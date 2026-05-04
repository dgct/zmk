/*
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 *
 * Dynamic BLE peripheral latency manager (Tier 1 + Tier 2 battery savings).
 *
 * Mirrors nRF Desktop's `applications/nrf_desktop/src/modules/ble_latency.c`
 * pattern: keep `BT_PERIPHERAL_PREF_LATENCY=0` while HID activity is flowing
 * (so the Mac never skips an ACK window — Bug 6 fix), but raise the slave
 * latency during quiet periods to recover the radio battery cost.
 *
 * Tiers:
 *   Warmup: CI=15ms (from PREF params), tighten to 7.5ms after
 *           CONFIG_ZMK_BLE_CONN_PARAM_WARMUP_MS (Apple two-phase dance)
 *   Active : latency=0,                                     CI preserved
 *   Idle-1 : latency=CONFIG_ZMK_BLE_HID_IDLE_LATENCY_INTERVALS
 *            (after CONFIG_ZMK_BLE_HID_IDLE_TIMEOUT_MS of HID silence),
 *            CI preserved
 *   Idle-2 : full ZMK idle (CONFIG_ZMK_IDLE_TIMEOUT) →
 *            CI=CONFIG_ZMK_BLE_DEEP_IDLE_INTERVAL_MS,
 *            latency=CONFIG_ZMK_BLE_DEEP_IDLE_LATENCY_INTERVALS,
 *            timeout=CONFIG_ZMK_BLE_DEEP_IDLE_TIMEOUT_S
 *
 * Critical patterns ported from nRF Desktop:
 *   - CONN_IS_SECURED first-class state bit; never raise latency before
 *     pairing completes.
 *   - Two-phase low-latency restore: if a previous param-update is still
 *     pending when input arrives, set CONN_LOW_LATENCY_REQUIRED and let
 *     the `le_param_updated` callback drive the actual restore.
 *   - For Idle-1: only the `latency` field changes; interval/timeout
 *     are read from `bt_conn_get_info` and preserved (handles Mac
 *     renegotiation gracefully).
 *
 * Concurrency: state bits live in `latency_state`, protected by
 * `state_lock`. Three contexts touch it: (a) BT RX thread via the conn
 * callbacks, (b) the ZMK event thread via the listeners, (c) the
 * system workqueue via the idle-check + input-activity work items.
 * The lock is held only for read-modify-write of `latency_state` and
 * `active_conn`; no BT API call ever runs under it.
 */

#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/input/input.h>
#include <zephyr/logging/log.h>

#include <zmk/activity.h>
#include <zmk/ble.h>
#include <zmk/event_manager.h>
#include <zmk/events/activity_state_changed.h>
#include <zmk/events/ble_host_param_request.h>
#include <zmk/events/keycode_state_changed.h>
#include <zmk/events/position_state_changed.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#define IDLE_TIMEOUT_MS K_MSEC(CONFIG_ZMK_BLE_HID_IDLE_TIMEOUT_MS)

BUILD_ASSERT(
    (CONFIG_ZMK_BLE_DEEP_IDLE_TIMEOUT_S * 400U) >
        ((1U + CONFIG_ZMK_BLE_DEEP_IDLE_LATENCY_INTERVALS) *
         (CONFIG_ZMK_BLE_DEEP_IDLE_INTERVAL_MS * 1000U / 1250U)),
    "ZMK_BLE_DEEP_IDLE_TIMEOUT_S too small for BLE spec: "
    "supervision_timeout must exceed (1 + latency) * interval / 4 "
    "(bt_conn_le_param_update will reject with -EINVAL)");

enum {
    CONN_LOW_LATENCY_ENABLED = BIT(0),
    CONN_LOW_LATENCY_REQUIRED = BIT(1),
    CONN_IS_SECURED = BIT(2),
    CONN_IN_DEEP_IDLE = BIT(3),
    CONN_UPDATE_PENDING = BIT(4),
    CONN_WARMUP_DONE = BIT(5),
    CONN_CI_SETTLED = BIT(6),
};

#define MAX_CI_RETRIES 3
static uint8_t ci_retry_count;
static uint16_t best_ci = CONFIG_ZMK_BLE_FAST_CI_INTERVAL;
static bool explore_ci;
static enum zmk_activity_state prev_activity_state = ZMK_ACTIVITY_ACTIVE;

static struct bt_conn *active_conn;
static uint8_t latency_state;
static struct k_spinlock state_lock;
static struct k_work_delayable idle_check_work;
static struct k_work_delayable warmup_work;
static struct k_work_delayable conn_update_timeout_work;

struct snapshot {
    struct bt_conn *conn;
    uint8_t state;
};

static struct snapshot take_snapshot(void) {
    struct snapshot s;
    k_spinlock_key_t key = k_spin_lock(&state_lock);
    s.conn = active_conn;
    s.state = latency_state;
    k_spin_unlock(&state_lock, key);
    return s;
}

static void state_set_bit(uint8_t bit) {
    k_spinlock_key_t key = k_spin_lock(&state_lock);
    latency_state |= bit;
    k_spin_unlock(&state_lock, key);
}

static void state_clear_bit(uint8_t bit) {
    k_spinlock_key_t key = k_spin_lock(&state_lock);
    latency_state &= ~bit;
    k_spin_unlock(&state_lock, key);
}

#define CONN_UPDATE_TIMEOUT_MS K_MSEC(5000)

static void conn_update_timeout_fn(struct k_work *work) {
    ARG_UNUSED(work);
    k_spinlock_key_t key = k_spin_lock(&state_lock);
    if (latency_state & CONN_UPDATE_PENDING) {
        latency_state &= ~CONN_UPDATE_PENDING;
        LOG_WRN("ble_latency: conn param update timed out (L2CAP rejection?)");
    }
    k_spin_unlock(&state_lock, key);
}

/* Issue a param update preserving the currently negotiated CI/timeout
 * and only changing the latency field (mirrors nRF Desktop pattern).
 * Caller must NOT hold state_lock.
 */
static int update_latency_only(struct bt_conn *conn, uint16_t new_latency) {
    struct bt_conn_info info;
    int err = bt_conn_get_info(conn, &info);
    if (err) {
        LOG_WRN("ble_latency: bt_conn_get_info failed (%d)", err);
        return err;
    }

    /* info.le.interval is already in 1.25ms units — the same units that
     * bt_le_conn_param expects for interval_min/max. No conversion needed.
     */
    struct bt_le_conn_param param = {
        .interval_min = info.le.interval,
        .interval_max = info.le.interval,
        .latency = new_latency,
        .timeout = info.le.timeout,
    };

    err = bt_conn_le_param_update(conn, &param);
    if (err && err != -EALREADY) {
        LOG_WRN("ble_latency: param update (latency=%u) failed (%d)", new_latency, err);
    }
    return err;
}

static void request_low_latency(void) {
    struct snapshot s = take_snapshot();
    if (!s.conn || !(s.state & CONN_IS_SECURED)) {
        return;
    }
    if (s.state & CONN_LOW_LATENCY_ENABLED) {
        explore_ci = false;
        k_work_reschedule(&idle_check_work, IDLE_TIMEOUT_MS);
        return;
    }
    if (s.state & CONN_UPDATE_PENDING) {
        state_set_bit(CONN_LOW_LATENCY_REQUIRED);
        k_work_reschedule(&idle_check_work, IDLE_TIMEOUT_MS);
        return;
    }

    /* If CI was widened by deep idle and warmup has completed, restore
     * the fast CI alongside the latency change. Without this, deep idle
     * recovery would leave CI at 20ms with only latency=0.
     */
    struct bt_conn_info info;
    if (bt_conn_get_info(s.conn, &info)) {
        return;
    }

    int err;
    uint16_t ci_target = best_ci;
    if (explore_ci) {
        ci_target = CONFIG_ZMK_BLE_FAST_CI_INTERVAL;
        explore_ci = false;
        LOG_INF("ble_latency: dormant recovery, exploring CI=%u", ci_target);
    }
    if ((s.state & CONN_WARMUP_DONE) && info.le.interval > ci_target) {
        struct bt_le_conn_param param = {
            .interval_min = ci_target,
            .interval_max = ci_target,
            .latency = 0,
            .timeout = info.le.timeout,
        };
        err = bt_conn_le_param_update(s.conn, &param);
    } else {
        err = update_latency_only(s.conn, 0);
    }

    if (err == 0) {
        state_set_bit(CONN_UPDATE_PENDING);
        k_work_reschedule(&conn_update_timeout_work, CONN_UPDATE_TIMEOUT_MS);
    }
    k_work_reschedule(&idle_check_work, IDLE_TIMEOUT_MS);
}

static void request_idle_1(void) {
    struct snapshot s = take_snapshot();
    if (!s.conn || !(s.state & CONN_IS_SECURED) || (s.state & CONN_UPDATE_PENDING)) {
        return;
    }
    int err = update_latency_only(s.conn, CONFIG_ZMK_BLE_HID_IDLE_LATENCY_INTERVALS);
    if (err == 0) {
        state_set_bit(CONN_UPDATE_PENDING);
        k_work_reschedule(&conn_update_timeout_work, CONN_UPDATE_TIMEOUT_MS);
    }
}

static void request_idle_2(void) {
    struct snapshot s = take_snapshot();
    if (!s.conn || !(s.state & CONN_IS_SECURED) || (s.state & CONN_UPDATE_PENDING)) {
        return;
    }

    uint16_t interval_units =
        BT_GAP_MS_TO_CONN_INTERVAL(CONFIG_ZMK_BLE_DEEP_IDLE_INTERVAL_MS);
    struct bt_le_conn_param param = {
        .interval_min = interval_units,
        .interval_max = interval_units,
        .latency = CONFIG_ZMK_BLE_DEEP_IDLE_LATENCY_INTERVALS,
        .timeout = CONFIG_ZMK_BLE_DEEP_IDLE_TIMEOUT_S * 100, /* 10ms units */
    };

    int err = bt_conn_le_param_update(s.conn, &param);
    if (err && err != -EALREADY) {
        LOG_WRN("ble_latency: deep idle update failed (%d)", err);
        return;
    }
    if (err == 0) {
        state_set_bit(CONN_UPDATE_PENDING);
        k_work_reschedule(&conn_update_timeout_work, CONN_UPDATE_TIMEOUT_MS);
    }
    /* -EALREADY: controller already has these params — no L2CAP procedure
     * was started, so on_le_param_updated won't fire.  Skip PENDING to
     * avoid blocking low-latency restore for the timeout duration. */
}

static void idle_check_fn(struct k_work *work) {
    ARG_UNUSED(work);
    struct snapshot s = take_snapshot();
    if (!s.conn || !(s.state & CONN_IS_SECURED)) {
        return;
    }
    if (s.state & CONN_IN_DEEP_IDLE) {
        return;
    }
    if (zmk_activity_get_state() != ZMK_ACTIVITY_ACTIVE) {
        return;
    }
    request_idle_1();
}

/* Apple two-phase conn param dance (Apple Accessory Design Guidelines +
 * RMK pattern): the host initially connects at 15ms CI (from
 * BT_PERIPHERAL_PREF_MIN_INT=12). After a warmup delay, we request the
 * fast CI (7.5ms). Apple rejects < 15ms on the first L2CAP conn param
 * update but accepts it once the link is established.
 */
static void warmup_fn(struct k_work *work) {
    ARG_UNUSED(work);
    struct snapshot s = take_snapshot();
    if (!s.conn || !(s.state & CONN_IS_SECURED) || (s.state & CONN_WARMUP_DONE)) {
        return;
    }

    struct bt_conn_info info;
    if (bt_conn_get_info(s.conn, &info)) {
        return;
    }

    if (info.le.interval <= CONFIG_ZMK_BLE_FAST_CI_INTERVAL) {
        /* Host already chose a fast CI (e.g. Windows/Linux) — skip. */
        state_set_bit(CONN_WARMUP_DONE);
        return;
    }

    if (s.state & CONN_UPDATE_PENDING) {
        /* A latency update is in flight — retry after it settles. */
        k_work_reschedule(&warmup_work, K_MSEC(500));
        return;
    }

    struct bt_le_conn_param param = {
        .interval_min = CONFIG_ZMK_BLE_FAST_CI_INTERVAL,
        .interval_max = CONFIG_ZMK_BLE_FAST_CI_INTERVAL,
        .latency = info.le.latency,
        .timeout = info.le.timeout,
    };

    int err = bt_conn_le_param_update(s.conn, &param);
    if (err == 0) {
        state_set_bit(CONN_UPDATE_PENDING);
        k_work_reschedule(&conn_update_timeout_work, CONN_UPDATE_TIMEOUT_MS);
    } else if (err == -EALREADY) {
        k_work_reschedule(&warmup_work, K_MSEC(500));
    } else {
        LOG_WRN("ble_latency: warmup CI tighten failed (%d)", err);
    }
}

static void on_connected(struct bt_conn *conn, uint8_t err) {
    struct bt_conn_info info;
    if (err || bt_conn_get_info(conn, &info) != 0) {
        return;
    }
    if (info.role != BT_CONN_ROLE_PERIPHERAL) {
        /* Split central→peripheral link is governed by
         * ZMK_SPLIT_BLE_PREF_LATENCY, not this module. We only manage
         * the host (Mac) link, where we are the BLE peripheral. */
        return;
    }
    k_spinlock_key_t key = k_spin_lock(&state_lock);
    active_conn = conn;
    latency_state = 0;
    best_ci = CONFIG_ZMK_BLE_FAST_CI_INTERVAL;
    ci_retry_count = 0;
    explore_ci = false;
    k_spin_unlock(&state_lock, key);
}

static void on_disconnected(struct bt_conn *conn, uint8_t reason) {
    ARG_UNUSED(reason);
    bool was_active = false;
    k_spinlock_key_t key = k_spin_lock(&state_lock);
    if (conn == active_conn) {
        active_conn = NULL;
        latency_state = 0;
        was_active = true;
    }
    k_spin_unlock(&state_lock, key);
    if (was_active) {
        k_work_cancel_delayable(&idle_check_work);
        k_work_cancel_delayable(&warmup_work);
        k_work_cancel_delayable(&conn_update_timeout_work);
    }
}

static void on_security_changed(struct bt_conn *conn, bt_security_t level,
                                enum bt_security_err err) {
    bool armed = false;
    k_spinlock_key_t key = k_spin_lock(&state_lock);
    if (conn == active_conn && !err && level >= BT_SECURITY_L2) {
        latency_state |= CONN_IS_SECURED;
        armed = true;
    }
    k_spin_unlock(&state_lock, key);
    if (armed) {
        k_work_reschedule(&idle_check_work, IDLE_TIMEOUT_MS);
        if (CONFIG_ZMK_BLE_CONN_PARAM_WARMUP_MS > 0) {
            k_work_reschedule(&warmup_work,
                              K_MSEC(CONFIG_ZMK_BLE_CONN_PARAM_WARMUP_MS));
        } else {
            state_set_bit(CONN_WARMUP_DONE);
        }
    }
}

static void on_le_param_updated(struct bt_conn *conn, uint16_t interval, uint16_t latency,
                                uint16_t timeout) {
    ARG_UNUSED(timeout);
    bool restore = false;
    k_spinlock_key_t key = k_spin_lock(&state_lock);
    if (conn == active_conn) {
        latency_state &= ~CONN_UPDATE_PENDING;
        if (interval <= CONFIG_ZMK_BLE_FAST_CI_INTERVAL) {
            latency_state |= CONN_WARMUP_DONE;
            ci_retry_count = 0;
            if (interval < best_ci) {
                best_ci = interval;
            }
        }
        if (latency == 0 && interval <= best_ci) {
            latency_state |= CONN_LOW_LATENCY_ENABLED;
            latency_state &= ~CONN_CI_SETTLED;
            if (interval < best_ci) {
                best_ci = interval;
            }
        } else if (latency == 0) {
            /* Central granted latency=0 but at wider CI than desired.
             * Retry fast CI unless we've exhausted attempts (Apple may
             * persistently refuse the aggressive interval). */
            if (ci_retry_count < MAX_CI_RETRIES &&
                (latency_state & CONN_WARMUP_DONE) &&
                !(latency_state & CONN_CI_SETTLED)) {
                ci_retry_count++;
                latency_state &= ~CONN_LOW_LATENCY_ENABLED;
                restore = true;
                LOG_INF("ble_latency: CI=%u > best_ci=%u, "
                        "retry %u/%u", interval,
                        best_ci, ci_retry_count, MAX_CI_RETRIES);
            } else {
                /* Accept wider CI — central insists */
                latency_state |= CONN_LOW_LATENCY_ENABLED;
                latency_state |= CONN_CI_SETTLED;
                if (!(latency_state & CONN_WARMUP_DONE)) {
                    latency_state |= CONN_WARMUP_DONE;
                }
                best_ci = interval;
                LOG_INF("ble_latency: accepted CI=%u as best_ci "
                        "(lat=0)", interval);
            }
        } else {
            latency_state &= ~CONN_LOW_LATENCY_ENABLED;
        }
        if (latency_state & CONN_LOW_LATENCY_REQUIRED) {
            latency_state &= ~CONN_LOW_LATENCY_REQUIRED;
            restore = true;
        }
    }
    k_spin_unlock(&state_lock, key);
    k_work_cancel_delayable(&conn_update_timeout_work);
    if (restore) {
        request_low_latency();
    }
}

BT_CONN_CB_DEFINE(ble_latency_conn_cb) = {
    .connected = on_connected,
    .disconnected = on_disconnected,
    .security_changed = on_security_changed,
    .le_param_updated = on_le_param_updated,
};

static int activity_event_listener(const zmk_event_t *eh) {
    const struct zmk_activity_state_changed *ev = as_zmk_activity_state_changed(eh);
    if (!ev) {
        return ZMK_EV_EVENT_BUBBLE;
    }
    switch (ev->state) {
    case ZMK_ACTIVITY_ACTIVE:
        state_clear_bit(CONN_IN_DEEP_IDLE | CONN_CI_SETTLED);
        ci_retry_count = 0;
        if (prev_activity_state == ZMK_ACTIVITY_SLEEP &&
            best_ci > CONFIG_ZMK_BLE_FAST_CI_INTERVAL) {
            explore_ci = true;
        }
        request_low_latency();
        break;
    case ZMK_ACTIVITY_IDLE:
        state_set_bit(CONN_IN_DEEP_IDLE);
        k_work_cancel_delayable(&idle_check_work);
        request_idle_2();
        break;
    case ZMK_ACTIVITY_SLEEP:
        break;
    }
    prev_activity_state = ev->state;
    return ZMK_EV_EVENT_BUBBLE;
}

static int hid_activity_listener(const zmk_event_t *eh) {
    ARG_UNUSED(eh);
    request_low_latency();
    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(ble_latency_activity, activity_event_listener);
ZMK_SUBSCRIPTION(ble_latency_activity, zmk_activity_state_changed);

ZMK_LISTENER(ble_latency_hid, hid_activity_listener);
ZMK_SUBSCRIPTION(ble_latency_hid, zmk_position_state_changed);
ZMK_SUBSCRIPTION(ble_latency_hid, zmk_keycode_state_changed);

#if IS_ENABLED(CONFIG_ZMK_POINTING)
/* TP/scroll motion goes through the Zephyr input subsystem and never
 * raises a ZMK event (it's converted directly to HID reports), so hook
 * the input subsystem the same way activity.c does. */
static void ble_latency_input_work_cb(struct k_work *_work) {
    ARG_UNUSED(_work);
    request_low_latency();
}
K_WORK_DEFINE(ble_latency_input_work, ble_latency_input_work_cb);

static void ble_latency_input_listener(struct input_event *ev, void *user_data) {
    ARG_UNUSED(ev);
    ARG_UNUSED(user_data);
    k_work_submit(&ble_latency_input_work);
}
INPUT_CALLBACK_DEFINE(NULL, ble_latency_input_listener, NULL);
#endif

/*
 * Host param request listener — single-writer interface for external modules
 * (e.g. subrating dormant) that need to change host connection parameters.
 * All host link param updates flow through ble_latency's state machine.
 */
static int host_param_request_listener(const zmk_event_t *eh) {
    const struct zmk_ble_host_param_request *req = as_zmk_ble_host_param_request(eh);
    if (!req) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    if (req->restore) {
        /* Caller wants us to restore our preferred params. If we're in
         * deep idle, request_idle_2 would re-apply deep-idle params.
         * If active, request_low_latency restores fast CI + lat=0. */
        struct snapshot s = take_snapshot();
        if (!s.conn || !(s.state & CONN_IS_SECURED)) {
            return ZMK_EV_EVENT_BUBBLE;
        }
        if (s.state & CONN_IN_DEEP_IDLE) {
            request_idle_2();
        } else {
            request_low_latency();
        }
        LOG_INF("ble_latency: host param restore requested");
        return ZMK_EV_EVENT_BUBBLE;
    }

    struct snapshot s = take_snapshot();
    if (!s.conn || !(s.state & CONN_IS_SECURED)) {
        return ZMK_EV_EVENT_BUBBLE;
    }
    if (s.state & CONN_UPDATE_PENDING) {
        /* Another update is in flight — log and skip. The dormant timer
         * will fire again or the caller can retry. */
        LOG_WRN("ble_latency: host param request dropped (update pending)");
        return ZMK_EV_EVENT_BUBBLE;
    }

    struct bt_le_conn_param param = {
        .interval_min = req->interval_min,
        .interval_max = req->interval_max,
        .latency = req->latency,
        .timeout = req->timeout,
    };

    int err = bt_conn_le_param_update(s.conn, &param);
    if (err == 0) {
        state_set_bit(CONN_UPDATE_PENDING);
        k_work_reschedule(&conn_update_timeout_work, CONN_UPDATE_TIMEOUT_MS);
        LOG_INF("ble_latency: host param request applied "
                "(CI=%u-%u, lat=%u)", req->interval_min,
                req->interval_max, req->latency);
    } else if (err != -EALREADY) {
        LOG_WRN("ble_latency: host param request failed (%d)", err);
    }

    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(ble_latency_host_param, host_param_request_listener);
ZMK_SUBSCRIPTION(ble_latency_host_param, zmk_ble_host_param_request);

static int ble_latency_init(void) {
    k_work_init_delayable(&idle_check_work, idle_check_fn);
    k_work_init_delayable(&warmup_work, warmup_fn);
    k_work_init_delayable(&conn_update_timeout_work, conn_update_timeout_fn);
    return 0;
}

SYS_INIT(ble_latency_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
