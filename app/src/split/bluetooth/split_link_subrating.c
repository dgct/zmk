/*
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

/*
 * Activity-driven connection subrating on the split link (central role).
 * Standard host API only: the same code runs on the open link layer and on
 * the SoftDevice.
 *
 * Three tiers follow ZMK's activity state: ACTIVE while typing, IDLE after
 * ZMK_BLE_SUBRATE_IDLE_DELAY of idle, DORMANT after ZMK_BLE_SUBRATE_DORMANT_DELAY
 * or when ZMK sleeps. Every tier is a standard LE Subrate Request; the
 * controller confirms through the host's subrate_changed callback and a
 * bounded retry covers a request the controller could not serve yet (for
 * example while another LL procedure is running).
 *
 * The short connection interval itself is negotiated by the fork
 * (zmk/app/src/split/bluetooth/split_link_sci.c). A rate change resets the
 * subrate to 1, so the tier is re-applied on conn_rate_changed.
 */

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/hci_types.h>
#include <zephyr/logging/log.h>

#include <zmk/event_manager.h>
#include <zmk/events/activity_state_changed.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#define SUBRATE_TIMEOUT CONFIG_ZMK_BLE_SUBRATE_TIMEOUT
#define SUBRATE_IDLE_DELAY_MS CONFIG_ZMK_BLE_SUBRATE_IDLE_DELAY
#define SUBRATE_DORMANT_DELAY_MS CONFIG_ZMK_BLE_SUBRATE_DORMANT_DELAY

BUILD_ASSERT(CONFIG_ZMK_BLE_SUBRATE_ACTIVE_MAX >= CONFIG_ZMK_BLE_SUBRATE_ACTIVE_MIN, "ACTIVE_MAX >= ACTIVE_MIN");
BUILD_ASSERT(CONFIG_ZMK_BLE_SUBRATE_IDLE_MAX >= CONFIG_ZMK_BLE_SUBRATE_IDLE_MIN, "IDLE_MAX >= IDLE_MIN");
BUILD_ASSERT(CONFIG_ZMK_BLE_SUBRATE_DORMANT_MAX >= CONFIG_ZMK_BLE_SUBRATE_DORMANT_MIN, "DORMANT_MAX >= DORMANT_MIN");
BUILD_ASSERT(CONFIG_ZMK_BLE_SUBRATE_ACTIVE_CN < CONFIG_ZMK_BLE_SUBRATE_ACTIVE_MAX, "ACTIVE_CN < ACTIVE_MAX");
BUILD_ASSERT(CONFIG_ZMK_BLE_SUBRATE_IDLE_CN < CONFIG_ZMK_BLE_SUBRATE_IDLE_MAX, "IDLE_CN < IDLE_MAX");
BUILD_ASSERT(CONFIG_ZMK_BLE_SUBRATE_DORMANT_CN < CONFIG_ZMK_BLE_SUBRATE_DORMANT_MAX, "DORMANT_CN < DORMANT_MAX");

enum subrate_tier { TIER_ACTIVE, TIER_IDLE, TIER_DORMANT };

static const struct bt_conn_le_subrate_param tier_params[] = {
    [TIER_ACTIVE] = {.subrate_min = CONFIG_ZMK_BLE_SUBRATE_ACTIVE_MIN,
                     .subrate_max = CONFIG_ZMK_BLE_SUBRATE_ACTIVE_MAX,
                     .max_latency = CONFIG_ZMK_BLE_SUBRATE_ACTIVE_MAX_LATENCY,
                     .continuation_number = CONFIG_ZMK_BLE_SUBRATE_ACTIVE_CN,
                     .supervision_timeout = SUBRATE_TIMEOUT},
    [TIER_IDLE] = {.subrate_min = CONFIG_ZMK_BLE_SUBRATE_IDLE_MIN,
                   .subrate_max = CONFIG_ZMK_BLE_SUBRATE_IDLE_MAX,
                   .max_latency = CONFIG_ZMK_BLE_SUBRATE_IDLE_MAX_LATENCY,
                   .continuation_number = CONFIG_ZMK_BLE_SUBRATE_IDLE_CN,
                   .supervision_timeout = SUBRATE_TIMEOUT},
    [TIER_DORMANT] = {.subrate_min = CONFIG_ZMK_BLE_SUBRATE_DORMANT_MIN,
                      .subrate_max = CONFIG_ZMK_BLE_SUBRATE_DORMANT_MAX,
                      .max_latency = CONFIG_ZMK_BLE_SUBRATE_DORMANT_MAX_LATENCY,
                      .continuation_number = CONFIG_ZMK_BLE_SUBRATE_DORMANT_CN,
                      .supervision_timeout = SUBRATE_TIMEOUT},
};
static const char *const tier_names[] = {"ACTIVE", "IDLE", "DORMANT"};

/* Retry when the controller could not take the request yet: 20 ms, doubling
 * to 640 ms, then give up until the next tier change. */
#define TIER_MAX_RETRIES 6
#define TIER_RETRY_BASE_MS 20

static enum subrate_tier current_tier = TIER_ACTIVE;
static bool tier_confirmed = true;
static uint8_t tier_retry_count;

static void dormant_timer_handler(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(dormant_work, dormant_timer_handler);
static void idle_timer_handler(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(idle_work, idle_timer_handler);
static void tier_retry_handler(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(tier_retry_work, tier_retry_handler);

static void apply_subrate_to_conn(struct bt_conn *conn, void *data) {
    const struct bt_conn_le_subrate_param *params = data;
    struct bt_conn_info info;

    if (bt_conn_get_info(conn, &info)) {
        return;
    }
    if (info.role == BT_CONN_ROLE_CENTRAL && info.state == BT_CONN_STATE_CONNECTED) {
        int err = bt_conn_le_subrate_request(conn, params);
        if (err) {
            LOG_DBG("Subrate request failed: %d", err);
        }
    }
}

static void apply_tier(enum subrate_tier tier, bool force) {
    if (tier == current_tier && tier_confirmed && !force) {
        return;
    }
    if (tier != current_tier) {
        tier_retry_count = 0;
    }
    current_tier = tier;
    tier_confirmed = false;

    const struct bt_conn_le_subrate_param *params = &tier_params[tier];

    LOG_INF("Subrating: %s (factor=%d-%d, lat=%d, cn=%d)", tier_names[tier], params->subrate_min,
            params->subrate_max, params->max_latency, params->continuation_number);

    bt_conn_le_subrate_set_defaults(params);
    bt_conn_foreach(BT_CONN_TYPE_LE, apply_subrate_to_conn, (void *)params);

    uint32_t retry_ms = TIER_RETRY_BASE_MS << MIN(tier_retry_count, TIER_MAX_RETRIES - 1);
    k_work_reschedule(&tier_retry_work, K_MSEC(retry_ms));
}

static void set_tier(enum subrate_tier tier) { apply_tier(tier, false); }

static void tier_retry_handler(struct k_work *work) {
    if (tier_confirmed) {
        return;
    }
    if (++tier_retry_count >= TIER_MAX_RETRIES) {
        LOG_WRN("Subrate tier %s not confirmed after %d attempts", tier_names[current_tier],
                TIER_MAX_RETRIES);
        tier_confirmed = true;
        return;
    }
    LOG_DBG("Retrying subrate tier %s (%d/%d)", tier_names[current_tier], tier_retry_count,
            TIER_MAX_RETRIES);
    apply_tier(current_tier, true);
}

static void dormant_timer_handler(struct k_work *work) { set_tier(TIER_DORMANT); }

static void enter_idle_tier(void) {
    k_work_cancel_delayable(&dormant_work);
    set_tier(TIER_IDLE);
    k_work_schedule(&dormant_work, K_MSEC(SUBRATE_DORMANT_DELAY_MS));
}

static void idle_timer_handler(struct k_work *work) { enter_idle_tier(); }

static void subrate_active(void) {
    k_work_cancel_delayable(&idle_work);
    k_work_cancel_delayable(&dormant_work);
    set_tier(TIER_ACTIVE);
}

/* ZMK's idle state can arrive within a second or two of the last key (the
 * keyboard's idle timeout serves display and trackpad power management).
 * Hold the ACTIVE tier for SUBRATE_IDLE_DELAY_MS more so that ordinary typing
 * pauses do not park the link at the subrated IDLE cadence. */
static void subrate_idle(void) {
    if (SUBRATE_IDLE_DELAY_MS == 0) {
        enter_idle_tier();
        return;
    }
    k_work_cancel_delayable(&dormant_work);
    k_work_schedule(&idle_work, K_MSEC(SUBRATE_IDLE_DELAY_MS));
}

/* ---- connection callbacks ---- */

static void subrate_changed_cb(struct bt_conn *conn,
                               const struct bt_conn_le_subrate_changed *params) {
    struct bt_conn_info info;
    char addr_str[BT_ADDR_LE_STR_LEN];

    if (bt_conn_get_info(conn, &info) || info.role != BT_CONN_ROLE_CENTRAL) {
        return;
    }
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr_str, sizeof(addr_str));

    if (params->status == BT_HCI_ERR_SUCCESS) {
        LOG_INF("Subrating [%s]: factor=%d, cn=%d", addr_str, params->factor,
                params->continuation_number);
        tier_confirmed = true;
        tier_retry_count = 0;
        k_work_cancel_delayable(&tier_retry_work);
        return;
    }

    LOG_WRN("Subrating failed [%s]: 0x%02x", addr_str, params->status);
    if (params->status == BT_HCI_ERR_UNSUPP_REMOTE_FEATURE ||
        params->status == BT_HCI_ERR_UNSUPP_FEATURE_PARAM_VAL) {
        LOG_ERR("Subrating unsupported by the peer; no further requests");
        tier_confirmed = true;
        tier_retry_count = 0;
        k_work_cancel_delayable(&tier_retry_work);
    }
}

#if IS_ENABLED(CONFIG_BT_SHORTER_CONNECTION_INTERVALS)
/* A connection rate change (split_link_sci.c) carries its own subrate
 * parameters, so the tier is applied again once it lands. */
static void conn_rate_changed_cb(struct bt_conn *conn, uint8_t status,
                                 const struct bt_conn_le_conn_rate_changed *params) {
    struct bt_conn_info info;

    if (status != BT_HCI_ERR_SUCCESS || bt_conn_get_info(conn, &info) ||
        info.role != BT_CONN_ROLE_CENTRAL) {
        return;
    }
    if (params->subrate_factor != tier_params[current_tier].subrate_max) {
        tier_retry_count = 0;
        apply_tier(current_tier, true);
    }
}
#endif

/* A new central link runs the ACTIVE defaults (subrating_init). If the
 * keyboard is idle while the link comes up, apply the idle tier once the link
 * is encrypted: by then the feature exchange has run and the request is
 * accepted. */
static void security_changed_cb(struct bt_conn *conn, bt_security_t level,
                                enum bt_security_err err) {
    struct bt_conn_info info;

    if (err || level < BT_SECURITY_L2 || bt_conn_get_info(conn, &info) ||
        info.role != BT_CONN_ROLE_CENTRAL) {
        return;
    }
    if (current_tier != TIER_ACTIVE) {
        tier_retry_count = 0;
        apply_tier(current_tier, true);
    }
}

static void disconnected_cb(struct bt_conn *conn, uint8_t reason) {
    struct bt_conn_info info;

    if (bt_conn_get_info(conn, &info) || info.role != BT_CONN_ROLE_CENTRAL) {
        return;
    }
    k_work_cancel_delayable(&tier_retry_work);
    tier_confirmed = true;
    tier_retry_count = 0;
}

BT_CONN_CB_DEFINE(zmk_ble_subrating) = {
    .security_changed = security_changed_cb,
    .disconnected = disconnected_cb,
    .subrate_changed = subrate_changed_cb,
#if IS_ENABLED(CONFIG_BT_SHORTER_CONNECTION_INTERVALS)
    .conn_rate_changed = conn_rate_changed_cb,
#endif
};

/* ---- activity ---- */

static int activity_listener(const zmk_event_t *eh) {
    struct zmk_activity_state_changed *ev = as_zmk_activity_state_changed(eh);

    if (!ev) {
        return -ENOTSUP;
    }
    switch (ev->state) {
    case ZMK_ACTIVITY_ACTIVE:
        subrate_active();
        break;
    case ZMK_ACTIVITY_IDLE:
        subrate_idle();
        break;
    case ZMK_ACTIVITY_SLEEP:
        k_work_cancel_delayable(&idle_work);
        k_work_cancel_delayable(&dormant_work);
        set_tier(TIER_DORMANT);
        break;
    default:
        break;
    }
    return 0;
}

ZMK_LISTENER(zmk_ble_subrating, activity_listener);
ZMK_SUBSCRIPTION(zmk_ble_subrating, zmk_activity_state_changed);

static int subrating_init(void) {
    /* New connections start at the ACTIVE tier so that pairing, discovery,
     * the PHY update and the interval negotiation run at full rate. */
    bt_conn_le_subrate_set_defaults(&tier_params[TIER_ACTIVE]);
    LOG_INF("Subrating: active=%d-%d, idle=%d-%d (after %d s), dormant=%d-%d (after %d s)",
            CONFIG_ZMK_BLE_SUBRATE_ACTIVE_MIN, CONFIG_ZMK_BLE_SUBRATE_ACTIVE_MAX,
            CONFIG_ZMK_BLE_SUBRATE_IDLE_MIN, CONFIG_ZMK_BLE_SUBRATE_IDLE_MAX,
            SUBRATE_IDLE_DELAY_MS / 1000, CONFIG_ZMK_BLE_SUBRATE_DORMANT_MIN,
            CONFIG_ZMK_BLE_SUBRATE_DORMANT_MAX, SUBRATE_DORMANT_DELAY_MS / 1000);
    return 0;
}

SYS_INIT(subrating_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
