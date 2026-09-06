/*
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

/*
 * Shorter Connection Intervals (Core 6.2) on the split link, central side.
 *
 * Once a split peripheral link is encrypted and on 2M PHY, negotiate the
 * frame space (optional) and then the target connection interval through
 * the standard host APIs. Everything advances on the host's callbacks
 * (frame_space_updated, conn_rate_changed, le_param_updated); the only
 * timer is a guard that re-issues a request when a completion never
 * arrives. Controller-agnostic: the open LL and the SoftDevice both
 * implement the same HCI commands and events.
 */

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/hci_types.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#define SCI_INTERVAL_US CONFIG_ZMK_SPLIT_BLE_SCI_INTERVAL_US
#define SCI_TIMEOUT_10MS CONFIG_ZMK_SPLIT_BLE_SCI_SUPERVISION_TIMEOUT
#define SCI_GUARD_MS 3000    /* a completion that never arrives */
#define SCI_RETRY_MS 500     /* between attempts */
#define SCI_MAX_ATTEMPTS 3   /* per connection */
#define SCI_MAX_REAPPLY 5    /* re-requests after the interval left the target */
#define SCI_INTERVAL_SLACK_US 250

enum sci_state {
    SCI_IDLE,          /* connected, waiting for encryption + 2M PHY */
    SCI_FSU_PENDING,   /* frame space update requested */
    SCI_RATE_PENDING,  /* connection rate request sent */
    SCI_ACTIVE,        /* target interval confirmed */
    SCI_GIVEN_UP,      /* peer lacks the feature or attempts exhausted */
};

struct sci_link {
    struct bt_conn *conn;
    enum sci_state state;
    uint8_t attempts;
    uint8_t reapplies;
    struct k_work_delayable guard;
};

static void guard_handler(struct k_work *work);

static struct sci_link links[CONFIG_ZMK_SPLIT_BLE_CENTRAL_PERIPHERALS] = {
    [0 ...(CONFIG_ZMK_SPLIT_BLE_CENTRAL_PERIPHERALS - 1)] = {
        .guard = Z_WORK_DELAYABLE_INITIALIZER(guard_handler),
    },
};

static const char *state_name(enum sci_state s) {
    static const char *const names[] = {"idle", "fsu", "rate", "active", "given-up"};
    return s < ARRAY_SIZE(names) ? names[s] : "?";
}

static struct sci_link *link_for(struct bt_conn *conn) {
    for (size_t i = 0; i < ARRAY_SIZE(links); i++) {
        if (links[i].conn == conn) {
            return &links[i];
        }
    }
    return NULL;
}

static bool is_split_central_conn(struct bt_conn *conn) {
    struct bt_conn_info info;
    return bt_conn_get_info(conn, &info) == 0 && info.role == BT_CONN_ROLE_CENTRAL;
}

static void set_state(struct sci_link *link, enum sci_state s) {
    LOG_DBG("split SCI: %s -> %s", state_name(link->state), state_name(s));
    link->state = s;
}

static void request_rate(struct sci_link *link);

static void request_fsu(struct sci_link *link) {
#if IS_ENABLED(CONFIG_ZMK_SPLIT_BLE_SCI_FSU)
    const struct bt_conn_le_frame_space_update_param fsu = {
        .phys = BT_HCI_LE_FRAME_SPACE_UPDATE_PHY_2M_MASK,
        .spacing_types = BT_CONN_LE_FRAME_SPACE_TYPES_MASK_ACL_IFS,
        .frame_space_min = 0,
        .frame_space_max = 150,
    };
    int err = bt_conn_le_frame_space_update(link->conn, &fsu);
    if (err == 0) {
        set_state(link, SCI_FSU_PENDING);
        k_work_reschedule(&link->guard, K_MSEC(SCI_GUARD_MS));
        return;
    }
    LOG_DBG("split SCI: frame space update not started (%d)", err);
#endif
    request_rate(link);
}

static void request_rate(struct sci_link *link) {
    const struct bt_conn_le_conn_rate_param rate = {
        .interval_min_125us = SCI_INTERVAL_US / 125,
        .interval_max_125us = SCI_INTERVAL_US / 125,
        .subrate_min = 1,
        .subrate_max = 1,
        .max_latency = 0,
        .continuation_number = 0,
        .supervision_timeout_10ms = SCI_TIMEOUT_10MS,
        .min_ce_len_125us = 0,
        .max_ce_len_125us = 0,
    };

    link->attempts++;
    int err = bt_conn_le_conn_rate_request(link->conn, &rate);
    if (err == 0) {
        set_state(link, SCI_RATE_PENDING);
        k_work_reschedule(&link->guard, K_MSEC(SCI_GUARD_MS));
        LOG_INF("split SCI: requested %u us (attempt %u)", SCI_INTERVAL_US, link->attempts);
        return;
    }
    if (err == -ENOTSUP || err == -EOPNOTSUPP) {
        LOG_WRN("split SCI: not supported on this link (%d)", err);
        set_state(link, SCI_GIVEN_UP);
        return;
    }
    if (link->attempts >= SCI_MAX_ATTEMPTS) {
        LOG_WRN("split SCI: giving up after %u attempts (%d)", link->attempts, err);
        set_state(link, SCI_GIVEN_UP);
        return;
    }
    LOG_DBG("split SCI: request failed (%d), retrying", err);
    set_state(link, SCI_IDLE);
    k_work_reschedule(&link->guard, K_MSEC(SCI_RETRY_MS));
}

static void guard_handler(struct k_work *work) {
    struct k_work_delayable *dw = k_work_delayable_from_work(work);
    struct sci_link *link = CONTAINER_OF(dw, struct sci_link, guard);

    if (link->conn == NULL) {
        return;
    }
    switch (link->state) {
    case SCI_FSU_PENDING:
        LOG_DBG("split SCI: no frame space completion, requesting the rate anyway");
        request_rate(link);
        break;
    case SCI_RATE_PENDING:
        if (link->attempts >= SCI_MAX_ATTEMPTS) {
            LOG_WRN("split SCI: no rate change confirmation, giving up");
            set_state(link, SCI_GIVEN_UP);
        } else {
            LOG_DBG("split SCI: no rate change confirmation, retrying");
            request_rate(link);
        }
        break;
    case SCI_IDLE:
        /* Scheduled retry after a failed request. */
        request_rate(link);
        break;
    default:
        break;
    }
}

/* The interval left the target (a parameter update from the peer, or a rate
 * change that settled elsewhere): ask again, a bounded number of times. */
static void reapply(struct sci_link *link, uint32_t interval_us) {
    if (link->reapplies >= SCI_MAX_REAPPLY) {
        LOG_WRN("split SCI: interval is %u us and stays there (re-apply budget spent)",
                interval_us);
        set_state(link, SCI_GIVEN_UP);
        return;
    }
    link->reapplies++;
    link->attempts = 0;
    LOG_INF("split SCI: interval is %u us, re-requesting", interval_us);
    set_state(link, SCI_IDLE);
    k_work_reschedule(&link->guard, K_MSEC(SCI_RETRY_MS));
}

static bool link_ready(struct bt_conn *conn) {
    struct bt_conn_info info;

    if (bt_conn_get_security(conn) < BT_SECURITY_L2) {
        return false;
    }
    if (bt_conn_get_info(conn, &info)) {
        return false;
    }
    return info.le.phy != NULL && info.le.phy->tx_phy == BT_GAP_LE_PHY_2M;
}

static void maybe_start(struct bt_conn *conn) {
    struct sci_link *link = link_for(conn);

    if (link == NULL || link->state != SCI_IDLE || !link_ready(conn)) {
        return;
    }
    request_fsu(link);
}

/* ---- connection callbacks ---- */

static void sci_connected(struct bt_conn *conn, uint8_t err) {
    if (err || !is_split_central_conn(conn)) {
        return;
    }
    for (size_t i = 0; i < ARRAY_SIZE(links); i++) {
        if (links[i].conn == NULL) {
            links[i].conn = bt_conn_ref(conn);
            links[i].state = SCI_IDLE;
            links[i].attempts = 0;
            links[i].reapplies = 0;
            return;
        }
    }
    LOG_WRN("split SCI: no free link slot");
}

static void sci_disconnected(struct bt_conn *conn, uint8_t reason) {
    struct sci_link *link = link_for(conn);

    if (link == NULL) {
        return;
    }
    k_work_cancel_delayable(&link->guard);
    bt_conn_unref(link->conn);
    link->conn = NULL;
    link->state = SCI_IDLE;
}

static void sci_security_changed(struct bt_conn *conn, bt_security_t level,
                                 enum bt_security_err err) {
    if (err == 0 && level >= BT_SECURITY_L2) {
        maybe_start(conn);
    }
}

static void sci_phy_updated(struct bt_conn *conn, struct bt_conn_le_phy_info *param) {
    if (param->tx_phy == BT_GAP_LE_PHY_2M) {
        maybe_start(conn);
    }
}

#if IS_ENABLED(CONFIG_ZMK_SPLIT_BLE_SCI_FSU)
static void sci_frame_space_updated(struct bt_conn *conn,
                                    const struct bt_conn_le_frame_space_updated *params) {
    struct sci_link *link = link_for(conn);

    if (link == NULL || link->state != SCI_FSU_PENDING) {
        return;
    }
    k_work_cancel_delayable(&link->guard);
    if (params->status == BT_HCI_ERR_SUCCESS) {
        LOG_INF("split SCI: frame space %u us", params->frame_space);
    } else {
        LOG_DBG("split SCI: frame space update status 0x%02x", params->status);
    }
    request_rate(link);
}
#endif

static void sci_conn_rate_changed(struct bt_conn *conn, uint8_t status,
                                  const struct bt_conn_le_conn_rate_changed *params) {
    struct sci_link *link = link_for(conn);

    if (link == NULL) {
        return;
    }
    if (status == BT_HCI_ERR_SUCCESS) {
        k_work_cancel_delayable(&link->guard);
        link->attempts = 0;
        LOG_INF("split SCI: interval %u us, subrate %u, latency %u", params->interval_us,
                params->subrate_factor, params->peripheral_latency);
        if (params->interval_us <= SCI_INTERVAL_US + SCI_INTERVAL_SLACK_US) {
            set_state(link, SCI_ACTIVE);
        } else if (link->state != SCI_GIVEN_UP) {
            reapply(link, params->interval_us);
        }
        return;
    }
    if (link->state != SCI_RATE_PENDING) {
        return;
    }
    k_work_cancel_delayable(&link->guard);
    if (status == BT_HCI_ERR_UNSUPP_REMOTE_FEATURE || status == BT_HCI_ERR_UNSUPP_FEATURE_PARAM_VAL ||
        status == BT_HCI_ERR_UNSUPP_LL_PARAM_VAL) {
        LOG_WRN("split SCI: peer rejected the rate (0x%02x)", status);
        set_state(link, SCI_GIVEN_UP);
        return;
    }
    if (link->attempts >= SCI_MAX_ATTEMPTS) {
        LOG_WRN("split SCI: rate change failed (0x%02x), giving up", status);
        set_state(link, SCI_GIVEN_UP);
        return;
    }
    LOG_DBG("split SCI: rate change failed (0x%02x), retrying", status);
    set_state(link, SCI_IDLE);
    k_work_reschedule(&link->guard, K_MSEC(SCI_RETRY_MS));
}

static void sci_param_updated(struct bt_conn *conn, uint16_t interval, uint16_t latency,
                              uint16_t timeout) {
    struct sci_link *link = link_for(conn);
    uint32_t interval_us = (uint32_t)interval * 1250U;

    if (link == NULL || link->state != SCI_ACTIVE) {
        return;
    }
    if (interval_us <= SCI_INTERVAL_US + SCI_INTERVAL_SLACK_US) {
        return;
    }
    reapply(link, interval_us);
}

BT_CONN_CB_DEFINE(split_link_sci) = {
    .connected = sci_connected,
    .disconnected = sci_disconnected,
    .security_changed = sci_security_changed,
    .le_phy_updated = sci_phy_updated,
    .le_param_updated = sci_param_updated,
    .conn_rate_changed = sci_conn_rate_changed,
#if IS_ENABLED(CONFIG_ZMK_SPLIT_BLE_SCI_FSU)
    .frame_space_updated = sci_frame_space_updated,
#endif
};

static int split_link_sci_init(void) {
    LOG_INF("split SCI: target %u us, supervision %u ms, frame space update %s", SCI_INTERVAL_US,
            SCI_TIMEOUT_10MS * 10, IS_ENABLED(CONFIG_ZMK_SPLIT_BLE_SCI_FSU) ? "on" : "off");
    return 0;
}

SYS_INIT(split_link_sci_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
