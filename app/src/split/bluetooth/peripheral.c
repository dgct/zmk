/*
 * Copyright (c) 2022 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/device.h>
#include <zephyr/init.h>

#include <errno.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

#include <zephyr/settings/settings.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci_types.h>

#include "peripheral.h"
#include "service.h"

#if IS_ENABLED(CONFIG_SETTINGS)

#include <zephyr/settings/settings.h>

#endif

#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#include <zmk/event_manager.h>
#include <zmk/events/split_peripheral_status_changed.h>
#include <zmk/events/activity_state_changed.h>
#include <zmk/ble.h>
#include <zmk/split/bluetooth/uuid.h>

static const struct bt_data zmk_ble_ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID16_SOME, 0x0f, 0x18 /* Battery Service */
                  ),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, ZMK_SPLIT_BT_SERVICE_UUID)};

static bool is_connected = false;

static bool is_bonded = false;

static bool enabled = false;

/* Reconnect advertising phases (bonded central only):
 *   DIRECTED  high-duty directed advertising to the bonded central: 3.75 ms
 *             events for 1.28 s, no payload. Ends with connected(err =
 *             BT_HCI_ERR_ADV_TIMEOUT) and a recycled() callback, which restarts
 *             advertising in the next phase.
 *   FAST      undirected + filter accept list at 30-60 ms for
 *             CONFIG_ZMK_SPLIT_BLE_PERIPHERAL_ADV_FAST_S seconds.
 *   NORMAL    undirected + filter accept list at 100-150 ms (stock ZMK).
 * A disconnect starts the sequence over. Both knobs default off, which is the
 * stock behaviour (NORMAL only). */
enum adv_phase { ADV_PHASE_DIRECTED, ADV_PHASE_FAST, ADV_PHASE_NORMAL };

#define ADV_FAST_S CONFIG_ZMK_SPLIT_BLE_PERIPHERAL_ADV_FAST_S

static enum adv_phase adv_phase_first(void) {
    if (IS_ENABLED(CONFIG_ZMK_SPLIT_BLE_PERIPHERAL_ADV_DIRECTED)) {
        return ADV_PHASE_DIRECTED;
    }
    return ADV_FAST_S > 0 ? ADV_PHASE_FAST : ADV_PHASE_NORMAL;
}

static enum adv_phase adv_phase = ADV_PHASE_NORMAL; /* set by set_enabled() */
static void adv_phase_handler(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(adv_phase_work, adv_phase_handler);

static void each_bond(const struct bt_bond_info *info, void *user_data) {
    bt_addr_le_t *addr = (bt_addr_le_t *)user_data;

    if (bt_addr_le_cmp(&info->addr, BT_ADDR_LE_NONE) != 0) {
        bt_addr_le_copy(addr, &info->addr);
    }
}

static int start_advertising(void) {
    bt_addr_le_t central_addr = bt_addr_le_none;

    bt_foreach_bond(BT_ID_DEFAULT, each_bond, &central_addr);

    if (bt_addr_le_cmp(&central_addr, BT_ADDR_LE_NONE) != 0) {
        is_bonded = true;

        // Undirected connectable advertising with a filter accept list
        // restricted to the bonded central; it advertises indefinitely with
        // the same peer restriction. The optional directed phase
        // (ZMK_SPLIT_BLE_PERIPHERAL_ADV_DIRECTED) runs before this for
        // 1.28 s and hands over here on its timeout.
        int err = bt_le_filter_accept_list_clear();
        if (err) {
            LOG_ERR("Failed to clear FAL (%d)", err);
            return err;
        }
        err = bt_le_filter_accept_list_add(&central_addr);
        if (err) {
            LOG_ERR("Failed to add central to FAL (%d)", err);
            return err;
        }

        if (adv_phase == ADV_PHASE_DIRECTED) {
            /* High duty cycle directed advertising carries no payload. */
            err = bt_le_adv_start(BT_LE_ADV_CONN_DIR(&central_addr), NULL, 0, NULL, 0);
            if (err == 0) {
                LOG_DBG("Directed advertising to the bonded central");
                return 0;
            }
            LOG_WRN("Directed advertising failed (%d), advertising undirected", err);
            adv_phase = ADV_FAST_S > 0 ? ADV_PHASE_FAST : ADV_PHASE_NORMAL;
        }

        bool fast = (adv_phase == ADV_PHASE_FAST);
        struct bt_le_adv_param adv_param = {
            .id = BT_ID_DEFAULT,
            .options = BT_LE_ADV_OPT_CONN | BT_LE_ADV_OPT_FILTER_CONN,
            .interval_min = fast ? BT_GAP_ADV_FAST_INT_MIN_1 : BT_GAP_ADV_FAST_INT_MIN_2,
            .interval_max = fast ? BT_GAP_ADV_FAST_INT_MAX_1 : BT_GAP_ADV_FAST_INT_MAX_2,
        };
        err = bt_le_adv_start(&adv_param, zmk_ble_ad, ARRAY_SIZE(zmk_ble_ad), NULL, 0);
        if (err == 0 && fast) {
            k_work_reschedule(&adv_phase_work, K_SECONDS(ADV_FAST_S));
        }
        return err;
    } else {
        is_bonded = false;
        return bt_le_adv_start(BT_LE_ADV_CONN_FAST_2, zmk_ble_ad, ARRAY_SIZE(zmk_ble_ad), NULL, 0);
    }
}

static void advertising_cb(struct k_work *work) {
    const int err = start_advertising();
    if (err < 0) {
        LOG_ERR("Failed to start advertising (%d)", err);
    } else {
        LOG_DBG("Split advertising started");
    }
}

K_WORK_DEFINE(advertising_work, advertising_cb);

/* FAST phase over: drop to the stock interval. Connectable advertising stops
 * by itself on connection, so a connected link means there is nothing to do. */
static void adv_phase_handler(struct k_work *work) {
    if (!enabled || is_connected || adv_phase != ADV_PHASE_FAST) {
        return;
    }
    adv_phase = ADV_PHASE_NORMAL;
    int err = bt_le_adv_stop();
    if (err < 0 && err != -EALREADY) {
        LOG_WRN("Failed to stop fast advertising (%d)", err);
    }
    k_work_submit(&advertising_work);
}

static void connected(struct bt_conn *conn, uint8_t err) {
    is_connected = (err == 0);

    if (err == 0) {
        k_work_cancel_delayable(&adv_phase_work);
    } else if (err == BT_HCI_ERR_ADV_TIMEOUT) {
        /* Directed phase elapsed without the central; recycled() restarts us. */
        adv_phase = ADV_FAST_S > 0 ? ADV_PHASE_FAST : ADV_PHASE_NORMAL;
        LOG_DBG("Directed advertising timed out, next phase %d", adv_phase);
    }

    raise_zmk_split_peripheral_status_changed(
        (struct zmk_split_peripheral_status_changed){.connected = is_connected});
}

static void recycled(void) {
    LOG_DBG("Connection recycled, restarting advertising (enabled=%d)", enabled);
    if (enabled) {
        k_work_submit(&advertising_work);
    }
}

static void disconnected(struct bt_conn *conn, uint8_t reason) {
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    LOG_DBG("Disconnected from %s (reason 0x%02x)", addr, reason);

    is_connected = false;
    adv_phase = adv_phase_first();

    raise_zmk_split_peripheral_status_changed(
        (struct zmk_split_peripheral_status_changed){.connected = is_connected});
}

static void security_changed(struct bt_conn *conn, bt_security_t level, enum bt_security_err err) {
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    if (!err) {
        LOG_DBG("Security changed: %s level %u", addr, level);
    } else {
        LOG_ERR("Security failed: %s level %u err %d", addr, level, err);
    }
}

static void le_param_updated(struct bt_conn *conn, uint16_t interval, uint16_t latency,
                             uint16_t timeout) {
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    LOG_DBG("%s: interval %d latency %d timeout %d", addr, interval, latency, timeout);
}

static struct bt_conn_cb conn_callbacks = {
    .connected = connected,
    .disconnected = disconnected,
    .recycled = recycled,
    .security_changed = security_changed,
    .le_param_updated = le_param_updated,
};

static void auth_pairing_complete(struct bt_conn *conn, bool bonded) { is_bonded = bonded; }

static struct bt_conn_auth_info_cb zmk_peripheral_ble_auth_info_cb = {
    .pairing_complete = auth_pairing_complete,
};

bool zmk_split_bt_peripheral_is_connected(void) { return is_connected; }

bool zmk_split_bt_peripheral_is_bonded(void) { return is_bonded; }

static zmk_split_transport_peripheral_status_changed_cb_t transport_status_cb;

static int
split_peripheral_bt_set_status_callback(zmk_split_transport_peripheral_status_changed_cb_t cb) {
    transport_status_cb = cb;
    return 0;
}

static void find_first_conn(struct bt_conn *conn, void *data) {
    struct bt_conn **cp = (struct bt_conn **)data;

    *cp = conn;
}

static int split_peripheral_bt_set_enabled(bool en) {
    int err;

    enabled = en;
    if (en) {
        adv_phase = adv_phase_first();
        k_work_submit(&advertising_work);
        return 0;
    } else {
        k_work_cancel_delayable(&adv_phase_work);
        struct bt_conn *conn = NULL;
        bt_conn_foreach(BT_CONN_TYPE_LE, find_first_conn, &conn);
        if (conn) {
            err = bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
            if (err < 0) {
                LOG_WRN("Failed to disconnect connection to central (%d)", err);
            }
        }

        err = bt_le_adv_stop();

        if (err < 0) {
            LOG_WRN("Failed to stop advertising (%d)", err);
        }

        return 0;
    }
}

static void notify_transport_status(void);

static void notify_status_work_cb(struct k_work *_work) { notify_transport_status(); }

static K_WORK_DEFINE(notify_status_work, notify_status_work_cb);

static bool settings_loaded = false;

static struct zmk_split_transport_status split_peripheral_bt_get_status(void) {
    return (struct zmk_split_transport_status){
        .available = !IS_ENABLED(CONFIG_ZMK_BLE_CLEAR_BONDS_ON_START) && settings_loaded,
        .enabled = enabled,
        .connections = zmk_split_bt_peripheral_is_connected()
                           ? ZMK_SPLIT_TRANSPORT_CONNECTIONS_STATUS_ALL_CONNECTED
                           : ZMK_SPLIT_TRANSPORT_CONNECTIONS_STATUS_DISCONNECTED,
    };
}

static const struct zmk_split_transport_peripheral_api peripheral_api = {
    .report_event = zmk_split_transport_peripheral_bt_report_event,
    .set_enabled = split_peripheral_bt_set_enabled,
    .set_status_callback = split_peripheral_bt_set_status_callback,
    .get_status = split_peripheral_bt_get_status,
};

ZMK_SPLIT_TRANSPORT_PERIPHERAL_REGISTER(bt_peripheral, &peripheral_api,
                                        CONFIG_ZMK_SPLIT_BLE_PRIORITY);

struct zmk_split_transport_peripheral *zmk_split_transport_peripheral_bt(void) {
    return &bt_peripheral;
}

static void notify_transport_status(void) {
    if (transport_status_cb) {
        transport_status_cb(&bt_peripheral, split_peripheral_bt_get_status());
    }
}

static int zmk_peripheral_ble_complete_startup(void) {
#if IS_ENABLED(CONFIG_ZMK_BLE_CLEAR_BONDS_ON_START)
    LOG_WRN("Clearing all existing BLE bond information from the keyboard");

    bt_unpair(BT_ID_DEFAULT, NULL);
#else
    bt_conn_cb_register(&conn_callbacks);
    bt_conn_auth_info_cb_register(&zmk_peripheral_ble_auth_info_cb);

    settings_loaded = true;
    k_work_submit(&notify_status_work);
#endif

    return 0;
}

#if IS_ENABLED(CONFIG_SETTINGS)

static int peripheral_ble_handle_set(const char *name, size_t len, settings_read_cb read_cb,
                                     void *cb_arg) {
    return 0;
}

static struct settings_handler ble_peripheral_settings_handler = {
    .name = "ble_peripheral",
    .h_set = peripheral_ble_handle_set,
    .h_commit = zmk_peripheral_ble_complete_startup};

#endif // IS_ENABLED(CONFIG_SETTINGS)

static int zmk_peripheral_ble_init(void) {
    int err = bt_enable(NULL);

    if (err) {
        LOG_ERR("BLUETOOTH FAILED (%d)", err);
        return err;
    }

#if IS_ENABLED(CONFIG_SETTINGS)
    settings_register(&ble_peripheral_settings_handler);
#else
    zmk_peripheral_ble_complete_startup();
#endif

    return 0;
}

SYS_INIT(zmk_peripheral_ble_init, APPLICATION, CONFIG_ZMK_BLE_INIT_PRIORITY);

/*
 * Quiesce the radio before deep sleep, as the central does (central.c stops
 * scanning and disconnects on ZMK_ACTIVITY_SLEEP) and as the ESB transport
 * does for its radio. With the SoftDevice Controller the radio and timer
 * interrupts are zero-latency and stay enabled through the irq_lock() in
 * sys_poweroff(); an advertising peripheral (this half sleeps within a
 * second of the link dropping, in the middle of directed advertising) can
 * therefore still be servicing radio events when SYSTEMOFF is written.
 * Stop advertising and drop any link first; re-enable on ACTIVE in case
 * the sleep is aborted.
 */
static bool disabled_for_sleep;

static int split_peripheral_bt_activity_listener(const zmk_event_t *eh) {
    const struct zmk_activity_state_changed *ev = as_zmk_activity_state_changed(eh);

    if (ev == NULL) {
        return ZMK_EV_EVENT_BUBBLE;
    }
    switch (ev->state) {
    case ZMK_ACTIVITY_SLEEP:
        if (enabled) {
            LOG_DBG("Sleep: stopping advertising and dropping the split link");
            disabled_for_sleep = true;
            split_peripheral_bt_set_enabled(false);
        }
        break;
    case ZMK_ACTIVITY_ACTIVE:
        if (disabled_for_sleep) {
            disabled_for_sleep = false;
            split_peripheral_bt_set_enabled(true);
        }
        break;
    default:
        break;
    }
    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(split_peripheral_bt_activity, split_peripheral_bt_activity_listener);
ZMK_SUBSCRIPTION(split_peripheral_bt_activity, zmk_activity_state_changed);
