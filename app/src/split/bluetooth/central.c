/*
 * Copyright (c) 2020 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/types.h>
#include <zephyr/init.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/byteorder.h>

#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#include <zmk/stdlib.h>
#include <zmk/ble.h>
#include <zmk/behavior.h>
#include <zmk/sensors.h>
#include <zmk/split/transport/central.h>
#include <zmk/split/bluetooth/uuid.h>
#include <zmk/split/bluetooth/service.h>
#include <zmk/event_manager.h>
#include <zmk/events/position_state_changed.h>
#include <zmk/events/sensor_event.h>
#include <zmk/events/battery_state_changed.h>
#include <zmk/events/activity_state_changed.h>
#include <zmk/pointing/input_split.h>
#include <zmk/hid_indicators_types.h>
#include <zmk/physical_layouts.h>
#include <zmk/activity.h>

static int start_scanning(void);
static void link_mgr_kick(void);
static int split_central_bt_set_enabled(bool enabled);

#define POSITION_STATE_DATA_LEN 16

/* Bug B: bound retries when bt_gatt_discover() fails on a freshly-connected
 * peripheral (e.g. ATT bearer not ready -> -ENOTCONN). After exhaustion we
 * disconnect so the standard reconnect/scan cycle gives us a clean slate.
 * 10 retries * K_MSEC(5) = 50ms window before giving up.
 */
#define MAX_CONN_PROCESS_RETRIES 10

/* Reconnect backoff after rapid disconnect storms.
 * If >3 disconnects within DISCONNECT_WINDOW_MS, exponentially delay
 * before restarting scan. Reset on successful GATT discovery. */
#define DISCONNECT_WINDOW_MS  10000
#define RECONNECT_DELAY_BASE_MS 100
#define RECONNECT_DELAY_MAX_MS 2000
static uint8_t consecutive_disconnect_count;
static int64_t last_disconnect_uptime;
static int64_t scan_hold_until; /* the link manager does not start a scan before this */
static void delayed_scan_handler(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(delayed_scan_work, delayed_scan_handler);

static void delayed_scan_handler(struct k_work *work) {
    link_mgr_kick();
}

enum peripheral_slot_state {
    PERIPHERAL_SLOT_STATE_OPEN,
    PERIPHERAL_SLOT_STATE_CONNECTING,
    PERIPHERAL_SLOT_STATE_CONNECTED,
};

struct peripheral_slot {
    enum peripheral_slot_state state;
    struct bt_conn *conn;
    struct bt_gatt_discover_params discover_params;
    struct bt_gatt_subscribe_params subscribe_params;
    struct bt_gatt_subscribe_params sensor_subscribe_params;
    struct bt_gatt_discover_params sub_discover_params;
    uint16_t run_behavior_handle;
    struct k_work_delayable conn_process_work;
    /* GATT discovery watchdog — disconnects if discovery hangs */
    struct k_work_delayable discovery_timeout_work;
#if IS_ENABLED(CONFIG_ZMK_SPLIT_BLE_CENTRAL_BATTERY_LEVEL_FETCHING)
    struct bt_gatt_subscribe_params batt_lvl_subscribe_params;
    struct bt_gatt_read_params batt_lvl_read_params;
#endif /* IS_ENABLED(CONFIG_ZMK_SPLIT_BLE_CENTRAL_BATTERY_LEVEL_FETCHING) */
#if IS_ENABLED(CONFIG_ZMK_SPLIT_PERIPHERAL_HID_INDICATORS)
    uint16_t update_hid_indicators;
#endif // IS_ENABLED(CONFIG_ZMK_SPLIT_PERIPHERAL_HID_INDICATORS)
    uint16_t selected_physical_layout_handle;
    uint8_t position_state[POSITION_STATE_DATA_LEN];
    uint8_t changed_positions[POSITION_STATE_DATA_LEN];
    /* Bug B: bounded retry counter for split_central_process_connection_work_handler */
    int conn_process_retries;
};

#if IS_ENABLED(CONFIG_ZMK_INPUT_SPLIT)

static const struct bt_uuid *gatt_ccc_uuid = BT_UUID_GATT_CCC;
static const struct bt_uuid *gatt_cpf_uuid = BT_UUID_GATT_CPF;

struct peripheral_input_slot {
    struct bt_conn *conn;
    struct bt_gatt_subscribe_params sub;
    uint8_t reg;
};

#define COUNT_INPUT_SPLIT(n) +1

static struct peripheral_input_slot
    peripheral_input_slots[(0 DT_FOREACH_STATUS_OKAY(zmk_input_split, COUNT_INPUT_SPLIT))];

static bool input_slot_is_open(size_t i) {
    return i < ARRAY_SIZE(peripheral_input_slots) && peripheral_input_slots[i].conn == NULL;
}

static bool input_slot_is_pending(size_t i) {
    return i < ARRAY_SIZE(peripheral_input_slots) && peripheral_input_slots[i].conn != NULL &&
           (!peripheral_input_slots[i].sub.value_handle ||
            !peripheral_input_slots[i].sub.ccc_handle || !peripheral_input_slots[i].reg);
}

static int reserve_next_open_input_slot(struct peripheral_input_slot **slot, struct bt_conn *conn) {
    for (size_t i = 0; i < ARRAY_SIZE(peripheral_input_slots); i++) {
        if (input_slot_is_open(i)) {
            peripheral_input_slots[i].conn = conn;

            // Clear out any previously set values
            peripheral_input_slots[i].sub.value_handle = 0;
            peripheral_input_slots[i].sub.ccc_handle = 0;
            peripheral_input_slots[i].reg = 0;
            *slot = &peripheral_input_slots[i];
            return i;
        }
    }

    return -ENOMEM;
}

static int find_pending_input_slot(struct peripheral_input_slot **slot, struct bt_conn *conn) {
    for (size_t i = 0; i < ARRAY_SIZE(peripheral_input_slots); i++) {
        if (peripheral_input_slots[i].conn == conn && input_slot_is_pending(i)) {
            *slot = &peripheral_input_slots[i];
            return i;
        }
    }

    return -ENODEV;
}

void release_peripheral_input_subs(struct bt_conn *conn) {
    for (size_t i = 0; i < ARRAY_SIZE(peripheral_input_slots); i++) {
        if (peripheral_input_slots[i].conn == conn) {
            peripheral_input_slots[i].conn = NULL;
            zmk_input_split_peripheral_disconnected(peripheral_input_slots[i].reg);
        }
    }
}

#endif // IS_ENABLED(CONFIG_ZMK_INPUT_SPLIT)

static zmk_split_transport_central_status_changed_cb_t transport_status_cb;
static bool is_enabled;

static struct peripheral_slot peripherals[ZMK_SPLIT_BLE_PERIPHERAL_COUNT];

/* Slot state is written from two contexts: reservation and confirmation on
 * the link manager (system work queue), release from the Bluetooth
 * connection callbacks. */
static K_MUTEX_DEFINE(slots_lock);

/* Scan state. Owned by the link manager: only link_mgr_handler() calls
 * bt_le_scan_start/stop and touches these. Every other path (connection
 * callbacks, the advertising report, timers, enable/disable) records intent
 * and calls link_mgr_kick(). */
static bool is_scanning = false;

#if IS_ENABLED(CONFIG_ZMK_SPLIT_BLE_CENTRAL_SCAN_BACKOFF)
/* Scan backoff: each scan starts at the stock full-duty parameters; after
 * CONFIG_ZMK_SPLIT_BLE_CENTRAL_SCAN_FAST_S the timer marks the slow phase due
 * and the link manager restarts the scan with the slow parameters. Both the
 * timer and the manager run on the system work queue, so there is no race
 * with the connection path. */
static bool scan_slow;      /* the running scan uses the slow parameters */
static bool scan_slow_due;  /* the fast phase has elapsed */
static void scan_slow_handler(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(scan_slow_work, scan_slow_handler);

static const struct bt_le_scan_param scan_slow_param = {
    .type = BT_LE_SCAN_TYPE_PASSIVE,
    .options = BT_LE_SCAN_OPT_FILTER_DUPLICATE,
    .interval = BT_GAP_MS_TO_SCAN_INTERVAL(CONFIG_ZMK_SPLIT_BLE_CENTRAL_SCAN_SLOW_INTERVAL_MS),
    .window = BT_GAP_MS_TO_SCAN_WINDOW(CONFIG_ZMK_SPLIT_BLE_CENTRAL_SCAN_SLOW_WINDOW_MS),
};
BUILD_ASSERT(CONFIG_ZMK_SPLIT_BLE_CENTRAL_SCAN_SLOW_WINDOW_MS <=
                 CONFIG_ZMK_SPLIT_BLE_CENTRAL_SCAN_SLOW_INTERVAL_MS,
             "slow scan window must not exceed the interval");
#endif

static const struct bt_uuid_128 split_service_uuid = BT_UUID_INIT_128(ZMK_SPLIT_BT_SERVICE_UUID);

struct peripheral_event_wrapper {
    uint8_t source;
    struct zmk_split_transport_peripheral_event event;
};

K_MSGQ_DEFINE(peripheral_event_msgq, sizeof(struct peripheral_event_wrapper),
              CONFIG_ZMK_SPLIT_BLE_CENTRAL_POSITION_QUEUE_SIZE, 4);

void peripheral_event_work_callback(struct k_work *work);

K_WORK_DEFINE(peripheral_event_work, peripheral_event_work_callback);

int peripheral_slot_index_for_conn(struct bt_conn *conn) {
    for (int i = 0; i < ZMK_SPLIT_BLE_PERIPHERAL_COUNT; i++) {
        if (peripherals[i].conn == conn) {
            return i;
        }
    }

    return -EINVAL;
}

struct peripheral_slot *peripheral_slot_for_conn(struct bt_conn *conn) {
    int idx = peripheral_slot_index_for_conn(conn);
    if (idx < 0) {
        return NULL;
    }

    return &peripherals[idx];
}

int release_peripheral_slot(int index) {
    if (index < 0 || index >= ZMK_SPLIT_BLE_PERIPHERAL_COUNT) {
        return -EINVAL;
    }

    struct peripheral_slot *slot = &peripherals[index];

    k_mutex_lock(&slots_lock, K_FOREVER);
    if (slot->state == PERIPHERAL_SLOT_STATE_OPEN) {
        k_mutex_unlock(&slots_lock);
        return -EINVAL;
    }

    LOG_DBG("Releasing peripheral slot at %d", index);

    k_work_cancel_delayable(&slot->conn_process_work);
    /* Cancel GATT discovery watchdog on slot release */
    k_work_cancel_delayable(&slot->discovery_timeout_work);

    if (slot->conn != NULL) {
        bt_conn_unref(slot->conn);
        slot->conn = NULL;
    }
    slot->state = PERIPHERAL_SLOT_STATE_OPEN;
    k_mutex_unlock(&slots_lock);

    // Raise events releasing any active positions from this peripheral
    for (int i = 0; i < POSITION_STATE_DATA_LEN; i++) {
        for (int j = 0; j < 8; j++) {
            if (slot->position_state[i] & BIT(j)) {
                uint32_t position = (i * 8) + j;
                struct peripheral_event_wrapper ev = {
                    .source = index,
                    .event = {.type = ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_KEY_POSITION_EVENT,
                              .data = {.key_position_event = {
                                           .position = position,
                                           .pressed = false,
                                       }}}};

                k_msgq_put(&peripheral_event_msgq, &ev, K_NO_WAIT);
                k_work_submit(&peripheral_event_work);
            }
        }
    }

    for (int i = 0; i < POSITION_STATE_DATA_LEN; i++) {
        slot->position_state[i] = 0U;
        slot->changed_positions[i] = 0U;
    }

    // Clean up previously discovered handles;
    slot->subscribe_params.value_handle = 0;
    slot->run_behavior_handle = 0;
    slot->selected_physical_layout_handle = 0;
    slot->conn_process_retries = 0;
#if IS_ENABLED(CONFIG_ZMK_SPLIT_PERIPHERAL_HID_INDICATORS)
    slot->update_hid_indicators = 0;
#endif // IS_ENABLED(CONFIG_ZMK_SPLIT_PERIPHERAL_HID_INDICATORS)

    return 0;
}

int reserve_peripheral_slot(const bt_addr_le_t *addr) {
    int i = zmk_ble_put_peripheral_addr(addr);
    if (i >= 0) {
        k_mutex_lock(&slots_lock, K_FOREVER);
        if (peripherals[i].state == PERIPHERAL_SLOT_STATE_OPEN) {
            // Be sure the slot is fully reinitialized (the mutex is recursive).
            release_peripheral_slot(i);
            peripherals[i].state = PERIPHERAL_SLOT_STATE_CONNECTING;
            k_mutex_unlock(&slots_lock);
            return i;
        }
        k_mutex_unlock(&slots_lock);
    }

    return -ENOMEM;
}

int release_peripheral_slot_for_conn(struct bt_conn *conn) {
    int idx = peripheral_slot_index_for_conn(conn);
    if (idx < 0) {
        return idx;
    }

    return release_peripheral_slot(idx);
}

int confirm_peripheral_slot_conn(struct bt_conn *conn) {
    int idx = peripheral_slot_index_for_conn(conn);
    if (idx < 0) {
        return idx;
    }

    k_mutex_lock(&slots_lock, K_FOREVER);
    peripherals[idx].state = PERIPHERAL_SLOT_STATE_CONNECTED;
    k_mutex_unlock(&slots_lock);
    return 0;
}

static void notify_transport_status(void);

static void notify_status_work_cb(struct k_work *_work) { notify_transport_status(); }

static K_WORK_DEFINE(notify_status_work, notify_status_work_cb);

#if ZMK_KEYMAP_HAS_SENSORS

static uint8_t split_central_sensor_notify_func(struct bt_conn *conn,
                                                struct bt_gatt_subscribe_params *params,
                                                const void *data, uint16_t length) {
    if (!data) {
        LOG_DBG("[UNSUBSCRIBED]");
        params->value_handle = 0U;
        return BT_GATT_ITER_STOP;
    }

    LOG_DBG("[SENSOR NOTIFICATION] data %p length %u", data, length);

    if (length < offsetof(struct sensor_event, channel_data)) {
        LOG_WRN("Ignoring sensor notify with insufficient data length (%d)", length);
        return BT_GATT_ITER_STOP;
    }

    struct sensor_event sensor_event;
    memcpy(&sensor_event, data, MIN(length, sizeof(sensor_event)));
    if (sensor_event.channel_data_size != 1) {
        return BT_GATT_ITER_STOP;
    }

    struct peripheral_event_wrapper event_wrapper = {
        .source = peripheral_slot_index_for_conn(conn),
        .event = {.type = ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_SENSOR_EVENT,
                  .data = {.sensor_event = {
                               .channel_data = sensor_event.channel_data[0],
                               .sensor_index = sensor_event.sensor_index,
                           }}}};

    k_msgq_put(&peripheral_event_msgq, &event_wrapper, K_NO_WAIT);
    k_work_submit(&peripheral_event_work);

    return BT_GATT_ITER_CONTINUE;
}
#endif /* ZMK_KEYMAP_HAS_SENSORS */

#if IS_ENABLED(CONFIG_ZMK_INPUT_SPLIT)

static uint8_t peripheral_input_event_notify_cb(struct bt_conn *conn,
                                                struct bt_gatt_subscribe_params *params,
                                                const void *data, uint16_t length) {
    if (!data) {
        LOG_DBG("[UNSUBSCRIBED]");
        params->value_handle = 0U;
        return BT_GATT_ITER_STOP;
    }

    LOG_DBG("[INPUT EVENT] data %p length %u", data, length);

    if (length != sizeof(struct zmk_split_input_event_payload)) {
        LOG_WRN("Ignoring input event notify with incorrect data length (%d)", length);
        return BT_GATT_ITER_STOP;
    }

    struct zmk_split_input_event_payload payload;
    memcpy(&payload, data, MIN(length, sizeof(struct zmk_split_input_event_payload)));

    for (size_t i = 0; i < ARRAY_SIZE(peripheral_input_slots); i++) {
        if (&peripheral_input_slots[i].sub == params) {
            struct peripheral_event_wrapper event_wrapper = {
                .source = peripheral_slot_index_for_conn(conn),
                .event = {.type = ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_INPUT_EVENT,
                          .data = {.input_event = {
                                       .reg = peripheral_input_slots[i].reg,
                                       .sync = payload.sync,
                                       .code = payload.code,
                                       .type = payload.type,
                                       .value = payload.value,
                                   }}}};

            k_msgq_put(&peripheral_event_msgq, &event_wrapper, K_NO_WAIT);
            k_work_submit(&peripheral_event_work);
            break;
        }
    }

    return BT_GATT_ITER_CONTINUE;
}

#endif

static uint8_t split_central_notify_func(struct bt_conn *conn,
                                         struct bt_gatt_subscribe_params *params, const void *data,
                                         uint16_t length) {
    struct peripheral_slot *slot = peripheral_slot_for_conn(conn);

    if (slot == NULL) {
        LOG_ERR("No peripheral state found for connection");
        return BT_GATT_ITER_CONTINUE;
    }

    if (!data) {
        LOG_DBG("[UNSUBSCRIBED]");
        params->value_handle = 0U;
        return BT_GATT_ITER_STOP;
    }

    LOG_DBG("[NOTIFICATION] data %p length %u", data, length);

    for (int i = 0; i < POSITION_STATE_DATA_LEN; i++) {
        slot->changed_positions[i] = ((uint8_t *)data)[i] ^ slot->position_state[i];
        slot->position_state[i] = ((uint8_t *)data)[i];
    }
    LOG_HEXDUMP_DBG(slot->position_state, POSITION_STATE_DATA_LEN, "data");

    for (int i = 0; i < POSITION_STATE_DATA_LEN; i++) {
        for (int j = 0; j < 8; j++) {
            if (slot->changed_positions[i] & BIT(j)) {
                uint32_t position = (i * 8) + j;
                bool pressed = slot->position_state[i] & BIT(j);
                struct peripheral_event_wrapper ev = {
                    .source = peripheral_slot_index_for_conn(conn),
                    .event = {.type = ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_KEY_POSITION_EVENT,
                              .data = {.key_position_event = {
                                           .position = position,
                                           .pressed = pressed,
                                       }}}};
                k_msgq_put(&peripheral_event_msgq, &ev, K_NO_WAIT);
                k_work_submit(&peripheral_event_work);
            }
        }
    }

    return BT_GATT_ITER_CONTINUE;
}

#if IS_ENABLED(CONFIG_ZMK_SPLIT_BLE_CENTRAL_BATTERY_LEVEL_FETCHING)

static uint8_t split_central_battery_level_notify_func(struct bt_conn *conn,
                                                       struct bt_gatt_subscribe_params *params,
                                                       const void *data, uint16_t length) {
    struct peripheral_slot *slot = peripheral_slot_for_conn(conn);

    if (!slot) {
        LOG_ERR("No peripheral state found for connection");
        return BT_GATT_ITER_CONTINUE;
    }

    if (!data) {
        LOG_DBG("[UNSUBSCRIBED]");
        params->value_handle = 0U;
        return BT_GATT_ITER_STOP;
    }

    if (length == 0) {
        LOG_ERR("Zero length battery notification received");
        return BT_GATT_ITER_CONTINUE;
    }

    LOG_DBG("[BATTERY LEVEL NOTIFICATION] data %p length %u", data, length);
    uint8_t battery_level = ((uint8_t *)data)[0];
    LOG_DBG("Battery level: %u", battery_level);

    struct peripheral_event_wrapper ev = {
        .source = peripheral_slot_index_for_conn(conn),
        .event = {.type = ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_BATTERY_EVENT,
                  .data = {.battery_event = {
                               .level = battery_level,
                           }}}};

    k_msgq_put(&peripheral_event_msgq, &ev, K_NO_WAIT);
    k_work_submit(&peripheral_event_work);

    return BT_GATT_ITER_CONTINUE;
}

static uint8_t split_central_battery_level_read_func(struct bt_conn *conn, uint8_t err,
                                                     struct bt_gatt_read_params *params,
                                                     const void *data, uint16_t length) {
    if (err > 0) {
        LOG_ERR("Error during reading peripheral battery level: %u", err);
        return BT_GATT_ITER_STOP;
    }

    struct peripheral_slot *slot = peripheral_slot_for_conn(conn);

    if (!slot) {
        LOG_ERR("No peripheral state found for connection");
        return BT_GATT_ITER_CONTINUE;
    }

    if (!data) {
        LOG_DBG("[READ COMPLETED]");
        return BT_GATT_ITER_STOP;
    }

    LOG_DBG("[BATTERY LEVEL READ] data %p length %u", data, length);

    if (length == 0) {
        LOG_ERR("Zero length battery notification received");
        return BT_GATT_ITER_CONTINUE;
    }

    uint8_t battery_level = ((uint8_t *)data)[0];

    LOG_DBG("Battery level: %u", battery_level);

    struct peripheral_event_wrapper ev = {
        .source = peripheral_slot_index_for_conn(conn),
        .event = {.type = ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_BATTERY_EVENT,
                  .data = {.battery_event = {
                               .level = battery_level,
                           }}}};

    k_msgq_put(&peripheral_event_msgq, &ev, K_NO_WAIT);
    k_work_submit(&peripheral_event_work);

    return BT_GATT_ITER_CONTINUE;
}

#endif /* IS_ENABLED(CONFIG_ZMK_SPLIT_BLE_CENTRAL_BATTERY_LEVEL_FETCHING) */

static int split_central_subscribe(struct bt_conn *conn, struct bt_gatt_subscribe_params *params) {
    atomic_set(params->flags, BT_GATT_SUBSCRIBE_FLAG_NO_RESUB);
    int err = bt_gatt_subscribe(conn, params);
    switch (err) {
    case -EALREADY:
        LOG_DBG("[ALREADY SUBSCRIBED]");
        break;
    case 0:
        LOG_DBG("[SUBSCRIBED]");
        break;
    default:
        LOG_ERR("Subscribe failed (err %d)", err);
        break;
    }

    return err;
}

static int update_peripheral_selected_layout(struct peripheral_slot *slot, uint8_t layout_idx) {
    if (slot->state != PERIPHERAL_SLOT_STATE_CONNECTED) {
        return -ENOTCONN;
    }

    if (slot->selected_physical_layout_handle == 0) {
        // It appears that sometimes the peripheral is considered connected
        // before the GATT characteristics have been discovered. If this is
        // the case, the selected_physical_layout_handle will not yet be set.
        return -EAGAIN;
    }

    if (bt_conn_get_security(slot->conn) < BT_SECURITY_L2) {
        return -EAGAIN;
    }

    int err = bt_gatt_write_without_response(slot->conn, slot->selected_physical_layout_handle,
                                             &layout_idx, sizeof(layout_idx), true);

    if (err < 0) {
        LOG_ERR("Failed to write physical layout index to peripheral (err %d)", err);
    }

    return err;
}

static void update_peripherals_selected_physical_layout(struct k_work *_work) {
    uint8_t layout_idx = zmk_physical_layouts_get_selected();
    for (int i = 0; i < ZMK_SPLIT_BLE_PERIPHERAL_COUNT; i++) {
        if (peripherals[i].state != PERIPHERAL_SLOT_STATE_CONNECTED) {
            continue;
        }

        update_peripheral_selected_layout(&peripherals[i], layout_idx);
    }
}

K_WORK_DEFINE(update_peripherals_selected_layouts_work,
              update_peripherals_selected_physical_layout);


static uint8_t split_central_chrc_discovery_func(struct bt_conn *conn,
                                                 const struct bt_gatt_attr *attr,
                                                 struct bt_gatt_discover_params *params) {
    if (!attr) {
        LOG_DBG("Discover complete");
        return BT_GATT_ITER_STOP;
    }

    if (!attr->user_data) {
        LOG_ERR("Required user data not passed to discovery");
        return BT_GATT_ITER_STOP;
    }

    struct peripheral_slot *slot = peripheral_slot_for_conn(conn);
    if (slot == NULL) {
        LOG_ERR("No peripheral state found for connection");
        return BT_GATT_ITER_STOP;
    }

    LOG_DBG("[ATTRIBUTE] handle %u", attr->handle);
    switch (params->type) {
    case BT_GATT_DISCOVER_CHARACTERISTIC: {
        const struct bt_uuid *chrc_uuid = ((struct bt_gatt_chrc *)attr->user_data)->uuid;

        if (bt_uuid_cmp(chrc_uuid, BT_UUID_DECLARE_128(ZMK_SPLIT_BT_CHAR_POSITION_STATE_UUID)) ==
            0) {
            LOG_DBG("Found position state characteristic");
            slot->subscribe_params.disc_params = &slot->sub_discover_params;
            slot->subscribe_params.end_handle = slot->discover_params.end_handle;
            slot->subscribe_params.value_handle = bt_gatt_attr_value_handle(attr);
            slot->subscribe_params.notify = split_central_notify_func;
            slot->subscribe_params.value = BT_GATT_CCC_NOTIFY;
            split_central_subscribe(conn, &slot->subscribe_params);
#if ZMK_KEYMAP_HAS_SENSORS
        } else if (bt_uuid_cmp(chrc_uuid,
                               BT_UUID_DECLARE_128(ZMK_SPLIT_BT_CHAR_SENSOR_STATE_UUID)) == 0) {
            slot->discover_params.uuid = NULL;
            slot->discover_params.start_handle = attr->handle + 2;
            slot->discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;

            slot->sensor_subscribe_params.disc_params = &slot->sub_discover_params;
            slot->sensor_subscribe_params.end_handle = slot->discover_params.end_handle;
            slot->sensor_subscribe_params.value_handle = bt_gatt_attr_value_handle(attr);
            slot->sensor_subscribe_params.notify = split_central_sensor_notify_func;
            slot->sensor_subscribe_params.value = BT_GATT_CCC_NOTIFY;
            split_central_subscribe(conn, &slot->sensor_subscribe_params);
#endif /* ZMK_KEYMAP_HAS_SENSORS */
#if IS_ENABLED(CONFIG_ZMK_INPUT_SPLIT)
        } else if (bt_uuid_cmp(chrc_uuid, BT_UUID_DECLARE_128(ZMK_SPLIT_BT_INPUT_EVENT_UUID)) ==
                   0) {
            LOG_DBG("Found an input characteristic");
            struct peripheral_input_slot *input_slot;
            int ret = reserve_next_open_input_slot(&input_slot, conn);
            if (ret < 0) {
                LOG_WRN("No available slot for peripheral input subscriptions (%d)", ret);

                slot->discover_params.uuid = NULL;
                slot->discover_params.start_handle = attr->handle + 1;
                slot->discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;
            } else {
                LOG_DBG("Reserved a slot for the input subscription");
                input_slot->sub.value_handle = bt_gatt_attr_value_handle(attr);

                slot->discover_params.uuid = gatt_ccc_uuid;
                slot->discover_params.start_handle = attr->handle;
                slot->discover_params.type = BT_GATT_DISCOVER_STD_CHAR_DESC;
            }
#endif // IS_ENABLED(CONFIG_ZMK_INPUT_SPLIT)
        } else if (bt_uuid_cmp(chrc_uuid,
                               BT_UUID_DECLARE_128(ZMK_SPLIT_BT_CHAR_RUN_BEHAVIOR_UUID)) == 0) {
            LOG_DBG("Found run behavior handle");
            slot->discover_params.uuid = NULL;
            slot->discover_params.start_handle = attr->handle + 2;
            slot->run_behavior_handle = bt_gatt_attr_value_handle(attr);
        } else if (!bt_uuid_cmp(((struct bt_gatt_chrc *)attr->user_data)->uuid,
                                BT_UUID_DECLARE_128(ZMK_SPLIT_BT_SELECT_PHYS_LAYOUT_UUID))) {
            LOG_DBG("Found select physical layout handle");
            slot->selected_physical_layout_handle = bt_gatt_attr_value_handle(attr);
            k_work_submit(&update_peripherals_selected_layouts_work);
#if IS_ENABLED(CONFIG_ZMK_SPLIT_PERIPHERAL_HID_INDICATORS)
        } else if (!bt_uuid_cmp(((struct bt_gatt_chrc *)attr->user_data)->uuid,
                                BT_UUID_DECLARE_128(ZMK_SPLIT_BT_UPDATE_HID_INDICATORS_UUID))) {
            LOG_DBG("Found update HID indicators handle");
            slot->update_hid_indicators = bt_gatt_attr_value_handle(attr);
#endif // IS_ENABLED(CONFIG_ZMK_SPLIT_PERIPHERAL_HID_INDICATORS)
#if IS_ENABLED(CONFIG_ZMK_SPLIT_BLE_CENTRAL_BATTERY_LEVEL_FETCHING)
        } else if (!bt_uuid_cmp(((struct bt_gatt_chrc *)attr->user_data)->uuid,
                                BT_UUID_BAS_BATTERY_LEVEL)) {
            LOG_DBG("Found battery level characteristics");
            slot->batt_lvl_subscribe_params.disc_params = &slot->sub_discover_params;
            slot->batt_lvl_subscribe_params.end_handle = slot->discover_params.end_handle;
            slot->batt_lvl_subscribe_params.value_handle = bt_gatt_attr_value_handle(attr);
            slot->batt_lvl_subscribe_params.notify = split_central_battery_level_notify_func;
            slot->batt_lvl_subscribe_params.value = BT_GATT_CCC_NOTIFY;
            split_central_subscribe(conn, &slot->batt_lvl_subscribe_params);

            slot->batt_lvl_read_params.func = split_central_battery_level_read_func;
            slot->batt_lvl_read_params.handle_count = 1;
            slot->batt_lvl_read_params.single.handle = bt_gatt_attr_value_handle(attr);
            slot->batt_lvl_read_params.single.offset = 0;
            bt_gatt_read(conn, &slot->batt_lvl_read_params);
#endif /* IS_ENABLED(CONFIG_ZMK_SPLIT_BLE_CENTRAL_BATTERY_LEVEL_FETCHING) */
        }
        break;
    }
    case BT_GATT_DISCOVER_STD_CHAR_DESC:
#if IS_ENABLED(CONFIG_ZMK_INPUT_SPLIT)
        if (bt_uuid_cmp(slot->discover_params.uuid, BT_UUID_GATT_CCC) == 0) {
            LOG_DBG("Found input CCC descriptor");
            struct peripheral_input_slot *input_slot;
            int ret = find_pending_input_slot(&input_slot, conn);
            if (ret < 0) {
                LOG_DBG("No pending input slot (%d)", ret);
                slot->discover_params.uuid = NULL;
                slot->discover_params.start_handle = attr->handle + 1;
                slot->discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;
            } else {
                LOG_DBG("Found pending input slot");
                input_slot->sub.ccc_handle = attr->handle;

                slot->discover_params.uuid = gatt_cpf_uuid;
                slot->discover_params.start_handle = attr->handle + 1;
                slot->discover_params.type = BT_GATT_DISCOVER_STD_CHAR_DESC;
            }
        } else if (bt_uuid_cmp(slot->discover_params.uuid, BT_UUID_GATT_CPF) == 0) {
            LOG_DBG("Found input CPF descriptor");
            struct bt_gatt_cpf *cpf = attr->user_data;
            struct peripheral_input_slot *input_slot;
            int ret = find_pending_input_slot(&input_slot, conn);
            if (ret < 0) {
                LOG_DBG("No pending input slot (%d)", ret);
            } else {
                LOG_DBG("Found pending input slot");
                input_slot->reg = cpf->description;
                input_slot->sub.notify = peripheral_input_event_notify_cb;
                input_slot->sub.value = BT_GATT_CCC_NOTIFY;
                int err = split_central_subscribe(conn, &input_slot->sub);
                if (err < 0) {
                    LOG_WRN("Failed to subscribe to input notifications %d", err);
                }
            }

            slot->discover_params.uuid = NULL;
            slot->discover_params.start_handle = attr->handle + 1;
            slot->discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;
        }
#endif // IS_ENABLED(CONFIG_ZMK_INPUT_SPLIT)
        break;
    }

    bool subscribed = slot->run_behavior_handle && slot->subscribe_params.value_handle &&
                      slot->selected_physical_layout_handle;

#if ZMK_KEYMAP_HAS_SENSORS
    subscribed = subscribed && slot->sensor_subscribe_params.value_handle;
#endif /* ZMK_KEYMAP_HAS_SENSORS */

#if IS_ENABLED(CONFIG_ZMK_SPLIT_PERIPHERAL_HID_INDICATORS)
    subscribed = subscribed && slot->update_hid_indicators;
#endif // IS_ENABLED(CONFIG_ZMK_SPLIT_PERIPHERAL_HID_INDICATORS)
#if IS_ENABLED(CONFIG_ZMK_SPLIT_BLE_CENTRAL_BATTERY_LEVEL_FETCHING)
    subscribed = subscribed && slot->batt_lvl_subscribe_params.value_handle;
#endif /* IS_ENABLED(CONFIG_ZMK_SPLIT_BLE_CENTRAL_BATTERY_LEVEL_FETCHING) */
#if IS_ENABLED(CONFIG_ZMK_INPUT_SPLIT)
    for (size_t i = 0; i < ARRAY_SIZE(peripheral_input_slots); i++) {
        if (input_slot_is_open(i) || input_slot_is_pending(i)) {
            subscribed = false;
            break;
        }
    }
#endif // IS_ENABLED(CONFIG_ZMK_INPUT_SPLIT)

    if (subscribed) {
        /* GATT discovery completed — cancel watchdog */
        k_work_cancel_delayable(&slot->discovery_timeout_work);
    }

    return subscribed ? BT_GATT_ITER_STOP : BT_GATT_ITER_CONTINUE;
}

/* GATT discovery watchdog — disconnects stalled connections */
static void discovery_timeout_handler(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct peripheral_slot *slot =
        CONTAINER_OF(dwork, struct peripheral_slot, discovery_timeout_work);
    if (slot->conn && !slot->subscribe_params.value_handle) {
        LOG_ERR("GATT discovery timed out after 5s — disconnecting");
        bt_conn_disconnect(slot->conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
    }
}

static uint8_t split_central_service_discovery_func(struct bt_conn *conn,
                                                    const struct bt_gatt_attr *attr,
                                                    struct bt_gatt_discover_params *params) {
    if (!attr) {
        LOG_DBG("Discover complete");
        (void)memset(params, 0, sizeof(*params));
        return BT_GATT_ITER_STOP;
    }

    LOG_DBG("[ATTRIBUTE] handle %u", attr->handle);

    struct peripheral_slot *slot = peripheral_slot_for_conn(conn);
    if (slot == NULL) {
        LOG_ERR("No peripheral state found for connection");
        return BT_GATT_ITER_STOP;
    }

    if (bt_uuid_cmp(slot->discover_params.uuid, BT_UUID_DECLARE_128(ZMK_SPLIT_BT_SERVICE_UUID)) !=
        0) {
        LOG_DBG("Found other service");
        return BT_GATT_ITER_CONTINUE;
    }

    LOG_DBG("Found split service");
    slot->discover_params.uuid = NULL;
    slot->discover_params.func = split_central_chrc_discovery_func;
    slot->discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;

    int err = bt_gatt_discover(conn, &slot->discover_params);
    if (err) {
        LOG_ERR("Failed to start discovering split service characteristics (err %d)", err);
    }
    return BT_GATT_ITER_STOP;
}

static void split_central_process_connection_work_handler(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct peripheral_slot *slot =
        CONTAINER_OF(dwork, struct peripheral_slot, conn_process_work);

    if (slot->state != PERIPHERAL_SLOT_STATE_CONNECTED || slot->conn == NULL) {
        return;
    }

    struct bt_conn *conn = slot->conn;
    int err;

    LOG_DBG("Current security for connection: %d", bt_conn_get_security(conn));

    /* Defensive: security_changed should trigger us only after encryption
     * is established. If we somehow run before L2, bail out silently —
     * security_changed will re-schedule us when encryption completes. */
    if (bt_conn_get_security(conn) < BT_SECURITY_L2) {
        LOG_WRN("GATT discovery deferred: encryption not yet established");
        return;
    }

    if (!slot->subscribe_params.value_handle) {
        slot->discover_params.uuid = &split_service_uuid.uuid;
        slot->discover_params.func = split_central_service_discovery_func;
        slot->discover_params.start_handle = 0x0001;
        slot->discover_params.end_handle = 0xffff;
        slot->discover_params.type = BT_GATT_DISCOVER_PRIMARY;

        err = bt_gatt_discover(slot->conn, &slot->discover_params);
        if (err) {
            // Bug B: previously only -ENOMEM was retried; any other error
            // (e.g. -ENOTCONN if ATT bearer isn't ready yet) left the
            // connection alive but unsubscribed forever, requiring a central
            // reset to recover. Retry all errors with bounded budget; on
            // exhaustion, disconnect so the standard reconnect path runs.
            if (++slot->conn_process_retries > MAX_CONN_PROCESS_RETRIES) {
                LOG_ERR("Discover failed after %d retries (err %d), disconnecting",
                        MAX_CONN_PROCESS_RETRIES, err);
                slot->conn_process_retries = 0;
                bt_conn_disconnect(slot->conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
                return;
            }
            LOG_WRN("Discover failed (err %d), retry %d/%d", err,
                    slot->conn_process_retries, MAX_CONN_PROCESS_RETRIES);
            k_work_schedule(&slot->conn_process_work, K_MSEC(5));
            return;
        }
        slot->conn_process_retries = 0;
        consecutive_disconnect_count = 0;  /* Stable link — reset backoff */
        /* Arm 5s GATT discovery watchdog. Cancelled by
         * release_peripheral_slot() on disconnect or by the
         * chrc_discovery_func when all subscriptions complete. */
        k_work_schedule(&slot->discovery_timeout_work, K_SECONDS(5));
    }

    struct bt_conn_info info;

    bt_conn_get_info(conn, &info);

    LOG_DBG("New connection params: Interval: %d, Latency: %d, PHY: %d", info.le.interval_us,
            info.le.latency, info.le.phy->rx_phy);

    // Restart scanning if necessary.
    link_mgr_kick();
}

static int stop_scanning(void) {
    LOG_DBG("Stopping peripheral scanning");
    is_scanning = false;
#if IS_ENABLED(CONFIG_ZMK_SPLIT_BLE_CENTRAL_SCAN_BACKOFF)
    scan_slow = false;
    scan_slow_due = false;
    k_work_cancel_delayable(&scan_slow_work);
#endif

    int err = bt_le_scan_stop();
    if (err == -EALREADY) {
        return 0;
    }
    if (err < 0) {
        LOG_ERR("Stop LE scan failed (err %d)", err);
        return err;
    }

    return 0;
}

/* ---- advertising reports: hand the address to the link manager ---- */

static bt_addr_le_t candidate_addr;
static bool candidate_valid;
static struct k_spinlock candidate_lock;

static void candidate_post(const bt_addr_le_t *addr) {
    k_spinlock_key_t key = k_spin_lock(&candidate_lock);
    bt_addr_le_copy(&candidate_addr, addr);
    candidate_valid = true;
    k_spin_unlock(&candidate_lock, key);
    link_mgr_kick();
}

static bool candidate_take(bt_addr_le_t *addr) {
    k_spinlock_key_t key = k_spin_lock(&candidate_lock);
    bool valid = candidate_valid;
    if (valid) {
        bt_addr_le_copy(addr, &candidate_addr);
        candidate_valid = false;
    }
    k_spin_unlock(&candidate_lock, key);
    return valid;
}

static bool split_central_eir_found(const bt_addr_le_t *addr) {
    LOG_DBG("Found the split service");
    candidate_post(addr);
    return false;
}

static bool split_central_eir_parse(struct bt_data *data, void *user_data) {
    bt_addr_le_t *addr = user_data;
    int i;

    LOG_DBG("[AD]: %u data_len %u", data->type, data->data_len);

    switch (data->type) {
    case BT_DATA_UUID128_SOME:
    case BT_DATA_UUID128_ALL:
        if (data->data_len % 16 != 0U) {
            LOG_ERR("AD malformed");
            return true;
        }

        for (i = 0; i < data->data_len; i += 16) {
            struct bt_uuid_128 uuid;

            if (!bt_uuid_create(&uuid.uuid, &data->data[i], 16)) {
                LOG_ERR("Unable to load UUID");
                continue;
            }

            if (bt_uuid_cmp(&uuid.uuid, BT_UUID_DECLARE_128(ZMK_SPLIT_BT_SERVICE_UUID)) != 0) {
                char uuid_str[BT_UUID_STR_LEN];
                char service_uuid_str[BT_UUID_STR_LEN];

                bt_uuid_to_str(&uuid.uuid, uuid_str, sizeof(uuid_str));
                bt_uuid_to_str(BT_UUID_DECLARE_128(ZMK_SPLIT_BT_SERVICE_UUID), service_uuid_str,
                               sizeof(service_uuid_str));
                LOG_DBG("UUID %s does not match split UUID: %s", uuid_str, service_uuid_str);
                continue;
            }

            return split_central_eir_found(addr);
        }
    }

    return true;
}

static void split_central_device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
                                       struct net_buf_simple *ad) {
    char dev[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(addr, dev, sizeof(dev));
    LOG_DBG("[DEVICE]: %s, AD evt type %u, AD data len %u, RSSI %i", dev, type, ad->len, rssi);

    /* We're only interested in connectable events */
    if (type == BT_GAP_ADV_TYPE_ADV_IND) {
        bt_data_parse(ad, split_central_eir_parse, (void *)addr);
    } else if (type == BT_GAP_ADV_TYPE_ADV_DIRECT_IND) {
        split_central_eir_found(addr);
    }
}

/* ---- scanning (link manager only) ---- */

static int start_scanning(void) {
    const struct bt_le_scan_param *param = BT_LE_SCAN_PASSIVE;

#if IS_ENABLED(CONFIG_ZMK_SPLIT_BLE_CENTRAL_SCAN_BACKOFF)
    bool slow = scan_slow_due;
    if (slow) {
        param = &scan_slow_param;
    }
#endif

    int err = bt_le_scan_start(param, split_central_device_found);
    if (err < 0) {
        LOG_ERR("Scanning failed to start (err %d)", err);
        return err;
    }
    is_scanning = true;

#if IS_ENABLED(CONFIG_ZMK_SPLIT_BLE_CENTRAL_SCAN_BACKOFF)
    scan_slow = slow;
    if (slow) {
        LOG_INF("Peripheral scan backed off to a %u ms window every %u ms",
                CONFIG_ZMK_SPLIT_BLE_CENTRAL_SCAN_SLOW_WINDOW_MS,
                CONFIG_ZMK_SPLIT_BLE_CENTRAL_SCAN_SLOW_INTERVAL_MS);
    } else {
        k_work_reschedule(&scan_slow_work, K_SECONDS(CONFIG_ZMK_SPLIT_BLE_CENTRAL_SCAN_FAST_S));
    }
#endif
    LOG_DBG("Scanning successfully started");
    return 0;
}

#if IS_ENABLED(CONFIG_ZMK_SPLIT_BLE_CENTRAL_SCAN_BACKOFF)
static void scan_slow_handler(struct k_work *work) {
    scan_slow_due = true;
    link_mgr_kick();
}
#endif

/* ---- link manager: the single owner of scan and connect decisions ---- */

static bool slots_connecting(void) {
    bool connecting = false;

    k_mutex_lock(&slots_lock, K_FOREVER);
    for (int i = 0; i < ZMK_SPLIT_BLE_PERIPHERAL_COUNT; i++) {
        if (peripherals[i].state == PERIPHERAL_SLOT_STATE_CONNECTING) {
            connecting = true;
            break;
        }
    }
    k_mutex_unlock(&slots_lock);
    return connecting;
}

static bool slots_have_open(void) {
    bool open = false;

    k_mutex_lock(&slots_lock, K_FOREVER);
    for (int i = 0; i < CONFIG_ZMK_SPLIT_BLE_CENTRAL_PERIPHERALS; i++) {
        if (peripherals[i].conn == NULL) {
            open = true;
            break;
        }
    }
    k_mutex_unlock(&slots_lock);
    return open;
}

static void try_connect(const bt_addr_le_t *addr) {
    // Reserve peripheral slot. Once the central has bonded to its peripherals,
    // the peripheral MAC addresses will be validated internally and the slot
    // reservation will fail if there is a mismatch.
    int slot_idx = reserve_peripheral_slot(addr);
    if (slot_idx < 0) {
        LOG_INF("Unable to reserve peripheral slot (err %d)", slot_idx);
        return;
    }
    struct peripheral_slot *slot = &peripherals[slot_idx];

    // Stop scanning so we can connect to the peripheral device.
    if (is_scanning && stop_scanning() < 0) {
        release_peripheral_slot(slot_idx);
        return;
    }

    LOG_DBG("Initiating new connection");
    struct bt_le_conn_param *param =
        BT_LE_CONN_PARAM(CONFIG_ZMK_SPLIT_BLE_PREF_INT, CONFIG_ZMK_SPLIT_BLE_PREF_INT,
                         CONFIG_ZMK_SPLIT_BLE_PREF_LATENCY, CONFIG_ZMK_SPLIT_BLE_PREF_TIMEOUT);
    int err = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN, param, &slot->conn);
    if (err < 0) {
        LOG_ERR("Create conn failed (err %d) (create conn? 0x%04x)", err, BT_HCI_OP_LE_CREATE_CONN);
        release_peripheral_slot(slot_idx);
    }
}

static void link_mgr_handler(struct k_work *work) {
    bt_addr_le_t addr;

    /* A candidate is consumed whether or not we can act on it: while a
     * connection is in progress, or when disabled, it is simply stale. */
    bool have_candidate = candidate_take(&addr);
    if (have_candidate && is_enabled && !slots_connecting()) {
        try_connect(&addr);
    }

    bool held = k_uptime_get() < scan_hold_until; /* disconnect-storm backoff */
    bool want_scan = is_enabled && slots_have_open() && !slots_connecting() && !held;

    if (!want_scan) {
        if (is_scanning) {
            if (is_enabled) {
                LOG_DBG("All devices are connected, scanning is unnecessary");
            } else {
                LOG_DBG("Not scanning, we're disabled");
            }
            stop_scanning();
        }
        return;
    }

    if (!is_scanning) {
        start_scanning();
        return;
    }

#if IS_ENABLED(CONFIG_ZMK_SPLIT_BLE_CENTRAL_SCAN_BACKOFF)
    if (scan_slow_due && !scan_slow) {
        /* The fast phase is over: restart with the slow parameters. */
        stop_scanning();
        scan_slow_due = true;
        start_scanning();
    }
#endif
}

static K_WORK_DEFINE(link_mgr_work, link_mgr_handler);

static void link_mgr_kick(void) { k_work_submit(&link_mgr_work); }

static void split_central_connected(struct bt_conn *conn, uint8_t conn_err) {
    char addr[BT_ADDR_LE_STR_LEN];
    struct bt_conn_info info;

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    bt_conn_get_info(conn, &info);

    if (info.role != BT_CONN_ROLE_CENTRAL) {
        LOG_DBG("SKIPPING FOR ROLE %d", info.role);
        return;
    }

    if (conn_err) {
        LOG_ERR("Failed to connect to %s (%u)", addr, conn_err);

        release_peripheral_slot_for_conn(conn);

        link_mgr_kick();
        return;
    }

    LOG_DBG("Connected: %s", addr);

    confirm_peripheral_slot_conn(conn);
    link_mgr_kick();

    /* Kick encryption — the security_changed callback will trigger GATT
     * discovery once LTK encryption is established. For bonded devices
     * the SoftDevice may auto-encrypt, but this ensures it. */
    bt_conn_set_security(conn, BT_SECURITY_L2);

    k_work_submit(&notify_status_work);
}

static void split_central_disconnected(struct bt_conn *conn, uint8_t reason) {
    char addr[BT_ADDR_LE_STR_LEN];
    int err;

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    LOG_DBG("Disconnected: %s (reason %d)", addr, reason);

#if IS_ENABLED(CONFIG_ZMK_SPLIT_BLE_CENTRAL_BATTERY_LEVEL_FETCHING) &&                             \
    IS_ENABLED(CONFIG_ZMK_SPLIT_BLE_CENTRAL_BATTERY_NOTIFY_DISCONNECT)
    /* Default-on for back-compat: raise a level=0 battery event so widgets
     * show the peripheral as offline. Disabling this prevents the brief
     * "0% then real value" flicker that hosts (e.g. macOS menu bar) show
     * on every transient peripheral reconnect; the central's GATT read
     * during re-discovery refreshes the real value within ms anyway. */
    struct peripheral_event_wrapper ev = {
        .source = peripheral_slot_index_for_conn(conn),
        .event = {.type = ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_BATTERY_EVENT,
                  .data = {.battery_event = {
                               .level = 0,
                           }}}};

    k_msgq_put(&peripheral_event_msgq, &ev, K_NO_WAIT);
    k_work_submit(&peripheral_event_work);
#endif // ..._BATTERY_LEVEL_FETCHING && ..._BATTERY_NOTIFY_DISCONNECT

#if IS_ENABLED(CONFIG_ZMK_INPUT_SPLIT)
    release_peripheral_input_subs(conn);
#endif

    err = release_peripheral_slot_for_conn(conn);

    if (err < 0) {
        LOG_WRN("Failed to release peripheral slot (%d)", err);
    }

    k_work_submit(&notify_status_work);

    /* Reconnect backoff after rapid disconnect storms.
     * Normal disconnects (count ≤ 3) scan immediately. Rapid storms
     * (>3 within 10s) get exponential backoff up to 2s. */
    int64_t now = k_uptime_get();
    if ((now - last_disconnect_uptime) > DISCONNECT_WINDOW_MS) {
        consecutive_disconnect_count = 0;
    }
    last_disconnect_uptime = now;
    consecutive_disconnect_count++;

    if (consecutive_disconnect_count > 3) {
        uint32_t delay = RECONNECT_DELAY_BASE_MS *
                         (1 << MIN(consecutive_disconnect_count - 4, 4));
        delay = MIN(delay, RECONNECT_DELAY_MAX_MS);
        LOG_WRN("Rapid disconnect #%d — backing off %u ms",
                consecutive_disconnect_count, delay);
        scan_hold_until = k_uptime_get() + delay;
        k_work_schedule(&delayed_scan_work, K_MSEC(delay));
    } else {
        link_mgr_kick();
    }
}

static void split_central_security_changed(struct bt_conn *conn, bt_security_t level,
                                           enum bt_security_err err) {
    struct peripheral_slot *slot = peripheral_slot_for_conn(conn);
    if (!slot) {
        return;
    }

    if (err) {
        LOG_ERR("Security failed for peripheral (err %d), disconnecting", err);
        bt_conn_disconnect(conn, BT_HCI_ERR_AUTH_FAIL);
        return;
    }

    if (level < BT_SECURITY_L2) {
        return;
    }

    /* Encryption established. If GATT discovery hasn't completed yet,
     * trigger it now. This is the event-driven entry point that replaces
     * the old poll-based security wait in conn_process_work. */
    if (!slot->subscribe_params.value_handle) {
        LOG_DBG("Security level %d reached, starting GATT discovery", level);
        k_work_schedule(&slot->conn_process_work, K_NO_WAIT);
    }

    /* Now that the link is encrypted, request 2M PHY. This replaces
     * CONFIG_BT_AUTO_PHY_CENTRAL_2M which fires PHY update BEFORE
     * encryption — competing for LL control procedure slots and
     * delaying security establishment by 60-120ms. */
    static const struct bt_conn_le_phy_param phy_2m = {
        .options = BT_CONN_LE_PHY_OPT_NONE,
        .pref_tx_phy = BT_GAP_LE_PHY_2M,
        .pref_rx_phy = BT_GAP_LE_PHY_2M,
    };
    int phy_err = bt_conn_le_phy_update(conn, &phy_2m);
    if (phy_err) {
        LOG_WRN("PHY update request failed: %d", phy_err);
    }

    /* If discovery already completed, push the physical layout. */
    if (slot->selected_physical_layout_handle) {
        k_work_submit(&update_peripherals_selected_layouts_work);
    }
}

static struct bt_conn_cb conn_callbacks = {
    .connected = split_central_connected,
    .disconnected = split_central_disconnected,
    .security_changed = split_central_security_changed,
};

struct central_cmd_wrapper {
    uint8_t source;
    struct zmk_split_transport_central_command cmd;
};

K_MSGQ_DEFINE(zmk_split_central_split_run_msgq, sizeof(struct central_cmd_wrapper),
              CONFIG_ZMK_SPLIT_BLE_CENTRAL_SPLIT_RUN_QUEUE_SIZE, 4);

#define SPLIT_RUN_MAX_RETRIES 5

static int split_run_retries;

static void split_central_split_run_callback(struct k_work *work);
K_WORK_DELAYABLE_DEFINE(split_central_split_run_work, split_central_split_run_callback);

static void split_central_split_run_callback(struct k_work *work) {
    struct central_cmd_wrapper payload_wrapper;

    LOG_DBG("");

    while (k_msgq_peek(&zmk_split_central_split_run_msgq, &payload_wrapper) == 0) {
        int err = 0;

        if (peripherals[payload_wrapper.source].state != PERIPHERAL_SLOT_STATE_CONNECTED) {
            LOG_ERR("Source not connected");
            goto dequeue;
        }

        switch (payload_wrapper.cmd.type) {
        case ZMK_SPLIT_TRANSPORT_CENTRAL_CMD_TYPE_INVOKE_BEHAVIOR: {
            if (!peripherals[payload_wrapper.source].run_behavior_handle) {
                LOG_ERR("Run behavior handle not found");
                goto dequeue;
            }

            struct zmk_split_run_behavior_payload payload = {
                .data = {
                    .param1 = payload_wrapper.cmd.data.invoke_behavior.param1,
                    .param2 = payload_wrapper.cmd.data.invoke_behavior.param2,
                    .position = payload_wrapper.cmd.data.invoke_behavior.position,
                    .source = payload_wrapper.cmd.data.invoke_behavior.event_source,
                    .state = payload_wrapper.cmd.data.invoke_behavior.state ? 1 : 0,
                }};
            const size_t payload_dev_size = sizeof(payload.behavior_dev);
            if (strlcpy(payload.behavior_dev, payload_wrapper.cmd.data.invoke_behavior.behavior_dev,
                        payload_dev_size) >= payload_dev_size) {
                LOG_ERR("Truncated behavior label %s to %s before invoking peripheral behavior",
                        payload_wrapper.cmd.data.invoke_behavior.behavior_dev,
                        payload.behavior_dev);
            }

            err = bt_gatt_write_without_response(
                peripherals[payload_wrapper.source].conn,
                peripherals[payload_wrapper.source].run_behavior_handle, &payload,
                sizeof(struct zmk_split_run_behavior_payload), true);

            if (err && err != -ENOMEM) {
                LOG_ERR("Failed to write the behavior characteristic (err %d)", err);
            }
            break;
        }
        case ZMK_SPLIT_TRANSPORT_CENTRAL_CMD_TYPE_SET_PHYSICAL_LAYOUT:
            err = update_peripheral_selected_layout(
                &peripherals[payload_wrapper.source],
                payload_wrapper.cmd.data.set_physical_layout.layout_idx);
            break;
#if IS_ENABLED(CONFIG_ZMK_SPLIT_PERIPHERAL_HID_INDICATORS)
        case ZMK_SPLIT_TRANSPORT_CENTRAL_CMD_TYPE_SET_HID_INDICATORS:
            LOG_WRN("do the indicators dance");
            if (peripherals[payload_wrapper.source].update_hid_indicators == 0) {
                // It appears that sometimes the peripheral is considered connected
                // before the GATT characteristics have been discovered. If this is
                // the case, the update_hid_indicators handle will not yet be set.
                LOG_WRN("NO HANDLE TO SET ON PERIPHERAL");
                goto dequeue;
            }

            err = bt_gatt_write_without_response(
                peripherals[payload_wrapper.source].conn,
                peripherals[payload_wrapper.source].update_hid_indicators,
                &payload_wrapper.cmd.data.set_hid_indicators.indicators,
                sizeof(payload_wrapper.cmd.data.set_hid_indicators.indicators), true);

            if (err && err != -ENOMEM) {
                LOG_ERR("Failed to write HID indicator characteristic (err %d)", err);
            }
            break;
#endif // IS_ENABLED(CONFIG_ZMK_SPLIT_PERIPHERAL_HID_INDICATORS)
        default:
            LOG_WRN("Unsupported wrapped central command type %d", payload_wrapper.cmd.type);
            goto dequeue;
        }

        if (err == -ENOMEM) {
            if (++split_run_retries > SPLIT_RUN_MAX_RETRIES) {
                LOG_WRN("ATT exhaustion: dropping split run queue after %d retries",
                        SPLIT_RUN_MAX_RETRIES);
                k_msgq_purge(&zmk_split_central_split_run_msgq);
                split_run_retries = 0;
                return;
            }
            k_work_schedule(&split_central_split_run_work, K_MSEC(2));
            return;
        }

    dequeue:
        k_msgq_get(&zmk_split_central_split_run_msgq, &payload_wrapper, K_NO_WAIT);
        split_run_retries = 0;
    }
}

static int split_bt_invoke_behavior_payload(struct central_cmd_wrapper payload_wrapper) {
    LOG_DBG("");

    int err = k_msgq_put(&zmk_split_central_split_run_msgq, &payload_wrapper, K_NO_WAIT);
    if (err) {
        switch (err) {
        case -ENOMSG:
        case -EAGAIN: {
            LOG_WRN("Run command message queue full, popping first message and queueing again");
            struct central_cmd_wrapper discarded_report;
            k_msgq_get(&zmk_split_central_split_run_msgq, &discarded_report, K_NO_WAIT);
            return split_bt_invoke_behavior_payload(payload_wrapper);
        }
        default:
            LOG_WRN("Failed to queue behavior to send (%d)", err);
            return err;
        }
    }

    k_work_schedule(&split_central_split_run_work, K_NO_WAIT);

    return 0;
};

static int finish_init();

static bool settings_loaded = false;

#if IS_ENABLED(CONFIG_SETTINGS)

static int central_ble_handle_set(const char *name, size_t len, settings_read_cb read_cb,
                                  void *cb_arg) {
    return 0;
}

static struct settings_handler ble_central_settings_handler = {
    .name = "ble_central", .h_set = central_ble_handle_set, .h_commit = finish_init};

#endif // IS_ENABLED(CONFIG_SETTINGS)

static int zmk_split_bt_central_init(void) {
    for (int i = 0; i < ZMK_SPLIT_BLE_PERIPHERAL_COUNT; i++) {
        k_work_init_delayable(&peripherals[i].conn_process_work,
                              split_central_process_connection_work_handler);
        k_work_init_delayable(&peripherals[i].discovery_timeout_work,
                              discovery_timeout_handler);
    }

    bt_conn_cb_register(&conn_callbacks);

#if IS_ENABLED(CONFIG_SETTINGS)
    settings_register(&ble_central_settings_handler);
    return 0;
#else
    return finish_init();
#endif // IS_ENABLED(CONFIG_SETTINGS)
}

SYS_INIT(zmk_split_bt_central_init, APPLICATION, CONFIG_ZMK_BLE_INIT_PRIORITY);

#if IS_ENABLED(CONFIG_ZMK_SPLIT_BLE_IDLE_CONN_PARAMS)

static void split_central_set_idle_conn_params(void) {
    struct bt_le_conn_param param = {
        .interval_min = CONFIG_ZMK_SPLIT_BLE_IDLE_CI,
        .interval_max = CONFIG_ZMK_SPLIT_BLE_IDLE_CI,
        .latency = CONFIG_ZMK_SPLIT_BLE_IDLE_LATENCY,
        .timeout = CONFIG_ZMK_SPLIT_BLE_IDLE_TIMEOUT,
    };

    for (int i = 0; i < ZMK_SPLIT_BLE_PERIPHERAL_COUNT; i++) {
        if (peripherals[i].state != PERIPHERAL_SLOT_STATE_CONNECTED || !peripherals[i].conn) {
            continue;
        }
        int err = bt_conn_le_param_update(peripherals[i].conn, &param);
        if (err && err != -EALREADY) {
            LOG_WRN("Failed to widen split CI for peripheral %d (%d)", i, err);
        }
    }
}

static void split_central_restore_fast_conn_params(void) {
    struct bt_le_conn_param param = {
        .interval_min = CONFIG_ZMK_SPLIT_BLE_PREF_INT,
        .interval_max = CONFIG_ZMK_SPLIT_BLE_PREF_INT,
        .latency = CONFIG_ZMK_SPLIT_BLE_PREF_LATENCY,
        .timeout = CONFIG_ZMK_SPLIT_BLE_PREF_TIMEOUT,
    };

    for (int i = 0; i < ZMK_SPLIT_BLE_PERIPHERAL_COUNT; i++) {
        if (peripherals[i].state != PERIPHERAL_SLOT_STATE_CONNECTED || !peripherals[i].conn) {
            continue;
        }
        int err = bt_conn_le_param_update(peripherals[i].conn, &param);
        if (err && err != -EALREADY) {
            LOG_WRN("Failed to restore split CI for peripheral %d (%d)", i, err);
        }
    }
}

#endif /* CONFIG_ZMK_SPLIT_BLE_IDLE_CONN_PARAMS */

static int zmk_split_bt_central_listener_cb(const zmk_event_t *eh) {
    if (as_zmk_physical_layout_selection_changed(eh)) {
        k_work_submit(&update_peripherals_selected_layouts_work);
    }

    const struct zmk_activity_state_changed *activity_ev = as_zmk_activity_state_changed(eh);
    if (activity_ev) {
        switch (activity_ev->state) {
        case ZMK_ACTIVITY_ACTIVE:
#if IS_ENABLED(CONFIG_ZMK_SPLIT_BLE_IDLE_CONN_PARAMS)
            split_central_restore_fast_conn_params();
#endif
            break;
        case ZMK_ACTIVITY_IDLE:
#if IS_ENABLED(CONFIG_ZMK_SPLIT_BLE_IDLE_CONN_PARAMS)
            split_central_set_idle_conn_params();
#endif
            break;
        case ZMK_ACTIVITY_SLEEP:
            LOG_DBG("Sleep state detected, disabling BLE central");
            split_central_bt_set_enabled(false);
            break;
        }
    }

    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(zmk_split_bt_central, zmk_split_bt_central_listener_cb);
ZMK_SUBSCRIPTION(zmk_split_bt_central, zmk_physical_layout_selection_changed);
ZMK_SUBSCRIPTION(zmk_split_bt_central, zmk_activity_state_changed);

static int split_central_bt_send_command(uint8_t source,
                                         struct zmk_split_transport_central_command cmd) {
    if (source >= ARRAY_SIZE(peripherals)) {
        return -EINVAL;
    }

    switch (cmd.type) {
    case ZMK_SPLIT_TRANSPORT_CENTRAL_CMD_TYPE_SET_HID_INDICATORS:
    case ZMK_SPLIT_TRANSPORT_CENTRAL_CMD_TYPE_SET_PHYSICAL_LAYOUT:
    case ZMK_SPLIT_TRANSPORT_CENTRAL_CMD_TYPE_INVOKE_BEHAVIOR: {
        struct central_cmd_wrapper wrapper = {.source = source, .cmd = cmd};
        return split_bt_invoke_behavior_payload(wrapper);
    }
    case ZMK_SPLIT_TRANSPORT_CENTRAL_CMD_TYPE_POLL_EVENTS:
        return -ENOTSUP;
    default:
        return -ENOTSUP;
    }

    return 0;
}

static int split_central_bt_get_available_source_ids(uint8_t *sources) {
    int count = 0;
    for (int i = 0; i < ZMK_SPLIT_BLE_PERIPHERAL_COUNT; i++) {
        if (peripherals[i].state != PERIPHERAL_SLOT_STATE_CONNECTED) {
            continue;
        }

        sources[count++] = i;
    }

    return count;
}

static int split_central_bt_set_enabled(bool enabled) {
    is_enabled = enabled;
    link_mgr_kick();
    if (enabled) {
        return 0;
    } else {
        int err;

        for (int i = 0; i < ZMK_SPLIT_BLE_PERIPHERAL_COUNT; i++) {
            if (peripherals[i].state != PERIPHERAL_SLOT_STATE_CONNECTED) {
                continue;
            }

            err = bt_conn_disconnect(peripherals[i].conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
            if (err < 0) {
                LOG_WRN("Failed to disconnect a peripheral (%d)", err);
            }
        }

        return 0;
    }
}

static int
split_central_bt_set_status_callback(zmk_split_transport_central_status_changed_cb_t cb) {
    transport_status_cb = cb;
    return 0;
}

static struct zmk_split_transport_status split_central_bt_get_status() {
    uint8_t _source_ids[ZMK_SPLIT_BLE_PERIPHERAL_COUNT];

    int count = split_central_bt_get_available_source_ids(_source_ids);

    enum zmk_split_transport_connections_status conn_status;

    if (count == 0) {
        conn_status = ZMK_SPLIT_TRANSPORT_CONNECTIONS_STATUS_DISCONNECTED;
    } else if (count == ZMK_SPLIT_BLE_PERIPHERAL_COUNT) {
        conn_status = ZMK_SPLIT_TRANSPORT_CONNECTIONS_STATUS_ALL_CONNECTED;
    } else {
        conn_status = ZMK_SPLIT_TRANSPORT_CONNECTIONS_STATUS_SOME_CONNECTED;
    }

    return (struct zmk_split_transport_status){
        .available = !IS_ENABLED(CONFIG_ZMK_BLE_CLEAR_BONDS_ON_START) && settings_loaded,
        .enabled = is_enabled,
        .connections = conn_status,
    };
}

static const struct zmk_split_transport_central_api central_api = {
    .send_command = split_central_bt_send_command,
    .get_available_source_ids = split_central_bt_get_available_source_ids,
    .set_enabled = split_central_bt_set_enabled,
    .set_status_callback = split_central_bt_set_status_callback,
    .get_status = split_central_bt_get_status,
};

ZMK_SPLIT_TRANSPORT_CENTRAL_REGISTER(bt_central, &central_api, CONFIG_ZMK_SPLIT_BLE_PRIORITY);

static void notify_transport_status(void) {
    if (transport_status_cb) {
        transport_status_cb(&bt_central, split_central_bt_get_status());
    }
}

static int finish_init() {
    settings_loaded = true;

    if (!transport_status_cb) {
        return 0;
    }

    return transport_status_cb(&bt_central, split_central_bt_get_status());
}

void peripheral_event_work_callback(struct k_work *work) {
    struct peripheral_event_wrapper ev;
    while (k_msgq_get(&peripheral_event_msgq, &ev, K_NO_WAIT) == 0) {
        LOG_DBG("Trigger key position state change of type %d", ev.event.type);
        zmk_split_transport_central_peripheral_event_handler(&bt_central, ev.source, ev.event);
    }
}
