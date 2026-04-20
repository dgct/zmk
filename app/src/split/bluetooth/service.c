/*
 * Copyright (c) 2020 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/drivers/sensor.h>
#include <zephyr/types.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/init.h>

#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>

#include <drivers/behavior.h>
#include <zmk/stdlib.h>
#include <zmk/behavior.h>
#include <zmk/matrix.h>
#include <zmk/physical_layouts.h>
#include <zmk/split/transport/peripheral.h>
#include <zmk/split/bluetooth/uuid.h>
#include <zmk/split/bluetooth/service.h>

#include "peripheral.h"

#include <zmk/split/bluetooth/peripheral.h>

#if IS_ENABLED(CONFIG_ZMK_SPLIT_PERIPHERAL_HID_INDICATORS)
#include <zmk/events/hid_indicators_changed.h>
#endif // IS_ENABLED(CONFIG_ZMK_SPLIT_PERIPHERAL_HID_INDICATORS)

#include <zmk/events/sensor_event.h>
#include <zmk/sensors.h>

#if ZMK_KEYMAP_HAS_SENSORS
static struct sensor_event last_sensor_event;

static ssize_t split_svc_sensor_state(struct bt_conn *conn, const struct bt_gatt_attr *attrs,
                                      void *buf, uint16_t len, uint16_t offset) {
    return bt_gatt_attr_read(conn, attrs, buf, len, offset, &last_sensor_event,
                             sizeof(last_sensor_event));
}

static void split_svc_sensor_state_ccc(const struct bt_gatt_attr *attr, uint16_t value) {
    LOG_DBG("value %d", value);
}
#endif /* ZMK_KEYMAP_HAS_SENSORS */

#define POS_STATE_LEN 16

static uint8_t num_of_positions = ZMK_KEYMAP_LEN;
static uint8_t position_state[POS_STATE_LEN];

static struct zmk_split_run_behavior_payload behavior_run_payload;

static ssize_t split_svc_pos_state(struct bt_conn *conn, const struct bt_gatt_attr *attrs,
                                   void *buf, uint16_t len, uint16_t offset) {
    return bt_gatt_attr_read(conn, attrs, buf, len, offset, &position_state,
                             sizeof(position_state));
}

static ssize_t split_svc_run_behavior(struct bt_conn *conn, const struct bt_gatt_attr *attrs,
                                      const void *buf, uint16_t len, uint16_t offset,
                                      uint8_t flags);

static ssize_t split_svc_num_of_positions(struct bt_conn *conn, const struct bt_gatt_attr *attrs,
                                          void *buf, uint16_t len, uint16_t offset) {
    return bt_gatt_attr_read(conn, attrs, buf, len, offset, attrs->user_data, sizeof(uint8_t));
}

static void split_svc_pos_state_ccc(const struct bt_gatt_attr *attr, uint16_t value) {
    LOG_DBG("value %d", value);
}

#if IS_ENABLED(CONFIG_ZMK_SPLIT_PERIPHERAL_HID_INDICATORS)

static zmk_hid_indicators_t hid_indicators = 0;

static void split_svc_update_indicators_callback(struct k_work *work) {
    LOG_DBG("Raising HID indicators changed event: %x", hid_indicators);
    raise_zmk_hid_indicators_changed(
        (struct zmk_hid_indicators_changed){.indicators = hid_indicators});
}

static K_WORK_DEFINE(split_svc_update_indicators_work, split_svc_update_indicators_callback);

static ssize_t split_svc_update_indicators(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                           const void *buf, uint16_t len, uint16_t offset,
                                           uint8_t flags) {
    if (offset + len > sizeof(zmk_hid_indicators_t)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    memcpy((uint8_t *)&hid_indicators + offset, buf, len);

    k_work_submit(&split_svc_update_indicators_work);

    return len;
}

#endif // IS_ENABLED(CONFIG_ZMK_SPLIT_PERIPHERAL_HID_INDICATORS)

static uint8_t selected_phys_layout = 0;

static void split_svc_select_phys_layout_callback(struct k_work *work) {
    LOG_DBG("Selecting physical layout after GATT write of %d", selected_phys_layout);
    zmk_physical_layouts_select(selected_phys_layout);
}

static K_WORK_DEFINE(split_svc_select_phys_layout_work, split_svc_select_phys_layout_callback);

static ssize_t split_svc_select_phys_layout(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                            const void *buf, uint16_t len, uint16_t offset,
                                            uint8_t flags) {
    if (offset + len > sizeof(uint8_t) || len == 0) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    selected_phys_layout = *(uint8_t *)buf;

    k_work_submit(&split_svc_select_phys_layout_work);

    return len;
}

static ssize_t split_svc_get_selected_phys_layout(struct bt_conn *conn,
                                                  const struct bt_gatt_attr *attrs, void *buf,
                                                  uint16_t len, uint16_t offset) {
    int selected_ret = zmk_physical_layouts_get_selected();
    if (selected_ret < 0) {
        return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    }

    uint8_t selected = (uint8_t)selected_ret;

    return bt_gatt_attr_read(conn, attrs, buf, len, offset, &selected, sizeof(selected));
}

#if IS_ENABLED(CONFIG_ZMK_INPUT_SPLIT)

static void split_input_events_ccc(const struct bt_gatt_attr *attr, uint16_t value) {
    LOG_DBG("value %d", value);
}

// Duplicated from Zephyr, since it is internal there
struct gatt_cpf {
    uint8_t format;
    int8_t exponent;
    uint16_t unit;
    uint8_t name_space;
    uint16_t description;
} __packed;

ssize_t bt_gatt_attr_read_input_split_cpf(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                          void *buf, uint16_t len, uint16_t offset) {
    uint16_t reg = (uint16_t)(uint32_t)attr->user_data;
    struct gatt_cpf value;

    value.format = 0x1B; // Struct
    value.exponent = 0;
    value.unit = sys_cpu_to_le16(0x2700); // Unitless
    value.name_space = 0x01;              // Bluetooth SIG
    value.description = sys_cpu_to_le16(reg);

    return bt_gatt_attr_read(conn, attr, buf, len, offset, &value, sizeof(value));
}

#define INPUT_SPLIT_CHARS(node_id)                                                                 \
    BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_128(ZMK_SPLIT_BT_INPUT_EVENT_UUID),                     \
                           BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_READ_ENCRYPT, NULL, NULL, NULL),      \
        BT_GATT_CCC(split_input_events_ccc,                                                        \
                    BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT),                       \
        BT_GATT_DESCRIPTOR(BT_UUID_GATT_CPF, BT_GATT_PERM_READ, bt_gatt_attr_read_input_split_cpf, \
                           NULL, (void *)DT_REG_ADDR(node_id)),

#endif

BT_GATT_SERVICE_DEFINE(
    split_svc, BT_GATT_PRIMARY_SERVICE(BT_UUID_DECLARE_128(ZMK_SPLIT_BT_SERVICE_UUID)),
    BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_128(ZMK_SPLIT_BT_CHAR_POSITION_STATE_UUID),
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_READ_ENCRYPT,
                           split_svc_pos_state, NULL, &position_state),
    BT_GATT_CCC(split_svc_pos_state_ccc, BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT),
    BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_128(ZMK_SPLIT_BT_CHAR_RUN_BEHAVIOR_UUID),
                           BT_GATT_CHRC_WRITE_WITHOUT_RESP, BT_GATT_PERM_WRITE_ENCRYPT, NULL,
                           split_svc_run_behavior, &behavior_run_payload),
    BT_GATT_DESCRIPTOR(BT_UUID_NUM_OF_DIGITALS, BT_GATT_PERM_READ, split_svc_num_of_positions, NULL,
                       &num_of_positions),
#if ZMK_KEYMAP_HAS_SENSORS
    BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_128(ZMK_SPLIT_BT_CHAR_SENSOR_STATE_UUID),
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_READ_ENCRYPT,
                           split_svc_sensor_state, NULL, &last_sensor_event),
    BT_GATT_CCC(split_svc_sensor_state_ccc, BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT),
#endif /* ZMK_KEYMAP_HAS_SENSORS */
    DT_FOREACH_STATUS_OKAY(zmk_input_split, INPUT_SPLIT_CHARS)
#if IS_ENABLED(CONFIG_ZMK_SPLIT_PERIPHERAL_HID_INDICATORS)
        BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_128(ZMK_SPLIT_BT_UPDATE_HID_INDICATORS_UUID),
                               BT_GATT_CHRC_WRITE_WITHOUT_RESP, BT_GATT_PERM_WRITE_ENCRYPT, NULL,
                               split_svc_update_indicators, NULL),
#endif // IS_ENABLED(CONFIG_ZMK_SPLIT_PERIPHERAL_HID_INDICATORS)
    BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_128(ZMK_SPLIT_BT_SELECT_PHYS_LAYOUT_UUID),
                           BT_GATT_CHRC_WRITE | BT_GATT_CHRC_READ,
                           BT_GATT_PERM_WRITE_ENCRYPT | BT_GATT_PERM_READ_ENCRYPT,
                           split_svc_get_selected_phys_layout, split_svc_select_phys_layout,
                           NULL), );

#define POSITION_NOTIFY_MAX_RETRIES 5
#define SENSOR_NOTIFY_MAX_RETRIES 5

K_MSGQ_DEFINE(position_state_msgq, sizeof(char[POS_STATE_LEN]),
              CONFIG_ZMK_SPLIT_BLE_PERIPHERAL_POSITION_QUEUE_SIZE, 4);

static int position_notify_retries;

static void send_position_state_callback(struct k_work *work);
K_WORK_DELAYABLE_DEFINE(service_position_notify_work, send_position_state_callback);

static void send_position_state_callback(struct k_work *work) {
    uint8_t state[POS_STATE_LEN];

    if (!zmk_split_bt_peripheral_is_connected()) {
        k_msgq_purge(&position_state_msgq);
        position_notify_retries = 0;
        return;
    }
    while (k_msgq_peek(&position_state_msgq, &state) == 0) {
        int err = bt_gatt_notify(NULL, &split_svc.attrs[1], &state, sizeof(state));
        if (err == -ENOMEM) {
            if (++position_notify_retries > POSITION_NOTIFY_MAX_RETRIES) {
                LOG_WRN("ATT exhaustion: dropping position notify after %d retries",
                        POSITION_NOTIFY_MAX_RETRIES);
                k_msgq_get(&position_state_msgq, &state, K_NO_WAIT);
                position_notify_retries = 0;
                continue;
            }
            k_work_schedule(&service_position_notify_work, K_MSEC(2));
            return;
        }
        if (err) {
            LOG_DBG("Error notifying %d", err);
        }
        k_msgq_get(&position_state_msgq, &state, K_NO_WAIT);
        position_notify_retries = 0;
    }
}

int send_position_state() {
    int err = k_msgq_put(&position_state_msgq, position_state, K_NO_WAIT);
    if (err) {
        switch (err) {
        case -EAGAIN: {
            LOG_WRN("Position state message queue full, popping first message and queueing again");
            uint8_t discarded_state[POS_STATE_LEN];
            k_msgq_get(&position_state_msgq, &discarded_state, K_NO_WAIT);
            return send_position_state();
        }
        default:
            LOG_WRN("Failed to queue position state to send (%d)", err);
            return err;
        }
    }

    k_work_schedule(&service_position_notify_work, K_NO_WAIT);

    return 0;
}

static int zmk_split_bt_position_pressed(uint8_t position) {
    WRITE_BIT(position_state[position / 8], position % 8, true);
    return send_position_state();
}

static int zmk_split_bt_position_released(uint8_t position) {
    WRITE_BIT(position_state[position / 8], position % 8, false);
    return send_position_state();
}

#if ZMK_KEYMAP_HAS_SENSORS
K_MSGQ_DEFINE(sensor_state_msgq, sizeof(struct sensor_event),
              CONFIG_ZMK_SPLIT_BLE_PERIPHERAL_POSITION_QUEUE_SIZE, 4);

static int sensor_notify_retries;

static void send_sensor_state_callback(struct k_work *work);
K_WORK_DELAYABLE_DEFINE(service_sensor_notify_work, send_sensor_state_callback);

static void send_sensor_state_callback(struct k_work *work) {
    while (k_msgq_peek(&sensor_state_msgq, &last_sensor_event) == 0) {
        int err = bt_gatt_notify(NULL, &split_svc.attrs[8], &last_sensor_event,
                                 sizeof(last_sensor_event));
        if (err == -ENOMEM) {
            if (++sensor_notify_retries > SENSOR_NOTIFY_MAX_RETRIES) {
                LOG_WRN("ATT exhaustion: dropping sensor notify after %d retries",
                        SENSOR_NOTIFY_MAX_RETRIES);
                k_msgq_get(&sensor_state_msgq, &last_sensor_event, K_NO_WAIT);
                sensor_notify_retries = 0;
                continue;
            }
            k_work_schedule(&service_sensor_notify_work, K_MSEC(2));
            return;
        }
        if (err) {
            LOG_DBG("Error notifying %d", err);
        }
        k_msgq_get(&sensor_state_msgq, &last_sensor_event, K_NO_WAIT);
        sensor_notify_retries = 0;
    }
}

int send_sensor_state(struct sensor_event ev) {
    int err = k_msgq_put(&sensor_state_msgq, &ev, K_NO_WAIT);
    if (err) {
        // retry...
        switch (err) {
        case -EAGAIN: {
            LOG_WRN("Sensor state message queue full, popping first message and queueing again");
            struct sensor_event discarded_state;
            k_msgq_get(&sensor_state_msgq, &discarded_state, K_NO_WAIT);
            return send_sensor_state(ev);
        }
        default:
            LOG_WRN("Failed to queue sensor state to send (%d)", err);
            return err;
        }
    }

    k_work_schedule(&service_sensor_notify_work, K_NO_WAIT);
    return 0;
}

static int zmk_split_bt_sensor_triggered(uint8_t sensor_index,
                                         const struct zmk_sensor_channel_data channel_data[],
                                         size_t channel_data_size) {
    if (channel_data_size > ZMK_SENSOR_EVENT_MAX_CHANNELS) {
        return -EINVAL;
    }

    struct sensor_event ev =
        (struct sensor_event){.sensor_index = sensor_index, .channel_data_size = channel_data_size};
    memcpy(ev.channel_data, channel_data,
           channel_data_size * sizeof(struct zmk_sensor_channel_data));
    return send_sensor_state(ev);
}
#endif /* ZMK_KEYMAP_HAS_SENSORS */

#if IS_ENABLED(CONFIG_ZMK_INPUT_SPLIT)

/* Counter of input notifies handed to the BLE host (bt_gatt_notify_cb returned
 * 0) but not yet completed via the per-tx callback. Bounded by
 * CONFIG_ZMK_INPUT_SPLIT_MAX_IN_FLIGHT. atomic_inc/atomic_dec are safe across
 * the worker (sysworkq) and the completion CB (conn_tx_workq).
 */
static atomic_t input_notify_in_flight_count = ATOMIC_INIT(0);
/* 32-bit ms timestamp of the OLDEST currently-in-flight notify. Set on the
 * 0->1 transition only; the value remains valid as long as count >= 1 (the
 * oldest hasn't completed). Cleared by the completion CB on the N->0
 * transition. Used solely by the 500 ms stuck-recovery path. Tear-free on
 * 32-bit targets via atomic_set/atomic_get. k_uptime_get_32() wraps every
 * ~49.7 days; unsigned subtraction is wraparound-safe for the 500 ms window.
 */
static atomic_t input_notify_oldest_send_time = ATOMIC_INIT(0);

bool zmk_split_bt_input_notify_ready(uint8_t reg) {
    atomic_val_t count = atomic_get(&input_notify_in_flight_count);
    if (count >= CONFIG_ZMK_INPUT_SPLIT_MAX_IN_FLIGHT) {
        uint32_t now = k_uptime_get_32();
        uint32_t oldest = (uint32_t)atomic_get(&input_notify_oldest_send_time);
        uint32_t elapsed = now - oldest;
        if (elapsed > 500) {
            LOG_WRN("input notify in-flight stuck (%d) for %u ms, resetting",
                    (int)count, elapsed);
            atomic_clear(&input_notify_in_flight_count);
            atomic_set(&input_notify_oldest_send_time, 0);
            return true;
        }
        return false;
    }
    return true;
}

void input_notify_work_cb(struct k_work *work);
K_WORK_DELAYABLE_DEFINE(input_notify_work, input_notify_work_cb);

static void input_notify_complete(struct bt_conn *conn, void *user_data) {
    /* atomic_dec returns the value BEFORE decrement (Zephyr API mirrors
     * __atomic_fetch_sub). prev == 1 means new count is 0 — the last
     * in-flight has completed and the oldest timestamp is no longer
     * meaningful.
     */
    atomic_val_t prev = atomic_dec(&input_notify_in_flight_count);
    if (prev == 1) {
        atomic_set(&input_notify_oldest_send_time, 0);
    }
    /* Wake the worker to drain the next queued entry (and refill in-flight
     * slots) now that one has freed up.
     */
    k_work_schedule(&input_notify_work, K_NO_WAIT);
}

#define INPUT_NOTIFY_MAX_RETRIES 10

/* A single-slot 'pending_input_payload' was racy: a producer (input handler
 * thread) can call zmk_split_bt_report_input twice within one input-event
 * dispatch (e.g. a button release with sync=true triggers both the pass-through
 * else-branch and the coalesce flush block in input_split.c). The worker on
 * sysworkq cannot run between those two calls, so the second write overwrote
 * the first and the first event was silently dropped.
 *
 * Use a small msgq, drop-oldest on overflow (matches sensor_state_msgq).
 */
struct input_notify_entry {
    const struct bt_gatt_attr *attr;
    struct zmk_split_input_event_payload payload;
};

#define INPUT_NOTIFY_QUEUE_DEPTH 16

K_MSGQ_DEFINE(input_notify_msgq, sizeof(struct input_notify_entry), INPUT_NOTIFY_QUEUE_DEPTH, 4);

/* Worker-private state: only touched from input_notify_work_cb (sysworkq). */
static struct input_notify_entry current_entry;
static bool current_entry_held; /* true when retrying current_entry on -ENOMEM */
static int input_notify_retries;

void input_notify_work_cb(struct k_work *work) {
    if (!zmk_split_bt_peripheral_is_connected()) {
        struct input_notify_entry discard;
        while (k_msgq_get(&input_notify_msgq, &discard, K_NO_WAIT) == 0) {
        }
        current_entry_held = false;
        input_notify_retries = 0;
        atomic_clear(&input_notify_in_flight_count);
        atomic_set(&input_notify_oldest_send_time, 0);
        return;
    }

    /* Stuck-recovery: if all slots are taken and the oldest hasn't completed
     * in 500 ms, the LL/peer is wedged. Reset the counter so we stop blocking
     * on phantom in-flights. Mirrors the producer-side check in
     * zmk_split_bt_input_notify_ready().
     */
    if (atomic_get(&input_notify_in_flight_count) >=
        CONFIG_ZMK_INPUT_SPLIT_MAX_IN_FLIGHT) {
        uint32_t now = k_uptime_get_32();
        uint32_t oldest = (uint32_t)atomic_get(&input_notify_oldest_send_time);
        uint32_t elapsed = now - oldest;
        if (elapsed <= 500) {
            return; /* Wait for a completion CB to wake us. */
        }
        LOG_WRN("input notify in-flight stuck (%ld) for %u ms, recovering",
                (long)atomic_get(&input_notify_in_flight_count), elapsed);
        atomic_clear(&input_notify_in_flight_count);
        atomic_set(&input_notify_oldest_send_time, 0);
    }

    /* Drain the msgq up to MAX_IN_FLIGHT. Each bt_gatt_notify_cb call is
     * non-blocking (~1 µs: build PDU, memcpy payload, k_fifo_put). Looping
     * here is safe — no risk of starving sysworkq.
     *
     * The completion CB runs on conn_tx_workq (CONFIG_BT_CONN_TX_NOTIFY_WQ=y)
     * and may decrement the counter concurrently with this loop. That's fine:
     * atomic_get re-reads each iteration, so we naturally pick up freed slots.
     */
    while (atomic_get(&input_notify_in_flight_count) <
           CONFIG_ZMK_INPUT_SPLIT_MAX_IN_FLIGHT) {
        if (!current_entry_held) {
            if (k_msgq_get(&input_notify_msgq, &current_entry, K_NO_WAIT) != 0) {
                return; /* Queue empty. */
            }
            input_notify_retries = 0;
        }

        struct bt_gatt_notify_params params = {
            .attr = current_entry.attr,
            .data = &current_entry.payload,
            .len = sizeof(current_entry.payload),
            .func = input_notify_complete,
        };

        /* Optimistic increment BEFORE the send so a completion CB firing
         * concurrently can't underflow. On 0->1, stamp the oldest-send-time
         * for the stuck-detection window. On err, we roll the increment back.
         */
        if (atomic_get(&input_notify_in_flight_count) == 0) {
            atomic_set(&input_notify_oldest_send_time,
                       (atomic_val_t)k_uptime_get_32());
        }
        atomic_inc(&input_notify_in_flight_count);

        int err = bt_gatt_notify_cb(NULL, &params);
        if (err == 0) {
            /* Data has been memcpy'd into a net_buf from att_pool. The caller's
             * current_entry buffer is free to reuse next iteration.
             */
            current_entry_held = false;
            input_notify_retries = 0;
            continue;
        }

        /* Roll back the optimistic increment. We don't unset the timestamp
         * because other in-flights may still be active.
         */
        atomic_dec(&input_notify_in_flight_count);

        if (err == -ENOMEM) {
            /* ATT pool exhausted. Hold current_entry and back off — the next
             * completion CB will reschedule us with at least one freed slot.
             * The 5 ms timer is a safety net in case all CBs are also stalled.
             */
            if (++input_notify_retries > INPUT_NOTIFY_MAX_RETRIES) {
                LOG_WRN("ATT exhaustion: dropping input notify after %d retries",
                        INPUT_NOTIFY_MAX_RETRIES);
                current_entry_held = false;
                input_notify_retries = 0;
                k_work_schedule(&input_notify_work, K_MSEC(5));
                return;
            }
            current_entry_held = true;
            k_work_schedule(&input_notify_work, K_MSEC(5));
            return;
        }

        /* Other error (e.g. -ENOTCONN): drop this entry, try the next. */
        LOG_WRN("input notify err=%d, dropping entry", err);
        current_entry_held = false;
        input_notify_retries = 0;
    }
}

static int zmk_split_bt_report_input(uint8_t reg, uint8_t type, uint16_t code, int32_t value,
                                     bool sync) {

    for (size_t i = 0; i < split_svc.attr_count; i++) {
        if (bt_uuid_cmp(split_svc.attrs[i].uuid,
                        BT_UUID_DECLARE_128(ZMK_SPLIT_BT_INPUT_EVENT_UUID)) == 0 &&
            (uint8_t)(uint32_t)split_svc.attrs[i + 2].user_data == reg) {
            struct input_notify_entry entry = {
                .attr = &split_svc.attrs[i],
                .payload = {
                    .type = type,
                    .code = code,
                    .value = value,
                    .sync = sync ? 1 : 0,
                },
            };
            if (k_msgq_put(&input_notify_msgq, &entry, K_NO_WAIT) != 0) {
                /* Queue full: drop the oldest pending entry and requeue. */
                struct input_notify_entry discard;
                k_msgq_get(&input_notify_msgq, &discard, K_NO_WAIT);
                (void)k_msgq_put(&input_notify_msgq, &entry, K_NO_WAIT);
                LOG_WRN("input notify queue full; dropped oldest");
            }
            k_work_schedule(&input_notify_work, K_NO_WAIT);
            return 0;
        }
    }
    return -ENODEV;
}

#endif /* IS_ENABLED(CONFIG_ZMK_INPUT_SPLIT) */

int zmk_split_transport_peripheral_bt_report_event(
    const struct zmk_split_transport_peripheral_event *ev) {
    switch (ev->type) {
    case ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_KEY_POSITION_EVENT:
        if (ev->data.key_position_event.pressed) {
            zmk_split_bt_position_pressed(ev->data.key_position_event.position);
        } else {
            zmk_split_bt_position_released(ev->data.key_position_event.position);
        }
        break;
#if ZMK_KEYMAP_HAS_SENSORS
    case ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_SENSOR_EVENT:
        zmk_split_bt_sensor_triggered(ev->data.sensor_event.sensor_index,
                                      &ev->data.sensor_event.channel_data, 1);

        break;
#endif

#if IS_ENABLED(CONFIG_ZMK_INPUT_SPLIT)
    case ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_INPUT_EVENT:
        return zmk_split_bt_report_input(ev->data.input_event.reg, ev->data.input_event.type,
                                         ev->data.input_event.code, ev->data.input_event.value,
                                         ev->data.input_event.sync);
#endif

#if IS_ENABLED(CONFIG_ZMK_BATTERY_REPORTING)
    case ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_BATTERY_EVENT:
        // The BLE transport uses standard BAS service for propagation, so just return success here.
        return 0;
#endif
    default:
        LOG_WRN("Unhandled event type %d", ev->type);
        return -ENOTSUP;
    }

    return 0;
}

static ssize_t split_svc_run_behavior(struct bt_conn *conn, const struct bt_gatt_attr *attrs,
                                      const void *buf, uint16_t len, uint16_t offset,
                                      uint8_t flags) {
    struct zmk_split_run_behavior_payload *payload = attrs->user_data;
    uint16_t end_addr = offset + len;

    LOG_DBG("offset %d len %d", offset, len);

    if (end_addr > sizeof(struct zmk_split_run_behavior_payload)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    memcpy(payload + offset, buf, len);

    // We run if:
    // 1: We've gotten all the position/state/param data.
    // 2: We have a null terminated string for the behavior device label.
    const size_t behavior_dev_offset =
        offsetof(struct zmk_split_run_behavior_payload, behavior_dev);
    if ((end_addr > sizeof(struct zmk_split_run_behavior_data)) &&
        payload->behavior_dev[end_addr - behavior_dev_offset - 1] == '\0') {

        struct zmk_split_transport_central_command cmd = {
            .type = ZMK_SPLIT_TRANSPORT_CENTRAL_CMD_TYPE_INVOKE_BEHAVIOR,
            .data = {.invoke_behavior = {
                         .param1 = payload->data.param1,
                         .param2 = payload->data.param2,
                         .position = payload->data.position,
                         .state = payload->data.state,
                     }}};

        const size_t payload_dev_size = sizeof(cmd.data.invoke_behavior.behavior_dev);
        if (strlcpy(cmd.data.invoke_behavior.behavior_dev, payload->behavior_dev,
                    payload_dev_size) >= payload_dev_size) {
            LOG_ERR("Truncated behavior label %s to %s before invoking peripheral behavior",
                    payload->behavior_dev, cmd.data.invoke_behavior.behavior_dev);
        }

        LOG_DBG("%s with params %d %d: pressed? %d", cmd.data.invoke_behavior.behavior_dev,
                cmd.data.invoke_behavior.param1, cmd.data.invoke_behavior.param2,
                cmd.data.invoke_behavior.state);

        int err = zmk_split_transport_peripheral_command_handler(
            zmk_split_transport_peripheral_bt(), cmd);

        if (err) {
            LOG_ERR("Failed to invoke behavior %s: %d", payload->behavior_dev, err);
        }
    }

    return len;
}