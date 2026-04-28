/*
 * Copyright (c) 2020 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/settings/settings.h>
#include <zephyr/init.h>
#include <zephyr/sys/atomic.h>

#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>

#include <zmk/ble.h>
#include <zmk/endpoints.h>
#include <zmk/endpoints_types.h>
#include <zmk/hog.h>
#include <zmk/hid.h>
#if IS_ENABLED(CONFIG_ZMK_POINTING_SMOOTH_SCROLLING)
#include <zmk/pointing/resolution_multipliers.h>
#endif // IS_ENABLED(CONFIG_ZMK_POINTING_SMOOTH_SCROLLING)
#if IS_ENABLED(CONFIG_ZMK_HID_INDICATORS)
#include <zmk/hid_indicators.h>
#endif // IS_ENABLED(CONFIG_ZMK_HID_INDICATORS)

enum {
    HIDS_REMOTE_WAKE = BIT(0),
    HIDS_NORMALLY_CONNECTABLE = BIT(1),
};

struct hids_info {
    uint16_t version; /* version number of base USB HID Specification */
    uint8_t code;     /* country HID Device hardware is localized for. */
    uint8_t flags;
} __packed;

struct hids_report {
    uint8_t id;   /* report id */
    uint8_t type; /* report type */
} __packed;

static struct hids_info info = {
    .version = 0x0000,
    .code = 0x00,
    .flags = HIDS_NORMALLY_CONNECTABLE | HIDS_REMOTE_WAKE,
};

enum {
    HIDS_INPUT = 0x01,
    HIDS_OUTPUT = 0x02,
    HIDS_FEATURE = 0x03,
};

static struct hids_report input = {
    .id = ZMK_HID_REPORT_ID_KEYBOARD,
    .type = HIDS_INPUT,
};

#if IS_ENABLED(CONFIG_ZMK_HID_INDICATORS)

static struct hids_report led_indicators = {
    .id = ZMK_HID_REPORT_ID_LEDS,
    .type = HIDS_OUTPUT,
};

#endif // IS_ENABLED(CONFIG_ZMK_HID_INDICATORS)

static struct hids_report consumer_input = {
    .id = ZMK_HID_REPORT_ID_CONSUMER,
    .type = HIDS_INPUT,
};

#if IS_ENABLED(CONFIG_ZMK_POINTING)

static struct hids_report mouse_input = {
    .id = ZMK_HID_REPORT_ID_MOUSE,
    .type = HIDS_INPUT,
};

#if IS_ENABLED(CONFIG_ZMK_POINTING_SMOOTH_SCROLLING)

static struct hids_report mouse_feature = {
    .id = ZMK_HID_REPORT_ID_MOUSE,
    .type = HIDS_FEATURE,
};

#endif // IS_ENABLED(CONFIG_ZMK_POINTING_SMOOTH_SCROLLING)

#endif // IS_ENABLED(CONFIG_ZMK_POINTING)

static bool host_requests_notification = false;
static uint8_t ctrl_point;
// static uint8_t proto_mode;

static ssize_t read_hids_info(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
                              uint16_t len, uint16_t offset) {
    return bt_gatt_attr_read(conn, attr, buf, len, offset, attr->user_data,
                             sizeof(struct hids_info));
}

static ssize_t read_hids_report_ref(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                    void *buf, uint16_t len, uint16_t offset) {
    return bt_gatt_attr_read(conn, attr, buf, len, offset, attr->user_data,
                             sizeof(struct hids_report));
}

static ssize_t read_hids_report_map(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                    void *buf, uint16_t len, uint16_t offset) {
    return bt_gatt_attr_read(conn, attr, buf, len, offset, zmk_hid_report_desc,
                             sizeof(zmk_hid_report_desc));
}

static ssize_t read_hids_input_report(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                      void *buf, uint16_t len, uint16_t offset) {
    struct zmk_hid_keyboard_report_body *report_body = &zmk_hid_get_keyboard_report()->body;
    return bt_gatt_attr_read(conn, attr, buf, len, offset, report_body,
                             sizeof(struct zmk_hid_keyboard_report_body));
}

#if IS_ENABLED(CONFIG_ZMK_HID_INDICATORS)
static ssize_t write_hids_leds_report(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                      const void *buf, uint16_t len, uint16_t offset,
                                      uint8_t flags) {
    if (offset != 0) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }
    if (len != sizeof(struct zmk_hid_led_report_body)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    struct zmk_hid_led_report_body *report = (struct zmk_hid_led_report_body *)buf;
    int profile = zmk_ble_profile_index(bt_conn_get_dst(conn));
    if (profile < 0) {
        return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
    }

    struct zmk_endpoint_instance endpoint = {.transport = ZMK_TRANSPORT_BLE,
                                             .ble = {
                                                 .profile_index = profile,
                                             }};
    zmk_hid_indicators_process_report(report, endpoint);

    return len;
}

#endif // IS_ENABLED(CONFIG_ZMK_HID_INDICATORS)

static ssize_t read_hids_consumer_input_report(struct bt_conn *conn,
                                               const struct bt_gatt_attr *attr, void *buf,
                                               uint16_t len, uint16_t offset) {
    struct zmk_hid_consumer_report_body *report_body = &zmk_hid_get_consumer_report()->body;
    return bt_gatt_attr_read(conn, attr, buf, len, offset, report_body,
                             sizeof(struct zmk_hid_consumer_report_body));
}

#if IS_ENABLED(CONFIG_ZMK_POINTING)

static ssize_t read_hids_mouse_input_report(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                            void *buf, uint16_t len, uint16_t offset) {
    struct zmk_hid_mouse_report_body *report_body = &zmk_hid_get_mouse_report()->body;
    return bt_gatt_attr_read(conn, attr, buf, len, offset, report_body,
                             sizeof(struct zmk_hid_mouse_report_body));
}

#if IS_ENABLED(CONFIG_ZMK_POINTING_SMOOTH_SCROLLING)

static ssize_t read_hids_mouse_feature_report(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                              void *buf, uint16_t len, uint16_t offset) {

    int profile = zmk_ble_profile_index(bt_conn_get_dst(conn));
    if (profile < 0) {
        LOG_DBG("   BT_ATT_ERR_UNLIKELY");
        return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
    }

    struct zmk_endpoint_instance endpoint = {
        .transport = ZMK_TRANSPORT_BLE,
        .ble = {.profile_index = profile},
    };

    struct zmk_pointing_resolution_multipliers mult =
        zmk_pointing_resolution_multipliers_get_profile(endpoint);

    struct zmk_hid_mouse_resolution_feature_report_body report = {
        .wheel_res = mult.wheel,
        .hwheel_res = mult.hor_wheel,
    };

    return bt_gatt_attr_read(conn, attr, buf, len, offset, &report,
                             sizeof(struct zmk_hid_mouse_resolution_feature_report_body));
}

static ssize_t write_hids_mouse_feature_report(struct bt_conn *conn,
                                               const struct bt_gatt_attr *attr, const void *buf,
                                               uint16_t len, uint16_t offset, uint8_t flags) {
    if (offset != 0) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }
    if (len != sizeof(struct zmk_hid_mouse_resolution_feature_report_body)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    struct zmk_hid_mouse_resolution_feature_report_body *report =
        (struct zmk_hid_mouse_resolution_feature_report_body *)buf;
    int profile = zmk_ble_profile_index(bt_conn_get_dst(conn));
    if (profile < 0) {
        return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
    }

    struct zmk_endpoint_instance endpoint = {.transport = ZMK_TRANSPORT_BLE,
                                             .ble = {
                                                 .profile_index = profile,
                                             }};
    zmk_pointing_resolution_multipliers_process_report(report, endpoint);

    return len;
}

#endif // IS_ENABLED(CONFIG_ZMK_POINTING_SMOOTH_SCROLLING)

#endif // IS_ENABLED(CONFIG_ZMK_POINTING)

// static ssize_t write_proto_mode(struct bt_conn *conn,
//                                 const struct bt_gatt_attr *attr,
//                                 const void *buf, uint16_t len, uint16_t offset,
//                                 uint8_t flags)
// {
//     printk("PROTO CHANGED\n");
//     return 0;
// }

static void input_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value) {
    host_requests_notification = (value == BT_GATT_CCC_NOTIFY) ? 1 : 0;
}

static ssize_t write_ctrl_point(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                const void *buf, uint16_t len, uint16_t offset, uint8_t flags) {
    uint8_t *value = attr->user_data;

    if (offset + len > sizeof(ctrl_point)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    memcpy(value + offset, buf, len);

    return len;
}

/* HID Service Declaration */
BT_GATT_SERVICE_DEFINE(
    hog_svc, BT_GATT_PRIMARY_SERVICE(BT_UUID_HIDS),
    //    BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_PROTOCOL_MODE, BT_GATT_CHRC_WRITE_WITHOUT_RESP,
    //                           BT_GATT_PERM_WRITE, NULL, write_proto_mode, &proto_mode),
    BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_INFO, BT_GATT_CHRC_READ, BT_GATT_PERM_READ, read_hids_info,
                           NULL, &info),
    BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_REPORT_MAP, BT_GATT_CHRC_READ, BT_GATT_PERM_READ_ENCRYPT,
                           read_hids_report_map, NULL, NULL),

    BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_REPORT, BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ_ENCRYPT, read_hids_input_report, NULL, NULL),
    BT_GATT_CCC(input_ccc_changed, BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT),
    BT_GATT_DESCRIPTOR(BT_UUID_HIDS_REPORT_REF, BT_GATT_PERM_READ_ENCRYPT, read_hids_report_ref,
                       NULL, &input),

    BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_REPORT, BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ_ENCRYPT, read_hids_consumer_input_report, NULL, NULL),
    BT_GATT_CCC(input_ccc_changed, BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT),
    BT_GATT_DESCRIPTOR(BT_UUID_HIDS_REPORT_REF, BT_GATT_PERM_READ_ENCRYPT, read_hids_report_ref,
                       NULL, &consumer_input),

#if IS_ENABLED(CONFIG_ZMK_POINTING)
    BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_REPORT, BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ_ENCRYPT, read_hids_mouse_input_report, NULL, NULL),
    BT_GATT_CCC(input_ccc_changed, BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT),
    BT_GATT_DESCRIPTOR(BT_UUID_HIDS_REPORT_REF, BT_GATT_PERM_READ_ENCRYPT, read_hids_report_ref,
                       NULL, &mouse_input),

#if IS_ENABLED(CONFIG_ZMK_POINTING_SMOOTH_SCROLLING)
    BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_REPORT,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
                           BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT,
                           read_hids_mouse_feature_report, write_hids_mouse_feature_report, NULL),
    BT_GATT_DESCRIPTOR(BT_UUID_HIDS_REPORT_REF, BT_GATT_PERM_READ_ENCRYPT, read_hids_report_ref,
                       NULL, &mouse_feature),
#endif // IS_ENABLED(CONFIG_ZMK_POINTING_SMOOTH_SCROLLING)

#endif // IS_ENABLED(CONFIG_ZMK_POINTING)

#if IS_ENABLED(CONFIG_ZMK_HID_INDICATORS)
    BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_REPORT,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
                           BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT, NULL,
                           write_hids_leds_report, NULL),
    BT_GATT_DESCRIPTOR(BT_UUID_HIDS_REPORT_REF, BT_GATT_PERM_READ_ENCRYPT, read_hids_report_ref,
                       NULL, &led_indicators),
#endif // IS_ENABLED(CONFIG_ZMK_HID_INDICATORS)

    BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_CTRL_POINT, BT_GATT_CHRC_WRITE_WITHOUT_RESP,
                           BT_GATT_PERM_WRITE, NULL, write_ctrl_point, &ctrl_point));

#define HOG_NOTIFY_MAX_RETRIES 10

#if IS_ENABLED(CONFIG_ZMK_POINTING) && IS_ENABLED(CONFIG_ZMK_HOG_MOUSE_PIPELINE)
/* Mirrors nRF Desktop hids.c HIDS_SUBSCRIBER_PIPELINE_SIZE + RMK
 * hid_provider/mouse.rs motion-event synchronization. Mouse-only.
 */
static atomic_t mouse_in_flight;
/* Timestamp (k_uptime_get_32) of the oldest un-completed notify.
 * Used solely by the 500ms stuck-recovery path.  Reset to 0 when the
 * last in-flight completes or when stuck-recovery fires. */
static atomic_t mouse_oldest_send_time = ATOMIC_INIT(0);
static struct zmk_hid_mouse_report_body mouse_coalesce;
static bool mouse_coalesce_pending;
static struct k_spinlock mouse_coalesce_lock;
extern struct k_work_delayable hog_mouse_work;
#endif

static void hog_notify_complete(struct bt_conn *conn, void *user_data) {
    struct k_work_delayable *work = (struct k_work_delayable *)user_data;
#if IS_ENABLED(CONFIG_ZMK_POINTING) && IS_ENABLED(CONFIG_ZMK_HOG_MOUSE_PIPELINE)
    if (work == &hog_mouse_work) {
        /* Guard against underflow: a late completion callback can fire after
         * hog_host_disconnected() has already reset the counter to 0.
         * The underflow is benign (counter goes negative, gate stays open)
         * but guarding keeps the counter semantically correct. */
        atomic_val_t prev = atomic_get(&mouse_in_flight);
        if (prev > 0) {
            if (atomic_dec(&mouse_in_flight) == 1) {
                /* Last in-flight completed — clear the timestamp. */
                atomic_set(&mouse_oldest_send_time, 0);
            }
        }
    }
#endif
    k_work_reschedule(work, K_NO_WAIT);
}

K_MSGQ_DEFINE(zmk_hog_keyboard_msgq, sizeof(struct zmk_hid_keyboard_report_body),
              CONFIG_ZMK_BLE_KEYBOARD_REPORT_QUEUE_SIZE, 4);

static int hog_keyboard_retries;

void send_keyboard_report_callback(struct k_work *work);
K_WORK_DELAYABLE_DEFINE(hog_keyboard_work, send_keyboard_report_callback);

void send_keyboard_report_callback(struct k_work *work) {
    struct zmk_hid_keyboard_report_body report;

    while (k_msgq_peek(&zmk_hog_keyboard_msgq, &report) == 0) {
        struct bt_conn *conn = zmk_ble_active_profile_conn();
        if (conn == NULL) {
            return;
        }

        struct bt_gatt_notify_params notify_params = {
            .attr = &hog_svc.attrs[5],
            .data = &report,
            .len = sizeof(report),
            .func = hog_notify_complete,
            .user_data = &hog_keyboard_work,
        };

        int err = bt_gatt_notify_cb(conn, &notify_params);
        // -EINVAL: client not yet subscribed to this characteristic
        // (BT_GATT_ENFORCE_SUBSCRIPTION). Retry like -ENOMEM so we don't
        // silently drain the queue while the host is still subscribing.
        if (err == -ENOMEM || err == -EINVAL) {
            bt_conn_unref(conn);
            if (++hog_keyboard_retries > HOG_NOTIFY_MAX_RETRIES) {
                LOG_WRN("Notify exhaustion: dropping keyboard queue (err %d, %d retries)",
                        err, HOG_NOTIFY_MAX_RETRIES);
                k_msgq_purge(&zmk_hog_keyboard_msgq);
                hog_keyboard_retries = 0;
                return;
            }
            k_work_schedule(&hog_keyboard_work, K_MSEC(2));
            return;
        }

        k_msgq_get(&zmk_hog_keyboard_msgq, &report, K_NO_WAIT);
        hog_keyboard_retries = 0;

        if (err == -EPERM) {
            bt_conn_set_security(conn, BT_SECURITY_L2);
        } else if (err) {
            LOG_DBG("Error notifying %d", err);
        }

        bt_conn_unref(conn);
    }
}

int zmk_hog_send_keyboard_report(struct zmk_hid_keyboard_report_body *report) {
    int err = k_msgq_put(&zmk_hog_keyboard_msgq, report, K_NO_WAIT);
    if (err) {
        switch (err) {
        /* k_msgq_put() with K_NO_WAIT returns -ENOMSG when the queue is full;
         * older code checked for -EAGAIN which never matched, causing reports
         * to be silently dropped without rescheduling the drain work. */
        case -ENOMSG:
        case -EAGAIN: {
            LOG_WRN("Keyboard message queue full, popping first message and queueing again");
            struct zmk_hid_keyboard_report_body discarded_report;
            k_msgq_get(&zmk_hog_keyboard_msgq, &discarded_report, K_NO_WAIT);
            return zmk_hog_send_keyboard_report(report);
        }
        default:
            LOG_WRN("Failed to queue keyboard report to send (%d)", err);
            return err;
        }
    }

    k_work_schedule(&hog_keyboard_work, K_NO_WAIT);

    return 0;
};

K_MSGQ_DEFINE(zmk_hog_consumer_msgq, sizeof(struct zmk_hid_consumer_report_body),
              CONFIG_ZMK_BLE_CONSUMER_REPORT_QUEUE_SIZE, 4);

static int hog_consumer_retries;

void send_consumer_report_callback(struct k_work *work);
K_WORK_DELAYABLE_DEFINE(hog_consumer_work, send_consumer_report_callback);

void send_consumer_report_callback(struct k_work *work) {
    struct zmk_hid_consumer_report_body report;

    while (k_msgq_peek(&zmk_hog_consumer_msgq, &report) == 0) {
        struct bt_conn *conn = zmk_ble_active_profile_conn();
        if (conn == NULL) {
            return;
        }

        struct bt_gatt_notify_params notify_params = {
            .attr = &hog_svc.attrs[9],
            .data = &report,
            .len = sizeof(report),
            .func = hog_notify_complete,
            .user_data = &hog_consumer_work,
        };

        int err = bt_gatt_notify_cb(conn, &notify_params);
        // -EINVAL: client not yet subscribed (BT_GATT_ENFORCE_SUBSCRIPTION).
        // Retry like -ENOMEM.
        if (err == -ENOMEM || err == -EINVAL) {
            bt_conn_unref(conn);
            if (++hog_consumer_retries > HOG_NOTIFY_MAX_RETRIES) {
                LOG_WRN("Notify exhaustion: dropping consumer queue (err %d, %d retries)",
                        err, HOG_NOTIFY_MAX_RETRIES);
                k_msgq_purge(&zmk_hog_consumer_msgq);
                hog_consumer_retries = 0;
                return;
            }
            k_work_schedule(&hog_consumer_work, K_MSEC(2));
            return;
        }

        k_msgq_get(&zmk_hog_consumer_msgq, &report, K_NO_WAIT);
        hog_consumer_retries = 0;

        if (err == -EPERM) {
            bt_conn_set_security(conn, BT_SECURITY_L2);
        } else if (err) {
            LOG_DBG("Error notifying %d", err);
        }

        bt_conn_unref(conn);
    }
};

int zmk_hog_send_consumer_report(struct zmk_hid_consumer_report_body *report) {
    int err = k_msgq_put(&zmk_hog_consumer_msgq, report, K_NO_WAIT);
    if (err) {
        switch (err) {
        /* k_msgq_put() with K_NO_WAIT returns -ENOMSG when the queue is full. */
        case -ENOMSG:
        case -EAGAIN: {
            LOG_WRN("Consumer message queue full, popping first message and queueing again");
            struct zmk_hid_consumer_report_body discarded_report;
            k_msgq_get(&zmk_hog_consumer_msgq, &discarded_report, K_NO_WAIT);
            return zmk_hog_send_consumer_report(report);
        }
        default:
            LOG_WRN("Failed to queue consumer report to send (%d)", err);
            return err;
        }
    }

    k_work_schedule(&hog_consumer_work, K_NO_WAIT);

    return 0;
};

#if IS_ENABLED(CONFIG_ZMK_POINTING)

K_MSGQ_DEFINE(zmk_hog_mouse_msgq, sizeof(struct zmk_hid_mouse_report_body),
              CONFIG_ZMK_BLE_MOUSE_REPORT_QUEUE_SIZE, 4);

static int hog_mouse_retries;

void send_mouse_report_callback(struct k_work *work);
K_WORK_DELAYABLE_DEFINE(hog_mouse_work, send_mouse_report_callback);

void send_mouse_report_callback(struct k_work *work) {
    struct zmk_hid_mouse_report_body report;
    while (true) {
#if IS_ENABLED(CONFIG_ZMK_HOG_MOUSE_PIPELINE)
        /* Pipeline gate: don't dispatch more notifies than the configured
         * window. Notify-completion callback re-triggers this worker. */
        if (atomic_get(&mouse_in_flight) >= CONFIG_ZMK_HOG_MOUSE_PIPELINE_DEPTH) {
            /* Stuck-recovery: if the oldest in-flight has been pending for
             * >500ms the BT stack silently dropped the completion callback.
             * Reset the counter so the pipeline doesn't block forever.
             * Mirrors the pattern in service.c input_notify_work_cb. */
            uint32_t oldest = (uint32_t)atomic_get(&mouse_oldest_send_time);
            if (oldest != 0) {
                uint32_t elapsed = k_uptime_get_32() - oldest;
                if (elapsed > 500) {
                    LOG_WRN("HOG mouse pipeline stuck (%ld in-flight) for %u ms, recovering",
                            (long)atomic_get(&mouse_in_flight), elapsed);
                    atomic_clear(&mouse_in_flight);
                    atomic_set(&mouse_oldest_send_time, 0);
                    /* Fall through to drain. */
                } else {
                    return;
                }
            } else {
                return;
            }
        }
#endif

        bool from_coalesce = false;
        if (k_msgq_peek(&zmk_hog_mouse_msgq, &report) != 0) {
#if IS_ENABLED(CONFIG_ZMK_HOG_MOUSE_PIPELINE)
            /* Queue empty: flush any pending coalesced motion. */
            k_spinlock_key_t key = k_spin_lock(&mouse_coalesce_lock);
            if (!mouse_coalesce_pending) {
                k_spin_unlock(&mouse_coalesce_lock, key);
                return;
            }
            report = mouse_coalesce;
            mouse_coalesce_pending = false;
            k_spin_unlock(&mouse_coalesce_lock, key);
            from_coalesce = true;
#else
            return;
#endif
        }

        struct bt_conn *conn = zmk_ble_active_profile_conn();
        if (conn == NULL) {
#if IS_ENABLED(CONFIG_ZMK_HOG_MOUSE_PIPELINE)
            if (from_coalesce) {
                /* Restore the coalesced report so we don't lose it. */
                k_spinlock_key_t key = k_spin_lock(&mouse_coalesce_lock);
                if (!mouse_coalesce_pending) {
                    mouse_coalesce = report;
                    mouse_coalesce_pending = true;
                }
                k_spin_unlock(&mouse_coalesce_lock, key);
            }
#endif
            return;
        }

        struct bt_gatt_notify_params notify_params = {
            .attr = &hog_svc.attrs[13],
            .data = &report,
            .len = sizeof(report),
            .func = hog_notify_complete,
            .user_data = &hog_mouse_work,
        };

#if IS_ENABLED(CONFIG_ZMK_HOG_MOUSE_PIPELINE)
        /* Optimistic increment BEFORE send: if bt_gatt_notify_cb succeeds
         * (returns 0) the completion CB will decrement.  On any error we
         * roll back below.  This matches the pattern in service.c and
         * prevents a window where the CB could fire before the counter
         * is incremented. */
        if (atomic_get(&mouse_in_flight) == 0) {
            atomic_set(&mouse_oldest_send_time,
                       (atomic_val_t)k_uptime_get_32());
        }
        atomic_inc(&mouse_in_flight);
#endif

        int err = bt_gatt_notify_cb(conn, &notify_params);
        // -EINVAL: client not yet subscribed (BT_GATT_ENFORCE_SUBSCRIPTION).
        // At 80Hz TP, dropping reports here permanently kills the mouse
        // pipeline if we don't retry until the host completes subscription.
        if (err == -ENOMEM || err == -EINVAL) {
            bt_conn_unref(conn);
#if IS_ENABLED(CONFIG_ZMK_HOG_MOUSE_PIPELINE)
            atomic_dec(&mouse_in_flight);
            if (from_coalesce) {
                k_spinlock_key_t key = k_spin_lock(&mouse_coalesce_lock);
                if (!mouse_coalesce_pending) {
                    mouse_coalesce = report;
                    mouse_coalesce_pending = true;
                }
                k_spin_unlock(&mouse_coalesce_lock, key);
            }
#endif
            if (++hog_mouse_retries > HOG_NOTIFY_MAX_RETRIES) {
                LOG_WRN("Notify exhaustion: dropping mouse queue (err %d, %d retries)",
                        err, HOG_NOTIFY_MAX_RETRIES);
                k_msgq_purge(&zmk_hog_mouse_msgq);
                hog_mouse_retries = 0;
                return;
            }
            k_work_schedule(&hog_mouse_work, K_MSEC(2));
            return;
        }

        if (!from_coalesce) {
            k_msgq_get(&zmk_hog_mouse_msgq, &report, K_NO_WAIT);
        }
        hog_mouse_retries = 0;

        if (err == -EPERM) {
#if IS_ENABLED(CONFIG_ZMK_HOG_MOUSE_PIPELINE)
            atomic_dec(&mouse_in_flight);
#endif
            bt_conn_set_security(conn, BT_SECURITY_L2);
        } else if (err) {
#if IS_ENABLED(CONFIG_ZMK_HOG_MOUSE_PIPELINE)
            atomic_dec(&mouse_in_flight);
#endif
            LOG_DBG("Error notifying %d", err);
        }

        bt_conn_unref(conn);
    }
};

int zmk_hog_send_mouse_report(struct zmk_hid_mouse_report_body *report) {
#if IS_ENABLED(CONFIG_ZMK_HOG_MOUSE_PIPELINE)
    /* Pipeline saturated: merge into coalesce buffer instead of queueing.
     * Sums motion/scroll deltas, latches buttons (last-value-wins).
     * Mirrors RMK's hid_provider/mouse.rs accumulator pattern. */
    if (atomic_get(&mouse_in_flight) >= CONFIG_ZMK_HOG_MOUSE_PIPELINE_DEPTH) {
        k_spinlock_key_t key = k_spin_lock(&mouse_coalesce_lock);
        if (mouse_coalesce_pending) {
            int32_t x = (int32_t)mouse_coalesce.d_x + report->d_x;
            int32_t y = (int32_t)mouse_coalesce.d_y + report->d_y;
            int32_t sx = (int32_t)mouse_coalesce.d_scroll_x + report->d_scroll_x;
            int32_t sy = (int32_t)mouse_coalesce.d_scroll_y + report->d_scroll_y;
            mouse_coalesce.d_x = CLAMP(x, INT16_MIN, INT16_MAX);
            mouse_coalesce.d_y = CLAMP(y, INT16_MIN, INT16_MAX);
            mouse_coalesce.d_scroll_x = CLAMP(sx, INT16_MIN, INT16_MAX);
            mouse_coalesce.d_scroll_y = CLAMP(sy, INT16_MIN, INT16_MAX);
            mouse_coalesce.buttons = report->buttons;
        } else {
            mouse_coalesce = *report;
            mouse_coalesce_pending = true;
        }
        k_spin_unlock(&mouse_coalesce_lock, key);
        k_work_schedule(&hog_mouse_work, K_NO_WAIT);
        return 0;
    }
#endif

    int err = k_msgq_put(&zmk_hog_mouse_msgq, report, K_NO_WAIT);
    if (err) {
        switch (err) {
        /* k_msgq_put() with K_NO_WAIT returns -ENOMSG when the queue is full. */
        case -ENOMSG:
        case -EAGAIN: {
            LOG_WRN("Mouse message queue full, popping first message and queueing again");
            struct zmk_hid_mouse_report_body discarded_report;
            k_msgq_get(&zmk_hog_mouse_msgq, &discarded_report, K_NO_WAIT);
            return zmk_hog_send_mouse_report(report);
        }
        default:
            LOG_WRN("Failed to queue mouse report to send (%d)", err);
            return err;
        }
    }

    k_work_schedule(&hog_mouse_work, K_NO_WAIT);

    return 0;
};
#endif // IS_ENABLED(CONFIG_ZMK_POINTING)

#if IS_ENABLED(CONFIG_ZMK_HOG_RELEASE_ON_DISCONNECT)

/* Release HID state when a HOG host disconnects so reconnect doesn't carry
 * over stuck modifiers/keys/buttons. Pairs with the split-side
 * zmk_input_split_peripheral_disconnected() that releases pointing input
 * forwarded from peripherals (cherry-pick of zmk#2721). This handles the
 * complementary case: the host-facing HOG link dropping while keys are held.
 *
 * If a mouse pipeline is configured, also drains the coalesce buffer and
 * resets in-flight counters so a stale pending report can't ride a fresh
 * connection.
 */
static void hog_host_disconnected(struct bt_conn *conn, uint8_t reason) {
    struct bt_conn_info info;
    if (bt_conn_get_info(conn, &info) != 0) {
        return;
    }
    /* HOG runs in the peripheral role (the keyboard advertises to a host).
     * Ignore central-role disconnects (split peripheral link) — those have
     * their own cleanup path in app/src/split/bluetooth/central.c. */
    if (info.role != BT_HCI_ROLE_PERIPHERAL) {
        return;
    }

    LOG_INF("HOG host disconnect (reason 0x%02x), clearing HID state", reason);

#if IS_ENABLED(CONFIG_ZMK_POINTING) && IS_ENABLED(CONFIG_ZMK_HOG_MOUSE_PIPELINE)
    {
        k_spinlock_key_t key = k_spin_lock(&mouse_coalesce_lock);
        mouse_coalesce = (struct zmk_hid_mouse_report_body){0};
        mouse_coalesce_pending = false;
        k_spin_unlock(&mouse_coalesce_lock, key);
    }
    atomic_set(&mouse_in_flight, 0);
    atomic_set(&mouse_oldest_send_time, 0);
#endif

    /* Only clear the global HID register state if the dropped connection
     * was the active profile's link. Other (background) profile links
     * dropping must not wipe state shared with the active host. */
    bt_addr_le_t *active_addr = zmk_ble_active_profile_addr();
    if (active_addr != NULL && bt_addr_le_eq(&info.le.dst, active_addr)) {
        zmk_endpoint_clear_reports();
    }
}

BT_CONN_CB_DEFINE(hog_conn_callbacks) = {
    .disconnected = hog_host_disconnected,
};

#endif // IS_ENABLED(CONFIG_ZMK_HOG_RELEASE_ON_DISCONNECT)


