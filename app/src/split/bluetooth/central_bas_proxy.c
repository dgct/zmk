/*
 * Copyright (c) 2020 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/device.h>
#include <zephyr/init.h>
#include <sys/types.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/bluetooth/gatt.h>

#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#include <zmk/event_manager.h>
#include <zmk/battery.h>
#include <zmk/events/battery_state_changed.h>
#include <zmk/split/central.h>

#if IS_ENABLED(CONFIG_ZMK_BATTERY_PROXY_NOTIFY_THROTTLE)
/* Per-source notify-rate limiter.
 *
 * Background: the peripheral pushes a battery level event every
 * CONFIG_ZMK_BATTERY_REPORT_INTERVAL seconds (default 60). At small SoC
 * deltas (1-2%) those notifies waste host wake-ups and ATT TX buffers
 * for visually-identical UI updates.
 *
 * Strategy mirrors PR #2938 Genteure's hysteresis-and-min-interval
 * pattern, scope-reduced to the proxy path only:
 *   - skip if |delta_pct| < hysteresis AND time_since_last < min_interval
 *   - never throttle critical levels (<= 15%) — low-battery alerts must
 *     reach the host promptly
 *   - never throttle the first notify per source (let host see initial
 *     value immediately on boot/reconnect)
 */
struct proxy_throttle_state {
    uint8_t last_pct;
    int64_t last_ts_ms;
    bool valid;
};
static struct proxy_throttle_state
    proxy_throttle[CONFIG_ZMK_SPLIT_BLE_CENTRAL_PERIPHERALS];

static bool proxy_should_throttle(uint8_t source, uint8_t level) {
    if (source >= CONFIG_ZMK_SPLIT_BLE_CENTRAL_PERIPHERALS) {
        return false;
    }
    if (!proxy_throttle[source].valid) {
        return false;
    }
    if (level <= CONFIG_ZMK_BATTERY_PROXY_NOTIFY_CRITICAL_PCT) {
        return false;
    }
    int delta = (int)level - (int)proxy_throttle[source].last_pct;
    if (delta < 0) {
        delta = -delta;
    }
    int64_t since = k_uptime_get() - proxy_throttle[source].last_ts_ms;
    if (delta < CONFIG_ZMK_BATTERY_PROXY_NOTIFY_HYSTERESIS_PCT &&
        since < (int64_t)CONFIG_ZMK_BATTERY_PROXY_NOTIFY_MIN_INTERVAL_S * 1000) {
        return true;
    }
    return false;
}

static void proxy_throttle_record(uint8_t source, uint8_t level) {
    if (source >= CONFIG_ZMK_SPLIT_BLE_CENTRAL_PERIPHERALS) {
        return;
    }
    proxy_throttle[source].last_pct = level;
    proxy_throttle[source].last_ts_ms = k_uptime_get();
    proxy_throttle[source].valid = true;
}
#endif // CONFIG_ZMK_BATTERY_PROXY_NOTIFY_THROTTLE

static void blvl_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value) {
    ARG_UNUSED(attr);

    bool notif_enabled = (value == BT_GATT_CCC_NOTIFY);

    LOG_INF("BAS Notifications %s", notif_enabled ? "enabled" : "disabled");
}

static ssize_t read_blvl(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
                         uint16_t len, uint16_t offset) {
    const uint8_t source = *(uint8_t *)attr->user_data;
    uint8_t level = 0;
    int rc = zmk_split_central_get_peripheral_battery_level(source, &level);

    if (rc == -EINVAL) {
        LOG_ERR("Invalid peripheral index requested for battery level read: %d", source);
        return 0;
    }

    return bt_gatt_attr_read(conn, attr, buf, len, offset, &level, sizeof(uint8_t));
}

static const struct bt_gatt_cpf aux_level_cpf = {
    .format = 0x04, // uint8
    .exponent = 0x0,
    .unit = 0x27AD,        // Percentage
    .name_space = 0x01,    // Bluetooth SIG
    .description = 0x0108, // "auxiliary"
};

#define PERIPH_CUD_(x) "Peripheral " #x
#define PERIPH_CUD(x) PERIPH_CUD_(x)

// How many GATT attributes each battery level adds to our service
#define PERIPH_BATT_LEVEL_ATTR_COUNT 5
// The second generated attribute is the one used to send GATT notifications
#define PERIPH_BATT_LEVEL_ATTR_NOTIFY_IDX 1

#define PERIPH_BATT_LEVEL_ATTRS(i, _)                                                              \
    BT_GATT_CHARACTERISTIC(BT_UUID_BAS_BATTERY_LEVEL, BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,     \
                           BT_GATT_PERM_READ, read_blvl, NULL, ((uint8_t[]){i})),                  \
        BT_GATT_CCC(blvl_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),                 \
        BT_GATT_CPF(&aux_level_cpf), BT_GATT_CUD(PERIPH_CUD(i), BT_GATT_PERM_READ),

BT_GATT_SERVICE_DEFINE(bas_aux, BT_GATT_PRIMARY_SERVICE(BT_UUID_BAS),
                       LISTIFY(CONFIG_ZMK_SPLIT_BLE_CENTRAL_PERIPHERALS, PERIPH_BATT_LEVEL_ATTRS,
                               ()));

int peripheral_batt_lvl_listener(const zmk_event_t *eh) {
    const struct zmk_peripheral_battery_state_changed *ev =
        as_zmk_peripheral_battery_state_changed(eh);
    if (ev == NULL) {
        return ZMK_EV_EVENT_BUBBLE;
    };

    if (ev->source >= CONFIG_ZMK_SPLIT_BLE_CENTRAL_PERIPHERALS) {
        LOG_WRN("Got battery level event for an out of range peripheral index");
        return ZMK_EV_EVENT_BUBBLE;
    }

    LOG_DBG("Peripheral battery level event: %u", ev->state_of_charge);

#if IS_ENABLED(CONFIG_ZMK_BATTERY_PROXY_NOTIFY_THROTTLE)
    /* Invalidate throttle state when we see a level=0 event. ZMK fires
     * this on peripheral disconnect (see central.c split_central_disconnected)
     * and real cells are typically cut off by voltage protection well
     * before reaching 0%. Resetting `valid` here ensures the next real
     * reading after reconnect bypasses the throttle and reaches the host
     * immediately, regardless of how small the delta from 0 is. */
    if (ev->state_of_charge == 0 && ev->source < CONFIG_ZMK_SPLIT_BLE_CENTRAL_PERIPHERALS) {
        proxy_throttle[ev->source].valid = false;
    }

    if (proxy_should_throttle(ev->source, ev->state_of_charge)) {
        LOG_DBG("Throttling proxy notify for source %u at %u%%", ev->source,
                ev->state_of_charge);
        return ZMK_EV_EVENT_BUBBLE;
    }
#endif

    // Offset by the index of the source plus the specific offset to find the attribute to notify
    // on.
    int index = (PERIPH_BATT_LEVEL_ATTR_COUNT * ev->source) + PERIPH_BATT_LEVEL_ATTR_NOTIFY_IDX;

    int rc = bt_gatt_notify(NULL, &bas_aux.attrs[index], &ev->state_of_charge, sizeof(uint8_t));
    if (rc < 0 && rc != -ENOTCONN) {
        LOG_WRN("Failed to notify hosts of peripheral battery level: %d", rc);
    }

#if IS_ENABLED(CONFIG_ZMK_BATTERY_PROXY_NOTIFY_THROTTLE)
    /* Record only on success-or-not-connected; -ENOTCONN still consumes the
     * event from the host's perspective (no host to notify). Other errors
     * leave the throttle state unchanged so the next event retries. */
    if (rc >= 0 || rc == -ENOTCONN) {
        proxy_throttle_record(ev->source, ev->state_of_charge);
    }
#endif

    return ZMK_EV_EVENT_BUBBLE;
};

ZMK_LISTENER(peripheral_batt_lvl_listener, peripheral_batt_lvl_listener);
ZMK_SUBSCRIPTION(peripheral_batt_lvl_listener, zmk_peripheral_battery_state_changed);
