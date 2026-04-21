/*
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 *
 * HFXO pre-lock module (nRF52-only).
 *
 * Mirrors nRF Desktop's `applications/nrf_desktop/src/modules/hfclk_lock.c`:
 * pre-acquires the 64 MHz HFXO crystal via the Zephyr clock control
 * onoff manager so the BLE controller does not pay the ~360 µs HFXO
 * ramp before the first conn event after wake.
 *
 * Pairs with CONFIG_ZMK_BLE_DYNAMIC_HID_LATENCY: when slave latency
 * snaps from Idle-1 (30) back to Active (0) on the first input event,
 * the next conn event finds HFXO already running.
 *
 * Ported pattern differences vs upstream:
 *   - Subscribes to ZMK `zmk_activity_state_changed` instead of CAF.
 *   - Initial lock at SYS_INIT(APPLICATION) covers the first conn
 *     event after boot too.
 *   - We do not block on the request — `sys_notify_init_spinwait` is
 *     used per upstream so the ramp completes asynchronously while
 *     the controller does its own lazy request in parallel.
 */

#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#include <zephyr/logging/log.h>

#include <zmk/activity.h>
#include <zmk/event_manager.h>
#include <zmk/events/activity_state_changed.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

static struct onoff_manager *mgr;
static struct onoff_client cli;

static void hfclk_lock(void) {
    if (mgr) {
        return;
    }
    mgr = z_nrf_clock_control_get_onoff(CLOCK_CONTROL_NRF_SUBSYS_HF);
    if (!mgr) {
        LOG_ERR("hfclk onoff manager unavailable");
        return;
    }
    sys_notify_init_spinwait(&cli.notify);
    int err = onoff_request(mgr, &cli);
    if (err < 0) {
        LOG_ERR("hfclk onoff_request err %d", err);
        mgr = NULL;
        return;
    }
    LOG_DBG("HFXO locked");
}

static void hfclk_unlock(void) {
    if (!mgr) {
        return;
    }
    int err = onoff_cancel_or_release(mgr, &cli);
    if (err < 0) {
        LOG_WRN("hfclk release err %d", err);
    }
    mgr = NULL;
    LOG_DBG("HFXO released");
}

static int activity_listener(const zmk_event_t *eh) {
    if (!IS_ENABLED(CONFIG_ZMK_HFCLK_LOCK_DISABLE_ON_IDLE)) {
        return ZMK_EV_EVENT_BUBBLE;
    }
    const struct zmk_activity_state_changed *ev = as_zmk_activity_state_changed(eh);
    if (!ev) {
        return ZMK_EV_EVENT_BUBBLE;
    }
    if (ev->state == ZMK_ACTIVITY_ACTIVE) {
        hfclk_lock();
    } else {
        hfclk_unlock();
    }
    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(hfclk_lock, activity_listener);
ZMK_SUBSCRIPTION(hfclk_lock, zmk_activity_state_changed);

static int hfclk_lock_init(void) {
    hfclk_lock();
    return 0;
}

SYS_INIT(hfclk_lock_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
