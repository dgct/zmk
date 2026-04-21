/*
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 *
 * Constant-latency hotfix module (nRF52-only).
 *
 * Mirrors nRF Desktop's `applications/nrf_desktop/src/modules/constlat.c`:
 * requests CONSTLAT mode while ZMK is in ACTIVE state, frees it on
 * IDLE/SLEEP. CONSTLAT eliminates regulator ramp jitter on ISR entry
 * (~3-5 µs) at the cost of ~2 mA continuous current — acceptable while
 * typing/pointing, not during sleep.
 *
 * Ported pattern differences vs upstream:
 *   - Subscribes to ZMK `zmk_activity_state_changed` instead of CAF
 *     power_down/wake_up events (ZMK has no CAF).
 *   - Initial `constlat_on()` runs at SYS_INIT(APPLICATION) so the very
 *     first interrupts (kscan init, BLE bring-up) already get the win.
 */

#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <nrfx_power.h>

#include <zmk/activity.h>
#include <zmk/event_manager.h>
#include <zmk/events/activity_state_changed.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

static bool enabled;

static void constlat_on(void) {
    if (enabled) {
        return;
    }
    (void)nrfx_power_constlat_mode_request();
    enabled = true;
    LOG_DBG("constant latency requested");
}

static void constlat_off(void) {
    if (!enabled) {
        return;
    }
    (void)nrfx_power_constlat_mode_free();
    enabled = false;
    LOG_DBG("constant latency freed");
}

static int activity_listener(const zmk_event_t *eh) {
    if (!IS_ENABLED(CONFIG_ZMK_CONSTLAT_DISABLE_ON_IDLE)) {
        return ZMK_EV_EVENT_BUBBLE;
    }
    const struct zmk_activity_state_changed *ev = as_zmk_activity_state_changed(eh);
    if (!ev) {
        return ZMK_EV_EVENT_BUBBLE;
    }
    if (ev->state == ZMK_ACTIVITY_ACTIVE) {
        constlat_on();
    } else {
        constlat_off();
    }
    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(constlat, activity_listener);
ZMK_SUBSCRIPTION(constlat, zmk_activity_state_changed);

static int constlat_init(void) {
    constlat_on();
    return 0;
}

SYS_INIT(constlat_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
