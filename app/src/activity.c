/*
 * Copyright (c) 2020 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/poweroff.h>

#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#include <zmk/event_manager.h>
#include <zmk/events/activity_state_changed.h>
#include <zmk/events/position_state_changed.h>
#include <zmk/events/sensor_event.h>

#include <zmk/pm.h>
#include <zmk/sleep_trace.h>

#include <zmk/activity.h>

#if IS_ENABLED(CONFIG_USB_DEVICE_STACK)
#include <zmk/usb.h>
#endif

#if IS_ENABLED(CONFIG_ZMK_SLEEP_PREVENT_WHILE_BLE_CONNECTED)
#include <zmk/ble.h>
#if IS_ENABLED(CONFIG_ZMK_SPLIT) && !IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
#include <zmk/split/bluetooth/peripheral.h>
#endif
#endif

#if IS_ENABLED(CONFIG_ZMK_POINTING)
#include <zephyr/input/input.h>
#endif

#if IS_ENABLED(CONFIG_ZMK_SLEEP_DEBUG)
#define SLEEP_LOG(...) LOG_INF(__VA_ARGS__)
#else
#define SLEEP_LOG(...)
#endif

bool is_usb_power_present(void) {
#if IS_ENABLED(CONFIG_USB_DEVICE_STACK)
    /* Prefer the hardware VBUS bit to the USB-stack enumeration state.
     * KVM switches drop the host's USB enumeration (-> usb_status goes
     * to USB_DC_DISCONNECTED -> zmk_usb_is_powered() returns false)
     * while VBUS stays asserted. Without this check, the keyboard would
     * sys_poweroff() while connected to a KVM that's just temporarily
     * routed away, then crash on re-enumeration when KVM switches back.
     * On non-nRF or non-NRFX builds, zmk_usb_vbus_present() returns
     * false and we fall back to the enumeration-state check. */
    return zmk_usb_vbus_present() || zmk_usb_is_powered();
#else
    return false;
#endif /* IS_ENABLED(CONFIG_USB_DEVICE_STACK) */
}

static enum zmk_activity_state activity_state;

static uint32_t activity_last_uptime;

#define MAX_IDLE_MS CONFIG_ZMK_IDLE_TIMEOUT

#if IS_ENABLED(CONFIG_ZMK_SLEEP)
#define MAX_SLEEP_MS CONFIG_ZMK_IDLE_SLEEP_TIMEOUT
#endif

int raise_event(void) {
    return raise_zmk_activity_state_changed(
        (struct zmk_activity_state_changed){.state = activity_state});
}

int set_state(enum zmk_activity_state state) {
    if (activity_state == state)
        return 0;

    activity_state = state;
    return raise_event();
}

enum zmk_activity_state zmk_activity_get_state(void) { return activity_state; }

static int note_activity(void) {
    activity_last_uptime = k_uptime_get();

    return set_state(ZMK_ACTIVITY_ACTIVE);
}

int zmk_activity_note(void) { return note_activity(); }

static int activity_event_listener(const zmk_event_t *eh) { return note_activity(); }

void activity_work_handler(struct k_work *work) {
    int32_t current = k_uptime_get();
    int32_t inactive_time = current - activity_last_uptime;
#if IS_ENABLED(CONFIG_ZMK_SLEEP)
    bool prevent_sleep =
	    IS_ENABLED(CONFIG_ZMK_SLEEP_PREVENT_WHILE_USB_POWERED) && is_usb_power_present();
    #if IS_ENABLED(CONFIG_ZMK_SLEEP_PREVENT_WHILE_BLE_CONNECTED)
        #if !IS_ENABLED(CONFIG_ZMK_SPLIT) || IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
            prevent_sleep |= zmk_ble_active_profile_is_connected();
        #else
            prevent_sleep |= zmk_split_bt_peripheral_is_connected();
        #endif
    #endif
    if (inactive_time > MAX_SLEEP_MS && !prevent_sleep) {
        SLEEP_LOG("sleep: %d ms inactive, entering deep sleep", inactive_time);
        zmk_sleep_trace(ZMK_SLEEP_TRACE_ENTER);
        // Put devices in suspend power mode before sleeping
        set_state(ZMK_ACTIVITY_SLEEP);
        zmk_sleep_trace(ZMK_SLEEP_TRACE_LISTENERS_DONE);
        SLEEP_LOG("sleep: listeners done, preparing devices and wake sources");

#if IS_ENABLED(CONFIG_ZMK_SPLIT) && !IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
        // Peripheral: upstream's sequence. The key matrix is a wake-up
        // source (enabled at init) and zmk_pm_suspend_devices() leaves it
        // alone, so it keeps the armed level interrupts (GPIO SENSE) it has
        // while idle. The soft-off dance below (suspend everything, then
        // disconnect the matrix pins, resume and rescan it) was added for
        // the central half; on this half it is the only step that differs
        // from upstream, and the peripheral did not come back from sleep.
        SLEEP_LOG("sleep: peripheral, key matrix stays armed");
#else
        // Disable all wakeup sources, suspend all devices, then
        // re-enable only the designated wakeup sources (e.g. kscan)
        // so GPIO SENSE wake works correctly from SYSTEMOFF.
        zmk_pm_prepare_for_poweroff();
        SLEEP_LOG("sleep: wake sources armed, suspending the rest");
#endif
        zmk_sleep_trace(ZMK_SLEEP_TRACE_PREPARED);

        zmk_pm_suspend_devices();
        zmk_sleep_trace(ZMK_SLEEP_TRACE_DEVICES_SUSPENDED);
        zmk_pm_log_wake_pins();
        zmk_sleep_trace(ZMK_SLEEP_TRACE_WAKE_PINS_LOGGED);
        SLEEP_LOG("sleep: powering off");

        zmk_sleep_trace(ZMK_SLEEP_TRACE_POWERING_OFF);
        sys_poweroff();
        zmk_sleep_trace(ZMK_SLEEP_TRACE_POWEROFF_RETURNED);
        SLEEP_LOG("sleep: sys_poweroff returned");
    } else
#endif /* IS_ENABLED(CONFIG_ZMK_SLEEP) */
        if (inactive_time > MAX_IDLE_MS) {
            set_state(ZMK_ACTIVITY_IDLE);
        }
}

K_WORK_DEFINE(activity_work, activity_work_handler);

void activity_expiry_function(struct k_timer *_timer) { k_work_submit(&activity_work); }

K_TIMER_DEFINE(activity_timer, activity_expiry_function, NULL);

static int activity_init(void) {
    activity_last_uptime = k_uptime_get();

    k_timer_start(&activity_timer, K_SECONDS(1), K_SECONDS(1));
    return 0;
}

ZMK_LISTENER(activity, activity_event_listener);
ZMK_SUBSCRIPTION(activity, zmk_position_state_changed);
ZMK_SUBSCRIPTION(activity, zmk_sensor_event);

#if IS_ENABLED(CONFIG_ZMK_POINTING)

static void note_activity_work_cb(struct k_work *_work) { note_activity(); }

K_WORK_DEFINE(note_activity_work, note_activity_work_cb);

static void activity_input_listener(struct input_event *ev, void *user_data) {
    k_work_submit(&note_activity_work);
}

INPUT_CALLBACK_DEFINE(NULL, activity_input_listener, NULL);

#endif

SYS_INIT(activity_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);