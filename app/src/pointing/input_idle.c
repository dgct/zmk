/*
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/device.h>
#include <zephyr/pm/device.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#include <zmk/event_manager.h>
#include <zmk/events/activity_state_changed.h>

#define TRACKPAD_NODE DT_INST(0, azoteq_iqs5xx)

#if DT_NODE_EXISTS(TRACKPAD_NODE)

static const struct device *trackpad = DEVICE_DT_GET(TRACKPAD_NODE);

static int input_idle_event_handler(const zmk_event_t *eh) {
    struct zmk_activity_state_changed *ev = as_zmk_activity_state_changed(eh);
    if (ev == NULL) {
        return -ENOTSUP;
    }

    switch (ev->state) {
    case ZMK_ACTIVITY_ACTIVE:
        pm_device_action_run(trackpad, PM_DEVICE_ACTION_RESUME);
        break;
    default:
        break;
    }
    return 0;
}

ZMK_LISTENER(input_idle, input_idle_event_handler);
ZMK_SUBSCRIPTION(input_idle, zmk_activity_state_changed);

#endif /* DT_NODE_EXISTS(TRACKPAD_NODE) */
