/*
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 *
 * Deep-sleep breadcrumb.  Every step of the sleep entry writes a marker into
 * RAM that survives a reset (but not System OFF).  The next boot reports the
 * marker it finds and stores it in settings, so a half that has to be reset
 * by hand afterwards says how far it got.
 */
#pragma once

#include <stdint.h>

enum zmk_sleep_trace_step {
    ZMK_SLEEP_TRACE_BOOT = 1,
    ZMK_SLEEP_TRACE_ENTER = 10,
    ZMK_SLEEP_TRACE_LISTENERS_DONE = 11,
    ZMK_SLEEP_TRACE_PREPARED = 12,
    ZMK_SLEEP_TRACE_DEVICES_SUSPENDED = 13,
    ZMK_SLEEP_TRACE_WAKE_PINS_LOGGED = 14,
    ZMK_SLEEP_TRACE_POWERING_OFF = 15,
    ZMK_SLEEP_TRACE_POWEROFF_RETURNED = 16,
    ZMK_SLEEP_TRACE_WAKE_SOURCES = 30,
    /* Per-device steps: base + index of the device in the static device list. */
    ZMK_SLEEP_TRACE_PREPARE_DEV = 1000,
    ZMK_SLEEP_TRACE_SUSPEND_DEV = 2000,
};

#if IS_ENABLED(CONFIG_ZMK_SLEEP_TRACE)
void zmk_sleep_trace(uint32_t step);
#else
static inline void zmk_sleep_trace(uint32_t step) { (void)step; }
#endif
