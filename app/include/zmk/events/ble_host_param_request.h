/*
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <zephyr/kernel.h>
#include <zmk/event_manager.h>

struct zmk_ble_host_param_request {
    uint16_t interval_min; /* 1.25ms units */
    uint16_t interval_max; /* 1.25ms units */
    uint16_t latency;
    uint16_t timeout;      /* 10ms units */
    bool restore;          /* true = restore to ble_latency's preferred params */
};

ZMK_EVENT_DECLARE(zmk_ble_host_param_request);
