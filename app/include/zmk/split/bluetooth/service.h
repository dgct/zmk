/*
 * Copyright (c) 2020 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <zmk/events/sensor_event.h>
#include <zmk/sensors.h>

#define ZMK_SPLIT_RUN_BEHAVIOR_DEV_LEN 9

struct sensor_event {
    uint8_t sensor_index;

    uint8_t channel_data_size;
    struct zmk_sensor_channel_data channel_data[ZMK_SENSOR_EVENT_MAX_CHANNELS];
} __packed;

struct zmk_split_run_behavior_data {
    uint8_t position;
    uint8_t source;
    uint8_t state;
    uint32_t param1;
    uint32_t param2;
} __packed;

struct zmk_split_run_behavior_payload {
    struct zmk_split_run_behavior_data data;
    char behavior_dev[ZMK_SPLIT_RUN_BEHAVIOR_DEV_LEN];
} __packed;

struct zmk_split_input_event_payload {
    uint8_t type;
    uint16_t code;
    uint32_t value;
    uint8_t sync;
} __packed;

/* Sentinel `code` value indicating that `value` carries a packed XY pair:
 *   bits  0..15 -> int16_t REL_X delta
 *   bits 16..31 -> int16_t REL_Y delta
 * Used with type = INPUT_EV_REL to halve the on-air notification count for
 * trackpoint/mouse coalescing. 0xFFFF was chosen because Linux relative-axis
 * codes are all small (<=0x0F), making collision impossible.
 */
#define ZMK_SPLIT_BT_INPUT_PACKED_XY_CODE 0xFFFF

#if IS_ENABLED(CONFIG_ZMK_INPUT_SPLIT)
bool zmk_split_bt_input_notify_ready(uint8_t reg);
#endif
