/*
 * Copyright (c) 2024 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_input_split

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/input/input.h>
#include <zephyr/sys/util.h>
#include <drivers/input_processor.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>

#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#include <zmk/split/bluetooth/service.h>

#if IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)

struct zis_entry {
    uint8_t reg;
    const struct device *dev;
};

#define ZIS_ENTRY(n) {.reg = DT_INST_REG_ADDR(n), .dev = DEVICE_DT_GET(DT_DRV_INST(n))},

static const struct zis_entry proxy_inputs[] = {DT_INST_FOREACH_STATUS_OKAY(ZIS_ENTRY)};

static uint16_t proxy_active_keys[DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT)]
                                 [CONFIG_ZMK_INPUT_SPLIT_MAX_TRACKED_KEYS];

static int replace_active_key(size_t input, uint16_t find, uint16_t replace) {
    for (size_t k = 0; k < CONFIG_ZMK_INPUT_SPLIT_MAX_TRACKED_KEYS; k++) {
        if (proxy_active_keys[input][k] == find) {
            proxy_active_keys[input][k] = replace;
            return 0;
        }
    }

    return -ENODEV;
}

int zmk_input_split_report_peripheral_event(uint8_t reg, uint8_t type, uint16_t code, int32_t value,
                                            bool sync) {
    LOG_DBG("Got peripheral event for %d!", reg);
    for (size_t i = 0; i < ARRAY_SIZE(proxy_inputs); i++) {
        if (reg == proxy_inputs[i].reg) {
            /* Unpack a coalesced XY event into two input_reports. The peripheral
             * encodes (int16_t)X in low 16 bits and (int16_t)Y in high 16 bits
             * to halve the on-air notification count.
             */
            if (type == INPUT_EV_REL && code == ZMK_SPLIT_BT_INPUT_PACKED_XY_CODE) {
                uint32_t packed = (uint32_t)value;
                int16_t x = (int16_t)(uint16_t)(packed & 0xFFFFU);
                int16_t y = (int16_t)(uint16_t)((packed >> 16) & 0xFFFFU);
                int ret = input_report(proxy_inputs[i].dev, INPUT_EV_REL, INPUT_REL_X,
                                       (int32_t)x, false, K_NO_WAIT);
                if (ret < 0) {
                    return ret;
                }
                return input_report(proxy_inputs[i].dev, INPUT_EV_REL, INPUT_REL_Y,
                                    (int32_t)y, sync, K_NO_WAIT);
            }
            if (type == INPUT_EV_KEY) {
                uint16_t find, replace;

                if (value) {
                    find = 0;
                    replace = code;
                } else {
                    find = code;
                    replace = 0;
                }

                int ret = replace_active_key(i, find, replace);
                if (ret < 0) {
                    LOG_WRN("Failed to %s key %d", value ? "track pressed" : "untrack released",
                            ret);
                }
            }
            return input_report(proxy_inputs[i].dev, type, code, value, sync, K_NO_WAIT);
        }
    }

    return -ENODEV;
}

int zmk_input_split_peripheral_disconnected(uint8_t reg) {
    for (size_t i = 0; i < ARRAY_SIZE(proxy_inputs); i++) {
        if (reg == proxy_inputs[i].reg) {
            uint16_t prev = 0;
            for (size_t k = 0; k < CONFIG_ZMK_INPUT_SPLIT_MAX_TRACKED_KEYS; k++) {
                if (proxy_active_keys[i][k] != 0) {
                    if (prev != 0) {
                        int ret = input_report(proxy_inputs[i].dev, INPUT_EV_KEY, prev, 0, false,
                                               K_NO_WAIT);
                        if (ret < 0) {
                            LOG_WRN("Failed to report release event on disconnect (%d)", ret);
                        }
                    }

                    prev = proxy_active_keys[i][k];
                    proxy_active_keys[i][k] = 0;
                }
            }

            if (prev != 0) {
                int ret = input_report(proxy_inputs[i].dev, INPUT_EV_KEY, prev, 0, true, K_NO_WAIT);
                if (ret < 0) {
                    LOG_WRN("Failed to report release event on disconnect (%d)", ret);
                }
            }
        }
    }

    return -ENODEV;
}

#define ZIS_INST(n)                                                                                \
    DEVICE_DT_INST_DEFINE(n, NULL, NULL, NULL, NULL, POST_KERNEL,                                  \
                          CONFIG_ZMK_INPUT_SPLIT_INIT_PRIORITY, NULL);

#else

#include <zmk/split/peripheral.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <zmk/split/bluetooth/service.h>

static void split_input_send_event(uint8_t reg, uint8_t type, uint16_t code, int32_t value,
                                   bool sync) {
    struct zmk_split_transport_peripheral_event ev = {
        .type = ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_INPUT_EVENT,
        .data = {.input_event = {
                     .reg = reg,
                     .type = type,
                     .code = code,
                     .value = value,
                     .sync = sync,
                 }}};
    zmk_split_peripheral_report_event(&ev);
}

#define ZIS_INST(n)                                                                                \
    static const struct zmk_input_processor_entry processors_##n[] =                               \
        COND_CODE_1(DT_INST_NODE_HAS_PROP(n, input_processors),                                    \
                    ({LISTIFY(DT_INST_PROP_LEN(n, input_processors),                               \
                              ZMK_INPUT_PROCESSOR_ENTRY_AT_IDX, (, ), DT_DRV_INST(n))}),           \
                    ({}));                                                                         \
    BUILD_ASSERT(DT_INST_NODE_HAS_PROP(n, device),                                                 \
                 "Peripheral input splits need an `input` property set");                          \
    static int32_t acc_rel_x_##n;                                                                  \
    static int32_t acc_rel_y_##n;                                                                  \
    static bool acc_dirty_##n;                                                                     \
    static int64_t acc_last_time_##n;                                                              \
    void split_input_handler_##n(struct input_event *evt, void *user_data) {                       \
        for (size_t i = 0; i < ARRAY_SIZE(processors_##n); i++) {                                  \
            int ret = zmk_input_processor_handle_event(processors_##n[i].dev, evt,                 \
                                                       processors_##n[i].param1,                   \
                                                       processors_##n[i].param2, NULL);            \
            if (ret != ZMK_INPUT_PROC_CONTINUE) {                                                  \
                return;                                                                            \
            }                                                                                      \
        }                                                                                          \
        int64_t now = k_uptime_get();                                                              \
        if (acc_dirty_##n && (now - acc_last_time_##n) > 1000) {                                   \
            acc_rel_x_##n = 0;                                                                     \
            acc_rel_y_##n = 0;                                                                     \
            acc_dirty_##n = false;                                                                 \
        }                                                                                          \
        if (evt->type == INPUT_EV_REL && evt->code == INPUT_REL_X) {                               \
            acc_rel_x_##n += evt->value;                                                           \
            acc_dirty_##n = true;                                                                  \
            acc_last_time_##n = now;                                                               \
        } else if (evt->type == INPUT_EV_REL && evt->code == INPUT_REL_Y) {                        \
            acc_rel_y_##n += evt->value;                                                           \
            acc_dirty_##n = true;                                                                  \
            acc_last_time_##n = now;                                                               \
        } else {                                                                                   \
            split_input_send_event(DT_INST_REG_ADDR(n), evt->type,                                \
                                   evt->code, evt->value, evt->sync);                              \
        }                                                                                          \
        if (evt->sync && acc_dirty_##n &&                                                          \
            zmk_split_bt_input_notify_ready(DT_INST_REG_ADDR(n))) {                                \
            int32_t cx = CLAMP(acc_rel_x_##n, INT16_MIN, INT16_MAX);                               \
            int32_t cy = CLAMP(acc_rel_y_##n, INT16_MIN, INT16_MAX);                               \
            bool has_x = (cx != 0);                                                                \
            bool has_y = (cy != 0);                                                                \
            if (has_x && has_y) {                                                                  \
                /* Pack X (low 16) and Y (high 16) into one notification.        */                \
                /* Mask through uint16_t to drop sign-extension into upper bits. */                \
                uint32_t packed = ((uint32_t)(uint16_t)(int16_t)cx) |                              \
                                  (((uint32_t)(uint16_t)(int16_t)cy) << 16);                       \
                split_input_send_event(DT_INST_REG_ADDR(n), INPUT_EV_REL,                          \
                                       ZMK_SPLIT_BT_INPUT_PACKED_XY_CODE,                          \
                                       (int32_t)packed, true);                                     \
            } else if (has_x) {                                                                    \
                split_input_send_event(DT_INST_REG_ADDR(n), INPUT_EV_REL,                          \
                                       INPUT_REL_X, cx, true);                                     \
            } else if (has_y) {                                                                    \
                split_input_send_event(DT_INST_REG_ADDR(n), INPUT_EV_REL,                          \
                                       INPUT_REL_Y, cy, true);                                     \
            }                                                                                      \
            /* Preserve any residual motion CLAMP truncated (saturating flush). */                 \
            /* In branches where we did not send an axis, cx/cy is 0, so this is a no-op. */       \
            acc_rel_x_##n -= cx;                                                                   \
            acc_rel_y_##n -= cy;                                                                   \
            acc_dirty_##n = (acc_rel_x_##n != 0) || (acc_rel_y_##n != 0);                          \
        }                                                                                          \
    }                                                                                              \
    INPUT_CALLBACK_DEFINE(DEVICE_DT_GET(DT_INST_PHANDLE(n, device)), split_input_handler_##n, NULL);

#endif

DT_INST_FOREACH_STATUS_OKAY(ZIS_INST)
