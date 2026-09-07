/*
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#if IS_ENABLED(CONFIG_SETTINGS)
#include <zephyr/settings/settings.h>
#endif
#if defined(CONFIG_SOC_FAMILY_NORDIC_NRF)
#include <hal/nrf_power.h>
#endif

#include <zmk/sleep_trace.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#define SLEEP_TRACE_MAGIC 0x53545243 /* "STRC" */

struct sleep_trace_rec {
    uint32_t magic;
    uint32_t step;
    uint32_t uptime_ms;
};

/* Persisted form: the record found at boot plus that boot's reset reason. */
struct sleep_trace_stored {
    uint32_t step;
    uint32_t uptime_ms;
    uint32_t resetreas;
};

/* Not zeroed at boot: a reset keeps it, System OFF (or a power loss) does not. */
static struct sleep_trace_rec live __noinit;
static struct sleep_trace_rec found;
static bool have_found;
static uint32_t boot_resetreas;
#if IS_ENABLED(CONFIG_SETTINGS)
static struct sleep_trace_stored stored;
static bool have_stored;
#endif

void zmk_sleep_trace(uint32_t step) {
    live.magic = SLEEP_TRACE_MAGIC;
    live.step = step;
    live.uptime_ms = (uint32_t)k_uptime_get();
}

static const char *device_name_at(uint32_t index) {
    const struct device *devs;
    size_t count = z_device_get_all_static(&devs);

    return (index < count) ? devs[index].name : "?";
}

static void describe(uint32_t step, char *buf, size_t len) {
    const char *name;

    switch (step) {
    case ZMK_SLEEP_TRACE_BOOT:
        name = "booted, never tried to sleep";
        break;
    case ZMK_SLEEP_TRACE_ENTER:
        name = "sleep: entering";
        break;
    case ZMK_SLEEP_TRACE_LISTENERS_DONE:
        name = "sleep: activity listeners done";
        break;
    case ZMK_SLEEP_TRACE_PREPARED:
        name = "sleep: devices prepared, wake sources armed";
        break;
    case ZMK_SLEEP_TRACE_DEVICES_SUSPENDED:
        name = "sleep: devices suspended";
        break;
    case ZMK_SLEEP_TRACE_WAKE_PINS_LOGGED:
        name = "sleep: wake pins checked";
        break;
    case ZMK_SLEEP_TRACE_POWERING_OFF:
        name = "sleep: sys_poweroff called (System OFF not taken if this is the last step)";
        break;
    case ZMK_SLEEP_TRACE_POWEROFF_RETURNED:
        name = "sleep: sys_poweroff RETURNED";
        break;
    case ZMK_SLEEP_TRACE_WAKE_SOURCES:
        name = "sleep: re-enabling wake sources";
        break;
    default:
        if (step >= ZMK_SLEEP_TRACE_SUSPEND_DEV) {
            snprintk(buf, len, "sleep: suspending device %s", device_name_at(step - ZMK_SLEEP_TRACE_SUSPEND_DEV));
            return;
        }
        if (step >= ZMK_SLEEP_TRACE_PREPARE_DEV) {
            snprintk(buf, len, "sleep: preparing device %s", device_name_at(step - ZMK_SLEEP_TRACE_PREPARE_DEV));
            return;
        }
        name = "unknown step";
        break;
    }
    snprintk(buf, len, "%s", name);
}

#if IS_ENABLED(CONFIG_SETTINGS)
static int sleep_trace_settings_set(const char *name, size_t len, settings_read_cb read_cb,
                                    void *cb_arg) {
    const char *next;

    if (settings_name_steq(name, "last", &next) && !next) {
        if (len != sizeof(stored)) {
            return -EINVAL;
        }
        if (read_cb(cb_arg, &stored, sizeof(stored)) == sizeof(stored)) {
            have_stored = true;
        }
        return 0;
    }
    return -ENOENT;
}

SETTINGS_STATIC_HANDLER_DEFINE(sleep_trace, "sleep", NULL, sleep_trace_settings_set, NULL,
                               NULL);
#endif /* CONFIG_SETTINGS */

static void report_fn(struct k_work *work) {
    char what[80];

    ARG_UNUSED(work);
    LOG_WRN("SLEEP TRACE: this boot resetreas=0x%08x%s%s%s%s", boot_resetreas,
            (boot_resetreas & 0x01) ? " PIN" : "", (boot_resetreas & 0x02) ? " DOG" : "",
            (boot_resetreas & 0x10000) ? " OFF-wake" : "",
            (boot_resetreas & 0x100000) ? " VBUS" : "");
    if (have_found) {
        describe(found.step, what, sizeof(what));
        LOG_WRN("SLEEP TRACE: the previous run stopped at step %u: %s (uptime %u ms). "
                "RAM survived, so that run never powered off",
                found.step, what, found.uptime_ms);
#if IS_ENABLED(CONFIG_SETTINGS)
        struct sleep_trace_stored rec = {
            .step = found.step, .uptime_ms = found.uptime_ms, .resetreas = boot_resetreas};
        int err = settings_save_one("sleep/last", &rec, sizeof(rec));

        if (err) {
            LOG_WRN("SLEEP TRACE: could not persist (%d)", err);
        }
#endif
    } else {
        LOG_INF("SLEEP TRACE: no record from the previous run (RAM was cleared: a real "
                "power-off, or power was lost)");
    }
#if IS_ENABLED(CONFIG_SETTINGS)
    if (have_stored) {
        describe(stored.step, what, sizeof(what));
        LOG_WRN("SLEEP TRACE (stored): a run once stopped at step %u: %s (uptime %u ms); the "
                "reset that followed had resetreas=0x%08x",
                stored.step, what, stored.uptime_ms, stored.resetreas);
    }
#endif
}

static K_WORK_DELAYABLE_DEFINE(report_work, report_fn);

/* Before crash_capture clears RESETREAS and before anything touches the record. */
static int sleep_trace_early_init(void) {
#if defined(CONFIG_SOC_FAMILY_NORDIC_NRF)
    boot_resetreas = nrf_power_resetreas_get(NRF_POWER);
#endif
    if (live.magic == SLEEP_TRACE_MAGIC) {
        found = live;
        have_found = true;
    }
    live.magic = SLEEP_TRACE_MAGIC;
    live.step = ZMK_SLEEP_TRACE_BOOT;
    live.uptime_ms = 0;
    return 0;
}

SYS_INIT(sleep_trace_early_init, PRE_KERNEL_1, 0);

/* Report once the log backend (USB CDC) has had time to come up, like the
 * fault record; the settings write also needs the settings subsystem loaded. */
static int sleep_trace_init(void) {
    k_work_schedule(&report_work, K_SECONDS(6));
    return 0;
}

SYS_INIT(sleep_trace_init, APPLICATION, 98);
