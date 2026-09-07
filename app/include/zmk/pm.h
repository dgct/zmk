/*
 * Copyright (c) 2023 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

int zmk_pm_suspend_devices(void);
void zmk_pm_resume_devices(void);
void zmk_pm_prepare_for_poweroff(void);
/* With CONFIG_ZMK_SLEEP_DEBUG on nRF: log every GPIO pin whose SENSE is armed. */
void zmk_pm_log_wake_pins(void);

int zmk_pm_soft_off(void);