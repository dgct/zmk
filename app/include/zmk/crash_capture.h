/*
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <stdint.h>

/**
 * @brief Hand the previous run's fatal-error context to the board at boot.
 *
 * crash_capture.c keeps the context of a kernel fault -- which now includes a
 * stack-canary trip, a fortified-libc abort and a null-page exception -- in a
 * .noinit record that survives the reset it causes.  Nothing else survives: a
 * battery pull or the deep sleep ZMK enters on its idle timeout clears that
 * RAM, and an image built without a log backend cannot report it at all.  A
 * board with durable storage can persist it from here instead.
 *
 * Called once from crash_capture_init(), only when a record was found, before
 * the deferred report.  Weakly defined as a no-op in crash_capture.c.
 *
 * @param reason K_ERR_* code the fatal handler was given.
 * @param pc     Program counter from the exception frame.
 * @param lr     Link register from the exception frame.
 * @param cfsr   Configurable Fault Status Register.
 * @param hfsr   HardFault Status Register.
 */
void zmk_crash_capture_boot_record(uint32_t reason, uint32_t pc, uint32_t lr, uint32_t cfsr,
                                   uint32_t hfsr);
