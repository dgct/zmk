/*
 * Copyright (c) 2020 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/class/usb_hid.h>

#include <zmk/keys.h>
#include <zmk/hid.h>

enum zmk_usb_conn_state {
    ZMK_USB_CONN_NONE,
    ZMK_USB_CONN_POWERED,
    ZMK_USB_CONN_HID,
};

enum usb_dc_status_code zmk_usb_get_status(void);
enum zmk_usb_conn_state zmk_usb_get_conn_state(void);

/* True iff hardware VBUS is asserted on the device cable, regardless of
 * USB enumeration state. On nRF this reads USBREGSTATUS.VBUSDETECT;
 * elsewhere returns false. Use this (not zmk_usb_is_powered) to gate
 * sleep/poweroff: KVM switches drop enumeration but keep VBUS hot, and
 * sys_poweroff()'ing a chip with VBUS still applied leads to unclean
 * wake on re-enumeration. */
bool zmk_usb_vbus_present(void);

static inline bool zmk_usb_is_powered(void) {
    return zmk_usb_get_conn_state() != ZMK_USB_CONN_NONE;
}
bool zmk_usb_is_hid_ready(void);
