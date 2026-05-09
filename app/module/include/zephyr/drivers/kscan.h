/*
 * Copyright (c) 2019 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Public API for Keyboard scan matrix devices.
 *
 * This header was removed from upstream Zephyr in v4.4.
 * ZMK still uses the kscan API for its keyboard scan drivers,
 * so this is kept as a local shim. Syscall annotations removed
 * (ZMK kscan drivers are never called from userspace).
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_KB_SCAN_H_
#define ZEPHYR_INCLUDE_DRIVERS_KB_SCAN_H_

#include <errno.h>
#include <zephyr/types.h>
#include <stddef.h>
#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Keyboard scan callback called when user press/release
 * a key on a matrix keyboard.
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @param row Describes row change.
 * @param column Describes column change.
 * @param pressed Describes the kind of key event.
 */
typedef void (*kscan_callback_t)(const struct device *dev, uint32_t row,
				 uint32_t column,
				 bool pressed);

typedef int (*kscan_config_t)(const struct device *dev,
			      kscan_callback_t callback);
typedef int (*kscan_disable_callback_t)(const struct device *dev);
typedef int (*kscan_enable_callback_t)(const struct device *dev);

__subsystem struct kscan_driver_api {
	kscan_config_t config;
	kscan_disable_callback_t disable_callback;
	kscan_enable_callback_t enable_callback;
};

static inline int kscan_config(const struct device *dev,
			       kscan_callback_t callback)
{
	const struct kscan_driver_api *api =
			(struct kscan_driver_api *)dev->api;

	return api->config(dev, callback);
}

static inline int kscan_enable_callback(const struct device *dev)
{
	const struct kscan_driver_api *api =
			(const struct kscan_driver_api *)dev->api;

	if (api->enable_callback == NULL) {
		return -ENOSYS;
	}

	return api->enable_callback(dev);
}

static inline int kscan_disable_callback(const struct device *dev)
{
	const struct kscan_driver_api *api =
			(const struct kscan_driver_api *)dev->api;

	if (api->disable_callback == NULL) {
		return -ENOSYS;
	}

	return api->disable_callback(dev);
}

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_KB_SCAN_H_ */
