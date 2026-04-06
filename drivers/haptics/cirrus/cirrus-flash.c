/*
 * Copyright (c) 2026 Cirrus Logic, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Flash memory functions for Cirrus Logic haptic drivers
 */

#include "cirrus.h"
#include <zephyr/drivers/flash.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device_runtime.h>

LOG_MODULE_DECLARE(HAPTICS_CIRRUS);

#define CIRRUS_FLASH_ERASED 0xFFFFFFFF

static inline bool cirrus_is_memory_erased(const struct cirrus_calibration *const calibration)
{
	return (calibration->f0 == CIRRUS_FLASH_ERASED && calibration->redc == CIRRUS_FLASH_ERASED);
}

int cirrus_load_calibration(const struct device *const dev)
{
	const struct cirrus_config *const config = dev->config;
	struct cirrus_data *const data = dev->data;
	struct cirrus_calibration calibration = {};
	int ret;

	if (!HAPTICS_CIRRUS_FLASH(config)) {
		return 0;
	}

	ret = pm_device_runtime_get(config->flash);
	if (ret < 0) {
		return ret;
	}

	ret = flash_read(config->flash, config->flash_offset, &calibration, sizeof(calibration));
	if (ret < 0) {
		goto error_pm;
	}

	if (!cirrus_is_memory_erased(&calibration)) {
		data->calibration = calibration;
	} else {
		LOG_INST_DBG(config->log, "calibration data not found");
	}

error_pm:
	(void)pm_device_runtime_put(config->flash);

	return ret;
}

int cirrus_store_calibration(const struct device *const dev)
{
	const struct cirrus_config *const config = dev->config;
	struct cirrus_data *const data = dev->data;
	struct cirrus_calibration calibration = {};
	int ret;

	if (!HAPTICS_CIRRUS_FLASH(config)) {
		return 0;
	}

	ret = pm_device_runtime_get(config->flash);
	if (ret < 0) {
		return ret;
	}

	ret = flash_read(config->flash, config->flash_offset, &calibration, sizeof(calibration));
	if (ret < 0) {
		goto error_pm;
	}

	if (!cirrus_is_memory_erased(&calibration)) {
		LOG_INST_WRN(config->log, "skipping write to flash memory");
		goto error_pm;
	}

	ret = flash_write(config->flash, config->flash_offset, &data->calibration,
			  sizeof(data->calibration));

error_pm:
	(void)pm_device_runtime_put(config->flash);

	return ret;
}
