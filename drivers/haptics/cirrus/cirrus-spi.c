/*
 * Copyright (c) 2026 Cirrus Logic, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief SPI functions for Cirrus Logic haptic drivers
 */

#if CONFIG_HAPTICS_CIRRUS_SPI
#include "cirrus.h"
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(HAPTICS_CIRRUS);

static bool cirrus_is_ready_spi(const struct device *const dev)
{
	const struct cirrus_config *const config = dev->config;

	return spi_is_ready_dt(&config->bus.spi);
}

static const struct device *cirrus_get_device_spi(const struct device *const dev)
{
	const struct cirrus_config *const config = dev->config;

	return config->bus.spi.bus;
}

static int cirrus_read_spi(const struct device *const dev, uint32_t addr, uint32_t *const rx,
			   const uint32_t len)
{
	__maybe_unused const struct cirrus_config *const config = dev->config;

	LOG_INST_ERR(config->log, "SPI not currently supported (%d)", -ENOTSUP);

	return -ENOTSUP;
}

static int cirrus_write_spi(const struct device *const dev, const uint32_t addr, uint32_t *const tx,
			    const uint32_t len)
{
	__maybe_unused const struct cirrus_config *const config = dev->config;

	LOG_INST_ERR(config->log, "SPI not currently supported (%d)", -ENOTSUP);

	return -ENOTSUP;
}

static int cirrus_raw_write_spi(const struct device *const dev, const uint32_t addr,
				const uint32_t *const tx, const uint32_t len)
{
	__maybe_unused const struct cirrus_config *const config = dev->config;

	LOG_INST_ERR(config->log, "SPI not currently supported (%d)", -ENOTSUP);

	return -ENOTSUP;
}

const struct cirrus_bus_io cirrus_bus_io_spi = {
	.is_ready = cirrus_is_ready_spi,
	.get_device = cirrus_get_device_spi,
	.read = cirrus_read_spi,
	.write = cirrus_write_spi,
	.raw_write = cirrus_raw_write_spi,
};
#endif /* CONFIG_HAPTICS_CIRRUS_SPI */
