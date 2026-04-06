/*
 * Copyright (c) 2026 Cirrus Logic, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief I2C functions for Cirrus Logic haptic drivers
 */

#if CONFIG_HAPTICS_CIRRUS_I2C
#include "cirrus.h"
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

LOG_MODULE_DECLARE(HAPTICS_CIRRUS);

static bool cirrus_is_ready_i2c(const struct device *const dev)
{
	const struct cirrus_config *const config = dev->config;

	return i2c_is_ready_dt(&config->bus.i2c);
}

static const struct device *cirrus_get_device_i2c(const struct device *const dev)
{
	const struct cirrus_config *const config = dev->config;

	return config->bus.i2c.bus;
}

static int cirrus_read_i2c(const struct device *const dev, const uint32_t addr, uint32_t *const rx,
			   const uint32_t len)
{
	const struct cirrus_config *const config = dev->config;
	uint8_t addr_be32[4];
	int ret;

	sys_put_be32(addr, addr_be32);

	ret = i2c_write_read_dt(&config->bus.i2c, addr_be32, CIRRUS_ADDRESS_WIDTH, (uint8_t *)rx,
				len * CIRRUS_REGISTER_WIDTH);
	if (ret < 0) {
		LOG_INST_DBG(config->log, "failed I2C read transaction (%d)", ret);
		return ret;
	}

	for (int i = 0; i < len; i++) {
		rx[i] = sys_get_be32((uint8_t *)&rx[i]);
	}

	return 0;
}

static int cirrus_raw_write_i2c(const struct device *const dev, const uint32_t addr,
				const uint32_t *const tx, const uint32_t len)
{
	const struct cirrus_config *const config = dev->config;
	struct i2c_msg msgs[2];
	uint8_t addr_be32[4];
	int ret;

	sys_put_be32(addr, addr_be32);

	msgs[0].buf = addr_be32;
	msgs[0].len = CIRRUS_ADDRESS_WIDTH;
	msgs[0].flags = I2C_MSG_WRITE;
	msgs[1].buf = (uint8_t *)tx;
	msgs[1].len = len * CIRRUS_REGISTER_WIDTH;
	msgs[1].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	ret = i2c_transfer_dt(&config->bus.i2c, msgs, ARRAY_SIZE(msgs));
	if (ret < 0) {
		LOG_INST_DBG(config->log, "failed I2C write transaction (%d)", ret);
		return ret;
	}

	return 0;
}

static int cirrus_write_i2c(const struct device *const dev, const uint32_t addr, uint32_t *const tx,
			    const uint32_t len)
{
	for (int i = 0; i < len; i++) {
		sys_put_be32(tx[i], (uint8_t *)&tx[i]);
	}

	return cirrus_raw_write_i2c(dev, addr, tx, len);
}

const struct cirrus_bus_io cirrus_bus_io_i2c = {
	.is_ready = cirrus_is_ready_i2c,
	.get_device = cirrus_get_device_i2c,
	.read = cirrus_read_i2c,
	.write = cirrus_write_i2c,
	.raw_write = cirrus_raw_write_i2c,
};
#endif /* CONFIG_HAPTICS_CIRRUS_I2C */
