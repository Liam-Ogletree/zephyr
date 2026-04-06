/*
 * Copyright (c) 2026 Cirrus Logic, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Common functions for Cirrus Logic haptic drivers
 */

#include "cirrus.h"
#include <zephyr/device.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(HAPTICS_CIRRUS, CONFIG_HAPTICS_LOG_LEVEL);

const uint8_t cirrus_source_attenuation[] = {
	0xFF, /* mute */
	0xA0, 0x88, 0x7A, 0x70, 0x68, 0x62, 0x5C, 0x58, 0x54, 0x50, 0x4D, 0x4A, 0x47,
	0x44, 0x42, 0x40, 0x3E, 0x3C, 0x3A, 0x38, 0x36, 0x35, 0x33, 0x32, 0x30, /* 25% */
	0x2F, 0x2D, 0x2C, 0x2B, 0x2A, 0x29, 0x28, 0x27, 0x25, 0x24, 0x23, 0x23, 0x22,
	0x21, 0x20, 0x1F, 0x1E, 0x1D, 0x1D, 0x1C, 0x1B, 0x1A, 0x1A, 0x19, 0x18, /* 50% */
	0x17, 0x17, 0x16, 0x15, 0x15, 0x14, 0x14, 0x13, 0x12, 0x12, 0x11, 0x11, 0x10,
	0x10, 0x0F, 0x0E, 0x0E, 0x0D, 0x0D, 0x0C, 0x0C, 0x0B, 0x0B, 0x0A, 0x0A, /* 75% */
	0x0A, 0x09, 0x09, 0x08, 0x08, 0x07, 0x07, 0x06, 0x06, 0x06, 0x05, 0x05, 0x04,
	0x04, 0x04, 0x03, 0x03, 0x03, 0x02, 0x02, 0x01, 0x01, 0x01, 0x00, 0x00 /* 100% */
};

bool cirrus_is_bus_ready(const struct device *const dev)
{
	const struct cirrus_config *const config = dev->config;

	return config->bus_io->is_ready(dev);
}

const struct device *cirrus_get_control_port(const struct device *const dev)
{
	const struct cirrus_config *const config = dev->config;

	return config->bus_io->get_device(dev);
}

int cirrus_burst_read(const struct device *const dev, const uint32_t addr, uint32_t *const rx,
		      const uint32_t len)
{
	const struct cirrus_config *const config = dev->config;

	return config->bus_io->read(dev, addr, rx, len);
}

int cirrus_read(const struct device *const dev, const uint32_t addr, uint32_t *const rx)
{
	return cirrus_burst_read(dev, addr, rx, 1);
}

int cirrus_burst_write(const struct device *const dev, const uint32_t addr, uint32_t *const tx,
		       const uint32_t len)
{
	const struct cirrus_config *const config = dev->config;

	return config->bus_io->write(dev, addr, tx, len);
}

int cirrus_write(const struct device *const dev, const uint32_t addr, uint32_t val)
{
	return cirrus_burst_write(dev, addr, &val, 1);
}

int cirrus_raw_burst_write(const struct device *const dev, const uint32_t addr,
			   const uint32_t *const tx, const uint32_t len)
{
	const struct cirrus_config *const config = dev->config;

	return config->bus_io->raw_write(dev, addr, tx, len);
}

int cirrus_raw_write(const struct device *const dev, const uint32_t addr, const uint32_t val)
{
	return cirrus_raw_burst_write(dev, addr, &val, 1);
}

int cirrus_multi_write(const struct device *const dev,
		       const struct cirrus_multi_write *const multi_write, const uint32_t len)
{
	int ret;

	for (int i = 0; i < len; i++) {
		ret = cirrus_burst_write(dev, multi_write[i].addr, multi_write[i].buf,
					 multi_write[i].len);
		if (ret < 0) {
			return ret;
		}
	}

	return 0;
}

int cirrus_raw_multi_write(const struct device *const dev,
			   const struct cirrus_multi_write *const multi_write, const uint32_t len)
{
	int ret;

	for (int i = 0; i < len; i++) {
		ret = cirrus_raw_burst_write(dev, multi_write[i].addr, multi_write[i].buf,
					     multi_write[i].len);
		if (ret < 0) {
			return ret;
		}
	}

	return 0;
}

int cirrus_poll(const struct device *const dev, const uint32_t addr, const uint32_t val,
		const k_timeout_t timeout)
{
	__maybe_unused const struct cirrus_config *const config = dev->config;
	const k_timepoint_t end = sys_timepoint_calc(timeout);
	uint32_t reg_val;
	int ret;

	do {
		ret = cirrus_read(dev, addr, &reg_val);
		if (ret < 0) {
			return ret;
		}

		if (reg_val == val) {
			return 0;
		}

		(void)k_sleep(CIRRUS_T_POLLING);
	} while (!sys_timepoint_expired(end));

	LOG_INST_DBG(config->log, "timed out polling 0x%08x, expected 0x%08x but received 0x%08x",
		     addr, val, reg_val);

	return -ETIMEDOUT;
}

int cirrus_wakeup(const struct device *const dev, const k_timeout_t timeout)
{
	__maybe_unused const struct cirrus_config *const config = dev->config;
	const k_timepoint_t end = sys_timepoint_calc(timeout);
	uint32_t dummy;
	int ret;

	do {
		ret = cirrus_read(dev, 0, &dummy);
		if (ret >= 0) {
			return 0;
		}

		(void)k_sleep(CIRRUS_T_POLLING);
	} while (!sys_timepoint_expired(end));

	LOG_INST_DBG(config->log, "failed to wake up device from hibernation");

	return -ETIMEDOUT;
}

void cirrus_error_callback(const struct device *const dev, const uint32_t error_bitmask)
{
	struct cirrus_data *const data = dev->data;

	if (data->error_callback != NULL) {
		data->error_callback(dev, error_bitmask, data->user_data);
	}
}

int cirrus_register_error_callback(const struct device *dev, haptics_error_callback_t cb,
				   void *const user_data)
{
	struct cirrus_data *const data = dev->data;

	data->error_callback = cb;
	data->user_data = user_data;

	return 0;
}

void cirrus_interrupt_handler(const struct device *port, struct gpio_callback *cb, uint32_t pins)
{
	struct cirrus_data *const data = CONTAINER_OF(cb, struct cirrus_data, interrupt_callback);
	__maybe_unused const struct cirrus_config *const config =
		HAPTICS_CIRRUS_CONFIG_FROM_DATA(data);
	int ret;

	ret = k_work_schedule(&data->interrupt_worker, CIRRUS_T_DEBOUNCE);
	if (ret < 0) {
		LOG_INST_DBG(config->log, "failed to queue interrupt worker (%d)", ret);
	}
}

int cirrus_interrupt_config(const struct device *const dev)
{
	const struct cirrus_config *const config = dev->config;
	struct cirrus_data *const data = dev->data;
	int ret;

	if (!HAPTICS_CIRRUS_INTERRUPT(config)) {
		return 0;
	}

	ret = gpio_pin_configure_dt(&config->interrupt_gpio, GPIO_INPUT);
	if (ret < 0) {
		return ret;
	}

	ret = gpio_pin_interrupt_configure_dt(&config->interrupt_gpio, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret < 0) {
		return ret;
	}

	gpio_init_callback(&data->interrupt_callback, cirrus_interrupt_handler,
			   BIT(config->interrupt_gpio.pin));

	return gpio_add_callback_dt(&config->interrupt_gpio, &data->interrupt_callback);
}

int cirrus_trigger_config(const struct device *const dev)
{
	const struct cirrus_config *const config = dev->config;
	int ret;

	if (!HAPTICS_CIRRUS_TRIGGER(config)) {
		return 0;
	}

	for (int i = 0; i < config->num_triggers; i++) {
		ret = gpio_pin_configure_dt(&config->trigger_gpios[i], GPIO_OUTPUT);
		if (ret < 0) {
			return ret;
		}
	}

	return 0;
}

int cirrus_get_trigger(const struct device *const dev, const struct gpio_dt_spec *const gpio,
		       uint8_t *const idx)
{
	const struct cirrus_config *const config = dev->config;
	uint8_t i;

	if (gpio == NULL) {
		return -EINVAL;
	}

	for (i = 0; i < config->num_triggers; i++) {
		if (gpio->port == config->trigger_gpios[i].port &&
		    gpio->pin == config->trigger_gpios[i].pin) {
			*idx = config->trigger_mapping[i];
			return 0;
		}
	}

	return -EINVAL;
}

int cirrus_interrupt_teardown(const struct device *const dev)
{
	const struct cirrus_config *const config = dev->config;
	struct cirrus_data *const data = dev->data;
	int ret;

	if (!HAPTICS_CIRRUS_INTERRUPT(config)) {
		return 0;
	}

	ret = gpio_pin_interrupt_configure_dt(&config->interrupt_gpio, GPIO_INT_DISABLE);
	if (ret < 0) {
		return ret;
	}

	return gpio_remove_callback_dt(&config->interrupt_gpio, &data->interrupt_callback);
}

int cirrus_reset_teardown(const struct device *const dev)
{
	const struct cirrus_config *const config = dev->config;
	int ret;

	if (!HAPTICS_CIRRUS_RESET(config)) {
		return 0;
	}

	ret = gpio_pin_set_dt(&config->reset_gpio, CIRRUS_GPIO_ACTIVE);
	if (ret < 0) {
		return ret;
	}

	if (gpio_pin_configure_dt(&config->reset_gpio, GPIO_DISCONNECTED) < 0) {
		(void)gpio_pin_configure_dt(&config->reset_gpio, GPIO_INPUT);
	}

	return 0;
}

int cirrus_init(const struct device *const dev, k_work_handler_t handler)
{
	const struct cirrus_config *const config = dev->config;
	struct cirrus_data *const data = dev->data;
	int ret;

	ret = k_mutex_init(&data->lock);
	if (ret < 0) {
		return ret;
	}

	if (HAPTICS_CIRRUS_INTERRUPT(config)) {
		k_work_init_delayable(&data->interrupt_worker, handler);
	}

	if (!cirrus_is_bus_ready(dev)) {
		LOG_ERR_DEVICE_NOT_READY(cirrus_get_control_port(dev));
		return -ENODEV;
	}

	if (HAPTICS_CIRRUS_RESET(config)) {
		if (!gpio_is_ready_dt(&config->reset_gpio)) {
			LOG_ERR_DEVICE_NOT_READY(config->reset_gpio.port);
			return -ENODEV;
		}

		ret = gpio_pin_configure_dt(&config->reset_gpio, GPIO_OUTPUT);
		if (ret < 0) {
			return ret;
		}
	}

	if (HAPTICS_CIRRUS_INTERRUPT(config) && !gpio_is_ready_dt(&config->interrupt_gpio)) {
		LOG_ERR_DEVICE_NOT_READY(config->interrupt_gpio.port);
		return -ENODEV;
	}

	if (HAPTICS_CIRRUS_TRIGGER(config)) {
		for (int i = 0; i < config->num_triggers; i++) {
			if (!gpio_is_ready_dt(&config->trigger_gpios[i])) {
				LOG_ERR_DEVICE_NOT_READY(config->trigger_gpios[i].port);
				return -ENODEV;
			}
		}
	}

	if (HAPTICS_CIRRUS_FLASH(config) && !device_is_ready(config->flash)) {
		LOG_ERR_DEVICE_NOT_READY(config->flash);
		return -ENODEV;
	}

	return 0;
}
