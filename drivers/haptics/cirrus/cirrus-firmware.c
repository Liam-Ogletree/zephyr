/*
 * Copyright (c) 2026 Cirrus Logic, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Firmware functions for Cirrus Logic haptic drivers
 */

#include "cirrus.h"
#if CONFIG_HAPTICS_CS40L26_A1 || CONFIG_HAPTICS_CS40L27_B2
#include "cs40l26.h"
#endif /* CONFIG_HAPTICS_CS40L26_A1 || CONFIG_HAPTICS_CS40L27_B2 */
#if CONFIG_HAPTICS_CS40L5X
#include "cs40l5x.h"
#endif /* CONFIG_HAPTICS_CS40L5X */
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(HAPTICS_CIRRUS);

#define CIRRUS_REG_UNUSED        0U
#define CIRRUS_REG_FIRMWARE_BASE 0x02800000U
#define CIRRUS_REG_FIRMWARE_EXT  0x00C00000U

enum cirrus_devices {
#if CONFIG_HAPTICS_CS40L26_A1
	CS40L26_A1,
#endif /* CONFIG_HAPTICS_CS40L26_A1 */
#if CONFIG_HAPTICS_CS40L27_B2
	CS40L27_B2,
#endif /* CONFIG_HAPTICS_CS40L27_B2 */
#if CONFIG_HAPTICS_CS40L5X
	CS40L5X_B0,
#endif /* CONFIG_HAPTICS_CS40L5X */
	CIRRUS_NUM_DEVICES,
};

static const uint16_t *const cirrus_firmware[CIRRUS_NUM_DEVICES] = {
/* CS40L26 A1 ROM register map */
#if CONFIG_HAPTICS_CS40L26_A1
	(uint16_t[]){
		[CS40L26_REG_PM_TIMER_TIMEOUT_TICKS] = 0x0350U,
		[CS40L26_REG_PM_ACTIVE_TIMEOUT] = 0x0360U,
		[CS40L26_REG_PM_STDBY_TIMEOUT] = 0x0378U,
		[CS40L26_REG_PM_POWER_ON_SEQUENCE] = 0x03E8U,
		[CS40L26_REG_DYNAMIC_F0_ENABLED] = 0x0F48U,
		[CS40L26_REG_HALO_STATE] = 0x0FA8U,
		[CS40L26_REG_RE_EST_STATUS] = 0x1E6CU,
		[CS40L26_REG_VIBEGEN_F0_OTP_STORED] = 0x2128U,
		[CS40L26_REG_VIBEGEN_REDC_OTP_STORED] = 0x212CU,
		[CS40L26_REG_VIBEGEN_COMPENSATION_ENABLE] = 0x2158U,
		[CS40L26_REG_F0_EST_REDC] = 0x64FCU,
		[CS40L26_REG_F0_EST] = 0x6504U,
		[CS40L26_REG_LOGGER_ENABLE] = 0x6CE0U,
		[CS40L26_REG_LOGGER_DATA] = 0x6D38U,
		[CS40L26_REG_GPIO_EVENT_BASE] = 0x6FC4U,
		[CS40L26_REG_BUZZ_FREQ] = 0x7008U,
		[CS40L26_REG_BUZZ_LEVEL] = 0x700CU,
		[CS40L26_REG_BUZZ_DURATION] = 0x7010U,
		[CS40L26_REG_MAILBOX_QUEUE_WT] = 0x7404U,
		[CS40L26_REG_MAILBOX_QUEUE_RD] = 0x7408U,
		[CS40L26_REG_MAILBOX_STATUS] = 0x740CU,
		[CS40L26_REG_SOURCE_ATTENUATION] = CIRRUS_REG_UNUSED,
	},
#endif /* CONFIG_HAPTICS_CS40L26_A1 */
/* CS40L27 B2 ROM register map */
#if CONFIG_HAPTICS_CS40L27_B2
	(uint16_t[]){
		[CS40L26_REG_PM_TIMER_TIMEOUT_TICKS] = 0x1F78U,
		[CS40L26_REG_PM_ACTIVE_TIMEOUT] = 0x1F88U,
		[CS40L26_REG_PM_STDBY_TIMEOUT] = 0x1FA0U,
		[CS40L26_REG_PM_POWER_ON_SEQUENCE] = 0x2018U,
		[CS40L26_REG_VIBEGEN_F0_OTP_STORED] = 0x2F30U,
		[CS40L26_REG_VIBEGEN_REDC_OTP_STORED] = 0x2F34U,
		[CS40L26_REG_VIBEGEN_COMPENSATION_ENABLE] = 0x2F54U,
		[CS40L26_REG_RE_EST_STATUS] = 0x666CU,
		[CS40L26_REG_LOGGER_ENABLE] = 0x67B4U,
		[CS40L26_REG_LOGGER_DATA] = 0x680CU,
		[CS40L26_REG_DYNAMIC_F0_ENABLED] = 0x69F4U,
		[CS40L26_REG_HALO_STATE] = 0x6AF8U,
		[CS40L26_REG_SOURCE_ATTENUATION] = 0x6B58U,
		[CS40L26_REG_BUZZ_FREQ] = 0x6D28U,
		[CS40L26_REG_BUZZ_LEVEL] = 0x6D2CU,
		[CS40L26_REG_BUZZ_DURATION] = 0x6D30U,
		[CS40L26_REG_GPIO_EVENT_BASE] = 0x6FB0U,
		[CS40L26_REG_F0_EST_REDC] = 0x720CU,
		[CS40L26_REG_F0_EST] = 0x7214U,
		[CS40L26_REG_MAILBOX_QUEUE_WT] = 0x74A0U,
		[CS40L26_REG_MAILBOX_QUEUE_RD] = 0x74A4U,
		[CS40L26_REG_MAILBOX_STATUS] = 0x74A8U,
	},
#endif /* CONFIG_HAPTICS_CS40L27_B2 */
/* CS40L5x B0 ROM register map */
#if CONFIG_HAPTICS_CS40L5X
	(uint16_t[]){
		[CS40L5X_REG_HALO_STATE] = 0x21E0U,
		[CS40L5X_REG_BUZZ_FREQ] = 0x27A0U,
		[CS40L5X_REG_BUZZ_LEVEL] = 0x27A4U,
		[CS40L5X_REG_BUZZ_DURATION] = 0x27A8U,
		[CS40L5X_REG_BUZZ_RES] = 0x27ECU,
		[CS40L5X_REG_DYNAMIC_F0] = 0x285CU,
		[CS40L5X_REG_REDC] = 0x2F7CU,
		[CS40L5X_REG_F0_EST] = 0x2F84U,
		[CS40L5X_REG_SOURCE_ATTENUATION] = 0x30B8U,
		[CS40L5X_REG_LOGGER_ENABLE] = 0x33E8U,
		[CS40L5X_REG_LOGGER_DATA] = 0x3440U,
		[CS40L5X_REG_GPIO_EVENT_BASE] = 0x4140U,
		[CS40L5X_REG_STDBY_TIMEOUT] = 0x42F8U,
		[CS40L5X_REG_ACTIVE_TIMEOUT] = 0x4300U,
		[CS40L5X_REG_MAILBOX_QUEUE_WT] = 0x42C8U,
		[CS40L5X_REG_MAILBOX_QUEUE_RD] = 0x42CCU,
		[CS40L5X_REG_MAILBOX_STATUS] = 0x42D0U,
		[CS40L5X_REG_WSEQ_POWER] = 0x4348U,
		[CS40L5X_REG_VIBEGEN_F0_OTP_STORED] = 0x5C00U,
		[CS40L5X_REG_VIBEGEN_REDC_OTP_STORED] = 0x5C08U,
		[CS40L5X_REG_VIBEGEN_COMPENSATION_ENABLE] = 0x5C30U,
		[CS40L5X_REG_CUSTOM_HEADER1_0] = 0x7770U,
		[CS40L5X_REG_CUSTOM_HEADER2_0] = 0x777CU,
		[CS40L5X_REG_CUSTOM_DATA_0] = 0x7784U,
		[CS40L5X_REG_CUSTOM_HEADER1_1] = 0x797CU,
		[CS40L5X_REG_CUSTOM_HEADER2_1] = 0x7988U,
		[CS40L5X_REG_CUSTOM_DATA_1] = 0x7990U,
		[CS40L5X_REG_CALIB_REDC_EST] = 0x1110U,
	},
#endif /* CONFIG_HAPTICS_CS40L5X */
};

static inline int cirrus_get_firmware_address(const struct device *const dev,
					      const uint32_t control, uint32_t *const addr)
{
	const struct cirrus_config *const config = dev->config;
	*addr = CIRRUS_REG_FIRMWARE_BASE;

	switch (config->device_id) {
#if CONFIG_HAPTICS_CS40L26_A1
	case CS40L26_DEVID_A1:
		*addr |= cirrus_firmware[CS40L26_A1][control];
		break;
#endif /* CONFIG_HAPTICS_CS40L26_A1 */
#if CONFIG_HAPTICS_CS40L27_B2
	case CS40L27_DEVID_B2:
		*addr |= cirrus_firmware[CS40L27_B2][control];
		break;
#endif /* CONFIG_HAPTICS_CS40L27_B2 */
#if CONFIG_HAPTICS_CS40L5X
	case CS40L5X_DEVID_50:
		__fallthrough;
	case CS40L5X_DEVID_51:
		__fallthrough;
	case CS40L5X_DEVID_52:
		__fallthrough;
	case CS40L5X_DEVID_53:
		if (control == CS40L5X_REG_CALIB_REDC_EST) {
			*addr += CIRRUS_REG_FIRMWARE_EXT;
		}

		*addr |= cirrus_firmware[CS40L5X_B0][control];
		break;
#endif /* CONFIG_HAPTICS_CS40L5X */
	default:
		return -ENODEV;
	}

	if (*addr == CIRRUS_REG_FIRMWARE_BASE) {
		LOG_INST_DBG(config->log, "unused firmware control");
		return -EINVAL;
	}

	return 0;
}

/* Make sure the offset does not overflow a uint32_t in either direction. */
static inline int cirrus_offset_overflow(const struct device *const dev, const uint32_t addr,
					 const off_t offset)
{
	__maybe_unused const struct cirrus_config *const config = dev->config;

	if (!IN_RANGE(offset, -(int64_t)addr, UINT32_MAX - addr)) {
		LOG_INST_DBG(config->log, "firmware offset not in range");
		return -EINVAL;
	}

	return 0;
}

int cirrus_firmware_burst_read(const struct device *const dev, const uint32_t control,
			       uint32_t *const rx, const uint32_t len)
{
	const struct cirrus_config *const config = dev->config;
	uint32_t addr;
	int ret;

	ret = cirrus_get_firmware_address(dev, control, &addr);
	if (ret < 0) {
		return ret;
	}

	return config->bus_io->read(dev, addr, rx, len);
}

int cirrus_firmware_burst_read_offset(const struct device *const dev, const uint32_t control,
				      uint32_t *const rx, const uint32_t len, const off_t offset)
{
	const struct cirrus_config *const config = dev->config;
	uint32_t addr;
	int ret;

	ret = cirrus_get_firmware_address(dev, control, &addr);
	if (ret < 0) {
		return ret;
	}

	ret = cirrus_offset_overflow(dev, addr, offset);
	if (ret < 0) {
		return ret;
	}

	return config->bus_io->read(dev, addr + offset, rx, len);
}

int cirrus_firmware_read(const struct device *const dev, const uint32_t control, uint32_t *const rx)
{
	return cirrus_firmware_burst_read(dev, control, rx, 1);
}

int cirrus_firmware_read_offset(const struct device *const dev, const uint32_t control,
				uint32_t *const rx, const off_t offset)
{
	return cirrus_firmware_burst_read_offset(dev, control, rx, 1, offset);
}

int cirrus_firmware_burst_write(const struct device *const dev, const uint32_t control,
				uint32_t *const tx, const uint32_t len)
{
	const struct cirrus_config *const config = dev->config;
	uint32_t addr;
	int ret;

	ret = cirrus_get_firmware_address(dev, control, &addr);
	if (ret < 0) {
		return ret;
	}

	return config->bus_io->write(dev, addr, tx, len);
}

int cirrus_firmware_burst_write_offset(const struct device *const dev, const uint32_t control,
				       uint32_t *const tx, const uint32_t len, const off_t offset)
{
	const struct cirrus_config *const config = dev->config;
	uint32_t addr;
	int ret;

	ret = cirrus_get_firmware_address(dev, control, &addr);
	if (ret < 0) {
		return ret;
	}

	ret = cirrus_offset_overflow(dev, addr, offset);
	if (ret < 0) {
		return ret;
	}

	return config->bus_io->write(dev, addr + offset, tx, len);
}

int cirrus_firmware_write(const struct device *const dev, const uint32_t control, uint32_t val)
{
	return cirrus_firmware_burst_write(dev, control, &val, 1);
}

int cirrus_firmware_write_offset(const struct device *const dev, const uint32_t control,
				 uint32_t val, const off_t offset)
{
	return cirrus_firmware_burst_write_offset(dev, control, &val, 1, offset);
}

int cirrus_firmware_raw_burst_write(const struct device *const dev, const uint32_t control,
				    const uint32_t *const tx, const uint32_t len)
{
	const struct cirrus_config *const config = dev->config;
	uint32_t addr;
	int ret;

	ret = cirrus_get_firmware_address(dev, control, &addr);
	if (ret < 0) {
		return ret;
	}

	return config->bus_io->raw_write(dev, addr, tx, len);
}

int cirrus_firmware_raw_burst_write_offset(const struct device *const dev, const uint32_t control,
					   const uint32_t *const tx, const uint32_t len,
					   const off_t offset)
{
	const struct cirrus_config *const config = dev->config;
	uint32_t addr;
	int ret;

	ret = cirrus_get_firmware_address(dev, control, &addr);
	if (ret < 0) {
		return ret;
	}

	ret = cirrus_offset_overflow(dev, addr, offset);
	if (ret < 0) {
		return ret;
	}

	return config->bus_io->raw_write(dev, addr + offset, tx, len);
}

int cirrus_firmware_raw_write(const struct device *const dev, const uint32_t control,
			      const uint32_t val)
{
	return cirrus_firmware_raw_burst_write(dev, control, &val, 1);
}

int cirrus_firmware_multi_write(const struct device *const dev,
				const struct cirrus_multi_write *const multi_write,
				const uint32_t len)
{
	int ret;

	for (int i = 0; i < len; i++) {
		ret = cirrus_firmware_burst_write(dev, multi_write[i].addr, multi_write[i].buf,
						  multi_write[i].len);
		if (ret < 0) {
			return ret;
		}
	}

	return 0;
}

int cirrus_firmware_raw_multi_write(const struct device *const dev,
				    const struct cirrus_multi_write *const multi_write,
				    const uint32_t len)
{
	int ret;

	for (int i = 0; i < len; i++) {
		ret = cirrus_firmware_raw_burst_write(dev, multi_write[i].addr, multi_write[i].buf,
						      multi_write[i].len);
		if (ret < 0) {
			return ret;
		}
	}

	return 0;
}

int cirrus_firmware_poll(const struct device *const dev, const uint32_t control, const uint32_t val,
			 const k_timeout_t timeout)
{
	uint32_t addr;
	int ret;

	ret = cirrus_get_firmware_address(dev, control, &addr);
	if (ret < 0) {
		return ret;
	}

	return cirrus_poll(dev, addr, val, timeout);
}
