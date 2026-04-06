/*
 * Copyright (c) 2025 Cirrus Logic, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Core driver for Cirrus Logic CS40L5x haptic drivers
 */

#include "cs40l5x.h"
#include <stdlib.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/haptics/cs40l5x.h>
#include <zephyr/drivers/regulator.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device_runtime.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/util_macro.h>

LOG_MODULE_REGISTER(CS40L5X, CONFIG_HAPTICS_LOG_LEVEL);

/* Register map */
#define CS40L5X_REG_DEVID               0x00000000U
#define CS40L5X_REG_REVID               (CS40L5X_REG_DEVID + 0x4)
#define CS40L5X_REG_OTPID               (CS40L5X_REG_REVID + 0xC)
#define CS40L5X_REG_SFT_RESET           0x00000020U
#define CS40L5X_REG_IRQ1_STATUS         0x0000E004U
#define CS40L5X_REG_IRQ1_INT_1          0x0000E010U
#define CS40L5X_REG_IRQ1_INT_2          (CS40L5X_REG_IRQ1_INT_1 + 0x4)
#define CS40L5X_REG_IRQ1_INT_8          (CS40L5X_REG_IRQ1_INT_2 + 0x18)
#define CS40L5X_REG_IRQ1_INT_9          (CS40L5X_REG_IRQ1_INT_8 + 0x4)
#define CS40L5X_REG_IRQ1_INT_10         (CS40L5X_REG_IRQ1_INT_9 + 0x4)
#define CS40L5X_REG_IRQ1_INT_14         0x0000E044U
#define CS40L5X_REG_IRQ1_INT_18         0x0000E054U
#define CS40L5X_REG_IRQ1_MASK_1         0x0000E090U
#define CS40L5X_REG_IRQ1_MASK_2         (CS40L5X_REG_IRQ1_MASK_1 + 0x4)
#define CS40L5X_REG_IRQ1_MASK_3         (CS40L5X_REG_IRQ1_MASK_2 + 0x4)
#define CS40L5X_REG_IRQ1_MASK_4         (CS40L5X_REG_IRQ1_MASK_3 + 0x4)
#define CS40L5X_REG_IRQ1_MASK_5         (CS40L5X_REG_IRQ1_MASK_4 + 0x4)
#define CS40L5X_REG_IRQ1_MASK_6         (CS40L5X_REG_IRQ1_MASK_5 + 0x4)
#define CS40L5X_REG_IRQ1_MASK_7         (CS40L5X_REG_IRQ1_MASK_6 + 0x4)
#define CS40L5X_REG_IRQ1_MASK_8         (CS40L5X_REG_IRQ1_MASK_7 + 0x4)
#define CS40L5X_REG_IRQ1_MASK_14        0x0000E0C4U
#define CS40L5X_REG_IRQ1_MASK_18        0x0000E0D4U
#define CS40L5X_REG_IRQ1_MASK_19        (CS40L5X_REG_IRQ1_MASK_18 + 0x4)
#define CS40L5X_REG_IRQ1_MASK_20        (CS40L5X_REG_IRQ1_MASK_19 + 0x4)
#define CS40L5X_REG_DSP_MBOX_2          0x00011004U
#define CS40L5X_REG_DSP_MBOX_8          (CS40L5X_REG_DSP_MBOX_2 + 0x18)
#define CS40L5X_REG_DSP_VIRTUAL1_MBOX_1 0x00011020U

/* Masks */
#define CS40L5X_MASK_AMP_SHORT_ERR_INT1        BIT(31)
#define CS40L5X_MASK_TEMP_ERR_INT8             BIT(31)
#define CS40L5X_MASK_BST_UVP_ERR_INT9          BIT(6)
#define CS40L5X_MASK_BST_SHORT_ERR_INT9        BIT(7)
#define CS40L5X_MASK_BST_ILIMIT_ERR_INT9       BIT(8)
#define CS40L5X_MASK_UVLO_VDDBATT_ERR_INT10    BIT(16)
#define CS40L5X_MASK_DSP_VIRTUAL2_MBOX_WR_INT2 BIT(21)
#define CS40L5X_MASK_ATTENUATION               GENMASK(11, 9)
#define CS40L5X_MASK_BUZ_BANK                  BIT(7)
#define CS40L5X_MASK_CUSTOM_BANK               BIT(16)

/* Outbound mailbox codes */
#define CS40L5X_MBOX_PREVENT_HIBERNATION 0x02000003U
#define CS40L5X_MBOX_ALLOW_HIBERNATION   0x02000004U
#define CS40L5X_MBOX_START_F0_EST        0x07000001U
#define CS40L5X_MBOX_START_REDC_EST      0x07000002U

/* Inbound mailbox codes */
#define CS40L5X_MBOX_PLAYBACK_COMPLETE_MBOX   0x01000000U
#define CS40L5X_MBOX_PLAYBACK_COMPLETE_GPIO   0x01000001U
#define CS40L5X_MBOX_PLAYBACK_START_MBOX      0x01000010U
#define CS40L5X_MBOX_PLAYBACK_START_GPIO      0x01000011U
#define CS40L5X_MBOX_INIT                     0x02000000U
#define CS40L5X_MBOX_AWAKE                    0x02000002U
#define CS40L5X_MBOX_F0_EST_START             0x07000011U
#define CS40L5X_MBOX_F0_EST_DONE              0x07000021U
#define CS40L5X_MBOX_REDC_EST_START           0x07000012U
#define CS40L5X_MBOX_REDC_EST_DONE            0x07000022U
#define CS40L5X_MBOX_PERMANENT_SHORT_DETECTED 0x0C000C1CU
#define CS40L5X_MBOX_RUNTIME_SHORT_DETECTED   0x0C000C1DU

/* Wavetable commands */
#define CS40L5X_ROM_BANK_CMD    0x01800000U
#define CS40L5X_CUSTOM_BANK_CMD 0x01400000U
#define CS40L5X_BUZ_BANK_CMD    0x01000080U

/* Timing specifications */
#define CS40L5X_T_RLPW              K_MSEC(1)
#define CS40L5X_T_IRS               K_MSEC(3)
#define CS40L5X_T_WAKESOURCE        K_MSEC(10)
#define CS40L5X_T_MBOX_CLEAR        K_MSEC(10)
#define CS40L5X_T_CALIBRATION_START K_MSEC(1000)
#define CS40L5X_T_REDC_EST_DONE     K_MSEC(30)
#define CS40L5X_T_REDC_CALIBRATION  K_MSEC(1030)
#define CS40L5X_T_F0_EST_DONE       K_MSEC(1500)
#define CS40L5X_T_F0_CALIBRATION    K_MSEC(2500)

/* Kconfig options */
#define CS40L5X_PM_ACTIVE_TIMEOUT CONFIG_HAPTICS_CS40L5X_PM_ACTIVE_TIMEOUT
#define CS40L5X_PM_STDBY_TIMEOUT  CONFIG_HAPTICS_CS40L5X_PM_STDBY_TIMEOUT

/* Miscellaneous helpers */
#define CS40L5X_WRITE_SFT_RESET         0x5A000000U
#define CS40L5X_WRITE_DYNAMIC_F0_ENABLE 0x00000001U
#define CS40L5X_WRITE_F0_COMP_ENABLE    0x00000001U
#define CS40L5X_WRITE_REDC_COMP_ENABLE  0x00000002U
#define CS40L5X_WRITE_PAUSE_PLAYBACK    0x05000000U
#define CS40L5X_WRITE_PCM               0x00000008U
#define CS40L5X_WRITE_PWLE              0x0000000CU
#define CS40L5X_EXP_MBOX_CLEAR          0x00000000U
#define CS40L5X_EXP_DSP_STANDBY         0x00000002U
#define CS40L5X_EXP_MBOX_OVERFLOW       0x00000006U
#define CS40L5X_EXP_REDC_EST_START      0x07000012U
#define CS40L5X_EXP_REDC_EST_DONE       0x07000022U
#define CS40L5X_EXP_F0_EST_START        0x07000011U
#define CS40L5X_EXP_F0_EST_DONE         0x07000021U
#define CS40L5X_LOGGER_SOURCE_STEP      12
#define CS40L5X_LOGGER_TYPE_STEP        4
#define CS40L5X_MAX_GAIN                100
#define CS40L5X_MAX_ATTENUATION         0x007FFFFFU
#define CS40L5X_NUM_ROM_EFFECTS         27
#define CS40L5X_NUM_BUZ_EFFECTS         1
#define CS40L5X_BUZ_1MS_RES             0x000020C5U
#define CS40L5X_BUZ_INF_DURATION        0
#define CS40L5X_WSEQ_TERMINATOR         0x00FF0000U
#define CS40L5X_HEADER_1                1
#define CS40L5X_HEADER_2                2
#define CS40L5X_HEADER_ERROR            0xFFFFFFFFU
#define CS40L5X_MAX_PCM_SAMPLES         378
#define CS40L5X_MAX_PWLE_SECTIONS       63
#define CS40L5X_PWLE_DEFAULT_FREQ       0x0320U
#define CS40L5X_PWLE_DEFAULT_FLAGS      0x1
#define CS40L5X_PWLE_RESERVED_VALUE     0x003FFFFFU

enum cs40l5x_irq {
	CS40L5X_INT1,
	CS40L5X_INT2,
	CS40L5X_INT3,
	CS40L5X_INT4,
	CS40L5X_INT5,
	CS40L5X_INT6,
	CS40L5X_INT7,
	CS40L5X_INT8,
	CS40L5X_INT9,
	CS40L5X_INT10,
	CS40L5X_INT14,
	CS40L5X_INT18,
	CS40L5X_INT19,
	CS40L5X_INT20,
	CS40L5X_INT21,
	CS40L5X_INT22,
	CS40L5X_NUM_INT,
};

static const uint8_t cs40l5x_trigger_offsets[] = {
	[CS40L5X_GPIO3] = 0x00,  [CS40L5X_GPIO4] = 0x08,  [CS40L5X_GPIO5] = 0x10,
	[CS40L5X_GPIO6] = 0x18,  [CS40L5X_GPIO10] = 0x20, [CS40L5X_GPIO11] = 0x28,
	[CS40L5X_GPIO12] = 0x30, [CS40L5X_GPIO13] = 0x38,
};

static const struct cirrus_multi_write cs40l5x_b0_internal_boost[] = {
	{.addr = 0x00002018U, CIRRUS_MULTI_WRITE_BE32(0x00003321U, 0x04000010U)},
};

static const struct cirrus_multi_write cs40l5x_b0_external_boost[] = {
	{.addr = 0x00002018U, CIRRUS_MULTI_WRITE_BE32(0x00003201U)},
	{.addr = 0x00004404U, CIRRUS_MULTI_WRITE_BE32(0x01000000U)},
};

static const struct cirrus_multi_write cs40l5x_b0_errata[] = {
	{.addr = 0x00000040U, CIRRUS_MULTI_WRITE_BE32(0x00000055U)},
	{.addr = 0x00000040U, CIRRUS_MULTI_WRITE_BE32(0x000000AAU)},
	{.addr = 0x00003014U, CIRRUS_MULTI_WRITE_BE32(0x08012E16U)},
	{.addr = 0x00003808U, CIRRUS_MULTI_WRITE_BE32(0xC0000004U)},
	{.addr = 0x0000380CU, CIRRUS_MULTI_WRITE_BE32(0xC8710230U)},
	{.addr = 0x0000388CU, CIRRUS_MULTI_WRITE_BE32(0x04E0FFFFU)},
	{.addr = 0x0000649CU, CIRRUS_MULTI_WRITE_BE32(0x01818461U)},
	{.addr = 0x00000040U, CIRRUS_MULTI_WRITE_BE32(0x00000000U)},
	{.addr = 0x02BC21B8U,
	 CIRRUS_MULTI_WRITE_BE32(0x00000302U, 0x00000001U, 0x00018B41U, 0x00009920U)},
};

static const struct cirrus_multi_write cs40l5x_b0_errata_external_boost[] = {
	{.addr = 0x00000040U, CIRRUS_MULTI_WRITE_BE32(0x00000055U)},
	{.addr = 0x00000040U, CIRRUS_MULTI_WRITE_BE32(0x000000AAU)},
	{.addr = 0x00005C00U, CIRRUS_MULTI_WRITE_BE32(0x00000400U)},
	{.addr = 0x00004220U, CIRRUS_MULTI_WRITE_BE32(0x8000007DU)},
	{.addr = 0x00004200U, CIRRUS_MULTI_WRITE_BE32(0x00000008U)},
	{.addr = 0x00004240U, CIRRUS_MULTI_WRITE_BE32(0x510002B5U)},
	{.addr = 0x00006024U, CIRRUS_MULTI_WRITE_BE32(0x00522303U)},
	{.addr = 0x00000040U, CIRRUS_MULTI_WRITE_BE32(0x00000000U)},
	{.addr = 0x02804348U,
	 CIRRUS_MULTI_WRITE_BE32(0x00040020U, 0x00183201U, 0x00050044U, 0x00040100U, 0x00FD0001U,
				 0x0004005CU, 0x00000400U, 0x00000000U, 0x00422080U, 0x0000007DU,
				 0x00040042U, 0x00000008U, 0x00050042U, 0x00405100U, 0x00040060U,
				 0x00242303U, 0x00FFFFFFU)}};

static const struct cirrus_multi_write cs40l5x_irq_clear[] = {
	{.addr = CS40L5X_REG_IRQ1_INT_1,
	 CIRRUS_MULTI_WRITE_BE32(0xFFFFFFFFU, 0xFFFFFFFFU, 0xFFFFFFFFU, 0xFFFFFFFFU, 0xFFFFFFFFU,
				 0xFFFFFFFFU, 0xFFFFFFFFU, 0xFFFFFFFFU, 0xFFFFFFFFU, 0xFFFFFFFFU)},
	{.addr = CS40L5X_REG_IRQ1_INT_14, CIRRUS_MULTI_WRITE_BE32(0xFFFFFFFFU)},
	{.addr = CS40L5X_REG_IRQ1_INT_18,
	 CIRRUS_MULTI_WRITE_BE32(0xFFFFFFFFU, 0xFFFFFFFFU, 0xFFFFFFFFU, 0xFFFFFFFFU, 0xFFFFFFFFU)},
};

static const struct cirrus_multi_write cs40l5x_irq_masks[] = {
	{.addr = CS40L5X_REG_IRQ1_MASK_1, CIRRUS_MULTI_WRITE_BE32(0x03FFFFFFU, 0xFFDF7FFFU)},
	{.addr = CS40L5X_REG_IRQ1_MASK_4, CIRRUS_MULTI_WRITE_BE32(0xE0FFFFFFU)},
	{.addr = CS40L5X_REG_IRQ1_MASK_8,
	 CIRRUS_MULTI_WRITE_BE32(0x7C000FFFU, 0x0101C033U, 0x0000F00CU)},
	{.addr = CS40L5X_REG_IRQ1_MASK_20, CIRRUS_MULTI_WRITE_BE32(0x15FFF000U)},
};

static const uint32_t cs40l5x_pseq[] = {CIRRUS_WRITE_BE32(
	0x00000000U, 0x00E09003U, 0x00FFFFFFU, 0x000304FFU, 0x00DF7FFFU, 0x00000000U, 0x00E09CE0U,
	0x00FFFFFFU, 0x00000000U, 0x00E0AC7CU, 0x00000FFFU, 0x00030401U, 0x0001C033U, 0x00030400U,
	0x0000F00CU, 0x00000000U, 0x00E0DC15U, 0x00FFF000U, 0x00000000U, 0x00004000U, 0x00000055U,
	0x00030000U, 0x000000AAU, 0x00000000U, 0x00301408U, 0x00012E16U, 0x00000000U, 0x003808C0U,
	0x00000004U, 0x000304C8U, 0x00710230U, 0x00038004U, 0x00E0FFFFU, 0x00000000U, 0x00649C01U,
	0x00818461U, 0x00000000U, 0x00004000U, 0x00000000U, CS40L5X_WSEQ_TERMINATOR)};

static const uint32_t cs40l5x_pseq_internal[] = {CIRRUS_WRITE_BE32(
	0x00000000U, 0x00201800U, 0x00003321U, 0x00030404U, 0x00000010U, CS40L5X_WSEQ_TERMINATOR)};

static const uint32_t cs40l5x_pseq_external[] = {CIRRUS_WRITE_BE32(
	0x00000000U, 0x00201800U, 0x00003201U, 0x00000000U, 0x00440401U, 0x00000000U, 0x00000000U,
	0x00004000U, 0x00000055U, 0x00030000U, 0x000000AAU, 0x00000000U, 0x005C0000U, 0x00000400U,
	0x00000000U, 0x00420000U, 0x00000008U, 0x00032080U, 0x0000007DU, 0x00032051U, 0x000002B5U,
	0x00000000U, 0x00602400U, 0x00522303U, 0x00000000U, 0x00004000U, 0x00000000U,
	CS40L5X_WSEQ_TERMINATOR)};

static inline bool cs40l5x_valid_wavetable_source(const struct device *const dev,
						  const enum cs40l5x_bank bank, const uint8_t index)
{
	struct cs40l5x_data *const data = dev->data;

	switch (bank) {
	case CS40L5X_ROM_BANK:
		return index < CS40L5X_NUM_ROM_EFFECTS;
	case CS40L5X_CUSTOM_BANK:
		return (index < CS40L5X_NUM_CUSTOM_EFFECTS &&
			IS_ENABLED(CONFIG_HAPTICS_CS40L5X_CUSTOM_EFFECTS))
			       ? data->custom_effects[index]
			       : false;
	case CS40L5X_BUZ_BANK:
		return index < CS40L5X_NUM_BUZ_EFFECTS;
	default:
		return false;
	}
}

static int cs40l5x_write_mailbox(const struct device *const dev, const uint32_t mailbox_command)
{
	const k_timepoint_t end = sys_timepoint_calc(CS40L5X_T_WAKESOURCE);
	__maybe_unused const struct cirrus_config *const config = dev->config;
	int ret;

	do {
		ret = cirrus_write(dev, CS40L5X_REG_DSP_VIRTUAL1_MBOX_1, mailbox_command);
		if (ret >= 0) {
			return cirrus_poll(dev, CS40L5X_REG_DSP_VIRTUAL1_MBOX_1,
					   CS40L5X_EXP_MBOX_CLEAR, CS40L5X_T_MBOX_CLEAR);
		}

		(void)k_sleep(CIRRUS_T_POLLING);
	} while (!sys_timepoint_expired(end));

	LOG_INST_DBG(config->log, "timed out writing to mailbox (%d)", ret);

	return -ETIMEDOUT;
}

static int cs40l5x_increment_mailbox(const struct device *const dev, uint32_t *const mbox_ptr)
{
	if (*mbox_ptr < CS40L5X_REG_DSP_MBOX_8) {
		*mbox_ptr += CIRRUS_REGISTER_WIDTH;
	} else {
		*mbox_ptr = CS40L5X_REG_DSP_MBOX_2;
	}

	return cirrus_firmware_write(dev, CS40L5X_REG_MAILBOX_QUEUE_RD, *mbox_ptr);
}

static int cs40l5x_poll_mailbox(const struct device *const dev, const uint32_t mailbox_command,
				const k_timeout_t timeout)
{
	uint32_t mbox_rd_ptr;
	int ret;

	ret = cirrus_firmware_read(dev, CS40L5X_REG_MAILBOX_QUEUE_RD, &mbox_rd_ptr);
	if (ret < 0) {
		return ret;
	}

	ret = cirrus_poll(dev, mbox_rd_ptr, mailbox_command, timeout);
	if (ret < 0) {
		return ret;
	}

	return cs40l5x_increment_mailbox(dev, &mbox_rd_ptr);
}

static int cs40l5x_reset_mailbox(const struct device *const dev)
{
	uint32_t mbox_ptr;
	int ret;

	ret = cirrus_firmware_read(dev, CS40L5X_REG_MAILBOX_QUEUE_WT, &mbox_ptr);
	if (ret < 0) {
		return ret;
	}

	return cirrus_firmware_write(dev, CS40L5X_REG_MAILBOX_QUEUE_RD, mbox_ptr);
}

static int cs40l5x_process_mailbox(const struct device *const dev)
{
	__maybe_unused const struct cirrus_config *const config = dev->config;
	uint32_t mbox_rd_ptr, mbox_status, mbox_val, mbox_wr_ptr;
	struct cs40l5x_data *const data = dev->data;
	int ret;

	ret = cirrus_firmware_read(dev, CS40L5X_REG_MAILBOX_STATUS, &mbox_status);
	if (ret < 0) {
		return ret;
	}

	ret = cirrus_firmware_read(dev, CS40L5X_REG_MAILBOX_QUEUE_RD, &mbox_rd_ptr);
	if (ret < 0) {
		return ret;
	}

	ret = cirrus_firmware_read(dev, CS40L5X_REG_MAILBOX_QUEUE_WT, &mbox_wr_ptr);
	if (ret < 0) {
		return ret;
	}

	if (mbox_status == CS40L5X_EXP_MBOX_OVERFLOW) {
		LOG_INST_DBG(config->log, "mailbox overflow");
	}

	do {
		ret = cirrus_read(dev, mbox_rd_ptr, &mbox_val);
		if (ret < 0) {
			return ret;
		}

		switch (mbox_val) {
		case CS40L5X_MBOX_PLAYBACK_COMPLETE_MBOX:
			LOG_INST_DBG(config->log, "mailbox playback complete");
			break;
		case CS40L5X_MBOX_PLAYBACK_COMPLETE_GPIO:
			LOG_INST_DBG(config->log, "trigger playback complete");
			break;
		case CS40L5X_MBOX_PLAYBACK_START_MBOX:
			LOG_INST_DBG(config->log, "mailbox playback started");
			break;
		case CS40L5X_MBOX_PLAYBACK_START_GPIO:
			LOG_INST_DBG(config->log, "trigger playback started");
			break;
		case CS40L5X_MBOX_INIT:
			LOG_INST_DBG(config->log, "awake after reset");
			break;
		case CS40L5X_MBOX_AWAKE:
			LOG_INST_DBG(config->log, "awake");
			break;
		case CS40L5X_MBOX_REDC_EST_START:
			LOG_INST_DBG(config->log, "ReDC calibration started");
			break;
		case CS40L5X_MBOX_REDC_EST_DONE:
			LOG_INST_DBG(config->log, "ReDC calibration complete");
			k_sem_give(&data->calibration_semaphore);
			break;
		case CS40L5X_MBOX_F0_EST_START:
			LOG_INST_DBG(config->log, "F0 calibration started");
			break;
		case CS40L5X_MBOX_F0_EST_DONE:
			LOG_INST_DBG(config->log, "F0 calibration complete");
			k_sem_give(&data->calibration_semaphore);
			break;
		case CS40L5X_MBOX_PERMANENT_SHORT_DETECTED:
			__fallthrough;
		case CS40L5X_MBOX_RUNTIME_SHORT_DETECTED:
			cirrus_error_callback(dev, HAPTICS_ERROR_OVERCURRENT);

			return 0;
		default:
			LOG_INST_DBG(config->log, "unexpected mailbox code: %08x", mbox_val);
			break;
		}

		ret = cs40l5x_increment_mailbox(dev, &mbox_rd_ptr);
		if (ret < 0) {
			return ret;
		}
	} while (mbox_rd_ptr != mbox_wr_ptr);

	return cirrus_write(dev, CS40L5X_REG_IRQ1_INT_2, CS40L5X_MASK_DSP_VIRTUAL2_MBOX_WR_INT2);
}

static int cs40l5x_process_interrupts(const struct device *const dev,
				      const uint32_t *const irq_ints)
{
	__maybe_unused const struct cirrus_config *const config = dev->config;
	uint32_t error_bitmask = 0;
	int ret;

	if (FIELD_GET(CS40L5X_MASK_AMP_SHORT_ERR_INT1, irq_ints[CS40L5X_INT1]) != 0) {
		LOG_INST_WRN(config->log, "amplifier short detected");

		error_bitmask |= HAPTICS_ERROR_OVERCURRENT;

		ret = cirrus_write(dev, CS40L5X_REG_IRQ1_INT_1, CS40L5X_MASK_AMP_SHORT_ERR_INT1);
		if (ret < 0) {
			return ret;
		}
	}

	if (FIELD_GET(CS40L5X_MASK_TEMP_ERR_INT8, irq_ints[CS40L5X_INT8]) != 0) {
		LOG_INST_WRN(config->log, "overtemperature detected");

		error_bitmask |= HAPTICS_ERROR_OVERTEMPERATURE;

		ret = cirrus_write(dev, CS40L5X_REG_IRQ1_INT_8, CS40L5X_MASK_TEMP_ERR_INT8);
		if (ret < 0) {
			return ret;
		}
	}

	if (FIELD_GET(CS40L5X_MASK_BST_UVP_ERR_INT9, irq_ints[CS40L5X_INT9]) != 0) {
		LOG_INST_WRN(config->log, "undervoltage detected");

		error_bitmask |= HAPTICS_ERROR_UNDERVOLTAGE;

		ret = cirrus_write(dev, CS40L5X_REG_IRQ1_INT_9, CS40L5X_MASK_BST_UVP_ERR_INT9);
		if (ret < 0) {
			return ret;
		}
	}

	if (FIELD_GET(CS40L5X_MASK_BST_SHORT_ERR_INT9, irq_ints[CS40L5X_INT9]) != 0) {
		LOG_INST_WRN(config->log, "inductor short detected");

		error_bitmask |= HAPTICS_ERROR_OVERCURRENT;

		ret = cirrus_write(dev, CS40L5X_REG_IRQ1_INT_9, CS40L5X_MASK_BST_SHORT_ERR_INT9);
		if (ret < 0) {
			return ret;
		}
	}

	if (FIELD_GET(CS40L5X_MASK_BST_ILIMIT_ERR_INT9, irq_ints[CS40L5X_INT9]) != 0) {
		LOG_INST_WRN(config->log, "current limited");

		ret = cirrus_write(dev, CS40L5X_REG_IRQ1_INT_9, CS40L5X_MASK_BST_ILIMIT_ERR_INT9);
		if (ret < 0) {
			return ret;
		}
	}

	if (FIELD_GET(CS40L5X_MASK_UVLO_VDDBATT_ERR_INT10, irq_ints[CS40L5X_INT10]) != 0) {
		LOG_INST_WRN(config->log, "battery undervoltage detected");

		error_bitmask |= HAPTICS_ERROR_UNDERVOLTAGE;

		ret = cirrus_write(dev, CS40L5X_REG_IRQ1_INT_10,
				   CS40L5X_MASK_UVLO_VDDBATT_ERR_INT10);
		if (ret < 0) {
			return ret;
		}
	}

	if (error_bitmask != 0) {
		cirrus_error_callback(dev, error_bitmask);
	}

	return 0;
}

static int cs40l5x_retrieve_interrupt_statuses(const struct device *const dev,
					       uint32_t *const irq_ints)
{
	uint32_t irq_masks[CS40L5X_NUM_INT];
	int ret;

	ret = cirrus_burst_read(dev, CS40L5X_REG_IRQ1_INT_1, irq_ints, 10);
	if (ret < 0) {
		return ret;
	}

	ret = cirrus_read(dev, CS40L5X_REG_IRQ1_INT_14, &irq_ints[CS40L5X_INT14]);
	if (ret < 0) {
		return ret;
	}

	ret = cirrus_burst_read(dev, CS40L5X_REG_IRQ1_INT_18, &irq_ints[CS40L5X_INT18], 5);
	if (ret < 0) {
		return ret;
	}

	ret = cirrus_burst_read(dev, CS40L5X_REG_IRQ1_MASK_1, irq_masks, 10);
	if (ret < 0) {
		return ret;
	}

	ret = cirrus_read(dev, CS40L5X_REG_IRQ1_MASK_14, &irq_masks[CS40L5X_INT14]);
	if (ret < 0) {
		return ret;
	}

	ret = cirrus_burst_read(dev, CS40L5X_REG_IRQ1_MASK_18, &irq_masks[CS40L5X_INT18], 5);
	if (ret < 0) {
		return ret;
	}

	for (int i = 0; i < CS40L5X_NUM_INT; i++) {
		irq_ints[i] &= ~irq_masks[i];
	}

	return 0;
}

static void cs40l5x_interrupt_worker(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct cirrus_data *data = CONTAINER_OF(dwork, struct cirrus_data, interrupt_worker);
	const struct device *const dev = data->dev;
	const struct cirrus_config *const config = dev->config;
	uint32_t irq1_status, irq_ints[CS40L5X_NUM_INT];
	int ret;

	if (gpio_pin_get_dt(&config->interrupt_gpio) == CIRRUS_GPIO_INACTIVE) {
		return;
	}

	ret = pm_device_runtime_get(dev);
	if (ret < 0) {
		return;
	}

	ret = cirrus_read(dev, CS40L5X_REG_IRQ1_STATUS, &irq1_status);
	if (ret < 0) {
		goto exit_pm;
	}

	if (irq1_status == 0) {
		LOG_INST_DBG(config->log, "IRQ status unset in interrupt worker");
		goto exit_pm;
	}

	ret = cs40l5x_retrieve_interrupt_statuses(dev, irq_ints);
	if (ret < 0) {
		LOG_INST_DBG(config->log, "failed to read IRQ registers (%d)", ret);
		goto exit_pm;
	}

	ret = cs40l5x_process_interrupts(dev, irq_ints);
	if (ret < 0) {
		LOG_INST_DBG(config->log, "failed to process non-mailbox interrupts (%d)", ret);
		goto exit_pm;
	}

	if (FIELD_GET(CS40L5X_MASK_DSP_VIRTUAL2_MBOX_WR_INT2, irq_ints[CS40L5X_INT2]) != 0) {
		ret = cs40l5x_process_mailbox(dev);
		if (ret < 0) {
			LOG_INST_DBG(config->log, "failed to process mailbox (%d)", ret);
			goto exit_pm;
		}
	}

	ret = cirrus_read(dev, CS40L5X_REG_IRQ1_STATUS, &irq1_status);
	if (ret < 0) {
		goto exit_pm;
	}

	if (irq1_status != 0) {
		LOG_INST_DBG(config->log, "IRQ still set in interrupt worker");

		ret = k_work_submit(work);
		if (ret < 0) {
			LOG_INST_DBG(config->log, "failed to resubmit worker (%d)", ret);
		}
	}

exit_pm:
	(void)pm_device_runtime_put(dev);
}

static int cs40l5x_click_compensation(const struct device *const dev)
{
	__maybe_unused const struct cirrus_config *const config = dev->config;
	const struct cirrus_data *const data = dev->data;
	int ret;

	if (!IS_ENABLED(CONFIG_HAPTICS_CIRRUS_CLICK_COMPENSATION)) {
		return 0;
	}

	if (data->calibration.f0 == 0 && data->calibration.redc == 0) {
		LOG_INST_WRN(config->log, "no calibration data provided");
		return 0;
	}

	ret = cirrus_firmware_write(dev, CS40L5X_REG_VIBEGEN_F0_OTP_STORED, data->calibration.f0);
	if (ret < 0) {
		return ret;
	}

	ret = cirrus_firmware_write(dev, CS40L5X_REG_VIBEGEN_REDC_OTP_STORED,
				    data->calibration.redc);
	if (ret < 0) {
		return ret;
	}

	return cirrus_firmware_write(dev, CS40L5X_REG_VIBEGEN_COMPENSATION_ENABLE,
				     CS40L5X_WRITE_F0_COMP_ENABLE | CS40L5X_WRITE_REDC_COMP_ENABLE);
}

static int cs40l5x_dynamic_f0(const struct device *const dev)
{
	if (!IS_ENABLED(CONFIG_HAPTICS_CIRRUS_DYNAMIC_F0)) {
		return 0;
	}

	return cirrus_firmware_write(dev, CS40L5X_REG_DYNAMIC_F0, CS40L5X_WRITE_DYNAMIC_F0_ENABLE);
}

static int cs40l5x_dsp_config(const struct device *const dev)
{
	int ret;

	ret = cirrus_load_calibration(dev);
	if (ret < 0) {
		return ret;
	}

	ret = cs40l5x_click_compensation(dev);
	if (ret < 0) {
		return ret;
	}

	return cs40l5x_dynamic_f0(dev);
}

static int cs40l5x_pseq_config(const struct device *const dev)
{
	const off_t offset = (ARRAY_SIZE(cs40l5x_pseq) - 1) * CIRRUS_REGISTER_WIDTH;
	const struct cs40l5x_config *const config = dev->config;
	int ret;

	ret = cirrus_firmware_raw_burst_write(dev, CS40L5X_REG_WSEQ_POWER, cs40l5x_pseq,
					      ARRAY_SIZE(cs40l5x_pseq));
	if (ret < 0) {
		return ret;
	}

	return (HAPTICS_CS40L5X_EXTERNAL_BOOST(config))
		       ? cirrus_firmware_raw_burst_write_offset(
				 dev, CS40L5X_REG_WSEQ_POWER, cs40l5x_pseq_external,
				 ARRAY_SIZE(cs40l5x_pseq_external), offset)
		       : cirrus_firmware_raw_burst_write_offset(
				 dev, CS40L5X_REG_WSEQ_POWER, cs40l5x_pseq_internal,
				 ARRAY_SIZE(cs40l5x_pseq_internal), offset);
}

static int cs40l5x_timeout_config(const struct device *const dev)
{
	int ret;

	ret = cirrus_firmware_write(dev, CS40L5X_REG_ACTIVE_TIMEOUT, CS40L5X_PM_ACTIVE_TIMEOUT);
	if (ret < 0) {
		return ret;
	}

	return cirrus_firmware_write(dev, CS40L5X_REG_STDBY_TIMEOUT, CS40L5X_PM_STDBY_TIMEOUT);
}

static int cs40l5x_write_errata(const struct device *const dev)
{
	const struct cs40l5x_config *const config = dev->config;
	int ret;

	ret = cirrus_raw_multi_write(dev, cs40l5x_b0_errata, ARRAY_SIZE(cs40l5x_b0_errata));
	if (ret < 0) {
		return ret;
	}

	if (HAPTICS_CS40L5X_EXTERNAL_BOOST(config)) {
		return cirrus_raw_multi_write(dev, cs40l5x_b0_errata_external_boost,
					      ARRAY_SIZE(cs40l5x_b0_errata_external_boost));
	}

	return 0;
}

static int cs40l5x_boost_configuration(const struct device *const dev)
{
	const struct cs40l5x_config *const config = dev->config;
	struct cirrus_data *const data = dev->data;

	if (data->revision_id != CS40L5X_REVID_B0) {
		return -ENOTSUP;
	}

	return (HAPTICS_CS40L5X_EXTERNAL_BOOST(config))
		       ? cirrus_raw_multi_write(dev, cs40l5x_b0_external_boost,
						ARRAY_SIZE(cs40l5x_b0_external_boost))
		       : cirrus_raw_multi_write(dev, cs40l5x_b0_internal_boost,
						ARRAY_SIZE(cs40l5x_b0_internal_boost));
}

static int cs40l5x_fingerprint(const struct device *const dev)
{
	const struct cirrus_config *const config = dev->config;
	struct cirrus_data *const data = dev->data;
	uint32_t otpid, rx[2];
	int ret;

	ret = cirrus_burst_read(dev, CS40L5X_REG_DEVID, rx, ARRAY_SIZE(rx));
	if (ret < 0) {
		return ret;
	}

	ret = cirrus_read(dev, CS40L5X_REG_OTPID, &otpid);
	if (ret < 0) {
		return ret;
	}

	if (rx[0] != config->device_id) {
		LOG_INST_ERR(config->log, "mismatched device ID: 0x%05X", rx[0]);
		return -ENOTSUP;
	}

	if (rx[1] != CS40L5X_REVID_B0) {
		LOG_INST_ERR(config->log, "unsupported revision: 0x%02X", rx[1]);
		return -ENOTSUP;
	}

	data->revision_id = (uint8_t)rx[1];

	if (!IN_RANGE(otpid, CS40L5X_OTPID_MIN, CS40L5X_OTPID_MAX)) {
		LOG_INST_ERR(config->log, "unsupported OTP: 0x%01X", otpid);
		return -ENOTSUP;
	}

	data->otp_id = (uint8_t)otpid;

	LOG_INST_INF(config->log, "Cirrus Logic CS40L%02X Revision %X (OTP %01X)",
		     (uint8_t)config->device_id, data->revision_id, data->otp_id);

	return 0;
}

static int cs40l5x_reset(const struct device *const dev)
{
	const struct cirrus_config *const config = dev->config;
	uint32_t halo_state;
	int ret;

	if (HAPTICS_CIRRUS_RESET(config)) {
		ret = gpio_pin_set_dt(&config->reset_gpio, CIRRUS_GPIO_ACTIVE);
		if (ret < 0) {
			return ret;
		}

		(void)k_sleep(CS40L5X_T_RLPW);

		ret = gpio_pin_set_dt(&config->reset_gpio, CIRRUS_GPIO_INACTIVE);
	} else {
		ret = cirrus_wakeup(dev, CS40L5X_T_WAKESOURCE);
		if (ret < 0) {
			return ret;
		}

		ret = cs40l5x_write_mailbox(dev, CS40L5X_MBOX_PREVENT_HIBERNATION);
		if (ret < 0) {
			return ret;
		}

		ret = cirrus_write(dev, CS40L5X_REG_SFT_RESET, CS40L5X_WRITE_SFT_RESET);
	}
	if (ret < 0) {
		return ret;
	}

	(void)k_sleep(CS40L5X_T_IRS);

	ret = cs40l5x_fingerprint(dev);
	if (ret < 0) {
		return ret;
	}

	ret = cirrus_firmware_read(dev, CS40L5X_REG_HALO_STATE, &halo_state);
	if (ret < 0) {
		return ret;
	}

	if (halo_state != CS40L5X_EXP_DSP_STANDBY) {
		LOG_INST_DBG(config->log, "expected DSP in standby after hardware reset (%d)", ret);
		return -ENODEV;
	}

	return cs40l5x_reset_mailbox(dev);
}

static int cs40l5x_bringup(const struct device *const dev)
{
	const struct cirrus_config *const config = dev->config;
	int ret;

	ret = cs40l5x_reset(dev);
	if (ret < 0) {
		LOG_INST_DBG(config->log, "failed reset (%d)", ret);
		return ret;
	}

	if (HAPTICS_CIRRUS_INTERRUPT(config)) {
		ret = cirrus_raw_multi_write(dev, cs40l5x_irq_masks, ARRAY_SIZE(cs40l5x_irq_masks));
		if (ret < 0) {
			return ret;
		}

		ret = cirrus_raw_multi_write(dev, cs40l5x_irq_clear, ARRAY_SIZE(cs40l5x_irq_clear));
		if (ret < 0) {
			return ret;
		}
	}

	ret = cirrus_interrupt_config(dev);
	if (ret < 0) {
		LOG_INST_DBG(config->log, "failed IRQ configuration (%d)", ret);
		return ret;
	}

	ret = cs40l5x_boost_configuration(dev);
	if (ret < 0) {
		LOG_INST_DBG(config->log, "failed boost configuration (%d)", ret);
		return ret;
	}

	ret = cs40l5x_write_errata(dev);
	if (ret < 0) {
		LOG_INST_DBG(config->log, "failed errata update (%d)", ret);
		return ret;
	}

	ret = cs40l5x_dsp_config(dev);
	if (ret < 0) {
		LOG_INST_DBG(config->log, "failed DSP configuration (%d)", ret);
		return ret;
	}

	ret = cs40l5x_timeout_config(dev);
	if (ret < 0) {
		LOG_INST_DBG(config->log, "failed to update timeouts (%d)", ret);
		return ret;
	}

	if (HAPTICS_CIRRUS_HIBERNATION) {
		ret = cs40l5x_pseq_config(dev);
		if (ret < 0) {
			LOG_INST_DBG(config->log, "failed write sequencer update (%d)", ret);
			return ret;
		}
	}

	ret = cirrus_trigger_config(dev);
	if (ret < 0) {
		LOG_INST_DBG(config->log, "failed trigger configuration (%d)", ret);
		return ret;
	}

	return cirrus_firmware_write(dev, CS40L5X_REG_BUZZ_RES, CS40L5X_BUZ_1MS_RES);
}

#if CONFIG_PM_DEVICE
static int cs40l5x_teardown(const struct device *const dev)
{
	__maybe_unused const struct cirrus_config *const config = dev->config;
	int ret;

	ret = cirrus_interrupt_teardown(dev);
	if (ret < 0) {
		LOG_INST_DBG(config->log, "failed to disable IRQ (%d)", ret);
	}

	ret = cirrus_reset_teardown(dev);
	if (ret < 0) {
		LOG_INST_DBG(config->log, "failed to teardown reset GPIO (%d)", ret);
	}

	return 0;
}
#endif /* CONFIG_PM_DEVICE */

static int cs40l5x_calibrate_redc(const struct device *const dev)
{
	const struct cirrus_config *const config = dev->config;
	struct cs40l5x_data *const data = dev->data;
	int ret;

	ret = cs40l5x_write_mailbox(dev, CS40L5X_MBOX_START_REDC_EST);
	if (ret < 0) {
		return ret;
	}

	if (HAPTICS_CIRRUS_INTERRUPT(config)) {
		ret = k_sem_take(&data->calibration_semaphore, CS40L5X_T_REDC_CALIBRATION);
	} else {
		ret = cs40l5x_poll_mailbox(dev, CS40L5X_MBOX_REDC_EST_START,
					   CS40L5X_T_CALIBRATION_START);
		if (ret < 0) {
			LOG_INST_DBG(config->log, "timed out waiting for ReDC start (%d)", ret);
			return ret;
		}

		ret = cs40l5x_poll_mailbox(dev, CS40L5X_MBOX_REDC_EST_DONE,
					   CS40L5X_T_REDC_EST_DONE);
	}
	if (ret < 0) {
		LOG_INST_DBG(config->log, "timed out waiting for ReDC completion (%d)", ret);
		return ret;
	}

	return cirrus_firmware_read(dev, CS40L5X_REG_CALIB_REDC_EST,
				    &data->common.calibration.redc);
}

static int cs40l5x_calibrate_f0(const struct device *const dev)
{
	const struct cirrus_config *const config = dev->config;
	struct cs40l5x_data *const data = dev->data;
	int ret;

	ret = cs40l5x_write_mailbox(dev, CS40L5X_MBOX_START_F0_EST);
	if (ret < 0) {
		return ret;
	}

	if (HAPTICS_CIRRUS_INTERRUPT(config)) {
		ret = k_sem_take(&data->calibration_semaphore, CS40L5X_T_F0_CALIBRATION);
	} else {
		ret = cs40l5x_poll_mailbox(dev, CS40L5X_MBOX_F0_EST_START,
					   CS40L5X_T_CALIBRATION_START);
		if (ret < 0) {
			LOG_INST_DBG(config->log, "timed out waiting for F0 start (%d)", ret);
			return ret;
		}

		ret = cs40l5x_poll_mailbox(dev, CS40L5X_MBOX_F0_EST_DONE, CS40L5X_T_F0_EST_DONE);
	}
	if (ret < 0) {
		LOG_INST_DBG(config->log, "timed out waiting for F0 completion (%d)", ret);
		return ret;
	}

	return cirrus_firmware_read(dev, CS40L5X_REG_F0_EST, &data->common.calibration.f0);
}

static int cs40l5x_run_calibration(const struct device *const dev)
{
	struct cirrus_data *const data = dev->data;
	int ret;

	ret = cs40l5x_calibrate_redc(dev);
	if (ret < 0) {
		return ret;
	}

	ret = cirrus_firmware_write(dev, CS40L5X_REG_REDC, data->calibration.redc);
	if (ret < 0) {
		return ret;
	}

	return cs40l5x_calibrate_f0(dev);
}

int cs40l5x_calibrate(const struct device *const dev)
{
	const struct cirrus_config *const config = dev->config;
	struct cirrus_data *const data = dev->data;
	int ret;

	if (!IS_ENABLED(CONFIG_HAPTICS_CIRRUS_CALIBRATION)) {
		LOG_INST_DBG(config->log, "calibration is disabled");
		return -EPERM;
	}

	ret = pm_device_runtime_get(dev);
	if (ret < 0) {
		return ret;
	}

	ret = k_mutex_lock(&data->lock, CIRRUS_T_TIMEOUT);
	if (ret < 0) {
		LOG_INST_DBG(config->log, "timed out waiting for lock (%d)", ret);
		goto error_pm;
	}

	if (!HAPTICS_CIRRUS_INTERRUPT(config)) {
		ret = cs40l5x_reset_mailbox(dev);
		if (ret < 0) {
			goto error_mutex;
		}
	}

	ret = cs40l5x_run_calibration(dev);
	if (ret < 0) {
		goto error_mutex;
	}

	ret = cs40l5x_click_compensation(dev);
	if (ret < 0) {
		LOG_INST_DBG(config->log, "failed to update click compensation (%d)", ret);
		goto error_mutex;
	}

	ret = cirrus_store_calibration(dev);
	if (ret < 0) {
		LOG_INST_DBG(config->log, "failed to store calibration (%d)", ret);
	}

error_mutex:
	(void)k_mutex_unlock(&data->lock);

error_pm:
	(void)pm_device_runtime_put(dev);

	return ret;
}

int cs40l5x_configure_buzz(const struct device *const dev, const uint32_t frequency,
			   const uint8_t level, const uint32_t duration)
{
	__maybe_unused const struct cirrus_config *const config = dev->config;
	struct cirrus_data *const data = dev->data;
	int ret;

	ret = pm_device_runtime_get(dev);
	if (ret < 0) {
		return ret;
	}

	ret = k_mutex_lock(&data->lock, CIRRUS_T_TIMEOUT);
	if (ret < 0) {
		LOG_INST_DBG(config->log, "timed out waiting for lock (%d)", ret);
		goto error_pm;
	}

	ret = cirrus_firmware_write(dev, CS40L5X_REG_BUZZ_FREQ, frequency);
	if (ret < 0) {
		goto error_mutex;
	}

	ret = cirrus_firmware_write(dev, CS40L5X_REG_BUZZ_LEVEL, level);
	if (ret < 0) {
		goto error_mutex;
	}

	ret = cirrus_firmware_write(dev, CS40L5X_REG_BUZZ_DURATION, duration);

error_mutex:
	(void)k_mutex_unlock(&data->lock);

error_pm:
	(void)pm_device_runtime_put(dev);

	return ret;
}

int cs40l5x_configure_trigger(const struct device *const dev, const struct gpio_dt_spec *const gpio,
			      const enum cs40l5x_bank bank, const uint8_t index,
			      const enum cs40l5x_attenuation attenuation,
			      const enum cs40l5x_trigger_edge edge)
{
	const struct cirrus_config *const config = dev->config;
	struct cirrus_data *const data = dev->data;
	uint8_t idx, offset;
	uint32_t playback;
	int ret;

	if (!HAPTICS_CIRRUS_TRIGGER(config)) {
		LOG_INST_DBG(config->log, "no trigger GPIOs provided");
		return -EPERM;
	}

	if (!cs40l5x_valid_wavetable_source(dev, bank, index)) {
		LOG_INST_ERR(config->log, "invalid wavetable selection (%d)", -EINVAL);
		return -EINVAL;
	}

	ret = cirrus_get_trigger(dev, gpio, &idx);
	if (ret < 0) {
		LOG_INST_DBG(config->log, "failed to retrieve trigger GPIO (%d)", ret);
		return ret;
	}

	offset = cs40l5x_trigger_offsets[idx] + ((edge == CS40L5X_FALLING_EDGE) ? 4 : 0);

	playback = FIELD_PREP(CS40L5X_MASK_ATTENUATION, abs(attenuation)) | index;

	switch (bank) {
	case CS40L5X_ROM_BANK:
		break;
	case CS40L5X_BUZ_BANK:
		playback |= CS40L5X_MASK_BUZ_BANK;
		break;
	case CS40L5X_CUSTOM_BANK:
		playback |= CS40L5X_MASK_CUSTOM_BANK;
		break;
	default:
		return -EINVAL;
	}

	ret = pm_device_runtime_get(dev);
	if (ret < 0) {
		return ret;
	}

	ret = k_mutex_lock(&data->lock, CIRRUS_T_TIMEOUT);
	if (ret < 0) {
		LOG_INST_DBG(config->log, "timed out waiting for lock (%d)", ret);
		goto error_pm;
	}

	ret = cirrus_firmware_write_offset(dev, CS40L5X_REG_GPIO_EVENT_BASE, playback, offset);

	(void)k_mutex_unlock(&data->lock);

error_pm:
	(void)pm_device_runtime_put(dev);

	return ret;
}

int cs40l5x_logger(const struct device *const dev, enum cs40l5x_logger logger_state)
{
	__maybe_unused const struct cirrus_config *const config = dev->config;
	struct cirrus_data *const data = dev->data;
	int ret;

	if (!IS_ENABLED(CONFIG_HAPTICS_CS40L5X_DSP_LOGGER)) {
		LOG_INST_DBG(config->log, "haptics logging is disabled");
		return -EPERM;
	}

	ret = pm_device_runtime_get(dev);
	if (ret < 0) {
		return ret;
	}

	ret = k_mutex_lock(&data->lock, CIRRUS_T_TIMEOUT);
	if (ret < 0) {
		LOG_INST_DBG(config->log, "timed out waiting for lock (%d)", ret);
		goto error_pm;
	}

	ret = cirrus_firmware_write(dev, CS40L5X_REG_LOGGER_ENABLE, (uint32_t)logger_state);

	(void)k_mutex_unlock(&data->lock);

error_pm:
	(void)pm_device_runtime_put(dev);

	return ret;
}

int cs40l5x_logger_get(const struct device *const dev, enum cs40l5x_logger_source source,
		       enum cs40l5x_logger_source_type type, uint32_t *const value)
{
	__maybe_unused const struct cirrus_config *const config = dev->config;
	struct cirrus_data *const data = dev->data;
	int offset, ret;

	if (!IS_ENABLED(CONFIG_HAPTICS_CS40L5X_DSP_LOGGER)) {
		LOG_INST_DBG(config->log, "haptics logging is disabled");
		return -EPERM;
	}

	offset = (source * CS40L5X_LOGGER_SOURCE_STEP) + (type * CS40L5X_LOGGER_TYPE_STEP);

	ret = pm_device_runtime_get(dev);
	if (ret < 0) {
		return ret;
	}

	ret = k_mutex_lock(&data->lock, CIRRUS_T_TIMEOUT);
	if (ret < 0) {
		LOG_INST_DBG(config->log, "timed out waiting for lock (%d)", ret);
		goto error_pm;
	}

	ret = cirrus_firmware_read_offset(dev, CS40L5X_REG_LOGGER_DATA, value, offset);

	(void)k_mutex_unlock(&data->lock);

error_pm:
	(void)pm_device_runtime_put(dev);

	return ret;
}

int cs40l5x_select_output(const struct device *const dev, const enum cs40l5x_bank bank,
			  const uint8_t index)
{
	__maybe_unused const struct cirrus_config *const config = dev->config;
	struct cs40l5x_data *const data = dev->data;
	uint32_t output;
	int ret;

	if (!cs40l5x_valid_wavetable_source(dev, bank, index)) {
		LOG_INST_ERR(config->log, "invalid wavetable selection (%d)", -EINVAL);
		return -EINVAL;
	}

	output = index;

	switch (bank) {
	case CS40L5X_ROM_BANK:
		output |= CS40L5X_ROM_BANK_CMD;
		break;
	case CS40L5X_CUSTOM_BANK:
		output |= CS40L5X_CUSTOM_BANK_CMD;
		break;
	case CS40L5X_BUZ_BANK:
		output |= CS40L5X_BUZ_BANK_CMD;
		break;
	default:
		return -EINVAL;
	}

	ret = k_mutex_lock(&data->common.lock, CIRRUS_T_TIMEOUT);
	if (ret < 0) {
		LOG_INST_DBG(config->log, "timed out waiting for lock (%d)", ret);
		return ret;
	}

	data->output = output;

	(void)k_mutex_unlock(&data->common.lock);

	return ret;
}

int cs40l5x_set_gain(const struct device *const dev, const uint8_t gain)
{
	__maybe_unused const struct cirrus_config *const config = dev->config;
	struct cirrus_data *const data = dev->data;
	uint32_t attenuation;
	int ret;

	if (gain > CS40L5X_MAX_GAIN) {
		LOG_INST_ERR(config->log, "invalid gain, %u >= %u", gain, CS40L5X_MAX_GAIN);
		return -EINVAL;
	}

	ret = pm_device_runtime_get(dev);
	if (ret < 0) {
		return ret;
	}

	ret = k_mutex_lock(&data->lock, CIRRUS_T_TIMEOUT);
	if (ret < 0) {
		LOG_INST_DBG(config->log, "timed out waiting for lock (%d)", ret);
		goto error_pm;
	}

	if (gain == 0) {
		attenuation = CS40L5X_MAX_ATTENUATION;
	} else {
		attenuation = (uint32_t)cirrus_source_attenuation[gain];
	}

	ret = cirrus_firmware_write(dev, CS40L5X_REG_SOURCE_ATTENUATION, attenuation);

	(void)k_mutex_unlock(&data->lock);

error_pm:
	(void)pm_device_runtime_put(dev);

	return ret;
}

static int cs40l5x_start_output(const struct device *const dev)
{
	__maybe_unused const struct cirrus_config *const config = dev->config;
	struct cs40l5x_data *const data = dev->data;
	int ret;

	ret = pm_device_runtime_get(dev);
	if (ret < 0) {
		return ret;
	}

	ret = k_mutex_lock(&data->common.lock, CIRRUS_T_TIMEOUT);
	if (ret < 0) {
		LOG_INST_DBG(config->log, "timed out waiting for lock (%d)", ret);
		goto error_pm;
	}

	ret = cs40l5x_write_mailbox(dev, data->output);

	(void)k_mutex_unlock(&data->common.lock);

error_pm:
	(void)pm_device_runtime_put(dev);

	return ret;
}

static int cs40l5x_stop_output(const struct device *const dev)
{
	int ret;

	ret = pm_device_runtime_get(dev);
	if (ret < 0) {
		return ret;
	}

	ret = cs40l5x_write_mailbox(dev, CS40L5X_WRITE_PAUSE_PLAYBACK);

	(void)pm_device_runtime_put(dev);

	return ret;
}

static inline uint32_t cs40l5x_custom_header(uint8_t index, uint8_t header)
{
	switch (header) {
	case CS40L5X_HEADER_1:
		return (index == CS40L5X_CUSTOM_0) ? CS40L5X_REG_CUSTOM_HEADER1_0
						   : CS40L5X_REG_CUSTOM_HEADER1_1;
	case CS40L5X_HEADER_2:
		return (index == CS40L5X_CUSTOM_0) ? CS40L5X_REG_CUSTOM_HEADER2_0
						   : CS40L5X_REG_CUSTOM_HEADER2_1;
	default:
		return CS40L5X_HEADER_ERROR;
	}
}

static int cs40l5x_upload_pcm_header(const struct device *const dev,
				     const enum cs40l5x_custom_index index, const uint16_t redc,
				     const uint16_t f0, const uint16_t num_samples)
{
	uint32_t header[2];
	int ret;

	header[0] = FIELD_PREP(GENMASK(21, 0), num_samples);
	header[1] = FIELD_PREP(GENMASK(23, 12), f0) | FIELD_PREP(GENMASK(11, 0), redc);

	ret = cirrus_firmware_write(dev, cs40l5x_custom_header(index, CS40L5X_HEADER_1),
				    CS40L5X_WRITE_PCM);
	if (ret < 0) {
		return ret;
	}

	return cirrus_firmware_burst_write(dev, cs40l5x_custom_header(index, CS40L5X_HEADER_2),
					   header, ARRAY_SIZE(header));
}

static int cs40l5x_upload_pcm_data(const struct device *const dev,
				   const enum cs40l5x_custom_index index,
				   const int8_t *const samples, const uint16_t num_samples)
{
	uint32_t addr, offset = 0, sample;
	uint16_t i = 0;
	int ret;

	addr = (index == CS40L5X_CUSTOM_0) ? CS40L5X_REG_CUSTOM_DATA_0 : CS40L5X_REG_CUSTOM_DATA_1;

	while (i < num_samples) {
		sample = FIELD_PREP(GENMASK(23, 16), (uint8_t)samples[i]);
		i += 1;

		if (i < num_samples) {
			sample |= FIELD_PREP(GENMASK(15, 8), (uint8_t)samples[i]);
			i += 1;
		}

		if (i < num_samples) {
			sample |= FIELD_PREP(GENMASK(7, 0), (uint8_t)samples[i]);
			i += 1;
		}

		ret = cirrus_firmware_write_offset(dev, addr, sample, offset);
		if (ret < 0) {
			return ret;
		}

		offset += CIRRUS_REGISTER_WIDTH;
	}

	return 0;
}

int cs40l5x_upload_pcm(const struct device *const dev, const enum cs40l5x_custom_index index,
		       const uint16_t redc, const uint16_t f0, const int8_t *const samples,
		       const uint16_t num_samples)
{
	__maybe_unused const struct cirrus_config *const config = dev->config;
	struct cs40l5x_data *const data = dev->data;
	int ret;

	if (!IS_ENABLED(CONFIG_HAPTICS_CS40L5X_CUSTOM_EFFECTS)) {
		LOG_INST_ERR(config->log, "custom effects are disabled (%d)", -EPERM);
		return -EPERM;
	}

	if (num_samples == 0 || num_samples > CS40L5X_MAX_PCM_SAMPLES) {
		LOG_INST_ERR(config->log, "invalid PCM sample length provided (%d)", -EINVAL);
		return -EINVAL;
	}

	ret = pm_device_runtime_get(dev);
	if (ret < 0) {
		return ret;
	}

	ret = k_mutex_lock(&data->common.lock, CIRRUS_T_TIMEOUT);
	if (ret < 0) {
		LOG_INST_DBG(config->log, "timed out waiting for lock (%d)", ret);
		goto error_pm;
	}

	ret = cs40l5x_upload_pcm_header(dev, index, redc, f0, num_samples);
	if (ret < 0) {
		goto error_mutex;
	}

	ret = cs40l5x_upload_pcm_data(dev, index, samples, num_samples);
	if (ret < 0) {
		goto error_mutex;
	}

	data->custom_effects[index] = true;

error_mutex:
	(void)k_mutex_unlock(&data->common.lock);

error_pm:
	(void)pm_device_runtime_put(dev);

	return ret;
}

static int cs40l5x_upload_pwle_header(const struct device *const dev,
				      const enum cs40l5x_custom_index index,
				      const struct cs40l5x_pwle_section *const sections,
				      const uint8_t num_sections)
{
	uint32_t header[4];
	int ret;

	header[0] = CS40L5X_PWLE_RESERVED_VALUE;
	header[1] = FIELD_PREP(GENMASK(3, 0), FIELD_GET(GENMASK(7, 4), num_sections));
	header[2] = FIELD_PREP(GENMASK(23, 20), FIELD_GET(GENMASK(3, 0), num_sections)) |
		    FIELD_PREP(GENMASK(3, 0), FIELD_GET(GENMASK(11, 8), sections[0].level));
	header[3] = FIELD_PREP(GENMASK(23, 16), FIELD_GET(GENMASK(7, 0), sections[0].level)) |
		    FIELD_PREP(GENMASK(15, 4), CS40L5X_PWLE_DEFAULT_FREQ) |
		    FIELD_PREP(GENMASK(3, 0), CS40L5X_PWLE_DEFAULT_FLAGS);

	ret = cirrus_firmware_write(dev, cs40l5x_custom_header(index, CS40L5X_HEADER_1),
				    CS40L5X_WRITE_PWLE);
	if (ret < 0) {
		return ret;
	}

	return cirrus_firmware_burst_write(dev, cs40l5x_custom_header(index, CS40L5X_HEADER_2),
					   header, ARRAY_SIZE(header));
}

static int cs40l5x_upload_pwle_data(const struct device *const dev,
				    const enum cs40l5x_custom_index index,
				    const struct cs40l5x_pwle_section *const sections,
				    const uint8_t num_sections)
{
	uint32_t addr, offset = 4 * CIRRUS_REGISTER_WIDTH, word;
	int ret;

	addr = cs40l5x_custom_header(index, CS40L5X_HEADER_2);

	for (int i = 1; i < num_sections; i++) {
		word = FIELD_PREP(GENMASK(19, 4), sections[i].duration) |
		       FIELD_PREP(GENMASK(3, 0), FIELD_GET(GENMASK(11, 8), sections[i].level));

		ret = cirrus_firmware_write_offset(dev, addr, word, offset);
		if (ret < 0) {
			return ret;
		}

		offset += CIRRUS_REGISTER_WIDTH;

		word = FIELD_PREP(GENMASK(23, 16), FIELD_GET(GENMASK(7, 0), sections[i].level)) |
		       FIELD_PREP(GENMASK(15, 4), sections[i].frequency) |
		       FIELD_PREP(GENMASK(3, 0), sections[i].flags);

		ret = cirrus_firmware_write_offset(dev, addr, word, offset);
		if (ret < 0) {
			return ret;
		}

		offset += CIRRUS_REGISTER_WIDTH;
	}

	return 0;
}

int cs40l5x_upload_pwle(const struct device *const dev, const enum cs40l5x_custom_index index,
			const struct cs40l5x_pwle_section *const sections,
			const uint8_t num_sections)
{
	__maybe_unused const struct cirrus_config *const config = dev->config;
	struct cs40l5x_data *const data = dev->data;
	int ret;

	if (!IS_ENABLED(CONFIG_HAPTICS_CS40L5X_CUSTOM_EFFECTS)) {
		LOG_INST_ERR(config->log, "custom effects are disabled (%d)", -EPERM);
		return -EPERM;
	}

	if (num_sections == 0 || num_sections > CS40L5X_MAX_PWLE_SECTIONS) {
		LOG_INST_ERR(config->log, "invalid PWLE section length provided (%d)", -EINVAL);
		return -EINVAL;
	}

	ret = pm_device_runtime_get(dev);
	if (ret < 0) {
		return ret;
	}

	ret = k_mutex_lock(&data->common.lock, CIRRUS_T_TIMEOUT);
	if (ret < 0) {
		LOG_INST_DBG(config->log, "timed out waiting for lock (%d)", ret);
		goto error_pm;
	}

	ret = cs40l5x_upload_pwle_header(dev, index, sections, num_sections);
	if (ret < 0) {
		goto error_mutex;
	}

	ret = cs40l5x_upload_pwle_data(dev, index, sections, num_sections);
	if (ret < 0) {
		goto error_mutex;
	}

	data->custom_effects[index] = true;

error_mutex:
	(void)k_mutex_unlock(&data->common.lock);

error_pm:
	(void)pm_device_runtime_put(dev);

	return ret;
}

static DEVICE_API(haptics, cs40l5x_driver_api) = {
	.start_output = &cs40l5x_start_output,
	.stop_output = &cs40l5x_stop_output,
	.register_error_callback = &cirrus_register_error_callback,
};

static int cs40l5x_pm_resume(const struct device *const dev)
{
	__maybe_unused const struct cirrus_config *const config = dev->config;
	int ret;

	ret = pm_device_runtime_get(cirrus_get_control_port(dev));
	if (ret < 0) {
		return ret;
	}

	ret = cs40l5x_write_mailbox(dev, CS40L5X_MBOX_PREVENT_HIBERNATION);
	if (ret < 0) {
		LOG_INST_DBG(config->log, "failed to disable hibernation (%d)", ret);

		(void)pm_device_runtime_put(cirrus_get_control_port(dev));

		return ret;
	}

	LOG_INST_DBG(config->log, "disabling hibernation");

	ret = cirrus_firmware_poll(dev, CS40L5X_REG_HALO_STATE, CS40L5X_EXP_DSP_STANDBY,
				   CS40L5X_T_WAKESOURCE);
	if (ret < 0) {
		(void)pm_device_runtime_put(cirrus_get_control_port(dev));

		return ret;
	}

	return 0;
}

#ifdef CONFIG_PM_DEVICE
static int cs40l5x_pm_suspend(const struct device *const dev)
{
	__maybe_unused const struct cirrus_config *const config = dev->config;
	int ret;

	ret = cs40l5x_write_mailbox(dev, CS40L5X_MBOX_ALLOW_HIBERNATION);
	if (ret < 0) {
		LOG_INST_DBG(config->log, "failed to allow hibernation (%d)", ret);
		return ret;
	}

	LOG_INST_DBG(config->log, "allowing hibernation");

	(void)pm_device_runtime_put(cirrus_get_control_port(dev));

	return 0;
}

static int cs40l5x_pm_turn_off(const struct device *const dev)
{
	const struct cs40l5x_config *const config = dev->config;
	int ret;

	if (HAPTICS_CIRRUS_RESET(config)) {
		ret = pm_device_runtime_get(config->common.reset_gpio.port);
		if (ret < 0) {
			return ret;
		}
	}

	ret = cs40l5x_teardown(dev);
	if (ret < 0) {
		LOG_INST_DBG(config->common.log, "failed device teardown (%d)", ret);
		return ret;
	}

	if (HAPTICS_CS40L5X_EXTERNAL_BOOST(config)) {
		(void)regulator_disable(config->external_boost);
	}

	if (HAPTICS_CIRRUS_RESET(config)) {
		(void)pm_device_runtime_put(config->common.reset_gpio.port);
	}

	if (HAPTICS_CIRRUS_INTERRUPT(config)) {
		(void)pm_device_runtime_put(config->common.interrupt_gpio.port);
	}

	return 0;
}
#endif /* CONFIG_PM_DEVICE */

static int cs40l5x_pm_turn_on(const struct device *const dev)
{
	const struct cs40l5x_config *const config = dev->config;
	int ret;

	if (HAPTICS_CIRRUS_RESET(config)) {
		ret = pm_device_runtime_get(config->common.reset_gpio.port);
		if (ret < 0) {
			return ret;
		}
	}

	ret = pm_device_runtime_get(cirrus_get_control_port(dev));
	if (ret < 0) {
		goto error_reset;
	}

	if (HAPTICS_CIRRUS_INTERRUPT(config)) {
		ret = pm_device_runtime_get(config->common.interrupt_gpio.port);
		if (ret < 0) {
			goto error_control_port;
		}
	}

	if (HAPTICS_CS40L5X_EXTERNAL_BOOST(config)) {
		ret = regulator_enable(config->external_boost);
		if (ret < 0) {
			LOG_INST_DBG(config->common.log, "failed to enable regulator (%d)", ret);
			goto error_interrupt;
		}
	}

	ret = cs40l5x_bringup(dev);
	if (ret < 0) {
		LOG_INST_ERR(config->common.log, "failed device bringup (%d)", ret);
	}

	if (HAPTICS_CS40L5X_EXTERNAL_BOOST(config) && ret < 0) {
		(void)regulator_disable(config->external_boost);
	}

error_interrupt:
	if (HAPTICS_CIRRUS_INTERRUPT(config) && ret < 0) {
		(void)pm_device_runtime_put(config->common.interrupt_gpio.port);
	}

error_control_port:
	(void)pm_device_runtime_put(cirrus_get_control_port(dev));

error_reset:
	if (HAPTICS_CIRRUS_RESET(config)) {
		(void)pm_device_runtime_put(config->common.reset_gpio.port);
	}

	return ret;
}

static int cs40l5x_pm_action(const struct device *dev, enum pm_device_action action)
{
	switch (action) {
	case PM_DEVICE_ACTION_RESUME:
		return cs40l5x_pm_resume(dev);
#if CONFIG_PM_DEVICE
	case PM_DEVICE_ACTION_SUSPEND:
		return cs40l5x_pm_suspend(dev);
	case PM_DEVICE_ACTION_TURN_OFF:
		return cs40l5x_pm_turn_off(dev);
#endif /* CONFIG_PM_DEVICE */
	case PM_DEVICE_ACTION_TURN_ON:
		return cs40l5x_pm_turn_on(dev);
	default:
		return -ENOTSUP;
	}
}

static int cs40l5x_init(const struct device *dev)
{
	struct cs40l5x_data *const data = dev->data;
	int ret;

	if (IS_ENABLED(CONFIG_HAPTICS_CIRRUS_CALIBRATION)) {
		ret = k_sem_init(&data->calibration_semaphore, 0, 1);
		if (ret < 0) {
			return ret;
		}
	}

	ret = cirrus_init(dev, cs40l5x_interrupt_worker);
	if (ret < 0) {
		return ret;
	}

	return pm_device_driver_init(dev, cs40l5x_pm_action);
}

#if CONFIG_PM_DEVICE
__maybe_unused static int cs40l5x_deinit(const struct device *dev)
{
	return pm_device_driver_deinit(dev, cs40l5x_pm_action);
}
#endif /* CONFIG_PM_DEVICE */

#define HAPTICS_CS40L5X_DEFINE(inst, name, id)                                                     \
	HAPTICS_CS40L5X_BUILD_ASSERTS(inst);                                                       \
	LOG_INSTANCE_REGISTER(DT_NODE_FULL_NAME_TOKEN(DT_DRV_INST(inst)), inst,                    \
			      CONFIG_HAPTICS_LOG_LEVEL);                                           \
	static const struct cs40l5x_config name##_config_##inst = {                                \
		.common = HAPTICS_CIRRUS_CONFIG(inst, id),                                         \
		.external_boost = DEVICE_DT_GET_OR_NULL(DT_INST_PHANDLE(inst, external_boost))};   \
	static struct cs40l5x_data name##_data_##inst = {.common = HAPTICS_CIRRUS_DATA(inst),      \
							 .output = CS40L5X_ROM_BANK_CMD,           \
							 .custom_effects = {false, false}};        \
	PM_DEVICE_DT_INST_DEFINE(inst, cs40l5x_pm_action);                                         \
	HAPTICS_CIRRUS_INIT(inst, cs40l5x, name);

#define DT_DRV_COMPAT cirrus_cs40l50
#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)
DT_INST_FOREACH_STATUS_OKAY_VARGS(HAPTICS_CS40L5X_DEFINE, cs40l50, CS40L5X_DEVID_50)
#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */
#undef DT_DRV_COMPAT

#define DT_DRV_COMPAT cirrus_cs40l51
#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)
DT_INST_FOREACH_STATUS_OKAY_VARGS(HAPTICS_CS40L5X_DEFINE, cs40l51, CS40L5X_DEVID_51)
#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */
#undef DT_DRV_COMPAT

#define DT_DRV_COMPAT cirrus_cs40l52
#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)
DT_INST_FOREACH_STATUS_OKAY_VARGS(HAPTICS_CS40L5X_DEFINE, cs40l52, CS40L5X_DEVID_52)
#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */
#undef DT_DRV_COMPAT

#define DT_DRV_COMPAT cirrus_cs40l53
#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)
DT_INST_FOREACH_STATUS_OKAY_VARGS(HAPTICS_CS40L5X_DEFINE, cs40l53, CS40L5X_DEVID_53)
#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */
#undef DT_DRV_COMPAT
