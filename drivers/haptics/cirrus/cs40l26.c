/*
 * Copyright (c) 2026 Cirrus Logic, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Core driver for Cirrus Logic CS40L26/27 haptic drivers
 */

#include "cs40l26.h"
#include <stdlib.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/haptics/cs40l26.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device_runtime.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/util_macro.h>

LOG_MODULE_REGISTER(CS40L26, CONFIG_HAPTICS_LOG_LEVEL);

/* Fixed register map */
#define CS40L26_REG_DEVID       0x00000000U
#define CS40L26_REG_REVID       0x00000004U
#define CS40L26_REG_OTPID       0x00000010U
#define CS40L26_REG_SFT_RESET   0x00000020U
#define CS40L26_REG_IRQ1_STATUS 0x00010004U
#define CS40L26_REG_IRQ1_EINT_1 0x00010010U
#define CS40L26_REG_IRQ1_MASK_1 0x00010110U
#define CS40L26_REG_DSP_MBOX_2  0x00013004U
#define CS40L26_REG_DSP_MBOX_8  0x0001301CU
#define CS40L26_REG_DSP_V1MBOX  0x00013020U

/* Masks */
#define CS40L26_MASK_F0_ENABLE   BIT(0)
#define CS40L26_MASK_REDC_ENABLE BIT(1)
#define CS40L26_MASK_ATTENUATION GENMASK(11, 9)
#define CS40L26_MASK_BUZ_BANK    BIT(7)

/* Unmasked interrupts */
#define CS40L26_DSP_VIRTUAL2_MBOX_WR_MASK1 BIT(31)
#define CS40L26_AMP_ERR_MASK1              BIT(27)
#define CS40L26_TEMP_ERR_MASK1             BIT(26)
#define CS40L26_BST_IPK_FLAG_MASK1         BIT(23)
#define CS40L26_BST_SHORT_ERR_MASK1        BIT(22)
#define CS40L26_BST_DCM_UVP_ERR_MASK1      BIT(21)
#define CS40L26_BST_OVP_ERR_MASK1          BIT(20)
#define CS40L26_IRQ_MASK1                                                                          \
	(CS40L26_DSP_VIRTUAL2_MBOX_WR_MASK1 | CS40L26_AMP_ERR_MASK1 | CS40L26_TEMP_ERR_MASK1 |     \
	 CS40L26_BST_IPK_FLAG_MASK1 | CS40L26_BST_SHORT_ERR_MASK1 |                                \
	 CS40L26_BST_DCM_UVP_ERR_MASK1 | CS40L26_BST_OVP_ERR_MASK1)

/* Outbound mailbox codes */
#define CS40L26_MBOX_PREVENT_HIBERNATION 0x02000003U
#define CS40L26_MBOX_ALLOW_HIBERNATION   0x02000004U
#define CS40L26_MBOX_PAUSE_PLAYBACK      0x05000000U

/* Wavetable commands */
#define CS40L26_ROM_BANK_CMD 0x01800000
#define CS40L26_BUZ_BANK_CMD 0x01800080

/* Inbound mailbox codes */
#define CS40L26_HAPTIC_COMPLETE_MBOX 0x01000000U
#define CS40L26_HAPTIC_COMPLETE_GPIO 0x01000001U
#define CS40L26_HAPTIC_COMPLETE_I2S  0x01000002U
#define CS40L26_HAPTIC_TRIGGER_MBOX  0x01000010U
#define CS40L26_HAPTIC_TRIGGER_GPIO  0x01000011U
#define CS40L26_HAPTIC_TRIGGER_I2S   0x01000012U
#define CS40L26_AWAKE                0x02000002U
#define CS40L26_F0_EST_START         0x07000011U
#define CS40L26_F0_EST_DONE          0x07000021U
#define CS40L26_REDC_EST_START       0x07000012U
#define CS40L26_REDC_EST_DONE        0x07000022U
#define CS40L26_PANIC                0x0C000000U

/* Miscellaneous codes */
#define CS40L26_SFT_RESET     0x5A000000U
#define CS40L26_MBOX_OVERFLOW 0x00000006U
#define CS40L26_DSP_STANDBY   0x00000002U

/* Timing specifications */
#define CS40L26_T_RLPW              K_MSEC(1)
#define CS40L26_T_IRS               K_USEC(5000)
#define CS40L26_T_IW                K_USEC(1500)
#define CS40L26_T_MBOX_CLEAR        K_MSEC(10)
#define CS40L26_T_REDC              K_MSEC(1000)
#define CS40L26_T_F0                K_MSEC(2500)
#define CS40L26_T_CALIBRATION_START K_MSEC(1000)

/* Kconfig options */
#define CS40L26_PM_ACTIVE_TIMEOUT CONFIG_HAPTICS_CS40L26_PM_ACTIVE_TIMEOUT
#define CS40L26_PM_STDBY_TIMEOUT  CONFIG_HAPTICS_CS40L26_PM_STDBY_TIMEOUT

/* Miscellaneous helpers */
#define CS40L26_NUM_IRQ1_INT    5
#define CS40L26_MAX_GAIN        100
#define CS40L26_MAX_ATTENUATION 0x7FFFFF
#define CS40L26_NUM_ROM_EFFECTS 39
#define CS40L26_NUM_BUZ_EFFECTS 1
#define CS40L26_WSEQ_TERMINATOR 0x00FF0000U

static const uint8_t cs40l26_trigger_offsets[] = {
	[CS40L26_GPIO1] = 0x00,
	[CS40L26_GPIO2] = 0x08,
	[CS40L26_GPIO3] = 0x10,
	[CS40L26_GPIO4] = 0x18,
};

static const uint32_t cs40l26_irq_clear[] = {
	CIRRUS_WRITE_BE32(0xFFFFFFFFU, 0xFFFFFFFFU, 0xFFFFFFFFU, 0xFFFFFFFFU, 0xFFFFFFFFU)};

static const uint32_t cs40l26_irq_masks[] = {
	CIRRUS_WRITE_BE32(~CS40L26_IRQ_MASK1, 0xFFFFFFFFU, 0xFFFFFFFFU, 0xFFFFFFFFU, 0xFFFFFFFFU)};

static const uint32_t cs40l26_pseq[] = {CIRRUS_WRITE_BE32(
	0x00000001U, 0x00011073U, 0x000FFFFFU, 0x000304FFU, 0x00FFFFFFU, 0x000304FFU, 0x00FFFFFFU,
	0x000304FFU, 0x00FFFFFFU, 0x000304FFU, 0x00FFFFFFU, CS40L26_WSEQ_TERMINATOR)};

static inline bool cs40l26_valid_wavetable_source(const struct device *const dev,
						  const enum cs40l26_bank bank,
						  const uint16_t index)
{
	switch (bank) {
	case CS40L26_ROM_BANK:
		return index < CS40L26_NUM_ROM_EFFECTS;
	case CS40L26_BUZ_BANK:
		return index < CS40L26_NUM_BUZ_EFFECTS;
	default:
		return false;
	}
}

static int cs40l26_write_mailbox(const struct device *const dev, const uint32_t mailbox_command)
{
	__maybe_unused const struct cirrus_config *const config = dev->config;
	const k_timepoint_t end = sys_timepoint_calc(CS40L26_T_IW);
	int ret;

	do {
		ret = cirrus_write(dev, CS40L26_REG_DSP_V1MBOX, mailbox_command);
		if (ret >= 0) {
			return cirrus_poll(dev, CS40L26_REG_DSP_V1MBOX, 0, CS40L26_T_MBOX_CLEAR);
		}

		(void)k_sleep(CIRRUS_T_POLLING);
	} while (!sys_timepoint_expired(end));

	LOG_INST_DBG(config->log, "timed out writing to mailbox (%d)", ret);

	return -ETIMEDOUT;
}

static int cs40l26_increment_mailbox(const struct device *const dev, uint32_t *const mbox_ptr)
{
	if (*mbox_ptr < CS40L26_REG_DSP_MBOX_8) {
		*mbox_ptr += CIRRUS_REGISTER_WIDTH;
	} else {
		*mbox_ptr = CS40L26_REG_DSP_MBOX_2;
	}

	return cirrus_firmware_write(dev, CS40L26_REG_MAILBOX_QUEUE_RD, *mbox_ptr);
}

static int cs40l26_poll_mailbox(const struct device *const dev, const uint32_t mailbox_command,
				const k_timeout_t timeout)
{
	uint32_t mbox_rd_ptr;
	int ret;

	ret = cirrus_firmware_read(dev, CS40L26_REG_MAILBOX_QUEUE_RD, &mbox_rd_ptr);
	if (ret < 0) {
		return ret;
	}

	ret = cirrus_poll(dev, mbox_rd_ptr, mailbox_command, timeout);
	if (ret < 0) {
		return ret;
	}

	return cs40l26_increment_mailbox(dev, &mbox_rd_ptr);
}

static int cs40l26_reset_mailbox(const struct device *const dev)
{
	uint32_t mbox_ptr;
	int ret;

	ret = cirrus_firmware_read(dev, CS40L26_REG_MAILBOX_QUEUE_WT, &mbox_ptr);
	if (ret < 0) {
		return ret;
	}

	return cirrus_firmware_write(dev, CS40L26_REG_MAILBOX_QUEUE_RD, mbox_ptr);
}

static int cs40l26_process_mailbox(const struct device *const dev)
{
	__maybe_unused const struct cirrus_config *const config = dev->config;
	uint32_t mbox_rd_ptr, mbox_status, mbox_val, mbox_wt_ptr;
	struct cs40l26_data *const data = dev->data;
	int ret;

	ret = cirrus_firmware_read(dev, CS40L26_REG_MAILBOX_STATUS, &mbox_status);
	if (ret < 0) {
		return ret;
	}

	ret = cirrus_firmware_read(dev, CS40L26_REG_MAILBOX_QUEUE_RD, &mbox_rd_ptr);
	if (ret < 0) {
		return ret;
	}

	ret = cirrus_firmware_read(dev, CS40L26_REG_MAILBOX_QUEUE_WT, &mbox_wt_ptr);
	if (ret < 0) {
		return ret;
	}

	if (mbox_status == CS40L26_MBOX_OVERFLOW) {
		LOG_INST_DBG(config->log, "mailbox overflow");
	}

	do {
		ret = cirrus_read(dev, mbox_rd_ptr, &mbox_val);
		if (ret < 0) {
			return ret;
		}

		switch (mbox_val) {
		case CS40L26_HAPTIC_COMPLETE_MBOX:
			LOG_INST_DBG(config->log, "mailbox playback complete");
			break;
		case CS40L26_HAPTIC_COMPLETE_GPIO:
			LOG_INST_DBG(config->log, "trigger playback complete");
			break;
		case CS40L26_HAPTIC_TRIGGER_MBOX:
			LOG_INST_DBG(config->log, "mailbox playback started");
			break;
		case CS40L26_HAPTIC_TRIGGER_GPIO:
			LOG_INST_DBG(config->log, "trigger playback started");
			break;
		case CS40L26_AWAKE:
			LOG_INST_DBG(config->log, "awake");
			break;
		case CS40L26_REDC_EST_START:
			LOG_INST_DBG(config->log, "ReDC calibration started");
			break;
		case CS40L26_REDC_EST_DONE:
			LOG_INST_DBG(config->log, "ReDC calibration complete");
			k_sem_give(&data->calibration_semaphore);
			break;
		case CS40L26_F0_EST_START:
			LOG_INST_DBG(config->log, "F0 calibration started");
			break;
		case CS40L26_F0_EST_DONE:
			LOG_INST_DBG(config->log, "F0 calibration complete");
			k_sem_give(&data->calibration_semaphore);
			break;
		case CS40L26_PANIC:
			LOG_INST_WRN(config->log, "device panic: 0x%08X", mbox_val);
			return 0;
		default:
			LOG_INST_DBG(config->log, "unexpected mailbox code: %08x", mbox_val);
			break;
		}

		ret = cs40l26_increment_mailbox(dev, &mbox_rd_ptr);
		if (ret < 0) {
			return ret;
		}
	} while (mbox_rd_ptr != mbox_wt_ptr);

	return cirrus_write(dev, CS40L26_REG_IRQ1_EINT_1, CS40L26_DSP_VIRTUAL2_MBOX_WR_MASK1);
}

static int cs40l26_process_interrupts(const struct device *const dev,
				      const uint32_t *const irq_ints)
{
	__maybe_unused const struct cirrus_config *const config = dev->config;
	uint32_t error_bitmask = 0;
	int ret;

	if (FIELD_GET(CS40L26_AMP_ERR_MASK1, irq_ints[0]) != 0) {
		LOG_INST_WRN(config->log, "amplifier short detected");

		error_bitmask |= HAPTICS_ERROR_OVERCURRENT;

		ret = cirrus_write(dev, CS40L26_REG_IRQ1_EINT_1, CS40L26_AMP_ERR_MASK1);
		if (ret < 0) {
			return ret;
		}
	}

	if (FIELD_GET(CS40L26_TEMP_ERR_MASK1, irq_ints[0]) != 0) {
		LOG_INST_WRN(config->log, "overtemperature detected");

		error_bitmask |= HAPTICS_ERROR_OVERTEMPERATURE;

		ret = cirrus_write(dev, CS40L26_REG_IRQ1_EINT_1, CS40L26_TEMP_ERR_MASK1);
		if (ret < 0) {
			return ret;
		}
	}

	if (FIELD_GET(CS40L26_BST_IPK_FLAG_MASK1, irq_ints[0]) != 0) {
		LOG_INST_WRN(config->log, "current limited");

		ret = cirrus_write(dev, CS40L26_REG_IRQ1_EINT_1, CS40L26_BST_IPK_FLAG_MASK1);
		if (ret < 0) {
			return ret;
		}
	}

	if (FIELD_GET(CS40L26_BST_SHORT_ERR_MASK1, irq_ints[0]) != 0) {
		LOG_INST_WRN(config->log, "inductor short detected");

		error_bitmask |= HAPTICS_ERROR_OVERCURRENT;

		ret = cirrus_write(dev, CS40L26_REG_IRQ1_EINT_1, CS40L26_BST_SHORT_ERR_MASK1);
		if (ret < 0) {
			return ret;
		}
	}

	if (FIELD_GET(CS40L26_BST_DCM_UVP_ERR_MASK1, irq_ints[0]) != 0) {
		LOG_INST_WRN(config->log, "boost undervoltage detected");

		error_bitmask |= HAPTICS_ERROR_UNDERVOLTAGE;

		ret = cirrus_write(dev, CS40L26_REG_IRQ1_EINT_1, CS40L26_BST_DCM_UVP_ERR_MASK1);
		if (ret < 0) {
			return ret;
		}
	}

	if (FIELD_GET(CS40L26_BST_OVP_ERR_MASK1, irq_ints[0]) != 0) {
		LOG_INST_WRN(config->log, "boost overvoltage detected");

		error_bitmask |= HAPTICS_ERROR_OVERVOLTAGE;

		ret = cirrus_write(dev, CS40L26_REG_IRQ1_EINT_1, CS40L26_BST_OVP_ERR_MASK1);
		if (ret < 0) {
			return ret;
		}
	}

	if (error_bitmask != 0) {
		cirrus_error_callback(dev, error_bitmask);
	}

	return 0;
}

static int cs40l26_retrieve_interrupt_statuses(const struct device *const dev,
					       uint32_t *const irq_ints)
{
	uint32_t irq_masks[CS40L26_NUM_IRQ1_INT];
	int ret;

	ret = cirrus_burst_read(dev, CS40L26_REG_IRQ1_EINT_1, irq_ints, CS40L26_NUM_IRQ1_INT);
	if (ret < 0) {
		return ret;
	}

	ret = cirrus_burst_read(dev, CS40L26_REG_IRQ1_MASK_1, irq_masks, CS40L26_NUM_IRQ1_INT);
	if (ret < 0) {
		return ret;
	}

	for (int i = 0; i < CS40L26_NUM_IRQ1_INT; i++) {
		irq_ints[i] &= ~irq_masks[i];
	}

	return 0;
}

static void cs40l26_interrupt_worker(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct cirrus_data *data = CONTAINER_OF(dwork, struct cirrus_data, interrupt_worker);
	const struct device *const dev = data->dev;
	const struct cirrus_config *const config = dev->config;
	uint32_t irq1_status, irq_ints[CS40L26_NUM_IRQ1_INT];
	int ret;

	if (gpio_pin_get_dt(&config->interrupt_gpio) == CIRRUS_GPIO_INACTIVE) {
		return;
	}

	ret = pm_device_runtime_get(dev);
	if (ret < 0) {
		return;
	}

	ret = cirrus_read(dev, CS40L26_REG_IRQ1_STATUS, &irq1_status);
	if (ret < 0) {
		goto error_pm;
	}

	if (irq1_status == 0) {
		LOG_INST_DBG(config->log, "IRQ status unset in interrupt worker");
		goto error_pm;
	}

	ret = cs40l26_retrieve_interrupt_statuses(dev, irq_ints);
	if (ret < 0) {
		LOG_INST_DBG(config->log, "failed to read IRQ registers (%d)", ret);
		goto error_pm;
	}

	ret = cs40l26_process_interrupts(dev, irq_ints);
	if (ret < 0) {
		LOG_INST_DBG(config->log, "failed to process non-mailbox interrupts (%d)", ret);
		goto error_pm;
	}

	if (FIELD_GET(CS40L26_DSP_VIRTUAL2_MBOX_WR_MASK1, irq_ints[0]) != 0) {
		ret = cs40l26_process_mailbox(dev);
		if (ret < 0) {
			LOG_INST_DBG(config->log, "failed to process mailbox (%d)", ret);
			goto error_pm;
		}
	}

	ret = cirrus_read(dev, CS40L26_REG_IRQ1_STATUS, &irq1_status);
	if (ret < 0) {
		goto error_pm;
	}

	if (irq1_status != 0) {
		LOG_INST_DBG(config->log, "IRQ still set in interrupt worker");

		ret = k_work_submit(work);
		if (ret < 0) {
			LOG_INST_DBG(config->log, "failed to resubmit worker (%d)", ret);
		}
	}

error_pm:
	(void)pm_device_runtime_put(dev);
}

static int cs40l26_click_compensation(const struct device *const dev)
{
	__maybe_unused const struct cirrus_config *const config = dev->config;
	struct cirrus_data *const data = dev->data;
	int ret;

	if (!IS_ENABLED(CONFIG_HAPTICS_CIRRUS_CLICK_COMPENSATION)) {
		return 0;
	}

	if (data->calibration.redc == 0 && data->calibration.f0 == 0) {
		LOG_INST_WRN(config->log, "no calibration data provided");
		return 0;
	}

	ret = cirrus_firmware_write(dev, CS40L26_REG_VIBEGEN_F0_OTP_STORED, data->calibration.f0);
	if (ret < 0) {
		return ret;
	}

	ret = cirrus_firmware_write(dev, CS40L26_REG_VIBEGEN_REDC_OTP_STORED,
				    data->calibration.redc);
	if (ret < 0) {
		return ret;
	}

	return cirrus_firmware_write(dev, CS40L26_REG_VIBEGEN_COMPENSATION_ENABLE,
				     CS40L26_MASK_F0_ENABLE | CS40L26_MASK_REDC_ENABLE);
}

static int cs40l26_dynamic_f0(const struct device *const dev)
{
	if (!IS_ENABLED(CONFIG_HAPTICS_CIRRUS_DYNAMIC_F0)) {
		return 0;
	}

	return cirrus_firmware_write(dev, CS40L26_REG_DYNAMIC_F0_ENABLED, 1);
}

static int cs40l26_dsp_config(const struct device *const dev)
{
	int ret;

	ret = cirrus_load_calibration(dev);
	if (ret < 0) {
		return ret;
	}

	ret = cs40l26_click_compensation(dev);
	if (ret < 0) {
		return ret;
	}

	return cs40l26_dynamic_f0(dev);
}

static int cs40l26_timeout_config(const struct device *const dev)
{
	int ret;

	ret = cirrus_firmware_write(dev, CS40L26_REG_PM_ACTIVE_TIMEOUT, CS40L26_PM_ACTIVE_TIMEOUT);
	if (ret < 0) {
		return ret;
	}

	return cirrus_firmware_write(dev, CS40L26_REG_PM_STDBY_TIMEOUT, CS40L26_PM_STDBY_TIMEOUT);
}

static int cs40l26_fingerprint(const struct device *const dev)
{
	const struct cirrus_config *const config = dev->config;
	struct cirrus_data *const data = dev->data;
	uint32_t otpid, ids[2];
	int ret;

	ret = cirrus_burst_read(dev, CS40L26_REG_DEVID, ids, ARRAY_SIZE(ids));
	if (ret < 0) {
		return ret;
	}

	ret = cirrus_read(dev, CS40L26_REG_OTPID, &otpid);
	if (ret < 0) {
		return ret;
	}

	if (ids[0] != config->device_id) {
		LOG_INST_ERR(config->log, "mismatched device ID: 0x%05X", ids[0]);
		return -ENOTSUP;
	}

	if ((ids[0] == CS40L26_DEVID_A1 && ids[1] != CS40L26_REVID_A1) ||
	    (ids[0] == CS40L27_DEVID_B2 && ids[1] != CS40L27_REVID_B2)) {
		LOG_INST_ERR(config->log, "unsupported revision: 0x%02X", ids[1]);
		return -ENOTSUP;
	}

	data->revision_id = (uint8_t)ids[1];

	if (!IN_RANGE(otpid, CS40L26_OTPID_MIN, CS40L26_OTPID_MAX)) {
		LOG_INST_ERR(config->log, "unsupported OTP: 0x%01X", otpid);
		return -ENOTSUP;
	}

	data->otp_id = (uint8_t)otpid;

	LOG_INST_INF(config->log, "Cirrus Logic CS40L%02X Revision %X (OTP %01X)",
		     (uint8_t)FIELD_GET(GENMASK(11, 4), config->device_id), data->revision_id,
		     data->otp_id);

	return 0;
}

static int cs40l26_reset(const struct device *const dev)
{
	const struct cirrus_config *const config = dev->config;
	uint32_t halo_state;
	int ret;

	if (HAPTICS_CIRRUS_RESET(config)) {
		ret = gpio_pin_set_dt(&config->reset_gpio, CIRRUS_GPIO_ACTIVE);
		if (ret < 0) {
			return ret;
		}

		(void)k_sleep(CS40L26_T_RLPW);

		ret = gpio_pin_set_dt(&config->reset_gpio, CIRRUS_GPIO_INACTIVE);
	} else {
		ret = cirrus_wakeup(dev, CS40L26_T_IW);
		if (ret < 0) {
			return ret;
		}

		ret = cs40l26_write_mailbox(dev, CS40L26_MBOX_PREVENT_HIBERNATION);
		if (ret < 0) {
			return ret;
		}

		ret = cirrus_write(dev, CS40L26_REG_SFT_RESET, CS40L26_SFT_RESET);
	}
	if (ret < 0) {
		return ret;
	}

	(void)k_sleep(CS40L26_T_IRS);

	ret = cs40l26_fingerprint(dev);
	if (ret < 0) {
		return ret;
	}

	ret = cirrus_firmware_read(dev, CS40L26_REG_HALO_STATE, &halo_state);
	if (ret < 0) {
		return ret;
	}

	if (halo_state != CS40L26_DSP_STANDBY) {
		LOG_INST_DBG(config->log, "expected DSP in standby after hardware reset (%d)", ret);
		return -ENODEV;
	}

	return cs40l26_reset_mailbox(dev);
}

static int cs40l26_bringup(const struct device *const dev)
{
	const struct cirrus_config *const config = dev->config;
	int ret;

	ret = cs40l26_reset(dev);
	if (ret < 0) {
		LOG_INST_DBG(config->log, "failed reset (%d)", ret);
		return ret;
	}

	if (HAPTICS_CIRRUS_INTERRUPT(config)) {
		ret = cirrus_raw_burst_write(dev, CS40L26_REG_IRQ1_MASK_1, cs40l26_irq_masks,
					     ARRAY_SIZE(cs40l26_irq_masks));
		if (ret < 0) {
			return ret;
		}

		ret = cirrus_raw_burst_write(dev, CS40L26_REG_IRQ1_EINT_1, cs40l26_irq_clear,
					     ARRAY_SIZE(cs40l26_irq_clear));
		if (ret < 0) {
			return ret;
		}
	}

	ret = cirrus_interrupt_config(dev);
	if (ret < 0) {
		LOG_INST_DBG(config->log, "failed IRQ configuration (%d)", ret);
		return ret;
	}

	ret = cs40l26_dsp_config(dev);
	if (ret < 0) {
		return ret;
	}

	ret = cs40l26_timeout_config(dev);
	if (ret < 0) {
		return ret;
	}

	if (HAPTICS_CIRRUS_HIBERNATION) {
		ret = cirrus_firmware_raw_burst_write(dev, CS40L26_REG_PM_POWER_ON_SEQUENCE,
						      cs40l26_pseq, ARRAY_SIZE(cs40l26_pseq));
		if (ret < 0) {
			return ret;
		}
	}

	ret = cirrus_trigger_config(dev);
	if (ret < 0) {
		LOG_INST_DBG(config->log, "failed trigger configuration (%d)", ret);
		return ret;
	}

	return 0;
}

#if CONFIG_PM_DEVICE
static int cs40l26_teardown(const struct device *const dev)
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

static int cs40l26_calibrate_redc(const struct device *const dev)
{
	const struct cirrus_config *const config = dev->config;
	struct cs40l26_data *const data = dev->data;
	int ret;

	ret = cs40l26_write_mailbox(dev, CS40L26_REDC_EST_START);
	if (ret < 0) {
		return ret;
	}

	if (HAPTICS_CIRRUS_INTERRUPT(config)) {
		ret = k_sem_take(&data->calibration_semaphore, CS40L26_T_REDC);
	} else {
		ret = cs40l26_poll_mailbox(dev, CS40L26_REDC_EST_START,
					   CS40L26_T_CALIBRATION_START);
		if (ret < 0) {
			LOG_INST_DBG(config->log, "timed out waiting for ReDC start (%d)", ret);
			return ret;
		}

		ret = cs40l26_poll_mailbox(dev, CS40L26_REDC_EST_DONE, CS40L26_T_REDC);
	}
	if (ret < 0) {
		LOG_INST_DBG(config->log, "timed out waiting for ReDC complete (%d)", ret);
		return ret;
	}

	return cirrus_firmware_read(dev, CS40L26_REG_RE_EST_STATUS, &data->common.calibration.redc);
}

static int cs40l26_calibrate_f0(const struct device *const dev)
{
	const struct cirrus_config *const config = dev->config;
	struct cs40l26_data *const data = dev->data;
	int ret;

	ret = cs40l26_write_mailbox(dev, CS40L26_F0_EST_START);
	if (ret < 0) {
		return ret;
	}

	if (HAPTICS_CIRRUS_INTERRUPT(config)) {
		ret = k_sem_take(&data->calibration_semaphore, CS40L26_T_F0);
	} else {
		ret = cs40l26_poll_mailbox(dev, CS40L26_F0_EST_START, CS40L26_T_CALIBRATION_START);
		if (ret < 0) {
			LOG_INST_DBG(config->log, "timed out waiting for F0 start (%d)", ret);
			return ret;
		}

		ret = cs40l26_poll_mailbox(dev, CS40L26_F0_EST_DONE, CS40L26_T_F0);
	}
	if (ret < 0) {
		LOG_INST_DBG(config->log, "timed out waiting for F0 completion (%d)", ret);
		return ret;
	}

	return cirrus_firmware_read(dev, CS40L26_REG_F0_EST, &data->common.calibration.f0);
}

static int cs40l26_run_calibration(const struct device *const dev)
{
	struct cirrus_data *const data = dev->data;
	int ret;

	ret = cs40l26_calibrate_redc(dev);
	if (ret < 0) {
		return ret;
	}

	ret = cirrus_firmware_write(dev, CS40L26_REG_F0_EST_REDC, data->calibration.redc);
	if (ret < 0) {
		return ret;
	}

	return cs40l26_calibrate_f0(dev);
}

int cs40l26_calibrate(const struct device *const dev)
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
		ret = cs40l26_reset_mailbox(dev);
		if (ret < 0) {
			goto error_mutex;
		}
	}

	ret = cs40l26_run_calibration(dev);
	if (ret < 0) {
		goto error_mutex;
	}

	ret = cs40l26_click_compensation(dev);
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

int cs40l26_configure_buzz(const struct device *const dev, const uint32_t frequency,
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

	ret = cirrus_firmware_write(dev, CS40L26_REG_BUZZ_FREQ, frequency);
	if (ret < 0) {
		goto error_mutex;
	}

	ret = cirrus_firmware_write(dev, CS40L26_REG_BUZZ_LEVEL, level);
	if (ret < 0) {
		goto error_mutex;
	}

	ret = cirrus_firmware_write(dev, CS40L26_REG_BUZZ_DURATION, duration);

error_mutex:
	(void)k_mutex_unlock(&data->lock);

error_pm:
	(void)pm_device_runtime_put(dev);

	return ret;
}

int cs40l26_configure_trigger(const struct device *const dev, const struct gpio_dt_spec *const gpio,
			      const enum cs40l26_bank bank, const uint8_t index,
			      const enum cs40l26_attenuation attenuation,
			      const enum cs40l26_trigger_edge edge)
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

	if (!cs40l26_valid_wavetable_source(dev, bank, index)) {
		LOG_INST_ERR(config->log, "invalid wavetable selection (%d)", -EINVAL);
		return -EINVAL;
	}

	ret = cirrus_get_trigger(dev, gpio, &idx);
	if (ret < 0) {
		LOG_INST_DBG(config->log, "failed to retrieve trigger GPIO (%d)", ret);
		return ret;
	}

	offset = cs40l26_trigger_offsets[idx] + ((edge == CS40L26_FALLING_EDGE) ? 4 : 0);

	playback = FIELD_PREP(CS40L26_MASK_ATTENUATION, abs(attenuation)) | index;

	switch (bank) {
	case CS40L26_ROM_BANK:
		break;
	case CS40L26_BUZ_BANK:
		playback |= CS40L26_MASK_BUZ_BANK;
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

	ret = cirrus_firmware_write_offset(dev, CS40L26_REG_GPIO_EVENT_BASE, playback, offset);

	(void)k_mutex_unlock(&data->lock);

error_pm:
	(void)pm_device_runtime_put(dev);

	return ret;
}

int cs40l26_select_output(const struct device *const dev, const enum cs40l26_bank bank,
			  const uint16_t index)
{
	__maybe_unused const struct cirrus_config *const config = dev->config;
	struct cs40l26_data *const data = dev->data;
	uint32_t output;
	int ret;

	if (!cs40l26_valid_wavetable_source(dev, bank, index)) {
		LOG_INST_ERR(config->log, "invalid wavetable selection (%d)", -EINVAL);
		return -EINVAL;
	}

	switch (bank) {
	case CS40L26_ROM_BANK:
		output = index | CS40L26_ROM_BANK_CMD;
		break;
	case CS40L26_BUZ_BANK:
		output = CS40L26_BUZ_BANK_CMD;
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

int cs40l26_set_gain(const struct device *const dev, const uint8_t gain)
{
	__maybe_unused const struct cirrus_config *const config = dev->config;
	struct cirrus_data *const data = dev->data;
	uint32_t attenuation;
	int ret;

	if (gain > CS40L26_MAX_GAIN) {
		LOG_INST_ERR(config->log, "invalid gain, %u >= %u", gain, CS40L26_MAX_GAIN);
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
		attenuation = CS40L26_MAX_ATTENUATION;
	} else {
		attenuation = (uint32_t)cirrus_source_attenuation[gain];
	}

	ret = cirrus_firmware_write(dev, CS40L26_REG_SOURCE_ATTENUATION, attenuation);

	(void)k_mutex_unlock(&data->lock);

error_pm:
	(void)pm_device_runtime_put(dev);

	return ret;
}

static int cs40l26_start_output(const struct device *const dev)
{
	__maybe_unused const struct cirrus_config *const config = dev->config;
	struct cs40l26_data *const data = dev->data;
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

	ret = cs40l26_write_mailbox(dev, data->output);

	(void)k_mutex_unlock(&data->common.lock);

error_pm:
	(void)pm_device_runtime_put(dev);

	return ret;
}

static int cs40l26_stop_output(const struct device *const dev)
{
	int ret;

	ret = pm_device_runtime_get(dev);
	if (ret < 0) {
		return ret;
	}

	ret = cs40l26_write_mailbox(dev, CS40L26_MBOX_PAUSE_PLAYBACK);

	(void)pm_device_runtime_put(dev);

	return ret;
}

static DEVICE_API(haptics, cs40l26_driver_api) = {
	.start_output = &cs40l26_start_output,
	.stop_output = &cs40l26_stop_output,
	.register_error_callback = &cirrus_register_error_callback,
};

static int cs40l26_pm_resume(const struct device *const dev)
{
	__maybe_unused const struct cirrus_config *const config = dev->config;
	int ret;

	ret = pm_device_runtime_get(cirrus_get_control_port(dev));
	if (ret < 0) {
		return ret;
	}

	ret = cs40l26_write_mailbox(dev, CS40L26_MBOX_PREVENT_HIBERNATION);
	if (ret < 0) {
		LOG_INST_DBG(config->log, "failed to disable hibernation (%d)", ret);

		(void)pm_device_runtime_put(cirrus_get_control_port(dev));

		return ret;
	}

	LOG_INST_DBG(config->log, "disabling hibernation");

	ret = cirrus_firmware_poll(dev, CS40L26_REG_HALO_STATE, CS40L26_DSP_STANDBY, CS40L26_T_IRS);
	if (ret < 0) {
		(void)pm_device_runtime_put(cirrus_get_control_port(dev));

		return ret;
	}

	return 0;
}

#ifdef CONFIG_PM_DEVICE
static int cs40l26_pm_suspend(const struct device *const dev)
{
	__maybe_unused const struct cirrus_config *const config = dev->config;
	int ret;

	ret = cs40l26_write_mailbox(dev, CS40L26_MBOX_ALLOW_HIBERNATION);
	if (ret < 0) {
		LOG_INST_DBG(config->log, "failed to allow hibernation (%d)", ret);
		return ret;
	}

	LOG_INST_DBG(config->log, "allowing hibernation");

	(void)pm_device_runtime_put(cirrus_get_control_port(dev));

	return 0;
}

static int cs40l26_pm_turn_off(const struct device *const dev)
{
	const struct cirrus_config *const config = dev->config;
	int ret;

	if (HAPTICS_CIRRUS_RESET(config)) {
		ret = pm_device_runtime_get(config->reset_gpio.port);
		if (ret < 0) {
			return ret;
		}
	}

	ret = cs40l26_teardown(dev);
	if (ret < 0) {
		LOG_INST_DBG(config->log, "failed device teardown (%d)", ret);
		return ret;
	}

	if (HAPTICS_CIRRUS_RESET(config)) {
		(void)pm_device_runtime_put(config->reset_gpio.port);
	}

	if (HAPTICS_CIRRUS_INTERRUPT(config)) {
		(void)pm_device_runtime_put(config->interrupt_gpio.port);
	}

	return 0;
}
#endif /* CONFIG_PM_DEVICE */

static int cs40l26_pm_turn_on(const struct device *const dev)
{
	const struct cirrus_config *const config = dev->config;
	int ret;

	if (HAPTICS_CIRRUS_RESET(config)) {
		ret = pm_device_runtime_get(config->reset_gpio.port);
		if (ret < 0) {
			return ret;
		}
	}

	ret = pm_device_runtime_get(cirrus_get_control_port(dev));
	if (ret < 0) {
		goto error_reset;
	}

	if (HAPTICS_CIRRUS_INTERRUPT(config)) {
		ret = pm_device_runtime_get(config->interrupt_gpio.port);
		if (ret < 0) {
			goto error_control_port;
		}
	}

	ret = cs40l26_bringup(dev);
	if (ret < 0) {
		LOG_INST_ERR(config->log, "failed device bringup (%d)", ret);
	}

	if (HAPTICS_CIRRUS_INTERRUPT(config) && ret < 0) {
		(void)pm_device_runtime_put(config->interrupt_gpio.port);
	}

error_control_port:
	(void)pm_device_runtime_put(cirrus_get_control_port(dev));

error_reset:
	if (HAPTICS_CIRRUS_RESET(config)) {
		(void)pm_device_runtime_put(config->reset_gpio.port);
	}

	return ret;
}

static int cs40l26_pm_action(const struct device *dev, enum pm_device_action action)
{
	switch (action) {
	case PM_DEVICE_ACTION_RESUME:
		return cs40l26_pm_resume(dev);
#if CONFIG_PM_DEVICE
	case PM_DEVICE_ACTION_SUSPEND:
		return cs40l26_pm_suspend(dev);
	case PM_DEVICE_ACTION_TURN_OFF:
		return cs40l26_pm_turn_off(dev);
#endif /* CONFIG_PM_DEVICE */
	case PM_DEVICE_ACTION_TURN_ON:
		return cs40l26_pm_turn_on(dev);
	default:
		return -ENOTSUP;
	}
}

static int cs40l26_init(const struct device *dev)
{
	struct cs40l26_data *const data = dev->data;
	int ret;

	if (IS_ENABLED(CONFIG_HAPTICS_CIRRUS_CALIBRATION)) {
		ret = k_sem_init(&data->calibration_semaphore, 0, 1);
		if (ret < 0) {
			return ret;
		}
	}

	ret = cirrus_init(dev, cs40l26_interrupt_worker);
	if (ret < 0) {
		return ret;
	}

	return pm_device_driver_init(dev, cs40l26_pm_action);
}

#if CONFIG_PM_DEVICE
__maybe_unused static int cs40l26_deinit(const struct device *dev)
{
	return pm_device_driver_deinit(dev, cs40l26_pm_action);
}
#endif /* CONFIG_PM_DEVICE */

#define HAPTICS_CS40L26_DEFINE(inst, name, id)                                                     \
	HAPTICS_CS40L26_BUILD_ASSERTS(inst);                                                       \
	LOG_INSTANCE_REGISTER(DT_NODE_FULL_NAME_TOKEN(DT_DRV_INST(inst)), inst,                    \
			      CONFIG_HAPTICS_LOG_LEVEL);                                           \
	static const struct cirrus_config name##_config_##inst = HAPTICS_CIRRUS_CONFIG(inst, id);  \
	static struct cs40l26_data name##_data_##inst = {.common = HAPTICS_CIRRUS_DATA(inst),      \
							 .output = CS40L26_ROM_BANK_CMD};          \
	PM_DEVICE_DT_INST_DEFINE(inst, cs40l26_pm_action);                                         \
	HAPTICS_CIRRUS_INIT(inst, cs40l26, name);

#define DT_DRV_COMPAT cirrus_cs40l26
#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)
DT_INST_FOREACH_STATUS_OKAY_VARGS(HAPTICS_CS40L26_DEFINE, cs40l26, CS40L26_DEVID_A1)
#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */
#undef DT_DRV_COMPAT

#define DT_DRV_COMPAT cirrus_cs40l27
#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)
DT_INST_FOREACH_STATUS_OKAY_VARGS(HAPTICS_CS40L26_DEFINE, cs40l27, CS40L27_DEVID_B2)
#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */
#undef DT_DRV_COMPAT
