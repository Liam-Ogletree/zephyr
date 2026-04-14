/*
 * Copyright (c) 2025 Cirrus Logic, Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_HAPTICS_CIRRUS_CS40L5X_H_
#define ZEPHYR_DRIVERS_HAPTICS_CIRRUS_CS40L5X_H_

#include "cirrus.h"
#include <zephyr/drivers/haptics/cs40l5x.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/** @cond INTERNAL_HIDDEN */

/** Supported devices */
#define CS40L5X_DEVID_50  0x40A50U
#define CS40L5X_DEVID_51  0x40A51U
#define CS40L5X_DEVID_52  0x40A52U
#define CS40L5X_DEVID_53  0x40A53U
#define CS40L5X_REVID_B0  0xB0U
#define CS40L5X_OTPID_MIN 0x1U
#define CS40L5X_OTPID_MAX 0xEU

/** Non-deterministic firmware registers */
#define CS40L5X_REG_HALO_STATE                  0
#define CS40L5X_REG_BUZZ_FREQ                   1
#define CS40L5X_REG_BUZZ_LEVEL                  2
#define CS40L5X_REG_BUZZ_DURATION               3
#define CS40L5X_REG_BUZZ_RES                    4
#define CS40L5X_REG_DYNAMIC_F0                  5
#define CS40L5X_REG_REDC                        6
#define CS40L5X_REG_F0_EST                      7
#define CS40L5X_REG_SOURCE_ATTENUATION          8
#define CS40L5X_REG_LOGGER_ENABLE               9
#define CS40L5X_REG_LOGGER_DATA                 10
#define CS40L5X_REG_GPIO_EVENT_BASE             11
#define CS40L5X_REG_STDBY_TIMEOUT               12
#define CS40L5X_REG_ACTIVE_TIMEOUT              13
#define CS40L5X_REG_MAILBOX_QUEUE_WT            14
#define CS40L5X_REG_MAILBOX_QUEUE_RD            15
#define CS40L5X_REG_MAILBOX_STATUS              16
#define CS40L5X_REG_WSEQ_POWER                  17
#define CS40L5X_REG_VIBEGEN_F0_OTP_STORED       18
#define CS40L5X_REG_VIBEGEN_REDC_OTP_STORED     19
#define CS40L5X_REG_VIBEGEN_COMPENSATION_ENABLE 20
#define CS40L5X_REG_CUSTOM_HEADER1_0            21
#define CS40L5X_REG_CUSTOM_HEADER2_0            22
#define CS40L5X_REG_CUSTOM_DATA_0               23
#define CS40L5X_REG_CUSTOM_HEADER1_1            24
#define CS40L5X_REG_CUSTOM_HEADER2_1            25
#define CS40L5X_REG_CUSTOM_DATA_1               26
#define CS40L5X_REG_CALIB_REDC_EST              27

/** Configurable GPIOs */
#define CS40L5X_GPIO3  3
#define CS40L5X_GPIO4  4
#define CS40L5X_GPIO5  5
#define CS40L5X_GPIO6  6
#define CS40L5X_GPIO10 10
#define CS40L5X_GPIO11 11
#define CS40L5X_GPIO12 12
#define CS40L5X_GPIO13 13

/**
 * @brief Check if a CS40L5x driver has external boost enabled in the devicetree
 *
 * @details The Kconfig symbol `CONFIG_HAPTICS_CS40L5X_EXTERNAL_BOOST` is included in the macro to
 * exclude the encapsulated code at build time if no device instances are using external boost.
 *
 * @param[in] config Pointer to a @ref cs40l5x_config structure
 *
 * @retval true if an instance has external boost enabled in the devicetree
 * @retval false if an instance has external boost disabled in the devicetree
 */
#define HAPTICS_CS40L5X_EXTERNAL_BOOST(config)                                                     \
	(IS_ENABLED(CONFIG_HAPTICS_CS40L5X_EXTERNAL_BOOST) && config->external_boost != NULL)

/**
 * @brief Compare each pin provided via the `trigger-mapping` property against valid GPIOs
 *
 * @param[in] inst `DT_DRV_COMPAT` instance number
 * @param[in] prop lowercase-and-underscores property name
 * @param[in] idx logical index into @p prop
 */
#define HAPTICS_CS40L5X_TRIGGER_MAPPING(inst, prop, idx)                                           \
	(FOR_EACH_FIXED_ARG(IS_EQ, (||), DT_PROP_BY_IDX(inst, prop, idx), CS40L5X_GPIO3,           \
			    CS40L5X_GPIO4, CS40L5X_GPIO5, CS40L5X_GPIO6, CS40L5X_GPIO10,           \
			    CS40L5X_GPIO11, CS40L5X_GPIO12, CS40L5X_GPIO13))

/**
 * @brief Verifies that devicetree properties are valid for CS40L5x drivers
 *
 * @param[in] inst `DT_DRV_COMPAT` instance number
 */
#define HAPTICS_CS40L5X_BUILD_ASSERTS(inst)                                                        \
	BUILD_ASSERT(COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, trigger_mapping),			   \
				 (DT_INST_FOREACH_PROP_ELEM_SEP(inst, trigger_mapping,             \
						   HAPTICS_CS40L5X_TRIGGER_MAPPING, (&&))),	   \
				 (true)),       \
			"the pins provided to trigger_mapping must match a configurable GPIO");

/**
 * @brief Configuration details common to CS40L5x haptic drivers
 *
 * @details CS40L5x uniquely supports external boost configurations because it supports higher
 * voltages than other haptics drivers.
 */
struct cs40l5x_config {
	const struct cirrus_config common;         /**< Common Cirrus Logic configuration. */
	const struct device *const external_boost; /**< External boost regulator. */
};

/**
 * @brief Data common to CS40L5x haptic drivers
 */
struct cs40l5x_data {
	struct cirrus_data common;                       /**< Common Cirrus Logic data. */
	struct k_sem calibration_semaphore;              /**< Calibration synchronization. */
	uint32_t output;                                 /**< Mailbox playback selection. */
	bool custom_effects[CS40L5X_NUM_CUSTOM_EFFECTS]; /**< Statuses for custom effects. */
};

/** @endcond */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* ZEPHYR_DRIVERS_HAPTICS_CIRRUS_CS40L5X_H_ */
