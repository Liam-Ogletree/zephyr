/*
 * Copyright (c) 2026 Cirrus Logic, Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_HAPTICS_CIRRUS_CS40L26_H_
#define ZEPHYR_DRIVERS_HAPTICS_CIRRUS_CS40L26_H_

#include "cirrus.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/** @cond INTERNAL_HIDDEN */

/** Supported devices */
#define CS40L26_DEVID_A1  0x40A260U
#define CS40L26_REVID_A1  0xA1U
#define CS40L27_DEVID_B2  0x40A270U
#define CS40L27_REVID_B2  0xB2U
#define CS40L26_OTPID_MIN 0x1U
#define CS40L26_OTPID_MAX 0xEU

/** Non-deterministic firmware registers */
#define CS40L26_REG_PM_TIMER_TIMEOUT_TICKS      0
#define CS40L26_REG_PM_ACTIVE_TIMEOUT           1
#define CS40L26_REG_PM_STDBY_TIMEOUT            2
#define CS40L26_REG_PM_POWER_ON_SEQUENCE        3
#define CS40L26_REG_DYNAMIC_F0_ENABLED          4
#define CS40L26_REG_HALO_STATE                  5
#define CS40L26_REG_RE_EST_STATUS               6
#define CS40L26_REG_VIBEGEN_F0_OTP_STORED       7
#define CS40L26_REG_VIBEGEN_REDC_OTP_STORED     8
#define CS40L26_REG_VIBEGEN_COMPENSATION_ENABLE 9
#define CS40L26_REG_F0_EST_REDC                 10
#define CS40L26_REG_F0_EST                      11
#define CS40L26_REG_LOGGER_ENABLE               12
#define CS40L26_REG_LOGGER_DATA                 13
#define CS40L26_REG_BUZZ_FREQ                   14
#define CS40L26_REG_BUZZ_LEVEL                  15
#define CS40L26_REG_BUZZ_DURATION               16
#define CS40L26_REG_MAILBOX_QUEUE_WT            17
#define CS40L26_REG_MAILBOX_QUEUE_RD            18
#define CS40L26_REG_MAILBOX_STATUS              19
#define CS40L26_REG_SOURCE_ATTENUATION          20
#define CS40L26_REG_GPIO_EVENT_BASE             21

/** Configurable GPIOs */
#define CS40L26_GPIO1 1
#define CS40L26_GPIO2 2
#define CS40L26_GPIO3 3
#define CS40L26_GPIO4 4

/**
 * @brief Compare each pin provided via the `trigger-mapping` property against valid GPIOs
 *
 * @param[in] inst `DT_DRV_COMPAT` instance number
 * @param[in] prop lowercase-and-underscores property name
 * @param[in] idx logical index into @p prop
 */
#define HAPTICS_CS40L26_TRIGGER_MAPPING(inst, prop, idx)                                           \
	(FOR_EACH_FIXED_ARG(IS_EQ, (||), DT_PROP_BY_IDX(inst, prop, idx), CS40L26_GPIO1,           \
			    CS40L26_GPIO2, CS40L26_GPIO3, CS40L26_GPIO4))

/**
 * @brief Verifies that devicetree properties and Kconfig symbols are valid for CS40L26/27 drivers
 *
 * @param[in] inst `DT_DRV_COMPAT` instance number
 */
#define HAPTICS_CS40L26_BUILD_ASSERTS(inst)                                                        \
	BUILD_ASSERT(COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, trigger_mapping),			   \
				 (DT_INST_FOREACH_PROP_ELEM_SEP(inst, trigger_mapping,             \
						   HAPTICS_CS40L26_TRIGGER_MAPPING, (&&))),	   \
				 (true)),       \
			"the pins provided to trigger_mapping must match a configurable GPIO");

/**
 * @brief Data common to CS40L26/27 haptic drivers
 */
struct cs40l26_data {
	struct cirrus_data common;          /**< Common Cirrus Logic data. */
	struct k_sem calibration_semaphore; /**< Calibration synchronization. */
	uint32_t output;                    /**< Mailbox playback selection. */
};

/** @endcond */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* ZEPHYR_DRIVERS_HAPTICS_CIRRUS_CS40L26_H_ */
