/*
 * Copyright (c) 2026 Cirrus Logic, Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief API for the CS40L26 haptic driver
 * @ingroup haptics_interface_ext
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_HAPTICS_CS40L26_H_
#define ZEPHYR_INCLUDE_DRIVERS_HAPTICS_CS40L26_H_

#include <zephyr/kernel.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/**
 * @defgroup cs40l26_interface CS40L26
 * @ingroup haptics_interface_ext
 * @brief CS40L26 haptic driver
 * @{
 */

/**
 * @brief Attenuation options for triggered haptic effects
 * @details Provide to @ref cs40l26_configure_trigger().
 */
enum cs40l26_attenuation {
	CS40L26_ATTENUATION_7DB = -7, /**< Configure haptic effect with 7 dB attenuation */
	CS40L26_ATTENUATION_6DB,      /**< Configure haptic effect with 6 dB attenuation */
	CS40L26_ATTENUATION_5DB,      /**< Configure haptic effect with 5 dB attenuation */
	CS40L26_ATTENUATION_4DB,      /**< Configure haptic effect with 4 dB attenuation */
	CS40L26_ATTENUATION_3DB,      /**< Configure haptic effect with 3 dB attenuation */
	CS40L26_ATTENUATION_2DB,      /**< Configure haptic effect with 2 dB attenuation */
	CS40L26_ATTENUATION_1DB,      /**< Configure haptic effect with 1 dB attenuation */
	CS40L26_ATTENUATION_0DB,      /**< Configure haptic effect with no attenuation */
};

/**
 * @brief Wavetable sources for haptic effects
 * @details Provide to @ref cs40l26_select_output().
 */
enum cs40l26_bank {
	CS40L26_ROM_BANK, /**< Playback from the pre-programmed ROM library */
	CS40L26_BUZ_BANK, /**< Playback from buzz source programmed at runtime */
	CS40L26_NO_BANK,  /**< Reserved for driver error handling */
};

/**
 * @brief Run calibration to derive ReDC and F0 values and apply results for click compensation
 *
 * @param[in] dev Pointer to the device structure for haptic device instance
 *
 * @return 0 on success, negative errno value on failure.
 * @retval -EAGAIN ReDC or F0 estimation timed out.
 * @retval -EIO A control port transaction failed.
 */
int cs40l26_calibrate(const struct device *const dev);

/**
 * @brief Configure ROM buzz for haptic playback
 *
 * @details With large amplitudes and insufficient power, it's possible to reach the current limit.
 *
 * @param[in] dev Pointer to the device structure for haptic device instance
 * @param[in] frequency Frequency of haptic effect in Hz (default: 0xA5)
 * @param[in] level Amplitude of haptic effect, where UINT8_MAX is 100% (default: 0x40)
 * @param[in] duration Playback duration in units of 4 milliseconds
 *
 * @return 0 on success, negative errno value on failure.
 * @retval -EIO A control port transaction failed.
 */
int cs40l26_configure_buzz(const struct device *const dev, const uint32_t frequency,
			   const uint8_t level, const uint32_t duration);

/**
 * @brief Select haptic effect triggered via @ref haptics_start_output()
 *
 * @param[in] dev Pointer to the device structure for haptic device instance
 * @param[in] bank Refer to @ref cs40l26_bank
 * @param[in] index Wavetable index for desired haptic effect
 *
 * @return 0 on success, negative errno value on failure.
 * @retval -EINVAL Invalid wavetable source and index provided (e.g., index out of bounds).
 */
int cs40l26_select_output(const struct device *const dev, const enum cs40l26_bank bank,
			  const uint16_t index);

/**
 * @brief Configure gain for haptic effects triggered via @ref haptics_start_output()
 *
 * @param[in] dev Pointer to the device structure for haptic device instance
 * @param[in] gain Gain setting (valid values between 0 and 100)
 *
 * @return 0 on success, negative errno value on failure.
 * @retval -EINVAL Provided gain is greater than 100%.
 * @retval -EIO A control port transaction failed.
 */
int cs40l26_set_gain(const struct device *const dev, const uint8_t gain);

/** @} */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* ZEPHYR_INCLUDE_DRIVERS_HAPTICS_CS40L26_H_ */
