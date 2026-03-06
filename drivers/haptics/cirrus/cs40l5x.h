/*
 * Copyright (c) 2025 Cirrus Logic, Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_HAPTICS_CIRRUS_CS40L5X_H_
#define ZEPHYR_DRIVERS_HAPTICS_CIRRUS_CS40L5X_H_

#include <zephyr/drivers/haptics/cs40l5x.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/**
 * @brief Calibration data for click compensation
 */
struct cs40l5x_calibration {
	/**< Coil DC resistance in signed Q6.17 format (Ohms * (24 / 2.9)) */
	uint32_t redc;
	/**< Resonant frequency in signed Q9.14 format (Hz) */
	uint32_t f0;
};

/**
 * @brief Structure to store GPIO trigger configurations
 */
struct cs40l5x_trigger_config {
	/**< Wavetable source for desired haptic effect */
	enum cs40l5x_bank bank;
	/**< Wavetable index for desired haptic effect */
	uint8_t index;
	/**< Attenuation to be applied to haptic effect */
	int attenuation;
	/**< Offset register address that stores the GPIO trigger configuration */
	uint8_t address;
};

/**
 * @brief Structure to store and handle trigger GPIOs
 */
struct cs40l5x_trigger_gpios {
	/**< Array of GPIOs provided via devicetree property 'trigger-gpios' */
	struct gpio_dt_spec *gpio;
	/**< Number of GPIOs provided via devicetree property 'trigger-gpios' */
	const uint8_t num_gpio;
	/**< Trigger configurations for rising-edge events */
	struct cs40l5x_trigger_config *rising_edge;
	/**< Trigger configurations for falling-edge events */
	struct cs40l5x_trigger_config *falling_edge;
	/**< GPIO statuses */
	bool *ready;
};

/**
 * @brief Return pointer to control port instance
 *
 * @param dev Pointer to the device structure for the haptic device instance
 *
 * @return Returns a pointer to the control port device structure
 */
typedef struct device *(*cs40l5x_io_bus_get_device)(const struct device *const dev);

/**
 * @brief Function wrapper for @ref i2c_is_ready_dt() or @ref spi_is_ready_dt()
 *
 * @param dev Pointer to the device structure for the haptic device instance
 *
 * @retval true control port is ready for use
 * @retval false control port is not ready for use
 */
typedef bool (*cs40l5x_io_bus_is_ready)(const struct device *const dev);

/**
 * @brief Function wrapper for @ref i2c_write_read_dt() or @ref spi_read_dt()
 *
 * @param dev Pointer to the device structure for the haptic device instance
 * @param addr Starting register address
 * @param rx Pointer to unsigned 32-bit storage (value or array) to store read values
 * @param len Number of registers to read
 *
 * @return a value from @ref i2c_write_read_dt() or @ref spi_read_dt()
 */
typedef int (*cs40l5x_io_bus_read)(const struct device *const dev, const uint32_t addr,
				   uint32_t *const rx, const uint32_t len);

/**
 * @brief Function wrapper for @ref i2c_write_dt() or @ref spi_read_dt()
 *
 * @param dev Pointer to the device structure for the haptic device instance
 * @param tx Unsigned 32-bit array with the base register address followed by values to write
 * @param len Pointer to unsigned 32-bit storage (value or array) to store read values
 *
 * @return a value from @ref i2c_write_dt() or @ref spi_write_dt()
 */
typedef int (*cs40l5x_io_bus_write)(const struct device *const dev, uint32_t *const tx,
				    const uint32_t len);

/**
 * @brief Control port I/O functions
 */
struct cs40l5x_bus_io {
	/**< Get control device instance for PM runtime usage */
	cs40l5x_io_bus_get_device get_device;
	/**< Check if control port device is ready */
	cs40l5x_io_bus_is_ready is_ready;
	/**< Read from the device */
	cs40l5x_io_bus_read read;
	/**< Write to the device  */
	cs40l5x_io_bus_write write;
};

#if CONFIG_HAPTICS_CS40L5X_I2C
/**
 * @brief Expose control port I/O functions for CS40L5x I2C-based driver
 */
extern const struct cs40l5x_bus_io cs40l5x_bus_io_i2c;
#endif /* CONFIG_HAPTICS_CS40L5X_I2C */

#if CONFIG_HAPTICS_CS40L5X_SPI
/**
 * @brief Expose control port I/O functions for CS40L5x SPI-based driver
 */
extern const struct cs40l5x_bus_io cs40l5x_bus_io_spi;
#endif /* CONFIG_HAPTICS_CS40L5X_SPI */

/**
 * @brief Structure to store control port devices
 *
 * @details Note that I2C and SPI control port structures will be included if there are multiple
 * devices in the devicetree and there is at least one device on both I2C and SPI.
 */
struct cs40l5x_bus {
#if CONFIG_HAPTICS_CS40L5X_I2C
	/**< I2C-based control port */
	struct i2c_dt_spec i2c;
#endif /* CONFIG_HAPTICS_CS40L5X_I2C */
#if CONFIG_HAPTICS_CS40L5X_SPI
	/**< SPI-based control port */
	struct spi_dt_spec spi;
#endif /* CONFIG_HAPTICS_CS40L5X_SPI */
};

/**
 * @brief CS40L5x configuration structure for a device instance
 */
struct cs40l5x_config {
	/**< Pointer to CS40L5x device instance */
	const struct device *const dev;
	/**< Logger configuration for instance-based logging */
	LOG_INSTANCE_PTR_DECLARE(log);
	/**< Device ID corresponding to the part number */
	const uint32_t dev_id;
	/**< Control port devices */
	const struct cs40l5x_bus bus;
	/**< Control port I/O functions */
	const struct cs40l5x_bus_io *const bus_io;
	/**< Required GPIO for hardware resets */
	struct gpio_dt_spec reset_gpio;
	/**< Optional GPIO for hardware and DSP interrupts */
	struct gpio_dt_spec interrupt_gpio;
	/**< Optional GPIOs for edge-triggered haptic effects */
	struct cs40l5x_trigger_gpios trigger_gpios;
	/**< Optional flash storage, configurable via devicetree property 'flash-storage' */
	const struct device *flash;
	/**< Optional flash register offset, configurable via devicetree property 'flash-offset' */
	const off_t flash_offset;
	/**< Boost setting, configurable via devicetree property 'external-boost' */
	const struct device *const external_boost;
};

/**
 * @brief CS40L5x data structure for a device instance
 */
struct cs40l5x_data {
	/**< Pointer to CS40L5x device instance */
	const struct device *const dev;
	/**< Lock to deconflict haptic playback, device calibration, and configuration changes */
	struct k_mutex lock;
	/**< Revision ID corresponding to the silicon variant */
	uint8_t rev_id;
	/**< Callback handler for interrupt processing */
	struct gpio_callback interrupt_callback;
	/**< Worker for debounced interrupt processing */
	struct k_work_delayable interrupt_worker;
	/**< Callback handler for trigger logging */
	struct gpio_callback trigger_callback;
	/**< Application-provided callback to recover from fatal hardware errors */
	haptics_error_callback_t error_callback;
	/**< Application-provided user data for callback context */
	void *user_data;
	/**< Semaphore used to sequence the calibration routine */
	struct k_sem calibration_semaphore;
	/**< F0 and ReDC data derived from calibration */
	struct cs40l5x_calibration calibration;
	/**< Playback command for mailbox-triggered haptic effects */
	uint32_t output;
	/**< Ring buffer to cache mailbox playback history */
	struct ring_buf rb_mailbox_history;
	/**< Ring buffer to cache trigger playback history */
	struct ring_buf rb_trigger_history;
	/**< Ring buffer storage for cached mailbox playback history */
	uint8_t buf_mailbox_history[CONFIG_HAPTICS_CS40L5X_METADATA_CACHE_LEN];
	/**< Ring buffer storage for cached trigger playback history */
	uint8_t buf_trigger_history[CONFIG_HAPTICS_CS40L5X_METADATA_CACHE_LEN];
	/**< Upload status for custom effects at indices 0 and 1 */
	bool custom_effects[CS40L5X_NUM_CUSTOM_EFFECTS];
	/**< Number of haptic effects playing or suspended */
	int effects_in_flight;
};

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* ZEPHYR_DRIVERS_HAPTICS_CIRRUS_CS40L5X_H_ */
