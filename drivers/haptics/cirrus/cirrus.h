/*
 * Copyright (c) 2026 Cirrus Logic, Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_HAPTICS_CIRRUS_CIRRUS_H_
#define ZEPHYR_DRIVERS_HAPTICS_CIRRUS_CIRRUS_H_

#include <sys/types.h>
#include <zephyr/drivers/haptics.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log_instance.h>
#include <zephyr/pm/device.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/toolchain.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/** @cond INTERNAL_HIDDEN */

/** Device-agnostic helper macros */
#define CIRRUS_REGISTER_WIDTH sizeof(uint32_t)
#define CIRRUS_ADDRESS_WIDTH  sizeof(uint32_t)

#define CIRRUS_GPIO_LOGIC_LOW  0
#define CIRRUS_GPIO_LOGIC_HIGH 1
#define CIRRUS_GPIO_INACTIVE   CIRRUS_GPIO_LOGIC_LOW
#define CIRRUS_GPIO_ACTIVE     CIRRUS_GPIO_LOGIC_HIGH

#define CIRRUS_T_DEBOUNCE K_USEC(CONFIG_HAPTICS_CIRRUS_DEBOUNCE_US)
#define CIRRUS_T_POLLING  K_USEC(CONFIG_HAPTICS_CIRRUS_POLLING_INTERVAL_US)
#define CIRRUS_T_TIMEOUT  K_MSEC(CONFIG_HAPTICS_CIRRUS_TIMEOUT_MS)

/**
 * @brief Convert a list of uint32_t values to big-endian at compile-time
 *
 * @param[in] __VA_ARGS__ Comma-separated list of uint32_t values
 */
#define CIRRUS_WRITE_BE32(...) FOR_EACH(sys_cpu_to_be32, (,), __VA_ARGS__)

/**
 * @brief Convert a list of uint32_t values to big-endian at compile-time
 *
 * @details Use with @ref cirrus_multi_write to format register writes.
 *
 * @param[in] __VA_ARGS__ Comma-separated list of uint32_t values
 */
#define CIRRUS_MULTI_WRITE_BE32(...)                                                               \
	.buf = (uint32_t[]){FOR_EACH(sys_cpu_to_be32, (,), __VA_ARGS__)},                          \
	.len = NUM_VA_ARGS(__VA_ARGS__)

/**
 * @brief Storage for multi-register writes
 *
 * @details Intended for bulk register writes with contents known at compile-time. See
 * @ref cirrus_multi_write() and @ref cirrus_raw_multi_write() for usage.
 */
struct cirrus_multi_write {
	uint32_t addr; /**< Hardware register address. */
	uint32_t *buf; /**< Array of register contents to be written. */
	uint32_t len;  /**< Number of registers to be written. */
};

/**
 * @brief Storage for calibration data
 */
struct cirrus_calibration {
	uint32_t redc; /**< Coil DC resistance in signed Q6.17 format (Ohms * (24 / 2.9)). */
	uint32_t f0;   /**< Resonant frequency in signed Q9.14 format (Hz). */
};

/**
 * @brief Wrapper for @ref device_is_ready()
 *
 * @param[in] dev Pointer to the device structure for the haptic device instance
 *
 * @return a value from @ref device_is_ready()
 */
typedef bool (*cirrus_bus_io_is_ready)(const struct device *const dev);

/**
 * @brief Gets a pointer to the device structure for the I/O bus
 *
 * @param[in] dev Pointer to the device structure for the haptic device instance
 *
 * @return Pointer to the device structure for the I/O bus instance
 */
typedef const struct device *(*cirrus_bus_io_get_device)(const struct device *const dev);

/**
 * @brief Wrapper for @ref i2c_write_read_dt() or @ref spi_read_dt()
 *
 * @details Register contents are read from the haptic driver in big-endian format and converted
 * to the system's endianness.
 *
 * @param[in] dev Pointer to the device structure for the haptic device instance
 * @param[in] addr Hardware register address
 * @param[out] rx Array storage for register contents
 * @param[in] len Number of registers to read
 *
 * @return a value from @ref i2c_write_read_dt() or @ref spi_read_dt()
 */
typedef int (*cirrus_bus_io_read)(const struct device *const dev, const uint32_t addr,
				  uint32_t *const rx, const uint32_t len);

/**
 * @brief Wrapper for @ref i2c_transfer_dt() or @ref spi_write_dt()
 *
 * @details If the system is little-endian, array contents are converted to big-endian and then
 * written to the haptic device. Note that the contents of @p tx is modified in-place.
 *
 * @param[in] dev Pointer to the device structure for the haptic device instance
 * @param[in] addr Hardware register address
 * @param[in, out] tx Array storage for register contents
 * @param[in] len Number of registers to write
 *
 * @return a value from @ref i2c_transfer_dt() or @ref spi_write_dt()
 */
typedef int (*cirrus_bus_io_write)(const struct device *const dev, const uint32_t addr,
				   uint32_t *const tx, const uint32_t len);

/**
 * @brief Wrapper for @ref i2c_transfer_dt() or @ref spi_write_dt()
 *
 * @details Array contents are written to the haptic driver without changing the endianness.
 *
 * @param[in] dev Pointer to the device structure for the haptic device instance
 * @param[in] addr Hardware register address
 * @param[in] tx Array storage for register contents
 * @param[in] len Number of registers to write
 *
 * @return a value from @ref i2c_transfer_dt() or @ref spi_write_dt()
 */
typedef int (*cirrus_bus_io_raw_write)(const struct device *const dev, const uint32_t addr,
				       const uint32_t *const tx, const uint32_t len);

/**
 * @brief Storage for I/O bus function pointers
 *
 * @details Used to generically support I/O operations regardless of bus implementation details.
 * See @ref cirrus_bus_io_is_ready(), @ref cirrus_bus_io_get_device(), @ref cirrus_bus_io_read(),
 * @ref cirrus_bus_io_write(), and @ref cirrus_bus_io_raw_write() for details.
 */
struct cirrus_bus_io {
	cirrus_bus_io_is_ready is_ready;
	cirrus_bus_io_get_device get_device;
	cirrus_bus_io_read read;
	cirrus_bus_io_write write;
	cirrus_bus_io_raw_write raw_write;
};

#if CONFIG_HAPTICS_CIRRUS_I2C
/** I2C-based I/O functions */
extern const struct cirrus_bus_io cirrus_bus_io_i2c;
#endif /* CONFIG_HAPTICS_CIRRUS_I2C */

#if CONFIG_HAPTICS_CIRRUS_SPI
/** SPI-based I/O functions */
extern const struct cirrus_bus_io cirrus_bus_io_spi;
#endif /* CONFIG_HAPTICS_CIRRUS_SPI */

/**
 * @brief I/O bus for a haptic device instance
 */
union cirrus_bus {
#if CONFIG_HAPTICS_CIRRUS_I2C
	struct i2c_dt_spec i2c;
#endif /* CONFIG_HAPTICS_CIRRUS_I2C */
#if CONFIG_HAPTICS_CIRRUS_SPI
	struct spi_dt_spec spi;
#endif /* CONFIG_HAPTICS_CIRRUS_SPI */
};

/**
 * @brief Configuration details common to all Cirrus Logic haptic drivers
 *
 * @details This must be the first member of any device-specific configuration structure.
 */
struct cirrus_config {
	LOG_INSTANCE_PTR_DECLARE(log);
	/* Log instance declaration requires blank line. */
	const struct device *const dev;                 /**< Parent device structure. */
	const uint32_t device_id;                       /**< Hardware device ID. */
	const union cirrus_bus bus;                     /**< I/O bus. */
	const struct cirrus_bus_io *const bus_io;       /**< Generic I/O bus functions. */
	const struct gpio_dt_spec reset_gpio;           /**< Hardware reset line. */
	const struct gpio_dt_spec interrupt_gpio;       /**< Hardware interrupt line. */
	const struct gpio_dt_spec *const trigger_gpios; /**< Hardware trigger lines. */
	const int *const trigger_mapping;               /**< Device mapping for triggers. */
	const uint8_t num_triggers;                     /**< Number of trigger lines. */
	const struct device *const flash;               /**< Pointer to flash storage. */
	const off_t flash_offset;                       /**< Byte offset for data storage. */
};

/**
 * @brief Data common to all Cirrus Logic haptic drivers
 *
 * @details This must be the first member of any device-specific data structure.
 */
struct cirrus_data {
	const struct device *const dev;           /**< Pointer to the parent device structure. */
	struct k_mutex lock;                      /**< Mutex lock. */
	struct gpio_callback interrupt_callback;  /**< Interrupt callback. */
	struct k_work_delayable interrupt_worker; /**< Worker to process interrupts. */
	haptics_error_callback_t error_callback;  /**< Application callback for fatal errors. */
	void *user_data;                          /**< Application-provided data for callbacks. */
	struct cirrus_calibration calibration;    /**< Calibration data. */
	uint8_t revision_id;                      /**< Hardware revision ID. */
	uint8_t otp_id;                           /**< Hardware OTP ID. */
};

/**
 * @brief Retrieve flash details from a devicetree
 *
 * @details Supports discrete flash devices and flash memory partitions. Refer to
 * @ref cirrus_config.flash and @ref cirrus_config.flash_offset.
 *
 * @param[in] inst `DT_DRV_COMPAT` instance number
 */
#define HAPTICS_CIRRUS_CONFIG_FLASH(inst)                                                          \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, flash_storage),                                    \
		(COND_CODE_1(DT_PARTITION_EXISTS(DT_INST_PHANDLE(inst, flash_storage)),            \
			(.flash = DEVICE_DT_GET_OR_NULL(DT_MTD_FROM_PARTITION(			   \
				DT_INST_PHANDLE(inst, flash_storage))),				   \
			 .flash_offset = PARTITION_NODE_OFFSET(					   \
				DT_INST_PHANDLE(inst, flash_storage)) +				   \
				DT_INST_PROP_OR(inst, flash_offset, 0)),			   \
			(.flash = DEVICE_DT_GET(DT_INST_PHANDLE(inst, flash_storage)),		   \
			 .flash_offset = DT_INST_PROP_OR(inst, flash_offset, 0)))),		   \
		(.flash = NULL, .flash_offset = 0))

/**
 * @brief Retrieve I/O bus details from a devicetree
 *
 * @details Supports I2C and SPI. Refer to @ref cirrus_config.bus and @ref cirrus_config.bus_io.
 *
 * @param[in] inst `DT_DRV_COMPAT` instance number
 */
#define HAPTICS_CIRRUS_CONFIG_BUS(inst)                                                            \
	COND_CODE_1(DT_INST_ON_BUS(inst, i2c),                                                     \
		(.bus.i2c = I2C_DT_SPEC_INST_GET(inst), .bus_io = &cirrus_bus_io_i2c),             \
		(.bus.spi = SPI_DT_SPEC_INST_GET(inst, SPI_OP_MODE_MASTER),                        \
			.bus_io = &cirrus_bus_io_spi))

/**
 * @brief Retrieve @ref gpio_dt_spec objects from a devicetree
 *
 * @param[in] inst `DT_DRV_COMPAT` instance number
 * @param[in] prop lowercase-and-underscores property name
 * @param[in] idx logical index into @p prop
 */
#define HAPTICS_CIRRUS_GET_GPIO(inst, prop, idx) GPIO_DT_SPEC_GET_BY_IDX(inst, prop, idx),

/**
 * @brief Retrieve trigger GPIOs from a devicetree
 *
 * @param[in] inst `DT_DRV_COMPAT` instance number
 */
#define HAPTICS_CIRRUS_TRIGGER_GPIOS(inst)                                                         \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, trigger_gpios),					   \
		    ((struct gpio_dt_spec[]){DT_INST_FOREACH_PROP_ELEM(inst, trigger_gpios,	   \
								       HAPTICS_CIRRUS_GET_GPIO)}), \
		    (NULL))

/**
 * @brief Retrieve trigger mapping from a devicetree
 *
 * @param[in] inst `DT_DRV_COMPAT` instance number
 */
#define HAPTICS_CIRRUS_TRIGGER_MAPPING(inst)                                                       \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, trigger_mapping),				   \
		    ((int[])DT_INST_PROP(inst, trigger_mapping)), (NULL))

/**
 * @brief Initializes all member variables of @ref cirrus_config from the devicetree
 *
 * @details Refer to @ref cirrus_config.
 *
 * @param[in] inst `DT_DRV_COMPAT` instance number
 * @param[in] id Hardware device ID, see @ref cirrus_config.device_id
 */
#define HAPTICS_CIRRUS_CONFIG(inst, id)                                                            \
	{.dev = DEVICE_DT_INST_GET(inst),                                                          \
	 .device_id = id,                                                                          \
	 .reset_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, reset_gpios, {0}),                           \
	 .interrupt_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, int_gpios, {0}),                         \
	 .trigger_gpios = HAPTICS_CIRRUS_TRIGGER_GPIOS(inst),                                      \
	 .trigger_mapping = HAPTICS_CIRRUS_TRIGGER_MAPPING(inst),                                  \
	 .num_triggers = DT_INST_PROP_LEN_OR(inst, trigger_gpios, 0),                              \
	 HAPTICS_CIRRUS_CONFIG_BUS(inst),                                                          \
	 HAPTICS_CIRRUS_CONFIG_FLASH(inst),                                                        \
	 LOG_INSTANCE_PTR_INIT(log, DT_NODE_FULL_NAME_TOKEN(DT_DRV_INST(inst)), inst)}

/**
 * @brief Initializes a subset of @ref cirrus_data member variables from the devicetree
 *
 * @details Refer to @ref cirrus_data.
 *
 * @param[in] inst `DT_DRV_COMPAT` instance number
 */
#define HAPTICS_CIRRUS_DATA(inst)                                                                  \
	{                                                                                          \
		.dev = DEVICE_DT_INST_GET(inst),                                                   \
		.error_callback = NULL,                                                            \
		.user_data = NULL,                                                                 \
	}

/**
 * @brief Verifies that devicetree properties are valid
 *
 * @details Build asserts are checked in @ref HAPTICS_CIRRUS_INIT().
 *
 * @param[in] inst `DT_DRV_COMPAT` instance number
 */
#define HAPTICS_CIRRUS_BUILD_ASSERTS(inst)                                                         \
	BUILD_ASSERT(IS_EQ(DT_INST_NODE_HAS_PROP(inst, trigger_gpios),                             \
			   DT_INST_NODE_HAS_PROP(inst, trigger_mapping)),                          \
		     "cannot provide trigger_gpios or trigger_mapping separately");                \
	BUILD_ASSERT(COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, trigger_gpios),			   \
				 (IS_EQ(DT_INST_PROP_LEN(inst, trigger_gpios),			   \
					DT_INST_PROP_LEN(inst, trigger_mapping))),		   \
				 (true)								   \
				), "length of trigger_gpios must match length of trigger_mapping");

/**
 * @brief Initializes a device object for the provided driver
 *
 * @param[in] inst `DT_DRV_COMPAT` instance number
 * @param[in] driver Driver name, i.e., 'cs40l26' or 'cs40l5x'
 * @param[in] name Part designator, i.e., 'cs40l50' or 'cs40l51'
 */
#define HAPTICS_CIRRUS_INIT(inst, driver, name)                                                    \
	HAPTICS_CIRRUS_BUILD_ASSERTS(inst);                                                        \
	COND_CODE_1(CONFIG_PM_DEVICE,								   \
		(DEVICE_DT_INST_DEINIT_DEFINE(inst, driver##_init, driver##_deinit,		   \
			PM_DEVICE_DT_INST_GET(inst), &name##_data_##inst, &name##_config_##inst,   \
			POST_KERNEL, CONFIG_HAPTICS_INIT_PRIORITY, &driver##_driver_api)),	   \
		(DEVICE_DT_INST_DEFINE(inst, driver##_init, PM_DEVICE_DT_INST_GET(inst),	   \
			&name##_data_##inst, &name##_config_##inst, POST_KERNEL,		   \
			CONFIG_HAPTICS_INIT_PRIORITY, &driver##_driver_api)))

/**
 * @brief Retrieve a device instance's @ref cirrus_config from @ref cirrus_data
 *
 * @param[in] data Pointer to a @ref cirrus_data structure
 *
 * @return Pointer to a device instance's @ref cirrus_config structure
 */
#define HAPTICS_CIRRUS_CONFIG_FROM_DATA(data)                                                      \
	(const struct cirrus_config *const)((const struct device *const)data->dev)->config

/**
 * @brief Check if power management symbols are enabled to support DSP hibernation
 */
#define HAPTICS_CIRRUS_HIBERNATION                                                                 \
	(IS_ENABLED(CONFIG_PM_DEVICE) && IS_ENABLED(CONFIG_PM_DEVICE_RUNTIME))

/**
 * @brief Check if a device instance has a reset GPIO
 *
 * @details The Kconfig symbol `CONFIG_HAPTICS_CIRRUS_RESET` is included in the macro to exclude
 * the encapsulated code at build time if no device instances have reset GPIOs.
 *
 * @param[in] config Pointer to a @ref cirrus_config structure
 *
 * @retval true if an instance is provided a reset GPIO in the devicetree
 * @retval false if an instance is not provided a reset GPIO in the devicetree
 */
#define HAPTICS_CIRRUS_RESET(config)                                                               \
	(IS_ENABLED(CONFIG_HAPTICS_CIRRUS_RESET) &&                                                \
	 ((const struct cirrus_config *const)config)->reset_gpio.port != NULL)

/**
 * @brief Check if a device instance has an interrupt GPIO
 *
 * @details The Kconfig symbol `CONFIG_HAPTICS_CIRRUS_INTERRUPT` is included in the macro to
 * exclude the encapsulated code at build time if no device instances have interrupt GPIOs.
 *
 * @param[in] config Pointer to a @ref cirrus_config structure
 *
 * @retval true if an instance is provided an interrupt GPIO in the devicetree
 * @retval false if an instance is not provided an interrupt GPIO in the devicetree
 */
#define HAPTICS_CIRRUS_INTERRUPT(config)                                                           \
	(IS_ENABLED(CONFIG_HAPTICS_CIRRUS_INTERRUPT) &&                                            \
	 ((const struct cirrus_config *const)config)->interrupt_gpio.port != NULL)

/**
 * @brief Check if a device instance has any trigger GPIOs
 *
 * @details The Kconfig symbol `CONFIG_HAPTICS_CIRRUS_TRIGGER` is included in the macro to exclude
 * the encapsulated code at build time if no device instances have trigger GPIOs.
 *
 * @param[in] config Pointer to a @ref cirrus_config structure
 *
 * @retval true if an instance is provided trigger GPIO(s) in the devicetree
 * @retval false if an instance is not provided trigger GPIO(s) in the devicetree
 */
#define HAPTICS_CIRRUS_TRIGGER(config)                                                             \
	(IS_ENABLED(CONFIG_HAPTICS_CIRRUS_TRIGGER) &&                                              \
	 ((const struct cirrus_config *const)config)->num_triggers > 0)

/**
 * @brief Check if a device instance has flash storage
 *
 * @details The Kconfig symbol `CONFIG_HAPTICS_CIRRUS_FLASH` is included in the macro to exclude
 * the encapsulated code at build time if no device instances have flash storage.
 *
 * @param[in] config Pointer to a @ref cirrus_config structure
 *
 * @retval true if an instance is provided flash storage in the devicetree
 * @retval false if an instance is not provided flash storage in the devicetree
 */
#define HAPTICS_CIRRUS_FLASH(config)                                                               \
	(IS_ENABLED(CONFIG_HAPTICS_CIRRUS_FLASH) &&                                                \
	 ((const struct cirrus_config *const)config)->flash != NULL)

/**
 * Source attenuation in decibels in signed Q21.2 format. To capture volumes between 1% and 100%,
 * only the least significant byte is required. 0% volume gets a special value.
 */
extern const uint8_t cirrus_source_attenuation[];

/**
 * @brief Refer to @ref cirrus_bus_io_is_ready()
 */
bool cirrus_is_bus_ready(const struct device *const dev);

/**
 * @brief Refer to @ref cirrus_bus_io_get_device()
 */
const struct device *cirrus_get_control_port(const struct device *const dev);

/**
 * @brief Refer to @ref cirrus_bus_io_read()
 */
int cirrus_burst_read(const struct device *const dev, const uint32_t addr, uint32_t *const rx,
		      const uint32_t len);

/**
 * @brief Extension of @ref cirrus_burst_read()
 *
 * @details Read from a single register, i.e., call @ref cirrus_burst_read() with length 1.
 *
 * @param[in] dev Pointer to the device structure for the haptic device instance
 * @param[in] addr Hardware register address
 * @param[out] rx Storage for register contents
 *
 * @return a value from @ref cirrus_burst_read()
 */
int cirrus_read(const struct device *const dev, const uint32_t addr, uint32_t *const rx);

/**
 * @brief Refer to @ref cirrus_bus_io_write()
 */
int cirrus_burst_write(const struct device *const dev, const uint32_t addr, uint32_t *const tx,
		       const uint32_t len);

/**
 * @brief Extension of @ref cirrus_burst_write()
 *
 * @details Write to a single register, i.e., call @ref cirrus_burst_write() with length 1.
 *
 * @param[in] dev Pointer to the device structure for the haptic device instance
 * @param[in] addr Hardware register address
 * @param[in] val Register content to write
 *
 * @return a value from @ref cirrus_burst_write()
 */
int cirrus_write(const struct device *const dev, const uint32_t addr, uint32_t val);

/**
 * @brief Refer to @ref cirrus_bus_io_raw_write()
 */
int cirrus_raw_burst_write(const struct device *const dev, const uint32_t addr,
			   const uint32_t *const tx, const uint32_t len);

/**
 * @brief Extension of @ref cirrus_raw_burst_write()
 *
 * @details Write to a single register without changing endianness of register content, i.e., call
 * @ref cirrus_raw_burst_write() with length 1.
 *
 * @param[in] dev Pointer to the device structure for the haptic device instance
 * @param[in] addr Hardware register address
 * @param[in] val Register content to write
 *
 * @return a value from @ref cirrus_raw_burst_write()
 */
int cirrus_raw_write(const struct device *const dev, const uint32_t addr, const uint32_t val);

/**
 * @brief Write a sequence of multi-register transactions
 *
 * @param[in] dev Pointer to the device structure for the haptic device instance
 * @param[in] multi_write Refer to @ref cirrus_multi_write
 * @param[in] len Number of multi-register sequences to write
 *
 * @retval 0 if success
 * @retval <0 if failure
 */
int cirrus_multi_write(const struct device *const dev,
		       const struct cirrus_multi_write *const multi_write, const uint32_t len);

/**
 * @brief Write a sequence of multi-register transactions
 *
 * @details Similar to @ref cirrus_multi_write() except register contents are written without
 * changing endianness.
 *
 * @param[in] dev Pointer to the device structure for the haptic device instance
 * @param[in] multi_write Refer to @ref cirrus_multi_write
 * @param[in] len Number of multi-register sequences to write
 *
 * @retval 0 if success
 * @retval <0 if failure
 */
int cirrus_raw_multi_write(const struct device *const dev,
			   const struct cirrus_multi_write *const multi_write, const uint32_t len);

/**
 * @brief Poll a hardware register until an expected value is read
 *
 * @param[in] dev Pointer to the device structure for the haptic device instance
 * @param[in] addr Hardware register address
 * @param[in] val Expected value of register contents
 * @param[in] timeout Refer to @ref k_timeout_t
 *
 * @retval 0 if success
 * @retval -ETIMEDOUT if the expected value is not read before the timeout has elapsed
 * @retval -errno another negative error code on failure
 */
int cirrus_poll(const struct device *const dev, const uint32_t addr, const uint32_t val,
		const k_timeout_t timeout);

/**
 * @brief Wake up the device with repeated register reads
 *
 * @param[in] dev Pointer to the device structure for the haptic device instance
 * @param[in] timeout Refer to @ref k_timeout_t
 *
 * @retval 0 if success
 * @retval -ETIMEDOUT if no read operation succeeds before the timeout has elapsed
 */
int cirrus_wakeup(const struct device *const dev, const k_timeout_t timeout);

/**
 * @brief Extension of @ref cirrus_burst_read(), except @p control replaces `addr`
 *
 * @details @p control is not a hardware register address but an index used as a table lookup.
 * Refer to device-specific, private headers and @ref cirrus-firmware.c for details.
 */
int cirrus_firmware_burst_read(const struct device *const dev, const uint32_t control,
			       uint32_t *const rx, const uint32_t len);

/**
 * @brief Similar to @ref cirrus_firmware_burst_read(), except @p offset is added to the address
 * after @p control is used with the lookup table.
 */
int cirrus_firmware_burst_read_offset(const struct device *const dev, const uint32_t control,
				      uint32_t *const rx, const uint32_t len, const off_t offset);

/**
 * @brief Extension of @ref cirrus_read(), except @p control replaces `addr`
 *
 * @details @p control is not a hardware register address but an index used as a table lookup.
 * Refer to device-specific, private headers and @ref cirrus-firmware.c for details.
 */
int cirrus_firmware_read(const struct device *const dev, const uint32_t control,
			 uint32_t *const rx);

/**
 * @brief Similar to @ref cirrus_firmware_read(), except @p offset is added to the address after
 * @p control is used with the lookup table.
 */
int cirrus_firmware_read_offset(const struct device *const dev, const uint32_t control,
				uint32_t *const rx, const off_t offset);

/**
 * @brief Extension of @ref cirrus_burst_write(), except @p control replaces `addr`
 *
 * @details @p control is not a hardware register address but an index used as a table lookup.
 * Refer to device-specific, private headers and @ref cirrus-firmware.c for details.
 */
int cirrus_firmware_burst_write(const struct device *const dev, const uint32_t control,
				uint32_t *const tx, const uint32_t len);

/**
 * @brief Similar to @ref cirrus_firmware_burst_write(), except @p offset is added to the address
 * after @p control is used with the lookup table.
 */
int cirrus_firmware_burst_write_offset(const struct device *const dev, const uint32_t control,
				       uint32_t *const tx, const uint32_t len, const off_t offset);

/**
 * @brief Extension of @ref cirrus_write(), except @p control replaces `addr`
 *
 * @details @p control is not a hardware register address but an index used as a table lookup.
 * Refer to device-specific, private headers and @ref cirrus-firmware.c for details.
 */
int cirrus_firmware_write(const struct device *const dev, const uint32_t control, uint32_t val);

/**
 * @brief Similar to @ref cirrus_firmware_write(), except @p offset is added to the address after
 * @p control is used with the lookup table.
 */
int cirrus_firmware_write_offset(const struct device *const dev, const uint32_t control,
				 uint32_t val, const off_t offset);

/**
 * @brief Extension of @ref cirrus_raw_burst_write(), except @p control replaces `addr`
 *
 * @details @p control is not a hardware register address but an index used as a table lookup.
 * Refer to device-specific, private headers and @ref cirrus-firmware.c for details.
 */
int cirrus_firmware_raw_burst_write(const struct device *const dev, const uint32_t control,
				    const uint32_t *const tx, const uint32_t len);

/**
 * @brief Similar to @ref cirrus_firmware_raw_burst_write(), except @p offset is added to the
 * address after @p control is used with the lookup table.
 */
int cirrus_firmware_raw_burst_write_offset(const struct device *const dev, const uint32_t control,
					   const uint32_t *const tx, const uint32_t len,
					   const off_t offset);

/**
 * @brief Extension of @ref cirrus_raw_write(), except @p control replaces `addr`
 *
 * @details @p control is not a hardware register address but an index used as a table lookup.
 * Refer to device-specific, private headers and @ref cirrus-firmware.c for details.
 */
int cirrus_firmware_raw_write(const struct device *const dev, const uint32_t control,
			      const uint32_t val);

/**
 * @brief Extension of @ref cirrus_multi_write()
 *
 * @details @ref cirrus_multi_write.addr is not a hardware register address in this context but an
 * index used as a table lookup. Refer to device-specific, private headers and
 * @ref cirrus-firmware.c for details.
 */
int cirrus_firmware_multi_write(const struct device *const dev,
				const struct cirrus_multi_write *const multi_write,
				const uint32_t len);

/**
 * @brief Extension of @ref cirrus_raw_multi_write()
 *
 * @details @ref cirrus_multi_write.addr is not a hardware register address in this context but an
 * index used as a table lookup. Refer to device-specific, private headers and
 * @ref cirrus-firmware.c for details.
 */
int cirrus_firmware_raw_multi_write(const struct device *const dev,
				    const struct cirrus_multi_write *const multi_write,
				    const uint32_t len);

/**
 * @brief Extension of @ref cirrus_poll(), except @p control replaces `addr`
 *
 * @details @p control is not a hardware register address but an index used as a table lookup.
 * Refer to device-specific, private headers and @ref cirrus-firmware.c for details.
 */
int cirrus_firmware_poll(const struct device *const dev, const uint32_t control, const uint32_t val,
			 const k_timeout_t timeout);

/**
 * @brief Load calibration data from flash memory
 *
 * @param[in] dev Pointer to the device structure for the haptic device instance
 *
 * @retval 0 if success
 * @retval <0 if failure
 */
int cirrus_load_calibration(const struct device *const dev);

/**
 * @brief Store calibration data to flash memory
 *
 * @param[in] dev Pointer to the device structure for the haptic device instance
 *
 * @retval 0 if success
 * @retval <0 if failure
 */
int cirrus_store_calibration(const struct device *const dev);

/**
 * @brief Execute an application callback, if provided
 *
 * @param[in] dev Pointer to the device structure for the haptic device instance
 * @param[in] error_bitmask Refer to @ref haptics_error_type
 */
void cirrus_error_callback(const struct device *const dev, const uint32_t error_bitmask);

/**
 * @brief Register an application callback for fatal errors
 *
 * @param[in] dev Pointer to the device structure for the haptic device instance
 * @param[in] cb Refer to @ref haptics_error_callback_t
 * @param[in] user_data Data provided to the application if executing a callback
 *
 * @retval 0 if success
 * @retval <0 if failure
 */
int cirrus_register_error_callback(const struct device *dev, haptics_error_callback_t cb,
				   void *const user_data);

/**
 * @brief Callback handler for interrupts
 *
 * @param[in] port Pointer to the device structure for the GPIO device instance
 * @param[in] cb GPIO callback structure
 * @param[in] pins Bitmask of GPIO pins that triggered the interrupt
 */
void cirrus_interrupt_handler(const struct device *port, struct gpio_callback *cb, uint32_t pins);

/**
 * @brief Configure the interrupt GPIO and add a callback handler
 *
 * @details Configures @ref cirrus_config.interrupt_gpio and @ref cirrus_data.interrupt_callback
 * for interrupts.
 *
 * @param[in] dev Pointer to the device structure for the haptic device instance
 *
 * @retval 0 if success
 * @retval <0 if failure
 */
int cirrus_interrupt_config(const struct device *const dev);

/**
 * @brief Configure trigger GPIO(s)
 *
 * @param[in] dev Pointer to the device structure for the haptic device instance
 *
 * @retval 0 if success
 * @retval <0 if failure
 */
int cirrus_trigger_config(const struct device *const dev);

/**
 * @brief Retrieve index of @ref cirrus_config.trigger_gpios matching @p gpio
 *
 * @param[in] dev Pointer to the device structure for the haptic device instance
 * @param[in] gpio Pointer to @ref gpio_dt_spec object
 * @param[out] idx Index for @ref cirrus_config.trigger_mapping
 *
 * @retval 0 if success
 * @retval <0 if failure
 */
int cirrus_get_trigger(const struct device *const dev, const struct gpio_dt_spec *const gpio,
		       uint8_t *const idx);

/**
 * @brief Disconnect the interrupt GPIO and remove the device instance's callback handler
 *
 * @param[in] dev Pointer to the device structure for the haptic device instance
 *
 * @retval 0 if success
 * @retval <0 if failure
 */
int cirrus_interrupt_teardown(const struct device *const dev);

/**
 * @brief Assert and then disconnect the reset GPIO to prevent erroneously powering the device
 *
 * @param[in] dev Pointer to the device structure for the haptic device instance
 *
 * @retval 0 if success
 * @retval <0 if failure
 */
int cirrus_reset_teardown(const struct device *const dev);

/**
 * @brief Initializes member variables of @ref cirrus_config and @ref cirrus_data at run-time
 *
 * @param[in] dev Pointer to the device structure for the haptic device instance
 * @param[in] handler Refer to @ref k_work_handler_t
 *
 * @retval 0 if success
 * @retval <0 if failure
 */
int cirrus_init(const struct device *const dev, k_work_handler_t handler);

/** @endcond */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* ZEPHYR_DRIVERS_HAPTICS_CIRRUS_CIRRUS_H_ */
