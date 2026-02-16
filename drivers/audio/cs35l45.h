/*
 * Copyright (c) 2026 Cirrus Logic, Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_AUDIO_CODEC_CS35L45_H_
#define ZEPHYR_INCLUDE_DRIVERS_AUDIO_CODEC_CS35L45_H_

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/audio/codec.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log_instance.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/**
 * @brief Return pointer to control port instance
 *
 * @param dev Pointer to the device structure for the haptic device instance
 *
 * @return Returns a pointer to the control port device structure
 */
typedef struct device *(*cs35l45_io_bus_get_device)(const struct device *const dev);

/**
 * @brief Function wrapper for @ref i2c_is_ready_dt() or @ref spi_is_ready_dt()
 *
 * @param dev Pointer to the device structure for the haptic device instance
 *
 * @retval true control port is ready for use
 * @retval false control port is not ready for use
 */
typedef bool (*cs35l45_io_bus_is_ready)(const struct device *const dev);

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
typedef int (*cs35l45_io_bus_read)(const struct device *const dev, const uint32_t addr,
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
typedef int (*cs35l45_io_bus_write)(const struct device *const dev, uint32_t *const tx,
				    const uint32_t len);

/**
 * @brief Control port I/O functions
 */
struct cs35l45_bus_io {
	/**< Get control device instance for PM runtime usage */
	cs35l45_io_bus_get_device get_device;
	/**< Check if control port device is ready */
	cs35l45_io_bus_is_ready is_ready;
	/**< Read from the device */
	cs35l45_io_bus_read read;
	/**< Write to the device  */
	cs35l45_io_bus_write write;
};

#if CONFIG_AUDIO_CODEC_CS35L45_I2C
/**
 * @brief Expose control port I/O functions for CS35L45 I2C-based driver
 */
extern const struct cs35l45_bus_io cs35l45_bus_io_i2c;
#endif /* CONFIG_AUDIO_CODEC_CS35L45_I2C */

#if CONFIG_AUDIO_CODEC_CS35L45_SPI
/**
 * @brief Expose control port I/O functions for CS35L45 SPI-based driver
 */
extern const struct cs35l45_bus_io cs35l45_bus_io_spi;
#endif /* CONFIG_AUDIO_CODEC_CS35L45_SPI */

/**
 * @brief Structure to store control port devices
 *
 * @details Note that I2C and SPI control port structures will be included if there are multiple
 * devices in the devicetree and there is at least one device on both I2C and SPI.
 */
union cs35l45_bus {
#if CONFIG_AUDIO_CODEC_CS35L45_I2C
	/**< I2C-based control port */
	struct i2c_dt_spec i2c;
#endif /* CONFIG_AUDIO_CODEC_CS35L45_I2C */
#if CONFIG_AUDIO_CODEC_CS35L45_SPI
	/**< SPI-based control port */
	struct spi_dt_spec spi;
#endif /* CONFIG_AUDIO_CODEC_CS35L45_SPI */
};

struct cs35l45_config {
	/**< Pointer to CS35L45 device instance */
	const struct device *const dev;
	/**< Pointer to data structure for CS35L45 device instance */
	struct cs35l45_data *const data;
	/**< Logger configuration for instance-based logging */
	LOG_INSTANCE_PTR_DECLARE(log);
	/**< Control port devices */
	const union cs35l45_bus bus;
	/**< Control port I/O functions */
	const struct cs35l45_bus_io *const bus_io;
	/**< Optional GPIO for hardware resets */
	struct gpio_dt_spec reset_gpio;
	/**< Optional GPIO for hardware and DSP interrupts */
	struct gpio_dt_spec int_gpio;
	/**< Audio serial port SDOUT Hi-Z control */
	uint8_t asp_sdout_hiz_ctrl;
	/**< Analog gain setting */
	uint8_t amp_gain_pcm;
};

struct cs35l45_data {
	/**< Pointer to CS35L45 device instance */
	const struct device *const dev;
	/**< Pointer to configuration structure for CS35L45 device instance */
	const struct cs35l45_config *const config;
	/**< Callback handler for interrupt processing */
	struct gpio_callback interrupt_callback;
	/**< Worker for debounced interrupt processing */
	struct k_work_delayable interrupt_worker;
	/**< Application-provided callback to recover from fatal hardware errors */
	void (*error_callback)(const struct device *const dev, const uint32_t errors);
};

/** @} */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
