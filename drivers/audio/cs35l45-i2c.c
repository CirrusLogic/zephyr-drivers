/*
 * Copyright (c) 2026, Cirrus Logic, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief I2C handlers for Cirrus Logic CS35L45
 */

#define DT_DRV_COMPAT cirrus_cs35l45

#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>

#include "cs35l45.h"

#define CS35L45_REG_WIDTH  4
#define CS35L45_ADDR_WIDTH 4

static bool cs35l45_is_ready_i2c(const struct device *const dev)
{
	const struct cs35l45_config *const config = dev->config;

	return i2c_is_ready_dt(&config->bus.i2c);
}

static struct device *cs35l45_get_device_i2c(const struct device *const dev)
{
	const struct cs35l45_config *const config = dev->config;

	return (struct device *)&config->bus.i2c.bus;
}

static int cs35l45_read_i2c(const struct device *const dev, uint32_t addr, uint32_t *const rx,
			    const uint32_t len)
{
	const struct cs35l45_config *const config = dev->config;
	int error;

	(void)sys_put_be32(addr, (uint8_t *)&addr);

	error = i2c_write_read_dt(&config->bus.i2c, (uint8_t *)&addr, CS35L45_ADDR_WIDTH,
				  (uint8_t *)rx, len * CS35L45_REG_WIDTH);
	if (error < 0) {
		return error;
	}

	for (int i = 0; i < len; i++) {
		rx[i] = sys_get_be32((uint8_t *)&rx[i]);
	}

	return error;
}

static int cs35l45_write_i2c(const struct device *const dev, uint32_t *const tx, const uint32_t len)
{
	const struct cs35l45_config *const config = dev->config;

	for (int i = 0; i < len; i++) {
		(void)sys_put_be32(tx[i], (uint8_t *)&tx[i]);
	}

	return i2c_write_dt(&config->bus.i2c, (uint8_t *)tx, len * CS35L45_REG_WIDTH);
}

const struct cs35l45_bus_io cs35l45_bus_io_i2c = {
	.is_ready = cs35l45_is_ready_i2c,
	.get_device = cs35l45_get_device_i2c,
	.read = cs35l45_read_i2c,
	.write = cs35l45_write_i2c,
};
