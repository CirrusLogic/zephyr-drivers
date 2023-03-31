/*
 * Copyright 2023 Cirrus Logic, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT sbs_sbs_charger

#include "sbs_charger.h"

#include <zephyr/drivers/charger.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

LOG_MODULE_REGISTER(sbs_charger);

static int sbs_cmd_reg_read(const struct device *dev, uint8_t reg_addr, uint16_t *val)
{
	const struct sbs_charger_config *cfg;
	uint8_t i2c_data[2];
	int status;

	cfg = dev->config;
	status = i2c_burst_read_dt(&cfg->i2c, reg_addr, i2c_data, ARRAY_SIZE(i2c_data));
	if (status < 0) {
		LOG_ERR("Unable to read register");
		return status;
	}

	*val = sys_get_le16(i2c_data);

	return 0;
}

static int sbs_charger_get_prop(const struct device *dev, struct charger_get_property *prop)
{
	uint16_t val = 0;
	int ret = 0;

	switch (prop->property_type) {
	case CHARGER_HEALTH:
		ret = sbs_cmd_reg_read(dev, SBS_CHARGER_REG_STATUS, &val);
		if (val & SBS_CHARGER_STATUS_RES_COLD)
			prop->value.health = CHARGER_HEALTH_COLD;
		else if (val & SBS_CHARGER_STATUS_RES_HOT)
			prop->value.health = CHARGER_HEALTH_OVERHEAT;
		else
			prop->value.health = CHARGER_HEALTH_GOOD;

		break;
	case CHARGER_ONLINE:
		ret = sbs_cmd_reg_read(dev, SBS_CHARGER_REG_STATUS, &val);
		if (val & SBS_CHARGER_STATUS_AC_PRESENT)
			prop->value.online = CHARGER_ONLINE_FIXED;
		else
			prop->value.online = CHARGER_ONLINE_OFFLINE;

		break;
	case CHARGER_PRESENT:
		ret = sbs_cmd_reg_read(dev, SBS_CHARGER_REG_STATUS, &val);
		prop->value.present = !!(val & SBS_CHARGER_STATUS_BATTERY_PRESENT);

		break;
	case CHARGER_STATUS:
		ret = sbs_cmd_reg_read(dev, SBS_CHARGER_REG_STATUS, &val);
		if (!(val & SBS_CHARGER_STATUS_BATTERY_PRESENT))
			prop->value.status = CHARGER_STATUS_NOT_CHARGING;
		else if (val & SBS_CHARGER_STATUS_AC_PRESENT &&
			 !(val & SBS_CHARGER_STATUS_CHARGE_INHIBITED))
			prop->value.status = CHARGER_STATUS_CHARGING;
		else
			prop->value.status = CHARGER_STATUS_DISCHARGING;
		break;
	default:
		ret = -ENOTSUP;
	}

	prop->err = ret;

	return ret;
}

static int sbs_charger_get_props(const struct device *dev, struct charger_get_property *props,
				 size_t len)
{
	int err_count = 0;

	for (int i = 0; i < len; i++) {
		int ret = sbs_charger_get_prop(dev, props + i);

		err_count += ret ? 1 : 0;
	}

	err_count = (err_count == len) ? -1 : err_count;

	return err_count;
}

/**
 * @brief initialize the fuel gauge
 *
 * @return 0 for success
 */
static int sbs_charger_init(const struct device *dev)
{
	const struct sbs_charger_config *cfg;

	cfg = dev->config;

	if (!device_is_ready(cfg->i2c.bus)) {
		LOG_ERR("Bus device is not ready");
		return -ENODEV;
	}

	return 0;
}

static const struct charger_driver_api sbs_charger_driver_api = {
	.get_property = &sbs_charger_get_props,
};

#define SBS_CHARGER_INIT(index)	\
								\
	static const struct sbs_charger_config sbs_charger_config_##index = {	\
		.i2c = I2C_DT_SPEC_INST_GET(index),	\
	};	\
		\
	DEVICE_DT_INST_DEFINE(index, &sbs_charger_init, NULL, NULL,	\
						  &sbs_charger_config_##index,	\
						  POST_KERNEL, 90, &sbs_charger_driver_api);

DT_INST_FOREACH_STATUS_OKAY(SBS_CHARGER_INIT)
