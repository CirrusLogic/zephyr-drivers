/*
 * Copyright 2023 Cirrus Logic, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_SBS_CHARGER_H_
#define ZEPHYR_DRIVERS_SENSOR_SBS_CHARGER_H_

#include <stdint.h>
#include <zephyr/drivers/i2c.h>

#define SBS_CHARGER_REG_SPEC_INFO                   0x11
#define SBS_CHARGER_REG_STATUS                      0x13
#define SBS_CHARGER_REG_ALARM_WARNING               0x16

#define SBS_CHARGER_STATUS_CHARGE_INHIBITED         BIT(1)
#define SBS_CHARGER_STATUS_RES_COLD                 BIT(9)
#define SBS_CHARGER_STATUS_RES_HOT                  BIT(10)
#define SBS_CHARGER_STATUS_BATTERY_PRESENT          BIT(14)
#define SBS_CHARGER_STATUS_AC_PRESENT               BIT(15)

#define SBS_CHARGER_POLL_TIME   500

struct sbs_charger_config {
	struct i2c_dt_spec i2c;
};

#endif
