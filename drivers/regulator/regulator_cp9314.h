/*
 * Copyright (c) 2023 Cirrus Logic, Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_REGULATOR_CP9314_H_
#define ZEPHYR_DRIVERS_REGULATOR_CP9314_H_

#include "zephyr/drivers/gpio.h"
#include "zephyr/drivers/i2c.h"
#include "zephyr/drivers/regulator.h"

#define CP9314_REG_DEVICE_ID 0x0
#define CP9314_DEV_ID        0xA4

#define CP9314_REG_VOUT_UVP   0x2
#define CP9314_VOUT_UVP_DIS_0 BIT(7)
#define CP9314_VOUT_UVP_DIS_1 BIT(3)
#define CP9314_VOUT_UVP_DIS   CP9314_VOUT_UVP_DIS_0 | CP9314_VOUT_UVP_DIS_1
#define CP9314_VOUT_UVP       GENMASK(1, 0)

#define CP9314_REG_OPTION_REG_1 0x3
#define CP9314_LB1_DELAY_CFG    GENMASK(5, 4)
#define CP9314_LB1_DELTA_CFG_0  GENMASK(3, 0)

#define CP9314_REG_OPTION_REG_2     0x4
#define CP9314_LB2_DELTA_CFG_0      GENMASK(7, 5)
#define CP9314_MODE_CTRL_MIN_FREQ_0 GENMASK(2, 0)

#define CP9314_REG_IIN_OCP   0x5
#define CP9314_IIN_OCP_DIS   BIT(7)
#define CP9314_TM_IIN_OC_CFG GENMASK(2, 0)

#define CP9314_REG_IIN_PEAK_OCP 0x6
#define CP9314_IIN_PEAK_OCP_DIS BIT(7)
#define CP9314_IIN_PEAK_OCP     GENMASK(2, 0)

#define CP9314_REG_VIN2OUT_OVP 0x7
#define CP9314_VIN2OUT_OVP     GENMASK(1, 0)

#define CP9314_REG_VIN2OUT_UVP 0x8
#define CP9314_VIN2OUT_UVP     GENMASK(1, 0)

#define CP9314_REG_CONVERTER    0x9
#define CP9314_FASTSHDN_PIN_STS BIT(6)
#define CP9314_PGOOD_PIN_STS    BIT(5)
#define CP9314_ACTIVE_STS       BIT(1)

#define CP9314_REG_CTRL1    0xA
#define CP9314_CP_EN        BIT(7)
#define CP9314_MODE_CTRL_EN BIT(3)

#define CP9314_REG_CTRL4        0xD
#define CP9314_SYNC_FUNCTION_EN BIT(7)
#define CP9314_SYNC_HOST_EN     BIT(6)
#define CP9314_FRC_SYNC_MODE    BIT(5)
#define CP9314_FRC_OP_MODE      BIT(3)
#define CP9314_MODE_MASK        GENMASK(2, 0)
#define CP9314_MODE_2TO1        1
#define CP9314_MODE_3TO1        2

#define CP9314_REG_LION_CFG_1  0x31
#define CP9314_LB2_DELTA_CFG_1 GENMASK(7, 5)

#define CP9314_REG_LION_INT_MASK_2 0x32
#define CP9314_CLEAR_INT           BIT(6)

#define CP9314_REG_LION_CFG_3        0x34
#define CP9314_LB_MIN_FREQ_SEL_0     GENMASK(7, 6)
#define CP9314_MODE_CTRL_UPDATE_BW_1 GENMASK(5, 3)

#define CP9314_REG_LB_CTRL       0x38
#define CP9314_LB1_DELTA_CFG_1   GENMASK(6, 3)
#define CP9314_LB_MIN_FREQ_SEL_1 GENMASK(2, 1)

#define CP9314_REG_CRUS_CTRL       0x40
#define CP9314_CRUS_KEY_LOCK       0x0
#define CP9314_CRUS_KEY_UNLOCK     0xAA
#define CP9314_CRUS_KEY_SOFT_RESET 0xC6

#define CP9314_REG_TRIM_5  0x46
#define CP9314_CSI_CHOP_EN BIT(2)

#define CP9314_REG_TRIM_8            0x49
#define CP9314_MODE_CTRL_UPDATE_BW_0 GENMASK(2, 0)

#define CP9314_REG_TRIM_9         0x4A
#define CP9314_FORCE_KEY_POLARITY BIT(2)
#define CP9314_TM_KEY_POLARITY    BIT(1)
#define CP9314_KEY_ACTIVE_LOW     0
#define CP9314_KEY_ACTIVE_HIGH    CP9314_TM_KEY_POLARITY

#define CP9314_REG_BST_CP_PD_CFG 0x58
#define CP9314_LB1_BLANK_CFG     BIT(5)

#define CP9314_REG_CFG_9            0x59
#define CP9314_VOUT_PCHG_TIME_CFG_0 GENMASK(2, 1)

#define CP9314_REG_CFG_10           0x5A
#define CP9314_VOUT_PCHG_TIME_CFG_1 GENMASK(1, 0)

#define CP9314_REG_BC_STS_C  0x62
#define CP9314_CHIP_REV_MASK GENMASK(7, 4)
#define CP9314_CHIP_REV_B0   0x10

#define CP9314_REG_FORCE_SC_MISC 0x69
#define CP9314_FORCE_CSI_EN      BIT(0)

#define CP9314_REG_TSBAT_CTRL     0x72
#define CP9314_LB1_STOP_PHASE_SEL BIT(4)

#define CP9314_REG_TEST_MODE_CTRL 0x66
#define CP9314_SOFT_RESET_REQ     BIT(0)

#define CP9314_REG_LION_COMP_CTRL_1 0x79
#define CP9314_VIN_SWITCH_OK_DIS_0  BIT(3)
#define CP9314_VOUT_OV_CFG_0        GENMASK(5, 4)
#define CP9314_VIN_SWITCH_OK_CFG    GENMASK(1, 0)

#define CP9314_REG_LION_COMP_CTRL_2 0x7A
#define CP9314_VOUT_OV_CFG_1        GENMASK(3, 2)

#define CP9314_REG_LION_COMP_CTRL_3 0x7B
#define CP9314_VIN_OV_CFG_0         GENMASK(4, 3)
#define CP9314_VIN_OV_CFG_1         GENMASK(1, 0)
#define CP9314_VIN_OV_CFG           CP9314_VIN_OV_CFG_0 | CP9314_VIN_OV_CFG_1

#define CP9314_REG_LION_COMP_CTRL_4 0x7C
#define CP9314_FORCE_IIN_OC_CFG     BIT(1)
#define CP9314_VIN_SWITCH_OK_DIS_1  BIT(5)

#define CP9314_REG_PTE_REG_2 0x8B
#define CP9314_PTE_2_MASK	GENMASK(7, 5)
#define CP9314_PTE_2_OTP_1	0x0
#define CP9314_PTE_2_OTP_2	0x1

#define CP9314_FAULT1_STS 0x9A
#define CP9314_VIN_OV_STS BIT(4)

#define CP9314_SYS_STS    0x98
#define CP9314_VIN_UV_STS BIT(7)

#define CP9314_REG_TM_SEQ_CTRL_1 0xAA
#define CP9314_TM_CSI_EN         BIT(5)

enum cp9314_sync_roles {
	CP9314_ROLE_HOST,
	CP9314_ROLE_DEV2,
	CP9314_ROLE_DEV3,
	CP9314_ROLE_DEV4,
	CP9314_ROLE_STANDALONE,
};

struct regulator_cp9314_config {
	struct regulator_common_config common;
	struct i2c_dt_spec i2c;
	struct gpio_dt_spec en_pin;
	uint8_t sync_role;
};

struct regulator_cp9314_data {
	struct regulator_common_data data;
	uint8_t rev_id;
	uint8_t op_mode;
};

#endif
