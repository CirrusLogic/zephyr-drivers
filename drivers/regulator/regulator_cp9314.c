/*
 * Copyright (c) 2023 Cirrus Logic, Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT cirrus_cp9314

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include "zephyr/kernel.h"
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include "regulator_cp9314.h"

LOG_MODULE_REGISTER(CP9314);

static int regulator_cp9314_set_mode(const struct device *dev, regulator_mode_t mode)
{
	const struct regulator_cp9314_config *config = dev->config;
	struct regulator_cp9314_data *data = dev->data;
	int ret;

	ret = i2c_reg_update_byte_dt(&config->i2c, CP9314_REG_CTRL4, CP9314_MODE_MASK, mode);
	if (ret < 0) {
		return ret;
	}

	data->op_mode = mode;

	return 0;
}

static int regulator_cp9314_get_mode(const struct device *dev, regulator_mode_t *mode)
{
	const struct regulator_cp9314_config *config = dev->config;
	uint8_t reg_val;
	int ret;

	ret = i2c_reg_read_byte_dt(&config->i2c, CP9314_REG_CTRL4, &reg_val);
	if (ret < 0) {
		return ret;
	}

	*mode = reg_val & CP9314_MODE_MASK;

	return 0;
}

static int regulator_cp9314_disable(const struct device *dev)
{
	const struct regulator_cp9314_config *config = dev->config;

	if (config->en_pin.port != NULL) {
		return gpio_pin_set_dt(&config->en_pin, 0);
	}

	return i2c_reg_update_byte_dt(&config->i2c, CP9314_REG_CTRL1, CP9314_CP_EN, 0);
}

static int regulator_cp9314_enable(const struct device *dev)
{
	const struct regulator_cp9314_config *config = dev->config;
	uint8_t value;
	int ret;

	ret = i2c_reg_read_byte_dt(&config->i2c, CP9314_REG_CONVERTER, &value);
	if (ret < 0) {
		return ret;
	}

	if (value & CP9314_ACTIVE_STS) {
		return 0;
	}

	ret = i2c_reg_update_byte_dt(&config->i2c, CP9314_REG_LION_INT_MASK_2, CP9314_CLEAR_INT,
				     CP9314_CLEAR_INT);
	if (ret < 0) {
		return ret;
	}

	ret = i2c_reg_update_byte_dt(&config->i2c, CP9314_REG_LION_INT_MASK_2, CP9314_CLEAR_INT, 0);
	if (ret < 0) {
		return ret;
	}

	if (config->en_pin.port != NULL) {
		return gpio_pin_set_dt(&config->en_pin, 1);
	}

	ret = i2c_reg_update_byte_dt(&config->i2c, CP9314_REG_CTRL1, CP9314_CP_EN, CP9314_CP_EN);
	if (ret < 0) {
		LOG_ERR("Unable to set CP_EN");
		return ret;
	}

	return 0;
}

static int regulator_cp9314_b0_init(const struct device *dev)
{
	const struct regulator_cp9314_config *config = dev->config;
	int ret;

	ret = i2c_reg_write_byte_dt(&config->i2c, CP9314_REG_CRUS_CTRL, CP9314_CRUS_KEY_UNLOCK);
	if (ret < 0) {
		return ret;
	}

	ret = i2c_reg_update_byte_dt(&config->i2c, CP9314_REG_LION_COMP_CTRL_3, CP9314_VIN_OV_CFG,
				     0x1B);
	if (ret < 0) {
		return ret;
	}

	ret = i2c_reg_update_byte_dt(&config->i2c, CP9314_REG_LION_COMP_CTRL_1,
				     CP9314_VOUT_OV_CFG_0, 0x30);
	if (ret < 0) {
		return ret;
	}

	ret = i2c_reg_update_byte_dt(&config->i2c, CP9314_REG_LION_COMP_CTRL_2,
				     CP9314_VOUT_OV_CFG_1, 0xC);
	if (ret < 0) {
		return ret;
	}

	ret = i2c_reg_update_byte_dt(&config->i2c, CP9314_REG_VIN2OUT_OVP, CP9314_VIN2OUT_OVP, 0x2);
	if (ret < 0) {
		return ret;
	}

	ret = i2c_reg_update_byte_dt(&config->i2c, CP9314_REG_VIN2OUT_UVP, CP9314_VIN2OUT_UVP, 0x1);
	if (ret < 0) {
		return ret;
	}

	ret = i2c_reg_update_byte_dt(&config->i2c, CP9314_REG_VOUT_UVP, CP9314_VOUT_UVP_DIS, 0);
	if (ret < 0) {
		return ret;
	}

	ret = i2c_reg_update_byte_dt(&config->i2c, CP9314_REG_VOUT_UVP, CP9314_VOUT_UVP, 0);
	if (ret < 0) {
		return ret;
	}

	ret = i2c_reg_update_byte_dt(&config->i2c, CP9314_REG_LION_COMP_CTRL_1,
				     CP9314_VIN_SWITCH_OK_DIS_0, 0);
	if (ret < 0) {
		return ret;
	}

	ret = i2c_reg_update_byte_dt(&config->i2c, CP9314_REG_LION_COMP_CTRL_4,
				     CP9314_VIN_SWITCH_OK_DIS_1, 0);
	if (ret < 0) {
		return ret;
	}

	ret = i2c_reg_update_byte_dt(&config->i2c, CP9314_REG_LION_COMP_CTRL_1,
				     CP9314_VIN_SWITCH_OK_CFG, 0);
	if (ret < 0) {
		return ret;
	}

	ret = i2c_reg_update_byte_dt(&config->i2c, CP9314_REG_LION_CFG_3, CP9314_LB_MIN_FREQ_SEL_0,
				     0x80);
	if (ret < 0) {
		return ret;
	}

	ret = i2c_reg_update_byte_dt(&config->i2c, CP9314_REG_LB_CTRL, CP9314_LB_MIN_FREQ_SEL_1,
				     0x4);
	if (ret < 0) {
		return ret;
	}

	ret = i2c_reg_update_byte_dt(&config->i2c, CP9314_REG_TRIM_8, CP9314_MODE_CTRL_UPDATE_BW_0,
				     0x2);
	if (ret < 0) {
		return ret;
	}

	ret = i2c_reg_update_byte_dt(&config->i2c, CP9314_REG_LION_CFG_3,
				     CP9314_MODE_CTRL_UPDATE_BW_1, 0x2);
	if (ret < 0) {
		return ret;
	}

	ret = i2c_reg_update_byte_dt(&config->i2c, CP9314_REG_IIN_OCP, CP9314_IIN_OCP_DIS,
				     CP9314_IIN_OCP_DIS);
	if (ret < 0) {
		return ret;
	}

	ret = i2c_reg_update_byte_dt(&config->i2c, CP9314_REG_IIN_PEAK_OCP, CP9314_IIN_PEAK_OCP_DIS,
				     CP9314_IIN_PEAK_OCP_DIS);
	if (ret < 0) {
		return ret;
	}

	return i2c_reg_write_byte_dt(&config->i2c, CP9314_REG_CRUS_CTRL, CP9314_CRUS_KEY_LOCK);
}

static int cp9314_pte_otp_v0_patch(const struct device *dev)
{
	const struct regulator_cp9314_config *config = dev->config;
	int ret;

	ret = i2c_reg_write_byte_dt(&config->i2c, CP9314_REG_CRUS_CTRL, CP9314_CRUS_KEY_UNLOCK);
	if (ret < 0) {
		return ret;
	}

	ret = i2c_reg_update_byte_dt(&config->i2c, CP9314_REG_OPTION_REG_1, CP9314_LB1_DELAY_CFG,
				     0);
	if (ret < 0) {
		return ret;
	}

	ret = i2c_reg_update_byte_dt(&config->i2c, CP9314_REG_BST_CP_PD_CFG, CP9314_LB1_BLANK_CFG,
				     CP9314_LB1_BLANK_CFG);
	if (ret < 0) {
		return ret;
	}

	ret = i2c_reg_update_byte_dt(&config->i2c, CP9314_REG_TSBAT_CTRL, CP9314_LB1_STOP_PHASE_SEL,
				     CP9314_LB1_STOP_PHASE_SEL);
	if (ret < 0) {
		return ret;
	}

	return i2c_reg_write_byte_dt(&config->i2c, CP9314_REG_CRUS_CTRL, CP9314_CRUS_KEY_LOCK);
}

static int cp9314_cfg_sync(const struct device *dev)
{
	const struct regulator_cp9314_config *config = dev->config;
	uint8_t value = 0;
	int ret;

	if (config->sync_role == CP9314_ROLE_HOST) {
		value = CP9314_SYNC_HOST_EN;
	}

	ret = i2c_reg_update_byte_dt(&config->i2c, CP9314_REG_CTRL4, CP9314_SYNC_HOST_EN, value);
	if (ret < 0) {
		return ret;
	}

	ret = i2c_reg_update_byte_dt(&config->i2c, CP9314_REG_CTRL4, CP9314_SYNC_FUNCTION_EN,
				     CP9314_SYNC_FUNCTION_EN);
	if (ret < 0) {
		return ret;
	}

	return i2c_reg_update_byte_dt(&config->i2c, CP9314_REG_CTRL4, CP9314_FRC_SYNC_MODE,
				      CP9314_FRC_SYNC_MODE);
}

static int cp9314_do_soft_reset(const struct device *dev)
{
	const struct regulator_cp9314_config *config = dev->config;
	int ret;

	ret = i2c_reg_write_byte_dt(&config->i2c, CP9314_REG_CRUS_CTRL, CP9314_CRUS_KEY_SOFT_RESET);
	if (ret < 0) {
		return ret;
	}

	ret = i2c_reg_update_byte_dt(&config->i2c, CP9314_REG_TEST_MODE_CTRL, CP9314_SOFT_RESET_REQ,
				     CP9314_SOFT_RESET_REQ);
	if (ret < 0) {
		return ret;
	}

	k_msleep(200);

	return 0;
}

static int regulator_cp9314_cfg_en(const struct device *dev)
{
	const struct regulator_cp9314_config *config = dev->config;
	uint8_t value = CP9314_KEY_ACTIVE_LOW;
	int ret;

	ret = i2c_reg_write_byte_dt(&config->i2c, CP9314_REG_CRUS_CTRL, CP9314_CRUS_KEY_UNLOCK);
	if (ret < 0) {
		return ret;
	}

	if (config->en_pin.dt_flags == GPIO_ACTIVE_HIGH) {
		value = CP9314_KEY_ACTIVE_HIGH;
	}

	ret = i2c_reg_update_byte_dt(&config->i2c, CP9314_REG_TRIM_9, CP9314_TM_KEY_POLARITY,
				     value);
	if (ret < 0) {
		return ret;
	}

	ret = i2c_reg_update_byte_dt(&config->i2c, CP9314_REG_TRIM_9, CP9314_FORCE_KEY_POLARITY,
				     CP9314_FORCE_KEY_POLARITY);
	if (ret < 0) {
		return ret;
	}

	return i2c_reg_write_byte_dt(&config->i2c, CP9314_REG_CRUS_CTRL, CP9314_CRUS_KEY_LOCK);
}

static int regulator_cp9314_otp_init(const struct device *dev)
{
	const struct regulator_cp9314_config *config = dev->config;
	uint8_t value;
	int ret;

	ret = i2c_reg_read_byte_dt(&config->i2c, CP9314_REG_PTE_REG_2, &value);
	if (ret < 0) {
		return ret;
	}

	value = FIELD_GET(CP9314_PTE_2_MASK, value);

	if (value == CP9314_PTE_2_OTP_1) {
		ret = cp9314_pte_otp_v0_patch(dev);
		if (ret < 0) {
			return ret;
		}
	}

	return 0;
}

static int regulator_cp9314_init(const struct device *dev)
{
	const struct regulator_cp9314_config *config = dev->config;
	struct regulator_cp9314_data *data = dev->data;
	uint8_t value;
	int ret;

	if (!device_is_ready(config->i2c.bus)) {
		LOG_ERR("Bus device is not ready");
		return -ENODEV;
	}

	ret = i2c_reg_read_byte_dt(&config->i2c, CP9314_REG_DEVICE_ID, &value);
	if (ret < 0) {
		LOG_ERR("No device found:%d\n", ret);
		return ret;
	}

	if (value != CP9314_DEV_ID) {
		LOG_ERR("Invalid device ID found:0x%x!\n", value);
		return -ENOTSUP;
	}

	ret = cp9314_do_soft_reset(dev);
	if (ret < 0) {
		return ret;
	}

	ret = i2c_reg_read_byte_dt(&config->i2c, CP9314_REG_BC_STS_C, &value);
	if (ret < 0) {
		return ret;
	}

	value &= CP9314_CHIP_REV_MASK;

	switch (value) {
	case CP9314_CHIP_REV_B0:
		LOG_INF("Found CP9314 REV:0x%x\n", value);
		ret = regulator_cp9314_b0_init(dev);
		if (ret < 0) {
			return ret;
		}
		break;
	default:
		LOG_ERR("Invalid CP9314 REV:0x%x\n", value);
		return -ENOTSUP;
	}

	data->rev_id = value;

	ret = regulator_cp9314_otp_init(dev);
	if (ret < 0) {
		return ret;
	}

	ret = regulator_cp9314_cfg_en(dev);
	if (ret < 0) {
		return ret;
	}

	if (config->en_pin.port != NULL) {
		if (!gpio_is_ready_dt(&config->en_pin)) {
			return -ENODEV;
		}

		ret = gpio_pin_configure_dt(&config->en_pin, GPIO_OUTPUT_INACTIVE);
		if (ret < 0) {
			return ret;
		}
	}

	ret = i2c_reg_update_byte_dt(&config->i2c, CP9314_REG_CTRL4, CP9314_FRC_OP_MODE,
				     CP9314_FRC_OP_MODE);
	if (ret < 0) {
		return ret;
	}

	if (config->sync_role != CP9314_ROLE_STANDALONE) {
		ret = cp9314_cfg_sync(dev);
		if (ret < 0) {
			return ret;
		}
	}

	regulator_common_data_init(dev);

	return regulator_common_init(dev, false);
}

static const struct regulator_driver_api api = {
	.enable = regulator_cp9314_enable,
	.disable = regulator_cp9314_disable,
	.get_mode = regulator_cp9314_get_mode,
	.set_mode = regulator_cp9314_set_mode,
};

#define REGULATOR_CP9314_DEFINE(inst)                                                              \
	static struct regulator_cp9314_data data_##inst;                                           \
                                                                                                   \
	static const struct regulator_cp9314_config config_##inst = {                              \
		.common = REGULATOR_DT_INST_COMMON_CONFIG_INIT(inst),                              \
		.i2c = I2C_DT_SPEC_INST_GET(inst),                                                 \
		.en_pin = GPIO_DT_SPEC_INST_GET_OR(inst, cirrus_en_gpios, {}),                     \
		.sync_role = DT_INST_ENUM_IDX(inst, cirrus_sync_role),                             \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, regulator_cp9314_init, NULL, &data_##inst, &config_##inst,     \
			      POST_KERNEL, CONFIG_REGULATOR_CP9314_INIT_PRIORITY, &api);

DT_INST_FOREACH_STATUS_OKAY(REGULATOR_CP9314_DEFINE)
