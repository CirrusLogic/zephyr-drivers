
/*
 * Copyright 2022 Cirrus Logic, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(cp7420, CONFIG_SENSOR_LOG_LEVEL);

#include "cp7420.h"

#define DT_DRV_COMPAT cirrus_cp7420

/**
 * @brief Gets an ADC value and converts the raw data into a value in micro units
 * 
 * @param dev CP7420 device to access
 * @param chan the desired ADC channel
 * @param val converted ADC value
 * @return 0 if successful,
 * @return negative error code from I2C API
 */
static int cp7420_get_adc(const struct device *dev, const enum cp7420_adc chan, int *val)
{
	const struct cp7420_config *const config = dev->config;
	uint8_t read_buf[2] = { 0x0 };
	uint8_t start_addr;
	uint16_t raw_data;
	int ret;

	ret = i2c_reg_update_byte_dt(&config->i2c,
								 ADC_CTRL_1,
								 CP7420_PAUSE_UPDATES,
								 CP7420_PAUSE_UPDATES);
	if (ret) {
		LOG_ERR("Unable to pause ADC: %d\n", ret);
		return ret;
	}

	start_addr = (chan * 2) + ADC_NTC_STATUS;

	ret = i2c_burst_read_dt(&config->i2c, start_addr, read_buf, sizeof(read_buf));
	if (ret) {
		LOG_ERR("Unable to read ADC channel %d: %d\n", chan, ret);
		return ret;
	}

	raw_data = (read_buf[1] << 8) | read_buf[0];

	switch (chan) {
	case CP7420_ADC_VIN:
		*val = raw_data * CP7420_ADC_VIN_STEP_UV;
		break;
	case CP7420_ADC_IIN:
		*val = raw_data * CP7420_ADC_IIN_STEP_UA;
		break;
	case CP7420_ADC_VSYS:
		*val = raw_data * CP7420_ADC_VSYS_STEP_UV;
		break;
	case CP7420_ADC_VBAT:
		*val = raw_data * CP7420_ADC_VBAT_STEP_UV;
		break;
	case CP7420_ADC_IBAT_CHG:
		raw_data &= CP7420_ADC_IBAT_MASK;

		*val = raw_data * CP7420_ADC_IBAT_STEP_UA;

		if (raw_data & CP7420_ADC_IBAT_DISCHG)
			*val *= -1;
 
 		break;
	case CP7420_ADC_NTC:
		*val = raw_data * CP7420_ADC_NTC_STEP_UV;
		break;
	case CP7420_ADC_TDIE:
		*val = raw_data * CP7420_ADC_TDIE_STEP_UC;
		break;
	default:
		return -ENOTSUP;
	}

	return ret;
}

/**
 * @brief Sets the LPM_EN bit
 * 
 * @param dev CP7420 device to access
 * @param en value to be set
 * @return 0 if successful,
 * @return negative error code from I2C API
 */
static int cp7420_set_lpm_en(const struct device *dev, const bool en)
{
	const struct cp7420_config *const config = dev->config;
	struct cp7420_data *data = dev->data;
	uint8_t reg_code;
	int ret;

	if (en)
		reg_code = CP7420_LPM_EN;
	else
		reg_code = 0;

	ret = i2c_reg_update_byte_dt(&config->i2c, OPERATION_CTRL_0, CP7420_LPM_EN, reg_code);
	if (ret) {
		LOG_ERR("Failed to set LPM_EN: %d\n", ret);
		return ret;
	}

	data->lpm_en = en;

	return ret;
}

/**
 * @brief Sets the IIN_REG_EN bit
 * 
 * @param dev CP7420 device to access
 * @param en value to be set
 * @return 0 if successful,
 * @return negative error code from I2C API
 */
static int cp7420_set_iin_reg_en(const struct device *dev, const bool en)
{
	const struct cp7420_config *const config = dev->config;
	struct cp7420_data *data = dev->data;
	uint8_t reg_code;
	int ret;

	if (en)
		reg_code = CP7420_IIN_REG_EN;
	else
		reg_code = 0;

	ret = i2c_reg_update_byte_dt(&config->i2c, OPERATION_CTRL_0, CP7420_IIN_REG_EN, reg_code);
	if (ret) {
		LOG_ERR("Failed to set IIN_REG_EN: %d\n", ret);
		return ret;
	}

	data->iin_reg_en = en;

	return ret;
}

/**
 * @brief Sets the VIN_REG_EN bit
 * 
 * @param dev CP7420 device to access
 * @param en value to be set
 * @return 0 if successful,
 * @return negative error code from I2C API
 */
static int cp7420_set_vin_reg_en(const struct device *dev, const bool en)
{
	const struct cp7420_config *const config = dev->config;
	struct cp7420_data *data = dev->data;
	uint8_t reg_code;
	int ret;

	if (en)
		reg_code = CP7420_VIN_REG_EN;
	else
		reg_code = 0;

	ret = i2c_reg_update_byte_dt(&config->i2c, OPERATION_CTRL_0, CP7420_VIN_REG_EN, reg_code);
	if (ret) {
		LOG_ERR("Failed to set VIN_REG_EN: %d\n", ret);
		return ret;
	}

	data->vin_reg_en = en;

	return ret;
}

/**
 * @brief Sets the PSYS_EN bit
 * 
 * @param dev CP7420 device to access
 * @param en value to be set
 * @return 0 if successful,
 * @return negative error code from I2C API
 */
static int cp7420_set_psys_en(const struct device *dev, const bool en)
{
	const struct cp7420_config *const config = dev->config;
	struct cp7420_data *data = dev->data;
	uint8_t reg_code;
	int ret;

	if (en)
		reg_code = CP7420_PSYS_EN;
	else
		reg_code = 0;

	ret = i2c_reg_update_byte_dt(&config->i2c, POWER_MONITOR_CTRL, CP7420_PSYS_EN, reg_code);
	if (ret) {
		LOG_ERR("Failed to set PSYS_EN: %d\n", ret);
		return ret;
	}

	data->psys_en = en;

	return ret;
}

/**
 * @brief Sets the CHG_MODE
 * 
 * @param dev CP7420 device to access
 * @param val CHG_MODE value to set
 * @return 0 if successful,
 * @return negative error code from I2C API
 */
static int cp7420_set_chg_mode(const struct device *dev, uint8_t val)
{
	const struct cp7420_config *const config = dev->config;
	struct cp7420_data *data = dev->data;
	int ret;

	ret = i2c_reg_update_byte_dt(&config->i2c, OPERATION_CTRL_0, CP7420_CHG_MODE_MASK, val);
	if (ret) {
		LOG_ERR("Failed to set CHG_MODE: %d\n", ret);
		return ret;
	}

	data->chg_mode = val;

	return ret;
}

/**
 * @brief Configures the SWITCH_CURRENT_CTRL_2 register according to the devicetree
 * 
 * @param dev CP7420 device to access
 * @return 0 if successful,
 * @return negative error code from I2C API
 */
static int cp7420_cfg_switch_curr_ctrl_2(const struct device *dev)
{
	const struct cp7420_config *const config = dev->config;
	uint16_t ocp_pfm_cfg = config->ocp_pfm_cfg;
	uint8_t reg_code;

	ocp_pfm_cfg = CLAMP(ocp_pfm_cfg, CP7420_OCP_PFM_MIN_MA, CP7420_OCP_PFM_MAX_MA);

	reg_code = (ocp_pfm_cfg - CP7420_OCP_PFM_OFFSET_MA) / CP7420_OCP_PFM_STEP_MA;

	return i2c_reg_write_byte_dt(&config->i2c, SWITCH_CURRENT_CTRL_2, reg_code);
}

/**
 * @brief Configures the SWITCH_CURRENT_CTRL_1 register according to the devicetree
 * 
 * @param dev CP7420 device to access
 * @return 0 if successful,
 * @return negative error code from I2C API
 */
static int cp7420_cfg_switch_curr_ctrl_1(const struct device *dev)
{
	const struct cp7420_config *const config = dev->config;
	uint16_t ocp_pwm_cfg = config->ocp_pwm_cfg;
	uint8_t reg_code;

	ocp_pwm_cfg = CLAMP(ocp_pwm_cfg, CP7420_OCP_PWM_MIN_MA, CP7420_OCP_PWM_MAX_MA);

	reg_code = (ocp_pwm_cfg - CP7420_OCP_PWM_OFFSET_MA) / CP7420_OCP_PWM_STEP_MA;

	return i2c_reg_write_byte_dt(&config->i2c, SWITCH_CURRENT_CTRL_1, reg_code);
}

/**
 * @brief Configures the POWER_MONITOR_CTRL register according to the devicetree
 * 
 * @param dev CP7420 device to access
 * @return 0 if successful,
 * @return negative error code from I2C API
 */
static int cp7420_cfg_power_mon_ctrl(const struct device *dev)
{
	const struct cp7420_config *const config = dev->config;
	struct cp7420_data *data = dev->data;
	uint8_t reg_code = 0;
	int ret;

	if (config->psys_en) {
		reg_code |= CP7420_PSYS_EN;

		if (config->rsi_cfg)
			reg_code |= (config->rsi_cfg << CP7420_RSI_CFG_SHIFT) & CP7420_RSI_CFG_MASK;

		if (config->rso_cfg)
			reg_code |= (config->rso_cfg << CP7420_RSO_CFG_SHIFT) & CP7420_RSO_CFG_MASK;

		if (config->psys_ratio)
			reg_code |= (config->psys_ratio << CP7420_PSYS_RATIO_SHIFT) & CP7420_PSYS_RATIO_MASK;
	}

	ret = i2c_reg_write_byte_dt(&config->i2c, POWER_MONITOR_CTRL, reg_code);
	if (ret) {
		LOG_ERR("Failed to configure POWER_MONITOR_CTRL: %d", ret);
		return ret;
	}

	data->psys_en = config->psys_en;

	return ret;
}

/**
 * @brief Configures the DITHERING_CTRL register according to the devicetree
 * 
 * @param dev CP7420 device to access
 * @return 0 if successful,
 * @return negative error code from I2C API
 */
static int cp7420_cfg_dither_ctrl(const struct device *dev)
{
	const struct cp7420_config *const config = dev->config;
	uint8_t reg_code = 0;

	if (config->dither_en) {
		reg_code |= CP7420_DITHER_EN;

		if (config->dither_rate)
			reg_code |= (config->dither_rate << CP7420_DITHER_RATE_SHIFT) & CP7420_DITHER_RATE_MASK;

		if (config->dither_limit)
			reg_code |= config->dither_limit & CP7420_DITHER_LIMIT_MASK;
	}

	return i2c_reg_write_byte_dt(&config->i2c, DITHERING_CTRL, reg_code);
}

/**
 * @brief Sets the PROCHOT monitors
 * 
 * @param dev CP7420 device to access
 * @param prochot_mon code to be set
 * @return 0 if successful,
 * @return negative error code from I2C API
 */
static int cp7420_set_prochot_mon(const struct device *dev, uint16_t prochot_mon)
{
	const struct cp7420_config *const config = dev->config;
	struct cp7420_data *data = dev->data;
	uint8_t buf[3] = {PROCHOT_CTRL_1, prochot_mon >> 8, prochot_mon & 0xFF};
	int ret;	

	ret = i2c_write_dt(&config->i2c, buf, sizeof(buf));
	if (ret) {
		LOG_ERR("Failed to set PROCHOT monitors: %d", ret);
		return ret;
	}

	data->prochot_mon = prochot_mon;

	return ret;
}

/**
 * @brief Sets the PROCHOT debounce time
 * 
 * @param dev CP7420 device to access
 * @param prochot_dbounce code to be set
 * @return 0 if successful,
 * @return negative error code from I2C API
 */
static int cp7420_set_prochot_debounce(const struct device *dev, uint8_t prochot_dbounce)
{
	const struct cp7420_config *const config = dev->config;
	struct cp7420_data *data = dev->data;
	uint8_t reg_code;
	int ret;

	reg_code = prochot_dbounce << CP7420_PROCHOT_DEBOUNCE_SHIFT;

	ret = i2c_reg_update_byte_dt(&config->i2c,
								 PROCHOT_CTRL_0,
								 CP7420_PROCHOT_DEBOUNCE_MASK,
								 reg_code);
	if (ret) {
		LOG_ERR("Failed to set PROCHOT_DEBOUNCE: %d", ret);
		return ret;
	}

	data->prochot_debounce = prochot_dbounce;

	return ret;
}

/**
 * @brief Sets the PROCHOT minimum pulse width
 * 
 * @param dev CP7420 device to access
 * @param prochot_duration code to be set
 * @return 0 if successful,
 * @return negative error code from I2C API
 */
static int cp7420_set_prochot_duration(const struct device *dev, uint8_t prochot_duration)
{
	const struct cp7420_config *const config = dev->config;
	struct cp7420_data *data = dev->data;
	uint8_t reg_code;
	int ret;

	reg_code = prochot_duration << CP7420_PROCHOT_DUR_SHIFT;

	ret = i2c_reg_update_byte_dt(&config->i2c, PROCHOT_CTRL_0, CP7420_PROCHOT_DUR_MASK, reg_code);
	if (ret) {
		LOG_ERR("Failed to set PROCHOT_DURATION: %d", ret);
		return ret;
	}

	data->prochot_duration = prochot_duration;

	return ret;
}

/**
 * @brief Configures T_IBAT_MAX1 and T_IBAT_MAX2 according to the datasheet
 * 
 * @param dev CP7420 device to access
 * @return 0 if successful,
 * @return negative error code from I2C API
 */
static int cp7420_cfg_t_ibat(const struct device *dev)
{
	const struct cp7420_config *const config = dev->config;
	uint8_t reg_code = 0;

	if (config->t_ibat_max1)
		reg_code |= (config->t_ibat_max1 << CP7420_T_IBAT_MAX1_SHIFT) & CP7420_T_IBAT_MAX1_MASK;

	if (config->t_ibat_max2)
		reg_code |= (config->t_ibat_max2 << CP7420_T_IBAT_MAX1_SHIFT) & CP7420_T_IBAT_MAX2_MASK;

	return i2c_reg_update_byte_dt(&config->i2c,
								  TWO_LEVEL_CURRENT_CTRL,
								  CP7420_T_IBAT_MASK,
								  reg_code);
}

/**
 * @brief Sets the upper PROCHOT input current limit's deglitch timer
 * 
 * @param dev CP7420 device to access
 * @param t_iin_max2 code to be set
 * @return 0 if successful,
 * @return negative error code from I2C API
 */
static int cp7420_set_t_iin_max2(const struct device *dev, uint8_t t_iin_max2)
{
	const struct cp7420_config *const config = dev->config;
	struct cp7420_data *data = dev->data;
	int ret;

	ret = i2c_reg_update_byte_dt(&config->i2c,
								 TWO_LEVEL_CURRENT_CTRL,
								 CP7420_T_IIN_MAX2_MASK,
								 t_iin_max2);
	if (ret) {
		LOG_ERR("Failed to set T_IIN_MAX2: %d", ret);
		return ret;
	}

	data->t_iin_max_2 = t_iin_max2;

	return ret;
}

/**
 * @brief Sets the lower PROCHOT input current limit's deglitch timer
 * 
 * @param dev CP7420 device to access
 * @param t_iin_max1 code to be set
 * @return 0 if successful,
 * @return negative error code from I2C API
 */
static int cp7420_set_t_iin_max1(const struct device *dev, uint8_t t_iin_max1)
{
	const struct cp7420_config *const config = dev->config;
	struct cp7420_data *data = dev->data;
	uint8_t reg_code;
	int ret;

	reg_code = t_iin_max1 << CP7420_T_IIN_MAX1_SHIFT;

	ret = i2c_reg_update_byte_dt(&config->i2c,
								 TWO_LEVEL_CURRENT_CTRL,
								 CP7420_T_IIN_MAX1_MASK,
								 reg_code);
	if (ret) {
		LOG_ERR("Failed to set T_IIN_MAX1: %d", ret);
		return ret;
	}

	data->t_iin_max_1 = t_iin_max1;

	return ret;
}

/**
 * @brief Sets the upper PROCHOT battery discharge current threshold 
 * 
 * @param dev CP7420 device to access
 * @param ibat_dischg value to be set in milliamps
 * @return 0 if successful,
 * @return negative error code from I2C API
 */
static int cp7420_set_ibat_dchg_lim_2(const struct device *dev, uint16_t ibat_dischg)
{
	const struct cp7420_config *const config = dev->config;
	struct cp7420_data *data = dev->data;
	uint8_t reg_code = 0;
	int ret;

	ibat_dischg = CLAMP(ibat_dischg,
						CP7420_IBAT_DISCHG_LIM_2_MIN_MA,
						CP7420_IBAT_DISCHG_LIM_2_MAX_MA);

	reg_code = ibat_dischg / CP7420_IBAT_DISCHG_LIM_2_STEP_MA;

	ret = i2c_reg_write_byte_dt(&config->i2c, BATTERY_DISCHG_CTRL_1, reg_code);
	if (ret) {
		LOG_ERR("Failed to set IBAT_DISCHG_LIMIT_2: %d", ret);
		return ret;
	}

	data->ibat_dischg_limit_2_ma = ibat_dischg;

	return ret;
}

/**
 * @brief Sets the lower PROCHOT battery discharge current threshold 
 * 
 * @param dev CP7420 device to access
 * @param ibat_dischg value to be set in milliamps
 * @return 0 if successful,
 * @return negative error code from I2C API
 */
static int cp7420_set_ibat_dchg_lim_1(const struct device *dev, uint16_t ibat_dischg)
{
	const struct cp7420_config *const config = dev->config;
	struct cp7420_data *data = dev->data;
	uint8_t reg_code = 0;
	int ret;

	ibat_dischg = CLAMP(ibat_dischg,
						CP7420_IBAT_DISCHG_LIM_1_MIN_MA,
						CP7420_IBAT_DISCHG_LIM_1_MAX_MA);

	reg_code = ibat_dischg / CP7420_IBAT_DISCHG_LIM_1_STEP_MA;

	ret = i2c_reg_write_byte_dt(&config->i2c, BATTERY_DISCHG_CTRL_0, reg_code);
	if (ret) {
		LOG_ERR("Failed to set IBAT_DISCHG_LIMIT_1: %d", ret);
		return ret;
	}

	data->ibat_dischg_limit_1_ma = ibat_dischg;

	return ret;
}

/**
 * @brief Configures the STARTUP_CTRL register according to the devicetree
 * 
 * @param dev CP7420 device to access
 * @return 0 if successful,
 * @return negative error code from I2C API
 */
static int cp7420_cfg_startup_ctrl(const struct device *dev)
{
	const struct cp7420_config *const config = dev->config;
	uint8_t reg_code = 0;

	if (config->ibat_softstart)
		reg_code |= (config->ibat_softstart << CP7420_IBAT_SOFTSTART_SHIFT) &
					CP7420_IBAT_SOFTSTART_MASK;

	if (config->rechg_ibat_softstart)
		reg_code |= CP7420_RECHG_IBAT_SOFTSTART;

	if (config->vsys_softstart)
		reg_code |= (config->vsys_softstart << CP7420_VSYS_SOFTSTART_SHIFT) &
					CP7420_VSYS_SOFTSTART_MASK;

	return i2c_reg_write_byte_dt(&config->i2c, STARTUP_CTRL, reg_code);
}

/**
 * @brief Configures the DPM_CTRL register according to the devicetree
 * 
 * @param dev CP7420 device to access
 * @return 0 if successful,
 * @return negative error code from I2C API
 */
static int cp7420_cfg_dpm_ctrl(const struct device *dev)
{
	const struct cp7420_config *const config = dev->config;
	uint8_t reg_code = 0;

	if (config->vin_dpm_threshold)
		reg_code |= (config->vin_dpm_threshold << CP7420_VIN_DPM_THRESH_SHIFT) &
					CP7420_VIN_DPM_THRESH_MASK;

	if (config->fsw_cfg)
		reg_code |= config->fsw_cfg & CP7420_FSW_CFG_MASK;

	return i2c_reg_write_byte_dt(&config->i2c, DPM_CTRL, reg_code);
}

/**
 * @brief Configures the TEMP_REGULATION register according to the devicetree
 * 
 * @param dev CP7420 device to access
 * @return 0 if successful,
 * @return negative error code from I2C API
 */
static int cp7420_cfg_temp_reg(const struct device *dev)
{
	const struct cp7420_config *const config = dev->config;
	uint8_t reg_code = 0;

	if (config->ntc_en)
		reg_code |= CP7420_NTC_EN;

	if (config->tsdie_en)
		reg_code |= CP7420_TSDIE_EN;

	if (config->batgone_en)
		reg_code |= CP7420_BATGONE_EN;

	if (config->watchdog_timer) {
		reg_code |= (config->watchdog_timer & CP7420_WATCHDOG_TIMER_MASK);
	}

	return i2c_reg_write_byte_dt(&config->i2c, TEMP_REGULATION_CTRL, reg_code);
}

/**
 * @brief Sets the falling PROCHOT system voltage threshold 
 * 
 * @param dev CP7420 device to access
 * @param vsys_low Value to set in millivolts
 * @return 0 if successful,
 * @return negative error code from I2C API
 */
static int cp7420_set_vsys_low(const struct device *dev, uint16_t vsys_low)
{
	const struct cp7420_config *const config = dev->config;
	struct cp7420_data *data = dev->data;
	uint8_t reg_code;
    int ret;

	vsys_low = CLAMP(vsys_low, CP7420_VSYS_LOW_MIN_MV, CP7420_VSYS_LOW_MAX_MV);

	reg_code = (vsys_low - CP7420_VSYS_LOW_OFFSET_MV) / CP7420_VSYS_LOW_STEP_MV;

	ret = i2c_reg_write_byte_dt(&config->i2c, SYS_VOLTAGE_CTRL_2, reg_code);
	if (ret) {
		LOG_ERR("Failed to set VSYS_LOW: %d", ret);
		return ret;
	}

	data->vsys_low_mv = vsys_low;

	return ret;
}

/**
 * @brief Configures VSYS_MAX according to the devicetree
 * 
 * @param dev CP7420 device to access
 * @return 0 if successful,
 * @return negative error code from I2C API
 */
static int cp7420_cfg_vsys_max(const struct device *dev)
{
	const struct cp7420_config *const config = dev->config;
	uint16_t vsys_max_uv;
	uint8_t reg_code;
    int ret;

	vsys_max_uv = CLAMP(config->vsys_max_mv, CP7420_VSYS_MAX_MIN_MV, CP7420_VSYS_MAX_MAX_MV);

	reg_code = vsys_max_uv / CP7420_VSYS_MAX_STEP_MV;

	ret = i2c_reg_write_byte_dt(&config->i2c, SYS_VOLTAGE_CTRL_1, reg_code);
	if (ret)
		LOG_ERR("Failed to set VSYS_MAX: %d", ret);

	return ret;
}

/**
 * @brief Configures VSYS_MIN according to the devicetree
 * 
 * @param dev CP7420 device to access
 * @return 0 if successful,
 * @return negative error code from I2C API
 */
static int cp7420_cfg_vsys_min(const struct device *dev)
{
	const struct cp7420_config *const config = dev->config;
	uint16_t vsys_min_uv;
	uint8_t reg_code;
    int ret;

	vsys_min_uv = CLAMP(config->vsys_min_mv, CP7420_VSYS_MIN_MIN_MV, CP7420_VSYS_MIN_MAX_MV);

	reg_code = vsys_min_uv / CP7420_VSYS_MIN_STEP_MV;

	ret = i2c_reg_write_byte_dt(&config->i2c, SYS_VOLTAGE_CTRL_0, reg_code);
	if (ret)
		LOG_ERR("Failed to set VSYS_MIN: %d", ret);
	
	return ret;
}

/**
 * @brief Sets the falling PROCHOT input voltage threshold
 * 
 * @param dev CP7420 device to access
 * @param vin_low VIN_LOW value in millivolts
 * @return 0 if successful,
 * @return negative error code from I2C API
 */
__unused static int cp7420_set_vin_low(const struct device *dev, uint16_t vin_low)
{
	const struct cp7420_config *const config = dev->config;
	struct cp7420_data *data = dev->data;
	uint8_t reg_code;
    int ret;

	vin_low = CLAMP(vin_low, CP7420_VIN_LOW_MIN_MV, CP7420_VIN_LOW_MAX_MV);

	reg_code = (vin_low - CP7420_VIN_LOW_OFFSET_MV) / CP7420_VIN_LOW_STEP_MV;

	ret = i2c_reg_write_byte_dt(&config->i2c, INPUT_CTRL_5, reg_code);
	if (ret) {
		LOG_ERR("Failed to set VIN_LOW: %d", ret);
		return ret;
	}

	data->vin_low_mv = vin_low;

	return ret;
}

/**
 * @brief Sets the peak PROCHOT input current threshold 
 * 
 * @param dev CP7420 device to access
 * @param iin_limit_peak IIN_LIMIT_PEAK value in milliamps
 * @return 0 if successful,
 * @return negative error code from I2C API
 */
static int cp7420_set_iin_limit_peak(const struct device *dev, uint16_t iin_limit_peak)
{
	const struct cp7420_config *const config = dev->config;
	struct cp7420_data *data = dev->data;
	uint8_t reg_code;
    int ret;

	iin_limit_peak = CLAMP(iin_limit_peak, CP7420_IIN_LIMIT_PEAK_MIN_MA, CP7420_IIN_LIMIT_PEAK_MAX_MA);

	reg_code = iin_limit_peak / CP7420_IIN_LIMIT_PEAK_STEP_MA;

	ret = i2c_reg_write_byte_dt(&config->i2c, INPUT_CTRL_4, reg_code);
	if (ret) {
		LOG_ERR("Failed to set IIN_LIMIT_PEAK: %d", ret);
		return ret;
	}

	data->iin_limit_peak_ma = iin_limit_peak;

	return ret;
}

/**
 * @brief Sets the upper PROCHOT input current threshold 
 * 
 * @param dev CP7420 device to access
 * @param iin_limit_2 IIN_LIMIT_2 value in milliamps
 * @return 0 if successful,
 * @return negative error code from I2C API
 */
static int cp7420_set_iin_limit_2(const struct device *dev, uint16_t iin_limit_2)
{
	const struct cp7420_config *const config = dev->config;
	struct cp7420_data *data = dev->data;
	uint8_t reg_code;
    int ret;

	iin_limit_2 = CLAMP(iin_limit_2, CP7420_IIN_LIMIT_2_MIN_MA, CP7420_IIN_LIMIT_2_MAX_MA);

	reg_code = iin_limit_2 / CP7420_IIN_LIMIT_2_STEP_MA;

	ret = i2c_reg_write_byte_dt(&config->i2c, INPUT_CTRL_3, reg_code);
	if (ret) {
		LOG_ERR("Failed to set IIN_LIMIT_2: %d", ret);
		return ret;
	}

	data->iin_limit_2_ma = iin_limit_2;

	return ret;
}

/**
 * @brief Sets the lower PROCHOT input current threshold 
 * 
 * @param dev CP7420 device to access
 * @param iin_limit_1 IIN_LIMIT_1 value in milliamps
 * @return 0 if successful,
 * @return negative error code from I2C API
 */
static int cp7420_set_iin_limit_1(const struct device *dev, uint16_t iin_limit_1)
{
	const struct cp7420_config *const config = dev->config;
	struct cp7420_data *data = dev->data;
	uint8_t reg_code;
    int ret;

	iin_limit_1 = CLAMP(iin_limit_1, CP7420_IIN_LIMIT_1_MIN_MA, CP7420_IIN_LIMIT_1_MAX_MA);

	reg_code = iin_limit_1 / CP7420_IIN_LIMIT_1_STEP_MA;

	ret = i2c_reg_write_byte_dt(&config->i2c, INPUT_CTRL_2, reg_code);
	if (ret) {
		LOG_ERR("Failed to set IIN_LIMIT_1: %d", ret);
		return ret;
	}

	data->iin_limit_1_ma = iin_limit_1;

	return ret;
}

/**
 * @brief Sets the input current regulation target
 * 
 * @param dev CP7420 device to access
 * @param iin_reg IIN_REG value in milliamps
 * @return 0 if successful,
 * @return negative error code from I2C API
 */
static int cp7420_set_iin_reg(const struct device *dev, uint16_t iin_reg)
{
	const struct cp7420_config *const config = dev->config;
	struct cp7420_data *data = dev->data;
	uint8_t reg_code;
    int ret;

	iin_reg = CLAMP(iin_reg, CP7420_IIN_REG_MIN_MA, CP7420_IIN_REG_MAX_MA);

	reg_code = iin_reg / CP7420_IIN_REG_STEP_MA;

	ret = i2c_reg_write_byte_dt(&config->i2c, INPUT_CTRL_1, reg_code);
	if (ret) {
		LOG_ERR("Failed to set IIN_REG: %d", ret);
		return ret;
	}

	data->iin_reg_ma = iin_reg;

	return ret;
}

/**
 * @brief Sets the input voltage regulation target
 * 
 * @param dev CP7420 device to access
 * @param vin_reg VIN_REG value in millivolts
 * @return 0 if successful,
 * @return negative error code from I2C API
 */
static int cp7420_set_vin_reg(const struct device *dev, uint16_t vin_reg)
{
	const struct cp7420_config *const config = dev->config;
	struct cp7420_data *data = dev->data;
	uint8_t reg_code;
    int ret;

	vin_reg = CLAMP(vin_reg, CP7420_VIN_REG_MIN_MV, CP7420_VIN_REG_MAX_MV);

	reg_code = (vin_reg - CP7420_VIN_REG_OFFSET_MV) / CP7420_VIN_REG_STEP_MV;

	ret = i2c_reg_write_byte_dt(&config->i2c, INPUT_CTRL_0, reg_code);
	if (ret) {
		LOG_ERR("Failed to set VIN_REG: %d", ret);
		return ret;
	}

	data->vin_reg_mv = vin_reg;

	return ret;
}

/**
 * @brief Configures BATTERY_CHGING_CTRL_5 based on devicetree
 * 
 * @param dev CP7420 device to access
 * @return 0 if successful,
 * @return negative error code from I2C API
 */
static int cp7420_cfg_batt_chg_ctrl_5(const struct device *dev)
{
	const struct cp7420_config *const config = dev->config;
	uint8_t reg_code = 0;

	if (config->auto_chg_timer_en)
		reg_code |= CP7420_AUTO_CHG_TIMER_EN;

	if (config->prechg_timer)
		reg_code |= (config->prechg_timer << CP7420_PRECHG_TIMER_SHIFT) & CP7420_PRECHG_TIMER_MASK;

	if (config->fast_chg_timer)
		reg_code |= config->fast_chg_timer & CP7420_FAST_CHG_TIMER_MASK;

	return i2c_reg_write_byte_dt(&config->i2c, BATTERY_CHGING_CTRL_5, reg_code);;
}

/**
 * @brief Configures BATTERY_CHGING_CTRL_4 based on devicetree
 * 
 * @param dev CP7420 device to access
 * @return 0 if successful,
 * @return negative error code from I2C API
 */
static int cp7420_cfg_batt_chg_ctrl_4(const struct device *dev)
{
	const struct cp7420_config *const config = dev->config;
	uint8_t reg_code;
	uint16_t iterm;

	reg_code = config->vsys_ov_cfg << CP7420_VSYS_OV_CFG_SHIFT;

	if (config->iterm_en) {
		iterm = CLAMP(config->iterm_ma, CP7420_ITERM_CFG_MIN_MA, CP7420_ITERM_CFG_MAX_MA);
		reg_code |= ((iterm - CP7420_ITERM_CFG_OFFSET_MA) /
					 CP7420_ITERM_CFG_STEP_MA) |
					 CP7420_ITERM_EN;
	}

	return i2c_reg_write_byte_dt(&config->i2c, BATTERY_CHGING_CTRL_4, reg_code);
}

/**
 * @brief Configures BATTERY_CHGING_CTRL_3 based on devicetree
 * 
 * @param dev CP7420 device to access
 * @return 0 if successful,
 * @return negative error code from I2C API
 */
static int cp7420_cfg_batt_chg_ctrl_3(const struct device *dev)
{
	const struct cp7420_config *const config = dev->config;
	uint8_t reg_code;

    reg_code = config->vbat_ov_cfg << CP7420_VBAT_OV_CFG_SHIFT;

	if (config->recharge_en) {
		reg_code |= CP7420_RECHARGE_EN;

		reg_code |= (config->recharge_offset << CP7420_RECHARGE_OFFSET_SHIFT) &
					CP7420_RECHARGE_OFFSET_MASK;

		reg_code |= config->recharge_deglitch & CP7420_RECAHRGE_DEGLITCH_MASK;
	}

	return i2c_reg_write_byte_dt(&config->i2c, BATTERY_CHGING_CTRL_3, reg_code);
}

/**
 * @brief precharge current setter
 * 
 * @param dev CP7420 device to access
 * @param iprechg The prechagre current value in milliamps
 * @return 0 if successful,
 * @return negative error code from I2C API 
 */
static int cp7420_set_iprechg_cfg(const struct device *dev, uint16_t iprechg)
{
    const struct cp7420_config *const config = dev->config;
	struct cp7420_data *data = dev->data;
	uint8_t reg_code;
	int ret;

	iprechg = CLAMP(iprechg, CP7420_IPRECHG_MIN_MA, CP7420_IPRECHG_MAX_MA);

	reg_code = (iprechg - CP7420_IPRECHG_OFFSET_MA) / CP7420_IPRECHG_STEP_MA;

	ret = i2c_reg_update_byte_dt(&config->i2c,
								 BATTERY_CHGING_CTRL_3,
								 CP7420_IPRECHG_CFG_MASK,
								 reg_code);
	if (ret) {
		LOG_ERR("Failed to set IPRECHG_CFG: %d", ret);
        return ret;
    }

    data->iprechg_ma = iprechg;

    return ret;
}

/**
 * @brief Configures VBAT_LOW based on devicetree
 * 
 * @param dev CP7420 device to access
 * @return int 
 */
static int cp7420_cfg_vbat_low(const struct device *dev)
{
	const struct cp7420_config *const config = dev->config;
	uint8_t reg_code;

    reg_code = config->vbat_low << CP7420_VBAT_LOW_SHIFT;

    return i2c_reg_update_byte_dt(&config->i2c,
                                  BATTERY_CHGING_CTRL_2,
                                  CP7420_VBAT_LOW_MASK,
                                  reg_code);
}

/**
 * @brief charge voltage setter
 * 
 * @param dev CP7420 device to access
 * @param vbat_reg charge voltage value in millivolts
 * @return 0 if successful,
 * @return negative error code from I2C API 
 */
static int cp7420_set_vbat_reg(const struct device *dev, uint16_t vbat_reg)
{
	const struct cp7420_config *const config = dev->config;
	struct cp7420_data *data = dev->data;
	uint8_t reg_code;
	uint16_t tmp;
	int ret;

    vbat_reg = CLAMP(vbat_reg, CP7420_VBAT_REG_MIN_MV, CP7420_VBAT_REG_MAX_MV);

    tmp = (vbat_reg / CP7420_VBAT_REG_STEP_MV) << CP7420_VBAT_REG_SHIFT;

    reg_code = (tmp & 0xff00) >> 8;

    ret = i2c_reg_update_byte_dt(&config->i2c,
                                 BATTERY_CHGING_CTRL_1,
                                 CP7420_VBAT_REG_MSB_MASK,
                                 reg_code);
    if (ret) {
        LOG_ERR("Failed to set VBAT_REG MSB: %d", ret);
        return ret;
    }

    reg_code = tmp & 0xff;

    ret = i2c_reg_update_byte_dt(&config->i2c,
                                 BATTERY_CHGING_CTRL_1,
                                 CP7420_VBAT_REG_LSB_MASK,
                                 reg_code);  
    if (ret) {
        LOG_ERR("Failed to set VBAT_REG LSB: %d", ret);
        return ret;
    }

    data->vbat_reg_mv = vbat_reg;

    return ret;
}

/**
 * @brief charge current setter
 * 
 * @param dev CP7420 device to access
 * @param ibat_chg_cfg charge current value in milliamps
 * @return 0 if successful,
 * @return negative error code from I2C API 
 */
static int cp7420_set_ibat_chg_cfg(const struct device *dev, uint16_t ibat_chg_cfg)
{
    const struct cp7420_config *const config = dev->config;
	struct cp7420_data *data = dev->data;
    uint8_t reg_code;
	int ret; 

    ibat_chg_cfg = CLAMP(ibat_chg_cfg, CP7420_IBAT_CHG_MIN_MA, CP7420_IBAT_CHG_MAX_MA);

    reg_code = ibat_chg_cfg / CP7420_IBAT_CHG_STEP_MA;

    ret = i2c_reg_update_byte_dt(&config->i2c,
                                 BATTERY_CHGING_CTRL_0,
                                 CP7420_IBAT_CHG_CFG_MASK,
                                 reg_code);
    if (ret) {
        LOG_ERR("Failed to set IBAT_CHG_CFG: %d", ret);
        return ret;
    }

    data->ibat_chg_ma = ibat_chg_cfg;

    return ret;
}

/**
 * @brief Gets values from the ADC for the supported channels
 * 
 * @param dev CP7420 device to access
 * @param chan Channel number to read
 * @param valp Returns the sensor value read on success
 * @return 0 if successful,
 * @return -ENOTSUP for unsupported channels
 */
static int cp7420_channel_get(const struct device *dev,
                              enum sensor_channel chan,
                              struct sensor_value *valp)
{
	int ret, val;

	ret = cp7420_get_adc(dev, chan, &val);
	if (ret)
		return ret;

	valp->val1 = val / 1000;
	valp->val2 = (val % 1000) * 1000U;

    return ret;
}

/**
 * @brief Sets attributes for a supported channel
 * 
 * @param dev CP7420 device to access
 * @param chan Channel number of interest
 * @param attr Attribute of a channel to set
 * @param val Value to be set
 * @return -ENOTSUP for unsupported channels 
 */
static int cp7420_attribute_set(const struct device *dev,
                                enum sensor_channel chan,
                                enum sensor_attribute attr,
                                const struct sensor_value *val)
{
	switch (attr) {
	case SENSOR_ATTR_FEATURE_MASK:
		switch (chan) {
		case SENSOR_CHAN_GAUGE_DESIRED_CHARGING_CURRENT:
			return cp7420_set_chg_mode(dev, val->val1);
		case SENSOR_CHAN_POWER:
			return cp7420_set_psys_en(dev, val->val1);
		case SENSOR_CHAN_VOLTAGE:
			return cp7420_set_vin_reg_en(dev, val->val1);
		case SENSOR_CHAN_CURRENT:
			return cp7420_set_iin_reg_en(dev, val->val1);
		case CP7420_CHAN_LOW_POWER_MODE:
			return cp7420_set_lpm_en(dev, val->val1);
		case CP7420_CHAN_PROCHOT_ALL:
			return cp7420_set_prochot_mon(dev, val->val1);
		default:
			goto chan_err;
		}
		break;
	case SENSOR_ATTR_CONFIGURATION:
		switch (chan) {
		case SENSOR_CHAN_GAUGE_DESIRED_CHARGING_CURRENT:
			return cp7420_set_ibat_chg_cfg(dev, val->val1);
		case SENSOR_CHAN_GAUGE_DESIRED_VOLTAGE:
			return cp7420_set_vbat_reg(dev, val->val1);
		case SENSOR_CHAN_VOLTAGE:
			return cp7420_set_vin_reg(dev, val->val1);
		case SENSOR_CHAN_CURRENT:
			return cp7420_set_iin_reg(dev, val->val1);
		case CP7420_CHAN_IIN_LIMIT_1:
			return cp7420_set_iin_limit_1(dev, val->val1);
		case CP7420_CHAN_IIN_LIMIT_2:
			return cp7420_set_iin_limit_2(dev, val->val1);
		case CP7420_CHAN_IIN_LIMIT_PEAK:
			return cp7420_set_iin_limit_peak(dev, val->val1);
		case CP7420_CHAN_IBAT_DISCHARGE_LIMIT_1:
			return cp7420_set_ibat_dchg_lim_1(dev, val->val1);
		case CP7420_CHAN_IBAT_DISCHARGE_LIMIT_2:
			return cp7420_set_ibat_dchg_lim_2(dev, val->val1);
		case CP7420_CHAN_VIN_LOW:
			return cp7420_set_vin_low(dev, val->val1);
		case CP7420_CHAN_VSYSMIN:
			return cp7420_set_vsys_low(dev, val->val1);
		default:
			goto chan_err;
		}
		break;
	case CP7420_ATTR_PROCHOT_DEBOUNCE:
		switch (chan) {
		case CP7420_CHAN_PROCHOT_ALL:
			return cp7420_set_prochot_debounce(dev, val->val1);
		default:
			goto chan_err;
		}
		break;
	case CP7420_ATTR_PROCHOT_DURATION:
		switch (chan) {
		case CP7420_CHAN_IIN_LIMIT_1:
			return cp7420_set_t_iin_max1(dev, val->val1);
		case CP7420_CHAN_IIN_LIMIT_2:
			return cp7420_set_t_iin_max2(dev, val->val1);
		case CP7420_CHAN_PROCHOT_ALL:
			return cp7420_set_prochot_duration(dev, val->val1);
		default:
			goto chan_err;
		}
		break;
	default:
		LOG_ERR("Attribute: %d is not supported", attr);
		return -ENOTSUP;
	}

    return 0;

chan_err:
	LOG_ERR("Channel: %d feature mask is not supported", chan);
	return -ENOTSUP;
}

/**
 * @brief 
 * 
 * @param dev CP7420 device to access
 * @param chan Channel number of interest
 * @param attr Attribute of a channel to get
 * @param val Value to be set
 * @return -ENOTSUP for unsupported channels 
 */
static int cp7420_attribute_get(const struct device *dev,
                                enum sensor_channel chan,
                                enum sensor_attribute attr,
                                struct sensor_value *val)
{
	struct cp7420_data *data = dev->data;

	switch (attr) {
	case SENSOR_ATTR_FEATURE_MASK:
		switch (chan) {
		case SENSOR_CHAN_GAUGE_DESIRED_CHARGING_CURRENT:
			val->val1 = data->chg_mode;
			break;
		case SENSOR_CHAN_POWER:
			val->val1 = data->psys_en;
			break;
		case SENSOR_CHAN_VOLTAGE:
			val->val1 = data->vin_reg_en;
			break;
		case CP7420_CHAN_LOW_POWER_MODE:
			val->val1 = data->lpm_en;
			break;
		case CP7420_CHAN_PROCHOT_ALL:
			val->val1 = data->prochot_mon;
			break;
		default:
			goto chan_err;
		}
		break;
	case SENSOR_ATTR_CONFIGURATION:
		switch (chan) {
		case SENSOR_CHAN_GAUGE_DESIRED_CHARGING_CURRENT:
			val->val1 = data->ibat_chg_ma;
			break;
		case SENSOR_CHAN_GAUGE_DESIRED_VOLTAGE:
			val->val1 = data->vbat_reg_mv;
			break;
		case SENSOR_CHAN_VOLTAGE:
			val->val1 = data->vin_reg_mv;
			break;
		case SENSOR_CHAN_CURRENT:
			val->val1 = data->iin_reg_ma;
			break;
		case CP7420_CHAN_IIN_LIMIT_1:
			val->val1 = data->iin_limit_1_ma;
			break;
		case CP7420_CHAN_IIN_LIMIT_2:
			val->val1 = data->iin_limit_2_ma;
			break;
		case CP7420_CHAN_IIN_LIMIT_PEAK:
			val->val1 = data->iin_limit_peak_ma;
			break;
		case CP7420_CHAN_IBAT_DISCHARGE_LIMIT_1:
			val->val1 = data->ibat_dischg_limit_1_ma;
			break;
		case CP7420_CHAN_IBAT_DISCHARGE_LIMIT_2:
			val->val1 = data->ibat_dischg_limit_2_ma;
			break;
		case CP7420_CHAN_VIN_LOW:
			val->val1 = data->vin_low_mv;
			break;
		case CP7420_CHAN_VSYSMIN:
			val->val1 = data->vsys_low_mv;
			break;
		default:
			goto chan_err;
		}
		break;
	case CP7420_ATTR_PROCHOT_DEBOUNCE:
		switch (chan) {
		case CP7420_CHAN_PROCHOT_ALL:
			val->val1 = data->prochot_debounce;
		default:
			goto chan_err;
		}
		break;
	case CP7420_ATTR_PROCHOT_DURATION:
		switch (chan) {
		case CP7420_CHAN_IIN_LIMIT_1:
			val->val1 = data->t_iin_max_1;
			break;
		case CP7420_CHAN_IIN_LIMIT_2:
			val->val1 = data->t_iin_max_2;
			break;
		case CP7420_CHAN_PROCHOT_ALL:
			val->val1 = data->prochot_duration;
			break;
		default:
			goto chan_err;
		}
		break;
	default:
		LOG_ERR("Attribute: %d is not supported", attr);
		return -ENOTSUP;
	}

    return 0;

chan_err:
	LOG_ERR("Channel: %d feature mask is not supported", chan);
	return -ENOTSUP;
}

static int cp7420_apply_dt(const struct device *dev)
{
	const struct cp7420_config *const config = dev->config;
	int ret;

	ret = cp7420_set_ibat_chg_cfg(dev, config->ibat_chg_max_ma);
	if (ret)
		return ret;

	ret = cp7420_set_vbat_reg(dev, config->vbat_reg_max_mv);
	if (ret)
		return ret;

	ret = cp7420_cfg_vsys_max(dev);
	if (ret)
		return ret;

	ret = cp7420_cfg_vsys_min(dev);
	if (ret)
		return ret;

	ret = cp7420_cfg_vbat_low(dev);
	if (ret)
		return ret;

	ret = cp7420_set_iprechg_cfg(dev, config->iprechg_ma);
	if (ret)
		return ret;

	ret = cp7420_cfg_batt_chg_ctrl_3(dev);
	if (ret)
		return ret;

	ret = cp7420_cfg_batt_chg_ctrl_4(dev);
	if (ret)
		return ret;

	ret = cp7420_cfg_batt_chg_ctrl_5(dev);
	if (ret)
		return ret;

	ret = cp7420_cfg_temp_reg(dev);
	if (ret)
		return ret;

	ret = cp7420_cfg_dpm_ctrl(dev);
	if (ret)
		return ret;

	ret = cp7420_cfg_startup_ctrl(dev);
	if (ret)
		return ret;

	ret = cp7420_cfg_dither_ctrl(dev);
	if (ret)
		return ret;

	ret = cp7420_cfg_power_mon_ctrl(dev);
	if (ret)
		return ret;

	ret = cp7420_cfg_t_ibat(dev);
	if (ret)
		return ret;

	ret = cp7420_cfg_switch_curr_ctrl_1(dev);
	if (ret)
		return ret;

	return cp7420_cfg_switch_curr_ctrl_2(dev);
}


/**
 * @brief Initialize the battery charger
 * 
 * @param dev CP7420 device to access
 * @return 0 for success
 * @return -EINVAL if the I2C controller could not be found
 */
static int cp7420_charger_init(const struct device *dev)
{
    const struct cp7420_config *const config = dev->config;
    int ret;

    if (!device_is_ready(config->i2c.bus)) {
        LOG_ERR("Bus device is not ready");
        return -ENODEV;
    }

	ret = cp7420_apply_dt(dev);
	if (ret) {
		LOG_ERR("Failed to apply devicetree: %d", ret);
		return -EIO;
	}

    return 0;
}

static const struct sensor_driver_api cp7420_charger_driver_api = {
    .attr_get = cp7420_attribute_get,
    .attr_set = cp7420_attribute_set,
    .channel_get = cp7420_channel_get,
};

#define CP7420_INIT(n)						                        	\
	static struct cp7420_data cp7420_data_##n;			            	\
									                                	\
	static const struct cp7420_config cp7420_config_##n = {         	\
		.i2c = I2C_DT_SPEC_INST_GET(n),                             	\
        .ibat_chg_max_ma = DT_INST_PROP(n, charge_current_milliamps),  	\
        .vbat_reg_max_mv = DT_INST_PROP(n, charge_voltage_millivolts), 	\
        .vsys_max_mv = DT_INST_PROP(n, maximum_system_millivolts),  	\
        .vsys_min_mv = DT_INST_PROP(n, minimum_system_millivolts),  	\
        .vin_ovp_en = DT_INST_PROP(n, enable_input_ovp),				\
        .vsys_ovp_en = DT_INST_PROP(n, enable_system_ovp),				\
        .vbat_ovp_en = DT_INST_PROP(n, enable_battery_ovp),				\
        .sw_ocp_en = DT_INST_PROP(n, enable_switching_ocp),				\
        .vin_ov_cfg = DT_INST_PROP(n, input_ovp_config),				\
        .vbat_low = DT_INST_PROP(n, low_battery_threshold),				\
        .iprechg_ma = DT_INST_PROP(n, precharge_current_milliamps),		\
        .vbat_ov_cfg = DT_INST_PROP(n, battery_ovp_config),				\
        .recharge_en = DT_INST_PROP(n, enable_recharge),				\
        .recharge_offset = DT_INST_PROP(n, recharge_offset),			\
        .recharge_deglitch = DT_INST_PROP(n, recharge_deglitch),		\
        .vsys_ov_cfg = DT_INST_PROP(n, system_ovp_config),				\
        .iterm_en = DT_INST_PROP(n, enable_charge_termination),			\
        .iterm_ma = DT_INST_PROP(n, termination_current_milliamps),		\
        .auto_chg_timer_en = DT_INST_PROP(n, enable_auto_charge_timer),	\
        .prechg_timer = DT_INST_PROP(n, precharge_timer_config),		\
        .fast_chg_timer = DT_INST_PROP(n, fast_charge_timer_config),	\
        .ntc_en = DT_INST_PROP(n, enable_ntc_feedback),					\
        .tsdie_en = DT_INST_PROP(n, enable_tsdie_protection),			\
        .batgone_en = DT_INST_PROP(n, enable_batgone_detection),		\
        .watchdog_timer = DT_INST_PROP(n, watchdog_timer_config),		\
        .vin_dpm_threshold = DT_INST_PROP(n, input_voltage_dpm_config),	\
        .fsw_cfg = DT_INST_PROP(n, switching_frequency_config),			\
        .ibat_softstart = DT_INST_PROP(n, ibat_softstart_config),		\
        .rechg_ibat_softstart = DT_INST_PROP(n, rechg_ibat_softstart),	\
        .vsys_softstart = DT_INST_PROP(n, system_softstart),			\
        .dither_en = DT_INST_PROP(n, enable_dithering),					\
        .dither_rate = DT_INST_PROP(n, dither_rate_config),				\
        .dither_limit = DT_INST_PROP(n, dither_limit_config),			\
        .psys_en = DT_INST_PROP(n, psys_output_desired),				\
        .rsi_cfg = DT_INST_PROP(n, input_sense_resistor_config),		\
        .rso_cfg = DT_INST_PROP(n, output_sense_resistor_config),		\
        .psys_ratio = DT_INST_PROP(n, psys_output_ratio_config),		\
        .ocp_pwm_cfg = DT_INST_PROP(n, ocp_pwm_config),					\
        .ocp_pfm_cfg = DT_INST_PROP(n, ocp_pfm_config),					\
        .t_ibat_max1 = DT_INST_PROP(n, prochot_discharge_lower_limit),	\
        .t_ibat_max2 = DT_INST_PROP(n, prochot_discharge_upper_limit),	\
	};								                                	\
									                                	\
	DEVICE_DT_INST_DEFINE(n, &cp7420_charger_init,                  	\
			    NULL,					                            	\
			    &cp7420_data_##n,				                    	\
			    &cp7420_config_##n, POST_KERNEL,                    	\
			    CONFIG_SENSOR_INIT_PRIORITY,                        	\
			    &cp7420_charger_driver_api);

DT_INST_FOREACH_STATUS_OKAY(CP7420_INIT)
