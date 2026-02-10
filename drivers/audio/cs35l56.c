/*
 * Copyright (c) 2025 Cirrus Logic, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT cirrus_cs35l56

#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/audio/codec.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/regulator.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/device_runtime.h>
#if DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/byteorder.h>
#endif /* DT_ANY_INST_ON_BUS_STATUS_OKAY */

#include "cs35l56.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(cirrus_cs35l56, CONFIG_AUDIO_CODEC_LOG_LEVEL);

union cs35l56_bus {
#if DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)
	struct i2c_dt_spec i2c;
#endif /* DT_ANY_INST_ON_BUS_STATUS_OKAY */
};

typedef bool (*cs35l56_bus_is_ready_fn)(const union cs35l56_bus *bus);

struct cs35l56_config {
	struct gpio_dt_spec reset_gpio;
	const struct device *vdd_amp;
	const struct device *vdd_b;
	const struct device *vdd_a;
	const struct device *vdd_p;
	const union cs35l56_bus bus;
	cs35l56_bus_is_ready_fn bus_is_ready;
	uint8_t preempt_config;
};

struct cs35l56_reg_sequence {
	uint32_t addr;
	uint32_t data;
};

struct cs35l56_data {
	struct cs35l56_reg_sequence *asp_context;
	int asp_context_size;
	uint8_t asp1_rx[64];
	uint8_t asp1_tx[64];
	bool fw_patched;
};

#define REG_SEQ(_addr, _data)                                                                      \
	{                                                                                          \
		.addr = _addr,                                                                     \
		.data = _data,                                                                     \
	}

static uint32_t cs35l56_bclk_freq_hz[] = {
	[0xc] = 128000,    [0xf] = 256000,    [0x11] = 384000,   [0x12] = 512000,
	[0x15] = 768000,   [0x17] = 1024000,  [0x19] = 1411200,  [0x1a] = 1500000,
	[0x1b] = 1536000,  [0x1c] = 2000000,  [0x1d] = 2048000,  [0x1e] = 2400000,
	[0x1f] = 2822400,  [0x20] = 3000000,  [0x21] = 3072000,  [0x23] = 4000000,
	[0x24] = 4096000,  [0x25] = 4800000,  [0x26] = 5644800,  [0x27] = 6000000,
	[0x28] = 6144000,  [0x29] = 6250000,  [0x2a] = 6400000,  [0x2d] = 7526400,
	[0x2e] = 8000000,  [0x2f] = 8192000,  [0x30] = 9600000,  [0x31] = 11289600,
	[0x32] = 12000000, [0x33] = 12288000, [0x37] = 13500000, [0x38] = 19200000,
	[0x39] = 22579200, [0x3b] = 24576000,
};

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)
static int cs35l56_reg_read(const struct device *dev, uint32_t reg_addr, uint32_t *val)
{
	uint8_t read_buf[sizeof(uint32_t)], write_buf[sizeof(uint32_t)];
	const struct cs35l56_config *config = dev->config;
	int ret;

	sys_put_be32(reg_addr, write_buf);

	ret = i2c_write_read_dt(&config->bus.i2c, write_buf, sizeof(uint32_t), read_buf,
				sizeof(uint32_t));
	if (ret < 0) {
		return ret;
	}

	*val = sys_get_be32(read_buf);

	return 0;
}

static int cs35l56_reg_write(const struct device *dev, uint32_t reg_addr, uint32_t val)
{
	const struct cs35l56_config *config = dev->config;
	uint64_t msg = ((uint64_t)reg_addr << 32) | val;
	uint8_t buf[sizeof(uint64_t)];

	sys_put_be64(msg, buf);

	return i2c_write_dt(&config->bus.i2c, buf, sizeof(uint64_t));
}

static bool cs35l56_bus_is_ready_i2c(const union cs35l56_bus *bus)
{
	return device_is_ready(bus->i2c.bus);
}

__maybe_unused static int cs35l56_burst_write(const struct device *dev, const uint32_t reg_addr,
					      const uint8_t *buf, unsigned int num_bytes)
{
	const struct cs35l56_config *config = dev->config;
	uint8_t addr_buf[sizeof(reg_addr)];
	struct i2c_msg msg[2];

	LOG_INF("%s: reg=0x%x size=%d", __func__, reg_addr, num_bytes);

	sys_put_be32(reg_addr, addr_buf);

	msg[0].buf = addr_buf;
	msg[0].len = 4;
	msg[0].flags = I2C_MSG_WRITE;

	msg[1].buf = (uint8_t *)buf;
	msg[1].len = num_bytes;
	msg[1].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	return i2c_transfer_dt(&config->bus.i2c, msg, 2);
}
#endif /* DT_ANY_INST_ON_BUS_STATUS_OKAY */

static int cs35l56_reg_update(const struct device *dev, uint32_t reg_addr, uint32_t mask,
			      uint32_t val)
{
	uint32_t tmp, orig;
	int ret;

	ret = cs35l56_reg_read(dev, reg_addr, &orig);
	if (ret < 0) {
		return ret;
	}

	tmp = orig & ~mask;
	tmp |= val & mask;

	return cs35l56_reg_write(dev, reg_addr, tmp);
}

static int cs35l56_asp_context_restore(const struct device *dev)
{
	struct cs35l56_data *data = dev->data;
	int ret;

	for (int i = 0; i < data->asp_context_size; i++) {
		ret = cs35l56_reg_write(dev, data->asp_context[i].addr, data->asp_context[i].data);
		if (ret < 0) {
			return ret;
		}
	}

	return 0;
}

static int cs35l56_asp_context_get_idx(const struct device *dev, const uint32_t addr)
{
	struct cs35l56_data *data = dev->data;

	for (int i = 0; i < data->asp_context_size; i++) {
		if (addr == data->asp_context[i].addr) {
			return i;
		}
	}

	return -EINVAL;
}

static int cs35l56_asp_context_write(const struct device *dev, const uint32_t addr,
				     const uint32_t val)
{
	int i = cs35l56_asp_context_get_idx(dev, addr);
	struct cs35l56_data *data = dev->data;

	if (i < 0) {
		return i;
	}

	data->asp_context[i].data = val;

	return 0;
}

static int cs35l56_asp_context_update(const struct device *dev, const uint32_t addr,
				      const uint32_t mask, const uint32_t val)
{
	uint32_t tmp, orig;
	int ret;

	ret = cs35l56_reg_read(dev, addr, &orig);
	if (ret < 0) {
		return ret;
	}

	tmp = orig & ~mask;
	tmp |= val & mask;

	return cs35l56_asp_context_write(dev, addr, tmp);
}

static void cs35l56_log_dsp_status(const struct device *dev)
{
	uint32_t val;

	k_msleep(50);
	cs35l56_reg_read(dev, CS35L56_HALO_STATE, &val);
	LOG_INF("HALO_STATE: %x", val);
	cs35l56_reg_read(dev, CS35L56_PM_PM_CUR_STATE, &val);
	LOG_INF("CS35L56_PM_PM_CUR_STATE: %x", val);
	cs35l56_reg_read(dev, CS35L56_DSP1_FW_VER, &val);
	LOG_INF("CS35L56_DSP1_FW_VER: %ld.%ld.%ld", FIELD_GET(CS35L56_DSP1_FW_REV_MAJOR_MASK, val),
		FIELD_GET(CS35L56_DSP1_FW_REV_MINOR_MASK, val),
		FIELD_GET(CS35L56_DSP1_FW_REV_FIX_MASK, val));
}

#ifndef CONFIG_AUDIO_CODEC_CS35L56_DELEGATE_FW_LOADING
static int cs35l56_fw_reset(const struct device *dev)
{
	int i = 0, ret;
	uint32_t val;

	ret = cs35l56_reg_write(dev, CS35L56_HALO_STATE, CS35L56_DSP_STATE_PREBOOT);
	if (ret < 0) {
		return ret;
	}

	ret = cs35l56_reg_write(dev, CS35L56_DSP_VIRTUAL1_MBOX_1,
				CS35L56_DSP_MBOX_CMD_SYSTEM_RESET);
	if (ret < 0) {
		return ret;
	}

	while (i < CS35L56_DSP_SYSTEM_RESET_RETIRES) {
		ret = cs35l56_reg_read(dev, CS35L56_DSP_VIRTUAL1_MBOX_1, &val);
		if (ret < 0) {
			return ret;
		}

		if (val == 0) {
			return 0;
		}

		k_usleep(CS35L56_DSP_SYSTEM_RESET_POLL_US);
		i++;
	}

	return -ETIME;
}

static int cs35l56_write_fw_blocks(const struct device *dev, halo_boot_block_t *blocks,
				   int num_blocks)
{
	halo_boot_block_t block;
	uint32_t bytes, address;
	uint8_t *buffer;
	int ret;

	for (int i = 0; i < num_blocks; i++) {
		block = blocks[i];
		bytes = block.block_size;
		address = block.address;
		buffer = block.bytes;
		ret = cs35l56_burst_write(dev, address, buffer, bytes);
		if (ret < 0) {
			return ret;
		}
	}

	return 0;
}

static int cs35l56_fw_download(const struct device *dev)
{
	halo_boot_block_t *blocks;
	int num_blocks;

	num_blocks = cs35l56_total_fw_blocks;
	blocks = cs35l56_fw_blocks;
	return cs35l56_write_fw_blocks(dev, blocks, num_blocks);
}

static int cs35l56_tuning_download(const struct device *dev, audio_channel_t channel)
{
	halo_boot_block_t *blocks;
	int num_blocks;

	num_blocks = cs35l56_total_coeff_blocks[channel];
	blocks = cs35l56_coeff_blocks[channel];

	return cs35l56_write_fw_blocks(dev, blocks, num_blocks);
}

static int cs35l56_apply_tuning(const struct device *dev, audio_channel_t channel)
{
	struct cs35l56_data *data = dev->data;
	int ret;

	ret = cs35l56_tuning_download(dev, channel);
	if (ret < 0) {
		return ret;
	}

	ret = cs35l56_fw_reset(dev);
	if (ret < 0) {
		return ret;
	}

	cs35l56_log_dsp_status(dev);

	data->fw_patched = true;

	return 0;
}

static int cs35l56_fw_download_prepare(const struct device *dev)
{
	int i = 0, ret;
	uint32_t val;

	ret = cs35l56_reg_write(dev, CS35L56_DSP_VIRTUAL1_MBOX_1, CS35L56_DSP_MBOX_CMD_SHUTDOWN);
	if (ret < 0) {
		return ret;
	}

	while (i < CS35L56_DSP_SHUTDOWN_RETRIES) {
		ret = cs35l56_reg_read(dev, CS35L56_PM_PM_CUR_STATE, &val);
		if (ret < 0) {
			return ret;
		}

		if (val == CS35L56_PM_STATE_SHUTDOWN) {
			break;
		}

		k_usleep(CS35L56_DSP_SHUTDOWN_POLL_US);
		i++;
	}

	if (val != CS35L56_PM_STATE_SHUTDOWN) {
		return -ETIME;
	}

	return 0;
}
#endif

static int cs35l56_route_input(const struct device *dev, audio_channel_t channel, uint32_t input)
{
	struct cs35l56_data *data = dev->data;
	int ret;

	ret = pm_device_runtime_get(dev);
	if (ret < 0) {
		return ret;
	}

	switch (input) {
	case CS35L56_ASP1_TX1:
		ret = cs35l56_asp_context_update(dev, CS35L56_ASP1_FRAME_CONTROL1,
						 CS35L56_ASP1_TX1_SLOT, channel);
		if (ret < 0) {
			return ret;
		}
		break;
	case CS35L56_ASP1_TX2:
		ret = cs35l56_asp_context_update(dev, CS35L56_ASP1_FRAME_CONTROL1,
						 CS35L56_ASP1_TX2_SLOT,
						 channel << CS35L56_ASP1_TX2_SHIFT);
		if (ret < 0) {
			return ret;
		}
		break;
	case CS35L56_ASP1_TX3:
		ret = cs35l56_asp_context_update(dev, CS35L56_ASP1_FRAME_CONTROL1,
						 CS35L56_ASP1_TX3_SLOT,
						 channel << CS35L56_ASP1_TX3_SHIFT);
		if (ret < 0) {
			return ret;
		}
		break;
	case CS35L56_ASP1_TX4:
		ret = cs35l56_asp_context_update(dev, CS35L56_ASP1_FRAME_CONTROL1,
						 CS35L56_ASP1_TX4_SLOT,
						 channel << CS35L56_ASP1_TX4_SHIFT);
		if (ret < 0) {
			return ret;
		}
		break;
	default:
		return -EINVAL;
	}

	data->asp1_tx[channel] = input;

	return pm_device_runtime_put(dev);
}

static int cs35l56_route_output(const struct device *dev, audio_channel_t channel, uint32_t output)
{
	struct cs35l56_data *data = dev->data;
	int ret;

	ret = pm_device_runtime_get(dev);
	if (ret < 0) {
		return ret;
	}

	switch (output) {
	case CS35L56_ASP1_RX1:
		ret = cs35l56_asp_context_update(dev, CS35L56_ASP1_FRAME_CONTROL5,
						 CS35L56_ASP1_RX1_SLOT, channel);
		if (ret < 0) {
			return ret;
		}
		break;
	case CS35L56_ASP1_RX2:
		ret = cs35l56_asp_context_update(dev, CS35L56_ASP1_FRAME_CONTROL5,
						 CS35L56_ASP1_RX2_SLOT,
						 channel << CS35L56_ASP1_RX2_SHIFT);
		if (ret < 0) {
			return ret;
		}
		break;
	case CS35L56_ASP1_RX3:
		ret = cs35l56_asp_context_update(dev, CS35L56_ASP1_FRAME_CONTROL5,
						 CS35L56_ASP1_RX3_SLOT,
						 channel << CS35L56_ASP1_RX3_SHIFT);
		if (ret < 0) {
			return ret;
		}
		break;
	default:
		return -EINVAL;
	}

	data->asp1_rx[channel] = output;
#ifndef CONFIG_AUDIO_CODEC_CS35L56_DELEGATE_FW_LOADING
	ret = cs35l56_apply_tuning(dev, channel);
	if (ret < 0) {
		return ret;
	}
#endif
	return pm_device_runtime_put(dev);
}

static int cs35l56_apply_properties(const struct device *dev)
{
	return 0;
}

static int cs35l56_asp_alt_set_volume(const struct device *dev, audio_channel_t channel,
				      audio_property_value_t audio_val)
{
	int ret;

	if (channel != AUDIO_CHANNEL_ALL) {
		return -EINVAL;
	}

	/* Value must be decibels in S7.8 format */
	ret = cs35l56_reg_write(dev, CS35L56_ASP_ALT_VOLUME, (uint32_t)audio_val.vol);
	if (ret < 0) {
		return ret;
	}

	return cs35l56_asp_context_write(dev, CS35L56_ASP_ALT_VOLUME, (uint32_t)audio_val.vol);
}

static int cs35l56_asp1_tx_set_mute(const struct device *dev, audio_channel_t channel,
				    audio_property_value_t audio_val)
{
	struct cs35l56_data *data = dev->data;
	uint8_t input = data->asp1_tx[channel];
	uint32_t val;

	if (channel == AUDIO_CHANNEL_ALL) {
		if (audio_val.mute) {
			val = 0;
		} else {
			val = CS35L56_ASP1_TX_EN;
		}
		return cs35l56_asp_context_update(dev, CS35L56_ASP1_ENABLES1, CS35L56_ASP1_TX_EN,
						  val);
	}

	if (input == 0) {
		return -EINVAL;
	}

	switch (input) {
	case CS35L56_ASP1_TX1:
		if (audio_val.mute) {
			val = 0;
		} else {
			val = CS35L56_ASP1_TX1_EN;
		}
		return cs35l56_asp_context_update(dev, CS35L56_ASP1_ENABLES1, CS35L56_ASP1_TX1_EN,
						  val);
	case CS35L56_ASP1_TX2:
		if (audio_val.mute) {
			val = 0;
		} else {
			val = CS35L56_ASP1_TX2_EN;
		}
		return cs35l56_asp_context_update(dev, CS35L56_ASP1_ENABLES1, CS35L56_ASP1_TX2_EN,
						  val);
	case CS35L56_ASP1_TX3:
		if (audio_val.mute) {
			val = 0;
		} else {
			val = CS35L56_ASP1_TX3_EN;
		}
		return cs35l56_asp_context_update(dev, CS35L56_ASP1_ENABLES1, CS35L56_ASP1_TX3_EN,
						  val);
	case CS35L56_ASP1_TX4:
		if (audio_val.mute) {
			val = 0;
		} else {
			val = CS35L56_ASP1_TX4_EN;
		}
		return cs35l56_asp_context_update(dev, CS35L56_ASP1_ENABLES1, CS35L56_ASP1_TX4_EN,
						  val);
	default:
		return -EINVAL;
	}

	return 0;
}

static int cs35l56_asp1_rx_set_mute(const struct device *dev, audio_channel_t channel,
				    audio_property_value_t audio_val)
{
	struct cs35l56_data *data = dev->data;
	uint8_t input = data->asp1_rx[channel];
	uint32_t val;

	if (channel == AUDIO_CHANNEL_ALL) {
		if (audio_val.mute) {
			val = 0;
		} else {
			val = CS35L56_ASP1_RX_EN;
		}
		return cs35l56_asp_context_update(dev, CS35L56_ASP1_ENABLES1, CS35L56_ASP1_RX_EN,
						  val);
	}

	if (input == 0) {
		return -EINVAL;
	}

	switch (input) {
	case CS35L56_ASP1_RX1:
		if (audio_val.mute) {
			val = 0;
		} else {
			val = CS35L56_ASP1_RX1_EN;
		}
		return cs35l56_asp_context_update(dev, CS35L56_ASP1_ENABLES1, CS35L56_ASP1_RX1_EN,
						  val);
	case CS35L56_ASP1_RX2:
		if (audio_val.mute) {
			val = 0;
		} else {
			val = CS35L56_ASP1_RX2_EN;
		}
		return cs35l56_asp_context_update(dev, CS35L56_ASP1_ENABLES1, CS35L56_ASP1_RX2_EN,
						  val);
	case CS35L56_ASP1_RX3:
		if (audio_val.mute) {
			val = 0;
		} else {
			val = CS35L56_ASP1_RX3_EN;
		}
		return cs35l56_asp_context_update(dev, CS35L56_ASP1_ENABLES1, CS35L56_ASP1_RX3_EN,
						  val);
	default:
		return -EINVAL;
	}

	return 0;
}

static int cs35l56_set_property(const struct device *dev, audio_property_t property,
				audio_channel_t channel, audio_property_value_t val)
{
	int ret;

	ret = pm_device_runtime_get(dev);
	if (ret < 0) {
		return ret;
	}

	switch (property) {
	case AUDIO_PROPERTY_OUTPUT_MUTE:
		return cs35l56_asp1_rx_set_mute(dev, channel, val);
	case AUDIO_PROPERTY_INPUT_MUTE:
		return cs35l56_asp1_tx_set_mute(dev, channel, val);
#ifdef CONFIG_AUDIO_CODEC_CS35L56_SOUNDWIRE_ASP_ARBITRATION
	case AUDIO_PROPERTY_OUTPUT_VOLUME:
		return cs35l56_asp_alt_set_volume(dev, channel, val);
#endif
	default:
		return -ENOTSUP;
	}

	return pm_device_runtime_put(dev);
}

#ifdef CONFIG_AUDIO_CODEC_CS35L56_SOUNDWIRE_ASP_ARBITRATION
static int cs35l56_disable_sdca_power_settings(const struct device *dev)
{
	/* Undoing SDCA host power settings */
	cs35l56_reg_update(dev, CS35L56_AUX_NGATE_CH1_CFG, CS35L56_AUX_NGATE_CHx_EN, 0);
	cs35l56_reg_update(dev, CS35L56_AUX_NGATE_CH2_CFG, CS35L56_AUX_NGATE_CHx_EN, 0);
	cs35l56_reg_write(dev, CS35L56_LDPM_CONFIG, 0x10606);

	return 0;
}
#endif

static void cs35l56_stop_output(const struct device *dev)
{
#ifdef CONFIG_AUDIO_CODEC_CS35L56_SOUNDWIRE_ASP_ARBITRATION
	struct cs35l56_data *data = dev->data;

	if (data->fw_patched) {
		cs35l56_reg_write(dev, CS35L56_DSP_VIRTUAL1_MBOX_1,
				  CS35L56_DSP_MBOX_CMD_PAUSE_ASP_ALT);
	} else {
		LOG_DBG("RAM Firmware not booted, failed to stop output");
	}
#else
	cs35l56_reg_write(dev, CS35L56_DSP_VIRTUAL1_MBOX_1, CS35L56_DSP_MBOX_CMD_PAUSE);
#endif

	pm_device_runtime_put(dev);
}

static void cs35l56_start_output(const struct device *dev)
{
	pm_device_runtime_get(dev);
#ifdef CONFIG_AUDIO_CODEC_CS35L56_SOUNDWIRE_ASP_ARBITRATION
	struct cs35l56_data *data = dev->data;
	uint32_t val;
	int ret;

	if (data->fw_patched) {
		ret = cs35l56_reg_read(dev, CS35L56_PDE23_TRANSDUCER_REQUESTED_PS, &val);
		if (ret < 0) {
			LOG_DBG("Unable to determine PDE23 power state");
			return;
		}

		if (val == CS35L56_PDE23_STATE_OFF) {
			cs35l56_asp_context_restore(dev);
			cs35l56_log_dsp_status(dev);
			cs35l56_disable_sdca_power_settings(dev);
			cs35l56_reg_write(dev, CS35L56_DSP_VIRTUAL1_MBOX_1,
					  CS35L56_DSP_MBOX_CMD_PLAY_ASP_ALT);
			cs35l56_reg_update(dev, CS35L56_BLOCK_ENABLES2, CS35L56_ASP_EN,
					   CS35L56_ASP_EN);
		} else {
			LOG_ERR("PDE23 State: %x", val);
		}
	} else {
		LOG_DBG("RAM Firmware not booted, failed to start output");
	}
#else
	cs35l56_reg_write(dev, CS35L56_DSP_VIRTUAL1_MBOX_1, CS35L56_DSP_MBOX_CMD_PLAY);
#endif
}

static int cs35l56_asp1_set_clks(const struct device *dev, struct audio_codec_cfg *cfg)
{
	struct i2s_config i2s = cfg->dai_cfg.i2s;
	uint8_t asp1_bclk_freq = 0, clk_opt = 0;
	uint32_t bclk_freq_hz;
	int ret;

	if (i2s.frame_clk_freq != AUDIO_PCM_RATE_48K) {
		return -EINVAL;
	}

	if (i2s.word_size == AUDIO_PCM_WIDTH_16_BITS) {
		bclk_freq_hz = AUDIO_PCM_RATE_48K * i2s.channels * i2s.word_size;
	} else {
		bclk_freq_hz = AUDIO_PCM_RATE_48K * i2s.channels * AUDIO_PCM_WIDTH_32_BITS;
	}

	for (int i = 0xc; i < ARRAY_SIZE(cs35l56_bclk_freq_hz); i++) {
		if (cs35l56_bclk_freq_hz[i] == bclk_freq_hz) {
			asp1_bclk_freq = i;
			break;
		}
	}

	if (asp1_bclk_freq == 0) {
		return -EINVAL;
	}

	ret = cs35l56_asp_context_write(dev, CS35L56_ASP1_CONTROL1, asp1_bclk_freq);
	if (ret < 0) {
		return ret;
	}

	if (i2s.options & I2S_OPT_BIT_CLK_MASTER) {
		clk_opt |= CS35L56_ASP1_BCLK_MSTR;
	}

	if (i2s.options & I2S_OPT_FRAME_CLK_MASTER) {
		clk_opt |= CS35L56_ASP1_FSYNC_MSTR;
	}

	if (i2s.options & I2S_OPT_BIT_CLK_CONT) {
		clk_opt |= CS35L56_ASP1_BCLK_FRC;
	}

	if (FIELD_GET(I2S_FMT_CLK_FORMAT_MASK, i2s.format) & I2S_FMT_BIT_CLK_INV) {
		clk_opt |= CS35L56_ASP1_BCLK_INV;
	}

	if (FIELD_GET(I2S_FMT_CLK_FORMAT_MASK, i2s.format) & I2S_FMT_FRAME_CLK_INV) {
		clk_opt |= CS35L56_ASP1_FSYNC_INV;
	}

	return cs35l56_asp_context_update(dev, CS35L56_ASP1_CONTROL2, CS35L56_BCLK_FSYNC_MASK,
					  clk_opt);
}

static int cs35l56_asp1_set_word(const struct device *dev, struct audio_codec_cfg *cfg)
{
	struct i2s_config i2s = cfg->dai_cfg.i2s;
	uint8_t asp1_fmt, asp1_width;
	uint32_t val = 0;
	int ret;

	if (i2s.word_size > CS35L56_ASP1_WL_MAX) {
		i2s.word_size = CS35L56_ASP1_WL_MAX;
	}

	if (!IN_RANGE(i2s.word_size, CS35L56_ASP1_WL_MIN, CS35L56_ASP1_WL_MAX)) {
		return -EINVAL;
	}

	if (i2s.word_size == AUDIO_PCM_WIDTH_16_BITS) {
		asp1_width = i2s.word_size;
	} else {
		asp1_width = AUDIO_PCM_WIDTH_32_BITS;
	}

	if (!IN_RANGE(asp1_width, CS35L56_ASP1_WIDTH_MIN, CS35L56_ASP1_WIDTH_MAX)) {
		return -EINVAL;
	}

	switch (cfg->dai_route) {
	case AUDIO_ROUTE_PLAYBACK:
		ret = cs35l56_asp_context_write(dev, CS35L56_ASP1_DATA_CONTROL5, i2s.word_size);
		if (ret < 0) {
			return ret;
		}

		val = FIELD_PREP(CS35L56_ASP1_RX_WIDTH, asp1_width);
		break;
	case AUDIO_ROUTE_PLAYBACK_CAPTURE:
		ret = cs35l56_asp_context_write(dev, CS35L56_ASP1_DATA_CONTROL5, i2s.word_size);
		if (ret < 0) {
			return ret;
		}

		ret = cs35l56_asp_context_write(dev, CS35L56_ASP1_DATA_CONTROL1, i2s.word_size);
		if (ret < 0) {
			return ret;
		}

		val = FIELD_PREP(CS35L56_ASP1_RX_WIDTH, asp1_width);
		val |= FIELD_PREP(CS35L56_ASP1_TX_WIDTH, asp1_width);
		break;
	case AUDIO_ROUTE_CAPTURE:
		ret = cs35l56_asp_context_write(dev, CS35L56_ASP1_DATA_CONTROL1, i2s.word_size);
		if (ret < 0) {
			return ret;
		}

		val = FIELD_PREP(CS35L56_ASP1_TX_WIDTH, asp1_width);
		break;
	default:
		return -EINVAL;
	}

	switch (FIELD_GET(I2S_FMT_DATA_FORMAT_MASK, i2s.format)) {
	case I2S_FMT_DATA_FORMAT_I2S:
		asp1_fmt = CS35L56_ASP1_FMT_I2S;
		break;
	case I2S_FMT_DATA_FORMAT_PCM_SHORT:
		asp1_fmt = CS35L56_ASP1_FMT_TDM15;
		break;
	case I2S_FMT_DATA_FORMAT_PCM_LONG:
		asp1_fmt = CS35L56_ASP1_FMT_DSPA;
		break;
	default:
		return -ENOTSUP;
	}

	val |= FIELD_PREP(CS35L56_ASP1_FMT_MASK, asp1_fmt);

	return cs35l56_asp_context_update(dev, CS35L56_ASP1_CONTROL2,
					  (CS35L56_ASP1_FMT_MASK | CS35L56_ASP1_WIDTH), val);
}

static int cs35l56_configure(const struct device *dev, struct audio_codec_cfg *cfg)
{
	int ret;

	ret = pm_device_runtime_get(dev);
	if (ret < 0) {
		return ret;
	}

	ret = cs35l56_asp1_set_clks(dev, cfg);
	if (ret < 0) {
		LOG_ERR("Failed to set clocks");
		return ret;
	}

	ret = cs35l56_asp1_set_word(dev, cfg);
	if (ret < 0) {
		LOG_ERR("Failed to set word");
		return ret;
	}

	return pm_device_runtime_put(dev);
}

static int cs35l56_wait_for_rom_boot(const struct device *dev)
{
	uint32_t val = 0;
	int i = 0, ret;

	while (i < CS35L56_ROM_BOOT_RETRIES) {
		ret = cs35l56_reg_read(dev, CS35L56_HALO_STATE, &val);
		if (ret < 0) {
			return ret;
		}

		if (val == CS35L56_DSP_STATE_RUNNING) {
			return 0;
		}

		k_usleep(CS35L56_ROM_BOOT_POLL_US);
		i++;
	}

	return -EPERM;
}

__maybe_unused static int cs35l56_reset(const struct device *dev)
{
	const struct cs35l56_config *config = dev->config;
	int ret;

	if (config->reset_gpio.port != NULL) {
		if (!gpio_is_ready_dt(&config->reset_gpio)) {
			return -ENODEV;
		}

		ret = gpio_pin_configure_dt(&config->reset_gpio, GPIO_OUTPUT_ACTIVE);
		if (ret == -EBUSY) {
			LOG_DBG("Reset line is busy, assuming shared reset");
			return ret;
		} else if (ret < 0) {
			return ret;
		}

		k_usleep(CS35L56_T_RLPW_US);
		gpio_pin_set_dt(&config->reset_gpio, 0);
		k_usleep(CS35L56_T_IRS_US);
	} else {
		/*
		 * Note that the DSP firmware memory (RAM) contents are retained through software
		 * reset conditions.
		 */
		ret = cs35l56_reg_write(dev, CS35L56_SW_RESET_SFT_RESET_REG, 0x5A000000);
		if (ret < 0) {
			return ret;
		}
	}

	return 0;
}

static int cs35l56_check_ids(const struct device *dev)
{
	uint32_t val;
	int ret;

	ret = cs35l56_reg_read(dev, CS35L56_SW_RESET_DEVID_REG, &val);
	if (ret < 0) {
		return ret;
	}

	switch (val) {
	case 0x35A56:
		break;
	case 0x35A57:
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

static int cs35l56_init_regulators(const struct device *dev)
{
	const struct cs35l56_config *config = dev->config;
	int ret;

	ret = regulator_enable(config->vdd_p);
	if (ret < 0) {
		return ret;
	}

	if (config->vdd_a != NULL) {
		ret = regulator_enable(config->vdd_a);
		if (ret < 0) {
			return ret;
		}
	}

	if (config->vdd_b != NULL) {
		ret = regulator_enable(config->vdd_b);
		if (ret < 0) {
			return ret;
		}
	} else if (config->vdd_amp != NULL) {
		ret = regulator_enable(config->vdd_amp);
		if (ret < 0) {
			return ret;
		}
	} else {
		return -EINVAL;
	}

	return 0;
}

static int cs35l56_init(const struct device *dev)
{
	const struct cs35l56_config *config = dev->config;
	struct cs35l56_data *data = dev->data;
	int ret;

	ret = cs35l56_init_regulators(dev);
	if (ret < 0) {
		LOG_ERR("Failed to enable regulators: %d", ret);
		return ret;
	}
#ifndef CONFIG_AUDIO_CODEC_CS35L56_DELEGATE_FW_LOADING
	ret = cs35l56_reset(dev);
	if (ret < 0) {
		LOG_ERR("Fail to reset: %d", ret);
		return ret;
	}
#endif
	ret = cs35l56_wait_for_rom_boot(dev);
	if (ret < 0) {
		LOG_ERR("Failed to boot from ROM: %d", ret);
		return ret;
	}

	ret = cs35l56_check_ids(dev);
	if (ret < 0) {
		LOG_ERR("Failed to check IDs: %d", ret);
		return ret;
	}
#ifdef CONFIG_AUDIO_CODEC_CS35L56_DELEGATE_FW_LOADING
	data->fw_patched = true;
#else
	ret = cs35l56_fw_download_prepare(dev);
	if (ret < 0) {
		LOG_ERR("Failed to patch fw: %d", ret);
		return ret;
	}

	ret = cs35l56_fw_download(dev);
	if (ret < 0) {
		return ret;
	}
#endif

	ret = cs35l56_reg_write(dev, CS35L56_PREEMPT_CONFIG, config->preempt_config);
	if (ret < 0) {
		return ret;
	}

	return cs35l56_reg_update(dev, CS35L56_BLOCK_ENABLES2, CS35L56_ASP_EN, CS35L56_ASP_EN);
}

#ifdef CONFIG_PM_DEVICE
static void cs35l56_issue_wake_event(const struct device *dev)
{
	uint32_t value;

	(void)cs35l56_reg_read(dev, CS35L56_IRQ1_STATUS, &value);

	k_usleep(CS35L56_WAKE_HOLD_TIME_US);

	(void)cs35l56_reg_read(dev, CS35L56_IRQ1_STATUS, &value);

	k_usleep(CS35L56_T_IRS_US);
}

static int cs35l56_device_pm_action_suspend(const struct device *dev)
{
	return cs35l56_reg_write(dev, CS35L56_DSP_VIRTUAL1_MBOX_1,
				 CS35L56_DSP_MBOX_CMD_ALLOW_HIBER);
}

static int cs35l56_device_pm_action_resume(const struct device *dev)
{
	int ret;

	cs35l56_issue_wake_event(dev);

	ret = cs35l56_wait_for_rom_boot(dev);
	if (ret < 0) {
		return ret;
	}

	return cs35l56_reg_write(dev, CS35L56_DSP_VIRTUAL1_MBOX_1,
				 CS35L56_DSP_MBOX_CMD_PREVENT_HIBER);
}

static int cs35l56_device_pm_action(const struct device *dev, enum pm_device_action action)
{
	switch (action) {
	case PM_DEVICE_ACTION_RESUME:
		return cs35l56_device_pm_action_resume(dev);
	case PM_DEVICE_ACTION_SUSPEND:
		return cs35l56_device_pm_action_suspend(dev);
	default:
		return -ENOTSUP;
	}

	return 0;
}
#endif

static const struct audio_codec_api api = {
	.configure = cs35l56_configure,
	.start_output = cs35l56_start_output,
	.stop_output = cs35l56_stop_output,
	.set_property = cs35l56_set_property,
	.apply_properties = cs35l56_apply_properties,
	.route_input = cs35l56_route_input,
	.route_output = cs35l56_route_output,
};

#define CS35L56_DEVICE_INIT(inst)                                                                  \
	PM_DEVICE_DT_INST_DEFINE(inst, cs35l56_device_pm_action);                                  \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, cs35l56_init, PM_DEVICE_DT_INST_GET(inst),                     \
			      &cs35l56_data_##inst, &cs35l56_config_##inst, POST_KERNEL,           \
			      CONFIG_AUDIO_CODEC_INIT_PRIORITY, &api);

#define CS35L56_CONFIG(inst)                                                                       \
	.vdd_amp = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(vdd_amp)),                                   \
	.vdd_b = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(vdd_b)),                                       \
	.vdd_p = DEVICE_DT_GET(DT_NODELABEL(vdd_p)),                                               \
	.vdd_a = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(vdd_a)),                                       \
	.preempt_config = DT_INST_ENUM_IDX_OR(inst, cirrus_preempt_config, 0),                     \
	.reset_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, reset_gpios, {0}),

#define CS35L56_CONFIG_I2C(inst)                                                                   \
	{.bus = {.i2c = I2C_DT_SPEC_INST_GET(inst)},                                               \
	 .bus_is_ready = cs35l56_bus_is_ready_i2c,                                                 \
	 CS35L56_CONFIG(inst)}

#define CS35L56_DATA(inst)                                                                         \
	{.asp_context = cs35l56_asp_context_##inst,                                                \
	 .asp_context_size = ARRAY_SIZE(cs35l56_asp_context_##inst)}

#define CS35L56_DEFINE_I2C(inst)                                                                   \
	static struct cs35l56_reg_sequence cs35l56_asp_context_##inst[] = {                        \
		REG_SEQ(CS35L56_ASP1_ENABLES1, 0x0),                                               \
		REG_SEQ(CS35L56_ASP1_CONTROL1, 0x28),                                              \
		REG_SEQ(CS35L56_ASP1_CONTROL2, 0x18180200),                                        \
		REG_SEQ(CS35L56_ASP1_FRAME_CONTROL1, 0x3020100),                                   \
		REG_SEQ(CS35L56_ASP1_FRAME_CONTROL5, 0x20100),                                     \
		REG_SEQ(CS35L56_ASP1_DATA_CONTROL1, 0x18),                                         \
		REG_SEQ(CS35L56_ASP1_DATA_CONTROL5, 0x18),                                         \
		REG_SEQ(CS35L56_DSP1RX9_INPUT, CS35L56_DSP1_SRC_ASP1RX1),                          \
		REG_SEQ(CS35L56_DSP1RX10_INPUT, CS35L56_DSP1_SRC_ASP1RX2),                         \
		REG_SEQ(CS35L56_ASP_ALT_VOLUME, 0x0),                                              \
	};                                                                                         \
                                                                                                   \
	static struct cs35l56_config cs35l56_config_##inst = CS35L56_CONFIG_I2C(inst);             \
	static struct cs35l56_data cs35l56_data_##inst = CS35L56_DATA(inst);                       \
	CS35L56_DEVICE_INIT(inst)

#define AUDIO_CODEC_CS35L56_DEFINE(inst) CS35L56_DEFINE_I2C(inst)

DT_INST_FOREACH_STATUS_OKAY(AUDIO_CODEC_CS35L56_DEFINE)
