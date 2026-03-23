/*
 * Copyright (c) 2026, Cirrus Logic, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Core driver for Cirrus Logic CS35L45
 */

#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/audio/codec.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/regulator.h>
#include <zephyr/logging/log.h>

#include "cs35l45.h"

#define DT_DRV_COMPAT cirrus_cs35l45

LOG_MODULE_REGISTER(cirrus_cs35l45, CONFIG_AUDIO_CODEC_LOG_LEVEL);

#define CS35L45_DEVID 0x00000000

#define CS35L45_GLOBAL_ENABLES    0x00002014
#define CS35L45_GLOBAL_EN_MASK    BIT(0)

#define CS35L45_BLOCK_ENABLES      0x00002018
#define CS35L45_BST_EN_MASK        GENMASK(5, 4)
#define CS35L45_BST_DISABLE_FET_ON 0x1

#define CS35L45_BLOCK_ENABLES2 0x0000201C
#define CS35L45_ASP_EN         BIT(27)

#define CS35L45_ERROR_RELEASE  0x00002034
#define CS35L45_GLOBAL_ERR_RLS BIT(11)

#define CS35L45_INTB_GPIO2_MCLK_REF 0x00002434
#define CS35L45_GP2_CTRL            GENMASK(22, 20)
#define CS35L45_OPEN_DRAIN_INT      0x2

#define CS35L45_REFCLK_INPUT         0x00002C04
#define CS35L45_PLL_OPEN_LOOP_MASK   BIT(11)
#define CS35L45_PLL_REFCLK_FREQ_MASK GENMASK(10, 5)
#define CS35L45_PLL_REFCLK_EN_MASK   BIT(4)

#define CS35L45_GLOBAL_SAMPLE_RATE 0x00002C0C
#define CS35L45_GLOBAL_FS_MASK     GENMASK(4, 0)
#define CS35L45_GLOBAL_FS_44P1K    0xB
#define CS35L45_GLOBAL_FS_48K      0x3
#define CS35L45_GLOBAL_FS_96K      0x4

#define CS35L45_BOOST_CCM_CFG 0x00003808
#define CS35L45_BOOST_DCM_CFG 0x0000380C
#define CS35L45_BOOST_OV_CFG  0x0000382C

#define CS35L45_ASP_ENABLES1 0x00004800
#define CS35L45_ASP_RX2_EN   BIT(17)
#define CS35L45_ASP_RX1_EN   BIT(16)
#define CS35L45_ASP_TX5_EN   BIT(4)
#define CS35L45_ASP_TX4_EN   BIT(3)
#define CS35L45_ASP_TX3_EN   BIT(2)
#define CS35L45_ASP_TX2_EN   BIT(1)
#define CS35L45_ASP_TX1_EN   BIT(0)

#define CS35L45_ASP_CONTROL2      0x00004808
#define CS35L45_ASP_WIDTH_RX_MASK GENMASK(31, 24)
#define CS35L45_ASP_WIDTH_TX_MASK GENMASK(23, 16)
#define CS35L45_ASP_WIDTH_MIN     12
#define CS35L45_ASP_WIDTH_MAX     128
#define CS35L45_ASP_FMT_MASK      GENMASK(10, 8)
#define CS35L45_ASP_FMT_DSP_A     0
#define CS35L45_ASP_FMT_I2S       2
#define CS35L45_ASP_FMT_TDM_1_5   4
#define CS35L45_ASP_BCLK_INV      BIT(6)
#define CS35L45_ASP_FSYNC_INV     BIT(2)

#define CS35L45_ASP_CONTROL3       0x0000480C
#define CS35L45_ASP_FRAME_CONTROL1 0x00004810
#define CS35L45_ASP_FRAME_CONTROL2 0x00004814
#define CS35L45_ASP_FRAME_CONTROL5 0x00004820
#define CS35L45_ASP_DATA_CONTROL1  0x00004830

#define CS35L45_ASP_DATA_CONTROL5 0x00004840
#define CS35L45_ASP_WL_MASK       GENMASK(5, 0)
#define CS35L45_ASP_WL_MIN        12
#define CS35L45_ASP_WL_MAX        24

#define CS35L45_DSP1RX1_INPUT 0x00004C40
#define CS35L45_DSP1RX2_INPUT 0x00004C44
#define CS35L45_DSP1RX3_INPUT 0x00004C48
#define CS35L45_DSP1RX4_INPUT 0x00004C4C
#define CS35L45_DSP1RX5_INPUT 0x00004C50
#define CS35L45_DSP1RX6_INPUT 0x00004C54
#define CS35L45_DSP1RX7_INPUT 0x00004C58
#define CS35L45_DSP1RX8_INPUT 0x00004C5C

#define CS35L45_DACPCM1_INPUT         0x00004C00
#define CS35L45_DACPCM1_SRC_MASK      GENMASK(6, 0)
#define CS35L45_DACPCM1_SRC_ZERO_FILL 0x0
#define CS35L45_DACPCM1_SRC_ASP_RX1   0x8
#define CS35L45_DACPCM1_SRC_ASP_RX2   0x9
#define CS35L45_DACPCM1_SRC_DSP_TX1   0x32
#define CS35L45_DACPCM1_SRC_DSP_TX2   0x33

#define CS35L45_ASPTX1_INPUT 0x00004C20
#define CS35L45_ASPTX2_INPUT 0x00004C24
#define CS35L45_ASPTX3_INPUT 0x00004C28
#define CS35L45_ASPTX4_INPUT 0x00004C2C
#define CS35L45_ASPTX5_INPUT 0x00004C30
#define CS35L45_LDPM_CONFIG  0x00006404

#define CS35L45_AMP_PCM_CONTROL  0x00007000
#define CS35L45_AMP_VOL_PCM_MIN  -816
#define CS35L45_AMP_VOL_PCM_MAX  96
#define CS35L45_AMP_VOL_PCM_MASK GENMASK(10, 0)

#define CS35L45_AMP_PCM_HPF_TST 0x00007004

#define CS35L45_AMP_GAIN     0x00007800
#define CS35L45_AMP_GAIN_PCM GENMASK(9, 8)

#define CS35L45_AMP_OUTPUT_MUTE 0x00007C04
#define CS35L45_AMP_MUTE        BIT(0)

#define CS35L45_HPF_44P1    0x000108BD
#define CS35L45_HPF_88P2    0x0001045F
#define CS35L45_HPF_DEFAULT 0x0

#define CS35L45_IRQ1_STATUS 0x0000E004

#define CS35L45_IRQ1_EINT_1                0x0000E010
#define CS35L45_AMP_SHORT_ERR_EINT1        BIT(31)
#define CS35L45_UVLO_VDDBATT_ERR_EINT1     BIT(29)
#define CS35L45_TEMP_ERR_EINT1             BIT(17)
#define CS35L45_BST_SHORT_ERR_EINT1        BIT(8)
#define CS35L45_BST_UVP_ERR_EINT1          BIT(7)
#define CS35L45_IRQ1_EINT_2                0x0000E014
#define CS35L45_DSP_VIRT2_MBOX_EINT1       BIT(21)
#define CS35L45_DSP_WDT_EXPIRE_EINT1       BIT(4)
#define CS35L45_IRQ1_EINT_3                0x0000E018
#define CS35L45_AMP_CAL_ERR_EINT1          BIT(25)
#define CS35L45_PLL_UNLOCK_FLAG_RISE_EINT1 BIT(4)
#define CS35L45_PLL_LOCK_FLAG_EINT1        BIT(1)
#define CS35L45_IRQ1_EINT_4                0x0000E01C
#define CS35L45_OTP_BOOT_DONE_STS_MASK     BIT(1)
#define CS35L45_OTP_BUSY_MASK              BIT(0)
#define CS35L45_IRQ1_EINT_7                0x0000E028
#define CS35L45_IRQ1_EINT_14               0x0000E044
#define CS35L45_IRQ1_EINT_18               0x0000E054
#define CS35L45_UVLO_VDDLV_ERR_EINT1       BIT(16)
#define CS35L45_GLOBAL_ERROR_EINT1         BIT(15)

#define CS35L45_IRQ1_MASK_1  0x0000E110
#define CS35L45_IRQ1_MASK_7  0x0000E128
#define CS35L45_IRQ1_MASK_14 0x0000E144
#define CS35L45_IRQ1_MASK_18 0x0000E154

#define CS35L45_DSP_MBOX_2       0x00011004
#define CS35L45_DSP_VIRT1_MBOX_1 0x00011020
#define CS35L45_DSP_VIRT2_MBOX_3 0x00011048
#define CS35L45_MBOX3_DATA_MASK 0xFFFFFF00
#define CS35L45_MBOX3_CMD_MASK 0xFF
#define CS35L45_DSP_VIRT2_MBOX_4 0x0001104C

#define CS35L45_T_DEFAULT_DELAY       K_MSEC(1)
#define CS35L45_T_RLPW_US             K_USEC(1000)
#define CS35L45_T_IRS_US              K_USEC(1100)
#define CS35L45_T_INTERRUPT_DEBOUNCER K_USEC(500)
#define CS35L45_T_POST_GLOBAL_EN_US		K_USEC(5000)
#define CS35L45_T_PRE_GLOBAL_DIS_US		K_USEC(3000)

#define CS35L45_NUM_IRQ1_INT 9

static const struct {
	uint8_t cfg_id;
	uint32_t freq;
} cs35l45_pll_refclk_freq[] = {
	{0x0C, 128000},   {0x0F, 256000},   {0x11, 384000},   {0x12, 512000},   {0x15, 768000},
	{0x17, 1024000},  {0x19, 1411200},  {0x1B, 1536000},  {0x1C, 2116800},  {0x1D, 2048000},
	{0x1E, 2304000},  {0x1F, 2822400},  {0x21, 3072000},  {0x23, 4233600},  {0x24, 4096000},
	{0x25, 4608000},  {0x26, 5644800},  {0x27, 6000000},  {0x28, 6144000},  {0x29, 6350400},
	{0x2A, 6912000},  {0x2D, 7526400},  {0x2E, 8467200},  {0x2F, 8192000},  {0x30, 9216000},
	{0x31, 11289600}, {0x33, 12288000}, {0x37, 16934400}, {0x38, 18432000}, {0x39, 22579200},
	{0x3B, 24576000},
};

struct reg_sequence {
	uint32_t reg;
	uint32_t def;
};

static const struct reg_sequence cs35l45_patch[] = {
	{0x00000040, 0x00000055},
	{0x00000040, 0x000000AA},
	{0x00000044, 0x00000055},
	{0x00000044, 0x000000AA},
	{0x00006480, 0x0830500A},
	{0x00007C60, 0x1000850B},
	{CS35L45_BOOST_OV_CFG, 0x007000D0},
	{CS35L45_LDPM_CONFIG, 0x0001B636},
	{0x00002C08, 0x00000009},
	{0x00006850, 0x0A30FFC4},
	{0x00003820, 0x00040100},
	{0x00003824, 0x00000000},
	{0x00007CFC, 0x62870004},
	{0x00007C60, 0x1001850B},
	{0x00000040, 0x00000000},
	{0x00000044, 0x00000000},
	{CS35L45_BOOST_CCM_CFG, 0xF0000003},
	{CS35L45_BOOST_DCM_CFG, 0x08710220},
	{CS35L45_ERROR_RELEASE, 0x00200000},
};

static const struct reg_sequence cs35l45_irq_mask_seq[] = {
	{CS35L45_IRQ1_MASK_1, 0x1FEDFE3FU},
	{0x0000E114, 0xFFDFFFEFU},
	{0x0000E118, 0xFDFF87EDU},
	{CS35L45_IRQ1_MASK_18, 0x3FE450FFU},
};

static const struct reg_sequence cs35l45_irq_clear_seq[] = {
	{CS35L45_IRQ1_EINT_1, 0xFFFFFFFFU},
	{CS35L45_IRQ1_EINT_2, 0xFFFFFFFFU},
	{CS35L45_IRQ1_EINT_3, 0xFFFFFFFFU},
	{CS35L45_IRQ1_EINT_4, 0xFFFFFFFFU},
	{0x0000E020, 0xFFFFFFFFU},
	{CS35L45_IRQ1_EINT_7, 0xFFFFFFFFU},
	{0x0000E02C, 0xFFFFFFFFU},
	{CS35L45_IRQ1_EINT_14, 0xFFFFFFFFU},
	{CS35L45_IRQ1_EINT_18, 0xFFFFFFFFU},
};

static const struct reg_sequence cs35l45_dsp_routing[] = {
	{CS35L45_DSP1RX3_INPUT, DATA_SOURCE_VMON},
	{CS35L45_DSP1RX4_INPUT, DATA_SOURCE_IMON},
	{CS35L45_DSP1RX5_INPUT, DATA_SOURCE_VDD_BATTMON},
	{CS35L45_DSP1RX6_INPUT, DATA_SOURCE_VDD_BSTMON},
	{CS35L45_DSP1RX7_INPUT, DATA_SOURCE_CLASSH_TGT},
	{CS35L45_DSP1RX8_INPUT, DATA_SOURCE_VDD_BATTMON},
};

enum cs35l45_irq {
	CS35L45_INT1,
	CS35L45_INT2,
	CS35L45_INT3,
	CS35L45_INT4,
	CS35L45_INT5,
	CS35L45_INT7,
	CS35L45_INT8,
	CS35L45_INT14,
	CS35L45_INT18,
};

enum mbox3_events {
	EVENT_SPEAKER_STATUS = 0x66,
	EVENT_BOOT_DONE = 0x67,
};

static bool cs35l45_is_ready(const struct device *const dev)
{
	const struct cs35l45_config *const config = dev->config;

	return config->bus_io->is_ready(dev);
}

__maybe_unused static struct device *cs35l45_get_control_port(const struct device *const dev)
{
	const struct cs35l45_config *const config = dev->config;

	return config->bus_io->get_device(dev);
}

static int cs35l45_read(const struct device *const dev, const uint32_t addr, uint32_t *const rx)
{
	const struct cs35l45_config *const config = dev->config;

	return config->bus_io->read(dev, addr, rx, 1);
}

static int cs35l45_burst_read(const struct device *const dev, const uint32_t addr,
			      uint32_t *const rx, const uint32_t len)
{
	const struct cs35l45_config *const config = dev->config;

	return config->bus_io->read(dev, addr, rx, len);
}

static int cs35l45_write(const struct device *const dev, const uint32_t addr, const uint32_t val)
{
	const struct cs35l45_config *const config = dev->config;
	uint32_t tx[2] = {addr, val};

	return config->bus_io->write(dev, tx, ARRAY_SIZE(tx));
}

__maybe_unused static int cs35l45_burst_write(const struct device *const dev, uint32_t *const tx,
			       const uint32_t len)
{
	const struct cs35l45_config *const config = dev->config;

	return config->bus_io->write(dev, tx, len);
}

static int cs35l45_update_bits(const struct device *const dev, const uint32_t addr,
			       const uint32_t mask, const uint32_t val)
{
	uint32_t orig, tmp;
	int ret;

	ret = cs35l45_read(dev, addr, &orig);
	if (ret < 0) {
		return ret;
	}

	tmp = orig & ~mask;
	tmp |= val & mask;

	return cs35l45_write(dev, addr, tmp);
}

static bool cs35l45_check_cspl_mbox_sts(const enum cs35l45_cspl_mboxcmd cmd,
					enum cs35l45_cspl_mboxstate sts)
{
	switch (cmd) {
	case CSPL_MBOX_CMD_NONE:
	case CSPL_MBOX_CMD_UNKNOWN_CMD:
		return true;
	case CSPL_MBOX_CMD_PAUSE:
	case CSPL_MBOX_CMD_OUT_OF_HIBERNATE:
		return (sts == CSPL_MBOX_STS_PAUSED);
	case CSPL_MBOX_CMD_RESUME:
		return (sts == CSPL_MBOX_STS_RUNNING);
	case CSPL_MBOX_CMD_REINIT:
		return (sts == CSPL_MBOX_STS_RUNNING);
	case CSPL_MBOX_CMD_STOP_PRE_REINIT:
		return (sts == CSPL_MBOX_STS_RDY_FOR_REINIT);
	case CSPL_MBOX_CMD_HIBERNATE:
		return (sts == CSPL_MBOX_STS_HIBERNATE);
	default:
		return false;
	}
}

static int cs35l45_set_cspl_mbox_cmd(const struct device *dev, const enum cs35l45_cspl_mboxcmd cmd)
{
	const struct cs35l45_config *const config = dev->config;
	struct cs35l45_data *const data = dev->data;
	uint32_t sts = 0, i;
	int ret;

	if (!data->dsp_booted) {
		LOG_INST_ERR(config->log, "DSP not running");
		return -EPERM;
	}

	ret = cs35l45_write(dev, CS35L45_DSP_VIRT1_MBOX_1, cmd);
	if (ret < 0) {
		if (cmd != CSPL_MBOX_CMD_OUT_OF_HIBERNATE) {
			LOG_INST_ERR(config->log, "Failed to write MBOX: %d", ret);
		}
		return ret;
	}

	for (i = 0; i < 5; i++) {
		k_sleep(K_USEC(1000));

		ret = cs35l45_read(dev, CS35L45_DSP_MBOX_2, &sts);
		if (ret < 0) {
			LOG_INST_ERR(config->log, "Failed to read MBOX STS: %d", ret);
			continue;
		}

		if (!cs35l45_check_cspl_mbox_sts(cmd, sts)) {
			LOG_INST_DBG(config->log, "[%u] cmd %u returned invalid sts %u", i, cmd,
				     sts);
		} else {
			return 0;
		}
	}

	if (cmd != CSPL_MBOX_CMD_OUT_OF_HIBERNATE) {
		LOG_INST_ERR(config->log, "Failed to set mailbox cmd %u (status %u)", cmd, sts);
	}

	return -ENOMSG;
}

static int cs35l45_apply_properties(const struct device *dev)
{
	return 0;
}

static int cs35L45_set_volume(const struct device *dev, audio_property_value_t val)
{
	if (!IN_RANGE(val.vol, CS35L45_AMP_VOL_PCM_MIN, CS35L45_AMP_VOL_PCM_MAX)) {
		return -EINVAL;
	}

	return cs35l45_update_bits(dev, CS35L45_AMP_PCM_CONTROL, CS35L45_AMP_VOL_PCM_MASK, val.vol);
}

static int cs35L45_set_mute(const struct device *dev, audio_property_value_t val)
{
	uint32_t reg_val, hpf_tune;
	uint8_t global_fs;
	int ret;

	if (!val.mute) {
		ret = cs35l45_read(dev, CS35L45_GLOBAL_SAMPLE_RATE, &reg_val);
		if (ret < 0) {
			return ret;
		}

		global_fs = FIELD_GET(CS35L45_GLOBAL_FS_MASK, reg_val);

		switch (global_fs) {
		case CS35L45_GLOBAL_FS_44P1K:
			hpf_tune = CS35L45_HPF_44P1;
			break;
		default:
			hpf_tune = CS35L45_HPF_DEFAULT;
			break;
		}

		ret = cs35l45_read(dev, CS35L45_AMP_PCM_HPF_TST, &reg_val);
		if (ret < 0) {
			return ret;
		}

		if (reg_val != hpf_tune) {
			struct reg_sequence hpf_override_seq[] = {
				{0x00000040, 0x00000055},
				{0x00000040, 0x000000AA},
				{0x00000044, 0x00000055},
				{0x00000044, 0x000000AA},
				{CS35L45_AMP_PCM_HPF_TST, hpf_tune},
				{0x00000040, 0x00000000},
				{0x00000044, 0x00000000},
			};

			for (int i = 0; i < ARRAY_SIZE(hpf_override_seq); i++) {
				ret = cs35l45_write(dev, hpf_override_seq[i].reg,
						    hpf_override_seq[i].def);
				if (ret < 0) {
					return ret;
				}
			}
		}
	}

	return cs35l45_update_bits(dev, CS35L45_AMP_OUTPUT_MUTE, CS35L45_AMP_MUTE,
				   FIELD_PREP(CS35L45_AMP_MUTE, (uint32_t)val.mute));
}

static int cs35l45_set_property(const struct device *dev, audio_property_t property,
				audio_channel_t channel, audio_property_value_t val)
{
	switch (property) {
	case AUDIO_PROPERTY_OUTPUT_MUTE:
		return cs35L45_set_mute(dev, val);
	case AUDIO_PROPERTY_OUTPUT_VOLUME:
		return cs35L45_set_volume(dev, val);
	default:
		return -ENOTSUP;
	}
}

int cs35l45_set_tx_data_source(const struct device *dev, enum cs35l45_data_source data_source, uint32_t tx_idx)
{
	switch (tx_idx) {
	case 1:
		return cs35l45_write(dev, CS35L45_ASPTX1_INPUT, (uint32_t)data_source);
	case 2:
		return cs35l45_write(dev, CS35L45_ASPTX2_INPUT, (uint32_t)data_source);
	case 3:
		return cs35l45_write(dev, CS35L45_ASPTX3_INPUT, (uint32_t)data_source);
	case 4:
		return cs35l45_write(dev, CS35L45_ASPTX4_INPUT, (uint32_t)data_source);
	case 5:
		return cs35l45_write(dev, CS35L45_ASPTX5_INPUT, (uint32_t)data_source);
	default:
		return -EINVAL;
	}
}

static int cs35l45_route_input(const struct device *dev, audio_channel_t channel, uint32_t input)
{
	uint32_t val;

	switch (input) {
	case 1:
		val = CS35L45_ASP_TX1_EN;
		break;
	case 2:
		val = CS35L45_ASP_TX2_EN;
		break;
	case 3:
		val = CS35L45_ASP_TX3_EN;
		break;
	case 4:
		val = CS35L45_ASP_TX4_EN;
		break;
	case 5:
		val = CS35L45_ASP_TX5_EN;
		break;
	default:
		return -EINVAL;
	}

	return cs35l45_update_bits(dev, CS35L45_ASP_ENABLES1, val, val);
}

static int cs35l45_route_dsp(const struct device *dev, uint32_t output)
{
	uint32_t val;
	int ret;

	switch (output) {
	case 1:
		val = (uint32_t)DATA_SOURCE_ASP_RX1;
	case 2:
		val = (uint32_t)DATA_SOURCE_ASP_RX2;
	default:
		return -EINVAL;
	}

	ret = cs35l45_write(dev, CS35L45_DSP1RX1_INPUT, val);
	if (ret < 0) {
		return ret;
	}

	ret = cs35l45_write(dev, CS35L45_DSP1RX2_INPUT, val);
	if (ret < 0) {
		return ret;
	}

	for (int i = 0; i < ARRAY_SIZE(cs35l45_dsp_routing); i++) {
		ret = cs35l45_write(dev, cs35l45_dsp_routing[i].reg, cs35l45_dsp_routing[i].def);
		if (ret < 0) {
			return ret;
		}
	}

	return 0;
}

static int cs35l45_route_output(const struct device *dev, audio_channel_t channel, uint32_t output)
{
	struct cs35l45_data *const data = dev->data;
	uint32_t val;
	int ret;

	switch (output) {
	case 1:
		val = CS35L45_ASP_RX1_EN;
		break;
	case 2:
		val = CS35L45_ASP_RX2_EN;
		break;
	default:
		return -EINVAL;
	}

	ret = cs35l45_update_bits(dev, CS35L45_ASP_ENABLES1, val, val);
	if (ret < 0) {
		return ret;
	}

	ret = cs35l45_update_bits(dev, CS35L45_BLOCK_ENABLES2, CS35L45_ASP_EN, CS35L45_ASP_EN);
	if (ret < 0) {
		return ret;
	}

	if (!data->dsp_booted) {
		if (output == 1) {
			val = CS35L45_DACPCM1_SRC_ASP_RX1;
		} else {
			val = CS35L45_DACPCM1_SRC_ASP_RX2;
		}
	} else {
		ret = cs35l45_route_dsp(dev, output);
		if (ret < 0) {
			return ret;
		}

		val = CS35L45_DACPCM1_SRC_DSP_TX1;
	}

	return cs35l45_write(dev, CS35L45_DACPCM1_INPUT, val);
}

static int cs35l45_dsp_audio_ev(const struct device *dev, const bool event)
{
	if (event) {
		return cs35l45_set_cspl_mbox_cmd(dev, CSPL_MBOX_CMD_RESUME);
	} else {
		return cs35l45_set_cspl_mbox_cmd(dev, CSPL_MBOX_CMD_PAUSE);
	}
}

static int cs35l45_global_en_event(const struct device *dev, const bool enable)
{
	int ret;

	if (enable) {
		ret = cs35l45_write(dev, CS35L45_GLOBAL_ENABLES, CS35L45_GLOBAL_EN_MASK);
		if (ret < 0) {
			return ret;
		}

		(void)k_sleep(CS35L45_T_POST_GLOBAL_EN_US);
	} else {
		(void)k_sleep(CS35L45_T_PRE_GLOBAL_DIS_US);
		ret = cs35l45_write(dev, CS35L45_GLOBAL_ENABLES, 0);
		if (ret < 0) {
			return ret;
		}
	}

	return 0;
}

static void cs35l45_stop_output(const struct device *dev)
{
	struct cs35l45_data *const data = dev->data;

	(void)cs35l45_global_en_event(dev, false);

	if (data->dsp_booted) {
		(void)cs35l45_dsp_audio_ev(dev, false);
	}
}

static void cs35l45_start_output(const struct device *dev)
{
	struct cs35l45_data *const data = dev->data;

	(void)cs35l45_global_en_event(dev, true);

	if (data->dsp_booted) {
		(void)cs35l45_dsp_audio_ev(dev, true);
	}
}

static int cs35l45_get_clk_freq_id(const uint32_t freq)
{
	int i;

	if (freq == 0) {
		return -EINVAL;
	}

	for (i = 0; i < ARRAY_SIZE(cs35l45_pll_refclk_freq); ++i) {
		if (cs35l45_pll_refclk_freq[i].freq == freq) {
			return cs35l45_pll_refclk_freq[i].cfg_id;
		}
	}

	return -EINVAL;
}

static int cs35l45_set_pll(const struct device *dev, const uint32_t freq)
{
	const struct cs35l45_config *const config = dev->config;
	uint8_t freq_id;
	uint32_t val;
	int ret;

	freq_id = cs35l45_get_clk_freq_id(freq);
	if (freq_id < 0) {
		LOG_INST_DBG(config->log, "Invalid freq: %u", freq);
		return -EINVAL;
	}

	ret = cs35l45_read(dev, CS35L45_REFCLK_INPUT, &val);
	if (ret < 0) {
		return ret;
	}

	val = FIELD_GET(CS35L45_PLL_REFCLK_FREQ_MASK, val);
	if (val == freq_id) {
		return 0;
	}

	ret = cs35l45_update_bits(dev, CS35L45_REFCLK_INPUT, CS35L45_PLL_OPEN_LOOP_MASK,
				  CS35L45_PLL_OPEN_LOOP_MASK);
	if (ret < 0) {
		return ret;
	}

	ret = cs35l45_update_bits(dev, CS35L45_REFCLK_INPUT, CS35L45_PLL_REFCLK_FREQ_MASK,
				  FIELD_PREP(CS35L45_PLL_REFCLK_FREQ_MASK, freq_id));
	if (ret < 0) {
		return ret;
	}

	ret = cs35l45_update_bits(dev, CS35L45_REFCLK_INPUT, CS35L45_PLL_REFCLK_EN_MASK, 0);
	if (ret < 0) {
		return ret;
	}

	ret = cs35l45_update_bits(dev, CS35L45_REFCLK_INPUT, CS35L45_PLL_OPEN_LOOP_MASK, 0);
	if (ret < 0) {
		return ret;
	}

	return cs35l45_update_bits(dev, CS35L45_REFCLK_INPUT, CS35L45_PLL_REFCLK_EN_MASK,
				   CS35L45_PLL_REFCLK_EN_MASK);
}

static int cs35l45_set_frame_clock(const struct device *dev, const uint32_t freq)
{
	const struct cs35l45_config *const config = dev->config;
	uint8_t global_fs;

	switch (freq) {
	case AUDIO_PCM_RATE_44P1K:
		global_fs = CS35L45_GLOBAL_FS_44P1K;
		break;
	case AUDIO_PCM_RATE_48K:
		global_fs = CS35L45_GLOBAL_FS_48K;
		break;
	case AUDIO_PCM_RATE_96K:
		global_fs = CS35L45_GLOBAL_FS_96K;
		break;
	default:
		LOG_INST_DBG(config->log, "Unsupported frame clock frequency: %d Hz", freq);
		return -EINVAL;
	}

	return cs35l45_update_bits(dev, CS35L45_GLOBAL_SAMPLE_RATE, CS35L45_GLOBAL_FS_MASK,
				   global_fs);
}

static int cs35l45_configure_asp_fmt(const struct device *dev, struct audio_codec_cfg *cfg)
{
	const struct cs35l45_config *const config = dev->config;
	struct i2s_config i2s = cfg->dai_cfg.i2s;
	i2s_opt_t i2s_opt = i2s.options;
	i2s_fmt_t i2s_fmt = i2s.format;
	uint8_t asp_fmt;
	uint32_t val;

	if ((i2s_opt & (I2S_OPT_BIT_CLK_SLAVE | I2S_OPT_FRAME_CLK_SLAVE)) == 0U) {
		LOG_INST_DBG(config->log, "Invalid DAI clocking");
		return -EINVAL;
	}

	switch (FIELD_GET(I2S_FMT_DATA_FORMAT_MASK, i2s_fmt)) {
	case I2S_FMT_DATA_FORMAT_I2S:
		asp_fmt = CS35L45_ASP_FMT_I2S;
		break;
	case I2S_FMT_DATA_FORMAT_PCM_SHORT:
		asp_fmt = CS35L45_ASP_FMT_TDM_1_5;
		break;
	case I2S_FMT_DATA_FORMAT_PCM_LONG:
		asp_fmt = CS35L45_ASP_FMT_DSP_A;
		break;
	default:
		LOG_INST_DBG(config->log, "Invalid DAI format");
		return -EINVAL;
	}

	val = FIELD_PREP(CS35L45_ASP_FMT_MASK, asp_fmt);

	switch (FIELD_GET(I2S_FMT_CLK_FORMAT_MASK, i2s_fmt)) {
	case I2S_FMT_CLK_NF_NB:
		break;
	case I2S_FMT_CLK_NF_IB:
		val |= CS35L45_ASP_BCLK_INV;
		break;
	case I2S_FMT_CLK_IF_NB:
		val |= CS35L45_ASP_FSYNC_INV;
		break;
	case I2S_FMT_CLK_IF_IB:
		val |= (CS35L45_ASP_FSYNC_INV | CS35L45_ASP_BCLK_INV);
		break;
	default:
		LOG_INST_DBG(config->log, "Invalid DAI clock polarity");
	}

	return cs35l45_update_bits(
		dev, CS35L45_ASP_CONTROL2,
		(CS35L45_ASP_FMT_MASK | CS35L45_ASP_BCLK_INV | CS35L45_ASP_FSYNC_INV), val);
}

static int cs35l45_configure_asp_word(const struct device *dev, struct audio_codec_cfg *cfg)
{
	struct i2s_config i2s = cfg->dai_cfg.i2s;
	uint8_t asp_width;
	int ret;

	if (!IN_RANGE(i2s.word_size, CS35L45_ASP_WL_MIN, CS35L45_ASP_WL_MAX)) {
		return -EINVAL;
	}

	if (i2s.word_size == AUDIO_PCM_WIDTH_16_BITS) {
		asp_width = AUDIO_PCM_WIDTH_16_BITS;
	} else {
		asp_width = AUDIO_PCM_WIDTH_32_BITS;
	}

	ret = cs35l45_set_pll(dev, asp_width * i2s.channels * i2s.frame_clk_freq);
	if (ret < 0) {
		return ret;
	}

	switch (cfg->dai_route) {
	case AUDIO_ROUTE_PLAYBACK_CAPTURE:
		ret = cs35l45_update_bits(dev, CS35L45_ASP_CONTROL2, CS35L45_ASP_WIDTH_TX_MASK,
					  FIELD_PREP(CS35L45_ASP_WIDTH_TX_MASK, asp_width));
		if (ret < 0) {
			return ret;
		}

		ret = cs35l45_update_bits(dev, CS35L45_ASP_DATA_CONTROL1, CS35L45_ASP_WL_MASK,
					  i2s.word_size);
		if (ret < 0) {
			return ret;
		}

		__fallthrough;
	case AUDIO_ROUTE_PLAYBACK:
		ret = cs35l45_update_bits(dev, CS35L45_ASP_CONTROL2, CS35L45_ASP_WIDTH_RX_MASK,
					  FIELD_PREP(CS35L45_ASP_WIDTH_RX_MASK, asp_width));
		if (ret < 0) {
			return ret;
		}

		return cs35l45_update_bits(dev, CS35L45_ASP_DATA_CONTROL5, CS35L45_ASP_WL_MASK,
					   i2s.word_size);
	default:
		return -EINVAL;
	}
}

static int cs35l45_configure(const struct device *dev, struct audio_codec_cfg *cfg)
{
	int ret;

	ret = cs35l45_set_frame_clock(dev, cfg->dai_cfg.i2s.frame_clk_freq);
	if (ret < 0) {
		return ret;
	}

	ret = cs35l45_configure_asp_word(dev, cfg);
	if (ret < 0) {
		return ret;
	}

	ret = cs35l45_configure_asp_fmt(dev, cfg);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

static int cs35l45_apply_patch(const struct device *dev)
{
	int ret;

	for (int i = 0; i < ARRAY_SIZE(cs35l45_patch); i++) {
		ret = cs35l45_write(dev, cs35l45_patch[i].reg, cs35l45_patch[i].def);
		if (ret < 0) {
			return ret;
		}
	}

	return 0;
}

static void cs35l45_error_callback(const struct device *const dev, const uint32_t error_bitmask)
{
	struct cs35l45_data *const data = dev->data;

	if (data->error_callback != NULL) {
		(void)data->error_callback(dev, error_bitmask);
	}
}

static int cs35l45_process_mailbox3(const struct device *const dev, uint8_t cmd, uint32_t data)
{
	const struct cs35l45_config *const config = dev->config;
	static char *speaker_status = "Unknown";

	switch (cmd) {
	case EVENT_SPEAKER_STATUS:
		switch (data) {
		case 1:
			speaker_status = "All Clear";
			break;
		case 2:
			speaker_status = "Open Circuit";
			break;
		case 4:
			speaker_status = "Short Circuit";
			break;
		}

		LOG_INST_INF(config->log, "MBOX event (SPEAKER_STATUS): %s", speaker_status);
		break;
	case EVENT_BOOT_DONE:
		LOG_INST_DBG(config->log, "MBOX event (BOOT_DONE)");
		break;
	default:
		LOG_INST_ERR(config->log, "MBOX event not supported %u", cmd);
		return -EINVAL;
	}

	return 0;
}

static int cs35l45_process_mailbox(const struct device *const dev)
{
	const struct cs35l45_config *const config = dev->config;
	uint32_t mbox_val;
	int ret;

	ret = cs35l45_read(dev, CS35L45_DSP_VIRT2_MBOX_3, &mbox_val);
	if ((ret == 0) && (mbox_val)) {
		(void)cs35l45_process_mailbox3(dev, (mbox_val & CS35L45_MBOX3_CMD_MASK), FIELD_GET(CS35L45_MBOX3_DATA_MASK, mbox_val));
	}

	ret = cs35l45_read(dev, CS35L45_DSP_VIRT2_MBOX_4, &mbox_val);
	if ((ret == 0) && (mbox_val != 0)) {
		LOG_INST_ERR(config->log, "Spurious DSP MBOX4 IRQ");
	}

	return ret;
}

static int cs35l45_process_interrupts(const struct device *const dev,
				      const uint32_t *const irq_ints)
{
	__maybe_unused const struct cs35l45_config *const config = dev->config;
	uint32_t error_bitmask = 0;
	int ret;

	if (FIELD_GET(CS35L45_AMP_SHORT_ERR_EINT1, irq_ints[CS35L45_INT1]) != 0) {
		LOG_INST_ERR(config->log, "Amplifier short error");

		error_bitmask |= AUDIO_CODEC_ERROR_OVERCURRENT;

		ret = cs35l45_write(dev, CS35L45_IRQ1_EINT_1, CS35L45_AMP_SHORT_ERR_EINT1);
		if (ret < 0) {
			return ret;
		}
	}

	if (FIELD_GET(CS35L45_UVLO_VDDBATT_ERR_EINT1, irq_ints[CS35L45_INT1]) != 0) {
		LOG_INST_ERR(config->log, "VDDBATT undervoltage error");

		error_bitmask |= AUDIO_CODEC_ERROR_UNDERVOLTAGE;

		ret = cs35l45_write(dev, CS35L45_IRQ1_EINT_1, CS35L45_UVLO_VDDBATT_ERR_EINT1);
		if (ret < 0) {
			return ret;
		}
	}

	if (FIELD_GET(CS35L45_BST_SHORT_ERR_EINT1, irq_ints[CS35L45_INT1]) != 0) {
		LOG_INST_ERR(config->log, "Boost inductor error");

		error_bitmask |= AUDIO_CODEC_ERROR_OVERCURRENT;

		ret = cs35l45_write(dev, CS35L45_IRQ1_EINT_1, CS35L45_BST_SHORT_ERR_EINT1);
		if (ret < 0) {
			return ret;
		}
	}

	if (FIELD_GET(CS35L45_BST_UVP_ERR_EINT1, irq_ints[CS35L45_INT1]) != 0) {
		LOG_INST_ERR(config->log, "Boost undervoltage error");

		error_bitmask |= AUDIO_CODEC_ERROR_UNDERVOLTAGE;

		ret = cs35l45_write(dev, CS35L45_IRQ1_EINT_1, CS35L45_BST_UVP_ERR_EINT1);
		if (ret < 0) {
			return ret;
		}
	}

	if (FIELD_GET(CS35L45_TEMP_ERR_EINT1, irq_ints[CS35L45_INT1]) != 0) {
		LOG_INST_ERR(config->log, "Overtemperature error");

		error_bitmask |= AUDIO_CODEC_ERROR_OVERTEMPERATURE;

		ret = cs35l45_write(dev, CS35L45_IRQ1_EINT_1, CS35L45_TEMP_ERR_EINT1);
		if (ret < 0) {
			return ret;
		}
	}

	if (FIELD_GET(CS35L45_DSP_WDT_EXPIRE_EINT1, irq_ints[CS35L45_INT2]) != 0) {
		LOG_INST_ERR(config->log, "DSP Watchdog Timer");

		ret = cs35l45_write(dev, CS35L45_IRQ1_EINT_2, CS35L45_DSP_WDT_EXPIRE_EINT1);
		if (ret < 0) {
			return ret;
		}
	}

	if (FIELD_GET(CS35L45_DSP_VIRT2_MBOX_EINT1, irq_ints[CS35L45_INT2]) != 0) {
		LOG_INST_DBG(config->log, "DSP virtual MBOX 2 write flag");

		ret = cs35l45_write(dev, CS35L45_IRQ1_EINT_2, CS35L45_DSP_VIRT2_MBOX_EINT1);
		if (ret < 0) {
			return ret;
		}
	}

	if (FIELD_GET(CS35L45_AMP_CAL_ERR_EINT1, irq_ints[CS35L45_INT3]) != 0) {
		LOG_INST_ERR(config->log, "Amplifier calibration error");

		ret = cs35l45_write(dev, CS35L45_IRQ1_EINT_3, CS35L45_AMP_CAL_ERR_EINT1);
		if (ret < 0) {
			return ret;
		}
	}

	if (FIELD_GET(CS35L45_PLL_UNLOCK_FLAG_RISE_EINT1, irq_ints[CS35L45_INT3]) != 0) {
		LOG_INST_DBG(config->log, "PLL unlock");

		ret = cs35l45_write(dev, CS35L45_IRQ1_EINT_3, CS35L45_PLL_UNLOCK_FLAG_RISE_EINT1);
		if (ret < 0) {
			return ret;
		}
	}

	if (FIELD_GET(CS35L45_PLL_LOCK_FLAG_EINT1, irq_ints[CS35L45_INT3]) != 0) {
		LOG_INST_DBG(config->log, "PLL lock");

		ret = cs35l45_write(dev, CS35L45_IRQ1_EINT_3, CS35L45_PLL_LOCK_FLAG_EINT1);
		if (ret < 0) {
			return ret;
		}
	}

	if (FIELD_GET(CS35L45_UVLO_VDDLV_ERR_EINT1, irq_ints[CS35L45_INT18]) != 0) {
		LOG_INST_ERR(config->log, "LV threshold detector error");

		ret = cs35l45_write(dev, CS35L45_IRQ1_EINT_18, CS35L45_UVLO_VDDLV_ERR_EINT1);
		if (ret < 0) {
			return ret;
		}
	}

	if (FIELD_GET(CS35L45_GLOBAL_ERROR_EINT1, irq_ints[CS35L45_INT18]) != 0) {
		LOG_INST_ERR(config->log, "Global error");

		ret = cs35l45_write(dev, CS35L45_IRQ1_EINT_18, CS35L45_GLOBAL_ERROR_EINT1);
		if (ret < 0) {
			return ret;
		}
	}

	if (error_bitmask != 0) {
		(void)cs35l45_error_callback(dev, error_bitmask);
	}

	return 0;
}

static int cs35l45_retrieve_interrupt_statuses(const struct device *const dev,
					       uint32_t *const irq_ints)
{
	uint32_t irq_masks[CS35L45_NUM_IRQ1_INT];
	int ret;

	ret = cs35l45_burst_read(dev, CS35L45_IRQ1_EINT_1, irq_ints, 5);
	if (ret < 0) {
		return ret;
	}

	ret = cs35l45_burst_read(dev, CS35L45_IRQ1_EINT_7, &irq_ints[CS35L45_INT7], 2);
	if (ret < 0) {
		return ret;
	}

	ret = cs35l45_read(dev, CS35L45_IRQ1_EINT_14, &irq_ints[CS35L45_INT14]);
	if (ret < 0) {
		return ret;
	}

	ret = cs35l45_read(dev, CS35L45_IRQ1_EINT_18, &irq_ints[CS35L45_INT18]);
	if (ret < 0) {
		return ret;
	}

	ret = cs35l45_burst_read(dev, CS35L45_IRQ1_MASK_1, irq_masks, 5);
	if (ret < 0) {
		return ret;
	}

	ret = cs35l45_burst_read(dev, CS35L45_IRQ1_MASK_7, &irq_masks[CS35L45_INT7], 2);
	if (ret < 0) {
		return ret;
	}

	ret = cs35l45_read(dev, CS35L45_IRQ1_MASK_14, &irq_masks[CS35L45_INT14]);
	if (ret < 0) {
		return ret;
	}

	ret = cs35l45_read(dev, CS35L45_IRQ1_MASK_18, &irq_masks[CS35L45_INT18]);
	if (ret < 0) {
		return ret;
	}

	for (int i = 0; i < CS35L45_NUM_IRQ1_INT; i++) {
		irq_ints[i] &= ~irq_masks[i];
	}

	return ret;
}

static void cs35l45_interrupt_worker(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct cs35l45_data *data = CONTAINER_OF(dwork, struct cs35l45_data, interrupt_worker);
	const struct cs35l45_config *const config = data->config;
	uint32_t irq1_status, irq_ints[CS35L45_NUM_IRQ1_INT];
	int ret;

	if (gpio_pin_get_dt(&config->int_gpio) == 0) {
		LOG_INST_DBG(config->log, "filtered interrupt trigger with debouncer");
		return;
	}

	ret = cs35l45_read(data->dev, CS35L45_IRQ1_STATUS, &irq1_status);
	if (ret < 0) {
		LOG_INST_DBG(config->log, "failed to read IRQ status (%d)", ret);
		return;
	}

	if (irq1_status == 0) {
		LOG_INST_DBG(config->log, "IRQ status unset in interrupt worker");
		return;
	}

	ret = cs35l45_retrieve_interrupt_statuses(data->dev, irq_ints);
	if (ret < 0) {
		LOG_INST_DBG(config->log, "failed to read IRQ registers (%d)", ret);
		return;
	}

	ret = cs35l45_process_interrupts(data->dev, irq_ints);
	if (ret < 0) {
		return;
	}

	if (irq_ints[CS35L45_INT2] & CS35L45_DSP_VIRT2_MBOX_EINT1) {
		ret = cs35l45_process_mailbox(data->dev);
		if (ret < 0) {
			LOG_INST_DBG(config->log, "failed to read process mailbox (%d)", ret);
			return;
		}
	}

	ret = cs35l45_read(data->dev, CS35L45_IRQ1_STATUS, &irq1_status);
	if (ret < 0) {
		LOG_INST_DBG(config->log, "failed to read IRQ status (%d)", ret);
		return;
	}

	if (irq1_status != 0) {
		LOG_INST_WRN(config->log, "IRQ still set in interrupt worker");

		ret = k_work_submit(work);
		if (ret < 0) {
			LOG_INST_DBG(config->log, "failed to resubmit worker (%d)", ret);
		}
	}
}

static void cs35l45_interrupt_handler(const struct device *port, struct gpio_callback *cb,
				      uint32_t pins)
{
	struct cs35l45_data *const data = CONTAINER_OF(cb, struct cs35l45_data, interrupt_callback);
	__maybe_unused const struct cs35l45_config *const config = data->config;
	int ret;

	ret = k_work_schedule(&data->interrupt_worker, CS35L45_T_INTERRUPT_DEBOUNCER);
	if (ret < 0) {
		LOG_INST_DBG(config->log, "failed to queue interrupt worker (%d)", ret);
	}
}

static int cs35l45_irq_config(const struct device *const dev)
{
	const struct cs35l45_config *config = dev->config;
	struct cs35l45_data *const data = dev->data;
	int ret;

	ret = gpio_pin_configure_dt(&config->int_gpio, GPIO_INPUT);
	if (ret < 0) {
		return ret;
	}

	ret = gpio_pin_interrupt_configure_dt(&config->int_gpio, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret < 0) {
		return ret;
	}

	for (int i = 0; i < ARRAY_SIZE(cs35l45_irq_mask_seq); i++) {
		ret = cs35l45_write(dev, cs35l45_irq_mask_seq[i].reg, cs35l45_irq_mask_seq[i].def);
		if (ret < 0) {
			return ret;
		}
	}

	for (int i = 0; i < ARRAY_SIZE(cs35l45_irq_clear_seq); i++) {
		ret = cs35l45_write(dev, cs35l45_irq_clear_seq[i].reg, cs35l45_irq_clear_seq[i].def);
		if (ret < 0) {
			return ret;
		}
	}

	ret = cs35l45_update_bits(dev, CS35L45_INTB_GPIO2_MCLK_REF, CS35L45_GP2_CTRL,
				  FIELD_PREP(CS35L45_GP2_CTRL, CS35L45_OPEN_DRAIN_INT));
	if (ret < 0) {
		return ret;
	}

	gpio_init_callback(&data->interrupt_callback, cs35l45_interrupt_handler,
				 BIT(config->int_gpio.pin));
	ret = gpio_add_callback_dt(&config->int_gpio, &data->interrupt_callback);
	if (ret < 0) {
		LOG_INST_DBG(config->log, "failed to add interrupt callback (%d)", ret);
		return ret;
	}

	return ret;
}

static int cs35l45_reset(const struct device *dev)
{
	const struct cs35l45_config *config = dev->config;
	int ret;

	if (!gpio_is_ready_dt(&config->reset_gpio)) {
		LOG_INST_DBG(config->log, "reset GPIO is not ready");
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&config->reset_gpio, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		LOG_INST_DBG(config->log, "Failed to configure reset GPIO");
		return ret;
	}

	(void)k_sleep(CS35L45_T_RLPW_US);

	ret = gpio_pin_set_dt(&config->reset_gpio, 0);
	if (ret < 0) {
		LOG_INST_DBG(config->log, "Failed to release reset GPIO");
		return ret;
	}

	(void)k_sleep(CS35L45_T_IRS_US);

	return 0;
}

static int cs35l45_apply_property_config(const struct device *dev)
{
	const struct cs35l45_config *config = dev->config;
	uint32_t val;
	int ret;

	val = config->asp_sdout_hiz_ctrl;

	ret = cs35l45_write(dev, CS35L45_ASP_CONTROL3, val);
	if (ret < 0) {
		LOG_INST_DBG(config->log, "Failed to apply SDOUT setting");
		return ret;
	}

	val = FIELD_PREP(CS35L45_AMP_GAIN_PCM, (uint32_t)config->amp_gain_pcm);

	ret = cs35l45_update_bits(dev, CS35L45_AMP_GAIN, CS35L45_AMP_GAIN_PCM,
				  config->amp_gain_pcm);
	if (ret < 0) {
		LOG_INST_DBG(config->log, "Failed to apply analog gain");
		return ret;
	}

	return 0;
}

static int cs35l45_hw_init(const struct device *dev)
{
	const struct cs35l45_config *const config = dev->config;
	uint32_t sts = 0;
	uint32_t val[5];
	int ret, i = 5;

	while ((sts & CS35L45_OTP_BOOT_DONE_STS_MASK) == 0U) {
		(void)k_usleep(1000);
		(void)cs35l45_read(dev, CS35L45_IRQ1_EINT_4, &sts);
		i--;
		if (i < 0) {
			return -ETIMEDOUT;
		}
	}

	ret = cs35l45_burst_read(dev, CS35L45_DEVID, val, ARRAY_SIZE(val));
	if (ret < 0) {
		return ret;
	}

	switch (val[0]) {
	case 0x35A450:
	case 0x35A460:
		break;
	default:
		LOG_INST_ERR(config->log, "Bad DEVID 0x%x", val[0]);
		return -ENODEV;
	}

	LOG_INST_INF(config->log, "Cirrus Logic CS35L45: REVID %02X OTPID %02X\n", val[1], val[4]);

	ret = cs35l45_write(dev, CS35L45_IRQ1_EINT_4,
			    (CS35L45_OTP_BOOT_DONE_STS_MASK | CS35L45_OTP_BUSY_MASK));
	if (ret < 0) {
		LOG_INST_DBG(config->log, "OTP Failed to boot");
		return ret;
	}

	ret = cs35l45_apply_patch(dev);
	if (ret < 0) {
		LOG_INST_DBG(config->log, "Failed to apply OTP patch");
		return ret;
	}

	ret = cs35l45_apply_property_config(dev);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

static int cs35l45_init(const struct device *dev)
{
	const struct cs35l45_config *config = dev->config;
	struct cs35l45_data *const data = dev->data;
	int ret;

	if (!cs35l45_is_ready(dev)) {
		LOG_INST_DBG(config->log, "control port is not ready");
		return -ENODEV;
	}

	if (config->reset_gpio.port != NULL) {
		ret = cs35l45_reset(dev);
		if (ret < 0) {
			return ret;
		}
	} else {
		LOG_INST_DBG(config->log, "Assuming shared reset");
	}

	ret = cs35l45_hw_init(dev);
	if (ret < 0) {
		return ret;
	}

	if (config->int_gpio.port != NULL) {
		(void)k_work_init_delayable(&data->interrupt_worker, cs35l45_interrupt_worker);
		ret = cs35l45_irq_config(dev);
		if (ret < 0) {
			return ret;
		}
	}

	return 0;
}

static const struct audio_codec_api cs35l45_driver_api = {
	.configure = cs35l45_configure,
	.start_output = cs35l45_start_output,
	.stop_output = cs35l45_stop_output,
	.set_property = cs35l45_set_property,
	.apply_properties = cs35l45_apply_properties,
	.route_output = cs35l45_route_output,
	.route_input = cs35l45_route_input,
};

#define AUDIO_CODEC_CS35L45_DATA(inst)                                                             \
	.dev = DEVICE_DT_INST_GET(inst), .config = &cs35l45_config_##inst,

#define AUDIO_CODEC_CS35L45_BUS(inst)                                                              \
	COND_CODE_1(DT_INST_ON_BUS(inst, i2c),	\
			(.bus.i2c = I2C_DT_SPEC_INST_GET(inst), .bus_io = &cs35l45_bus_io_i2c,),	   \
			(.bus.spi = SPI_DT_SPEC_INST_GET(inst, SPI_OP_MODE_MASTER),		   \
				.bus_io = &cs35l45_bus_io_spi,))

#define AUDIO_CODEC_CS35L45_CONFIG(inst)                                                           \
	.dev = DEVICE_DT_INST_GET(inst), .data = &cs35l45_data_##inst,                             \
	.reset_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, reset_gpios, {0}),                            \
	.int_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, int_gpios, {0}),                                \
	.asp_sdout_hiz_ctrl = DT_INST_ENUM_IDX(inst, cirrus_asp_sdout_hiz_ctrl),                   \
	.amp_gain_pcm = DT_INST_ENUM_IDX(inst, cirrus_amp_gain_pcm),                               \
	LOG_INSTANCE_PTR_INIT(log, DT_NODE_FULL_NAME_TOKEN(DT_DRV_INST(inst)), inst)               \
		AUDIO_CODEC_CS35L45_BUS(inst)

#define AUDIO_CODEC_CS35L45_INIT(inst)                                                             \
	DEVICE_DT_INST_DEFINE(inst, cs35l45_init, NULL, &cs35l45_data_##inst,                      \
			      &cs35l45_config_##inst, POST_KERNEL,                                 \
			      CONFIG_AUDIO_CODEC_INIT_PRIORITY, &cs35l45_driver_api);

#define AUDIO_CODEC_CS35L45_DEFINE(inst)                                                           \
	LOG_INSTANCE_REGISTER(DT_NODE_FULL_NAME_TOKEN(DT_DRV_INST(inst)), inst,                    \
			      CONFIG_AUDIO_CODEC_LOG_LEVEL);                                       \
	static const struct cs35l45_config cs35l45_config_##inst;                                  \
	static struct cs35l45_data cs35l45_data_##inst;                                            \
	static const struct cs35l45_config cs35l45_config_##inst = {                               \
		AUDIO_CODEC_CS35L45_CONFIG(inst)};                                                 \
	static struct cs35l45_data cs35l45_data_##inst = {AUDIO_CODEC_CS35L45_DATA(inst)};         \
	AUDIO_CODEC_CS35L45_INIT(inst)

DT_INST_FOREACH_STATUS_OKAY(AUDIO_CODEC_CS35L45_DEFINE)
