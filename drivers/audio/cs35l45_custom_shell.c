/*
 * Copyright (c) 2026 Cirrus Logic, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdlib.h>
#include <zephyr/shell/shell.h>
#include <zephyr/audio/codec.h>
#include "cs35l45.h"

#define CODEC_SET_TX_SOURCE_HELP                                                                   \
	SHELL_HELP("Set transmitter output source", "<device> <tx_idx> <data source>")

static const char *const data_source_name[] = {
	[DATA_SOURCE_ZERO_FILL] = "zero_fill",
	[DATA_SOURCE_DIAG_GEN] = "diag_gen",
	[DATA_SOURCE_ASP_RX1] = "asp_rx1",
	[DATA_SOURCE_ASP_RX2] = "asp_rx2",
	[DATA_SOURCE_VMON] = "vmon",
	[DATA_SOURCE_IMON] = "imon",
	[DATA_SOURCE_ERR_VOL] = "err_vol",
	[DATA_SOURCE_VDD_BATTMON] = "vdd_battmon",
	[DATA_SOURCE_VDD_BSTMON] = "vdd_bstmon",
	[DATA_SOURCE_DSP_TX_CH1] = "dsp_tx_ch1",
	[DATA_SOURCE_DSP_TX_CH2] = "dsp_tx_ch2",
	[DATA_SOURCE_DSP_TX_CH3] = "dsp_tx_ch3",
	[DATA_SOURCE_DSP_TX_CH4] = "dsp_tx_ch4",
	[DATA_SOURCE_DSP_TX_CH5] = "dsp_tx_ch5",
	[DATA_SOURCE_DSP_TX_CH6] = "dsp_tx_ch6",
	[DATA_SOURCE_DSP_TX_CH7] = "dsp_tx_ch7",
	[DATA_SOURCE_DSP_TX_CH8] = "dsp_tx_ch8",
	[DATA_SOURCE_TEMPMON] = "tempmon",
	[DATA_SOURCE_IL_TARGET] = "il_target",
};

struct args_index {
	uint8_t device;
	uint8_t property;
	uint8_t data_source;
	uint8_t tx_idx;
	uint8_t channel;
	uint8_t value;
};

static const struct args_index args_indx = {
	.device = 1,
	.tx_idx = 2,
	.data_source = 3,
};

static int parse_named_int(const char *name, const char *const keystack[], size_t count)
{
	char *endptr;
	int i;

	/* Attempt to parse name as a number first */
	i = strtoul(name, &endptr, 0);
	if (*endptr == '\0') {
		return i;
	}

	/* Name is not a number, look it up */
	for (i = 0; i < count; i++) {
		if (strcmp(name, keystack[i]) == 0) {
			return i;
		}
	}

	return -ENOTSUP;
}

static int cmd_set_tx_source(const struct shell *sh, size_t argc, char *argv[])
{
	const struct device *dev;
	int data_source;
	long tx_idx;
	char *endptr;

	dev = shell_device_get_binding(argv[args_indx.device]);
	if (!dev) {
		shell_error(sh, "CS35L45 device not found");
		return -ENODEV;
	}

	data_source = parse_named_int(argv[args_indx.data_source], data_source_name,
				      ARRAY_SIZE(data_source_name));
	if (data_source < 0) {
		shell_error(sh, "Property '%s' unknown", argv[args_indx.data_source]);
		return -EINVAL;
	}

	tx_idx = strtol(argv[args_indx.tx_idx], &endptr, 0);
	if (*endptr != '\0') {
		return -EINVAL;
	}
	if (tx_idx > INT32_MAX || tx_idx < INT32_MIN) {
		return -EINVAL;
	}

	return cs35l45_set_tx_data_source(dev, data_source, tx_idx);
}

/* Device name autocompletion support */
static void device_name_get(size_t idx, struct shell_static_entry *entry)
{
	const struct device *dev = shell_device_lookup(idx, NULL);

	entry->syntax = (dev != NULL) ? dev->name : NULL;
	entry->handler = NULL;
	entry->help = NULL;
	entry->subcmd = NULL;
}

SHELL_DYNAMIC_CMD_CREATE(dsub_device_name, device_name_get);

/* clang-format off */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_codec,
	SHELL_CMD_ARG(set_tx_source, &dsub_device_name, CODEC_SET_TX_SOURCE_HELP, cmd_set_tx_source,
			4, 0),
	SHELL_SUBCMD_SET_END
);
/* clang-format on */

SHELL_CMD_REGISTER(cs35l45, &sub_codec, "CS35L45 custom commands", NULL);
