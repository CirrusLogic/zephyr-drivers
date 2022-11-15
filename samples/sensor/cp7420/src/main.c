/*
 * Copyright (c) 2022 Cirrus Logic Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>

#define CP7420 DT_INST(0, cirrus_cp7420)

#if DT_NODE_HAS_STATUS(CP7420, okay)
#define CP7420_LABEL DT_LABEL(CP7420)
#else
#error Your devicetree has no enabled nodes with compatible "cirrus,cp7420"
#define CP7420_LABEL "<none>"
#endif

void main(void)
{
	const struct device *const dev = DEVICE_DT_GET_ANY(cirrus_cp7420);

	if (dev == NULL) {
		printk("No device found...\n");
		return;
	}

	while (1) {
	        printk("Found device %s\n", dev->name);
		k_sleep(K_MSEC(1000));
	}
}