/*
 * Copyright (c) 2024 Cirrus Logic, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "zephyr/sys/printk.h"
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/regulator.h>

int main(void)
{
	const struct device *dev2 = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(cp9314_dev2));
	const struct device *host = DEVICE_DT_GET(DT_NODELABEL(cp9314));
	int ret;

	if (host == NULL) {
		printk("No host CP9314 found...\n");
		return -EIO;
	} else if (dev2 == NULL) {
		printk("No secondary CP9314 found, assuming standalone operation\n");
	}

	if (!device_is_ready(host)) {
		printk("\nError: Device \"%s\" is not ready; "
		       "check the driver initialization logs for errors.\n",
		       host->name);
		return -EIO;
	}

	printk("Found device \"%s\", getting regulator data\n", host->name);

	if (dev2) {
		if (!device_is_ready(dev2)) {
			printk("\nError: Device \"%s\" is not ready; "
			       "check the driver initialization logs for errors.\n",
			       dev2->name);
			return -EIO;
		}

		printk("Found device \"%s\", getting regulator data\n", dev2->name);
	}

	while (1) {
		printk("Found device %s\n", host->name);
		if (dev2) {
			printk("Found device %s\n", dev2->name);
		}

		ret = regulator_enable(host);
		if (ret < 0) {
			printk("Error enabling regulator: %d\n", ret);
			return ret;
		}

		if (dev2) {
			ret = regulator_enable(dev2);
			if (ret < 0) {
				printk("Error enabling regulator: %d\n", ret);
				return ret;
			}
		}

		printk("Converter(s) enabled\n");

		k_sleep(K_SECONDS(10));

		if (dev2) {
			ret = regulator_disable(dev2);
			if (ret < 0) {
				printk("Error disabling regulator: %d\n", ret);
				return ret;
			}
		}

		ret = regulator_disable(host);
		if (ret < 0) {
			printk("Error disabling regulator: %d\n", ret);
			return ret;
		}

		printk("Converter(s) disabled\n");

		k_sleep(K_SECONDS(10));
	}

	return 0;
}
