/*
 * Copyright 2023 Cirrus Logic, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/syscall_handler.h>
#include <zephyr/drivers/charger.h>

static inline int z_vrfy_charger_get_prop(const struct device *dev,
					     struct charger_get_property *props,
					     size_t props_len)
{
	struct charger_get_property k_props[props_len];

	Z_OOPS(Z_SYSCALL_DRIVER_CHARGER(dev, get_property));

	Z_OOPS(z_user_from_copy(k_props, props,
				props_len * sizeof(struct charger_get_property)));

	int ret = z_impl_charger_get_prop(dev, k_props, props_len);

	Z_OOPS(z_user_to_copy(props, k_props, props_len * sizeof(struct charger_get_property)));

	return ret;
}

#include <syscalls/charger_get_prop_mrsh.c>
