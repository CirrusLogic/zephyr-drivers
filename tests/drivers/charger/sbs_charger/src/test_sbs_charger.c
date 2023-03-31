/*
 * Copyright 2023 Cirrus Logic, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/charger.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
#include <zephyr/ztest.h>
#include <zephyr/ztest_assert.h>

struct sbs_charger_fixture {
	const struct device *dev;
	const struct charger_driver_api *api;
};

static void *sbs_charger_setup(void)
{
	static ZTEST_DMEM struct sbs_charger_fixture fixture;

	fixture.dev = DEVICE_DT_GET_ANY(sbs_sbs_charger);

	k_object_access_all_grant(fixture.dev);

	zassert_true(device_is_ready(fixture.dev), "Charger not found");

	return &fixture;
}

ZTEST_USER_F(sbs_charger, test_get_all_props_failed_returns_negative)
{
	struct charger_get_property props[] = {
		{
			/* Invalid property */
			.property_type = CHARGER_PROP_MAX,
		},
	};

	int ret = charger_get_prop(fixture->dev, props, ARRAY_SIZE(props));

	zassert_equal(props[0].err, -ENOTSUP, "Getting bad property %d has a good status.",
		      props[0].property_type);

	zassert_true(ret < 0);
}

ZTEST_USER_F(sbs_charger, test_get_some_props_failed_returns_failed_prop_count)
{
	struct charger_get_property props[] = {
		{
			/* First invalid property */
			.property_type = CHARGER_PROP_MAX,
		},
		{
			/* Second invalid property */
			.property_type = CHARGER_PROP_MAX,
		},
		{
			/* Valid property */
			.property_type = CHARGER_HEALTH,
		},

	};

	int ret = charger_get_prop(fixture->dev, props, ARRAY_SIZE(props));

	zassert_equal(props[0].err, -ENOTSUP, "Getting bad property %d has a good status.",
		      props[0].property_type);

	zassert_equal(props[1].err, -ENOTSUP, "Getting bad property %d has a good status.",
		      props[1].property_type);

	zassert_ok(props[2].err, "Property %d getting %d has a bad status.", 2,
		   props[2].property_type);

	zassert_equal(ret, 2);
}

ZTEST_USER_F(sbs_charger, test_get_props__returns_ok)
{
	/* Validate what props are supported by the driver */

	struct charger_get_property props[] = {
		{
			.property_type = CHARGER_HEALTH,
		},
		{
			.property_type = CHARGER_ONLINE,
		},
		{
			.property_type = CHARGER_PRESENT,
		},
		{
			.property_type = CHARGER_STATUS,
		},
	};

	int ret = charger_get_prop(fixture->dev, props, ARRAY_SIZE(props));

	for (int i = 0; i < ARRAY_SIZE(props); i++) {
		zassert_ok(props[i].err, "Property %d getting %d has a bad status.", i,
			   props[i].property_type);
	}

	zassert_ok(ret);
}

ZTEST_SUITE(sbs_charger, NULL, sbs_charger_setup, NULL, NULL, NULL);
