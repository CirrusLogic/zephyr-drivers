/*
 * Copyright 2023 Cirrus Logic, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_CHARGER_H_
#define ZEPHYR_INCLUDE_DRIVERS_CHARGER_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/device.h>

/* Keep these alphabetized wrt property name */

/** Runtime Dynamic Battery Parameters */

/** Represents the health of the charger */
#define CHARGER_HEALTH          0
/** Indicates if VBUS is present for the charger */
#define CHARGER_ONLINE          CHARGER_HEALTH + 1
/** Reports whether or not a battery is present */
#define CHARGER_PRESENT         CHARGER_ONLINE + 1
/** Represents the charging status of the charger */
#define CHARGER_STATUS          CHARGER_PRESENT + 1

/** Reserved to demark end of common charger properties */
#define CHARGER_COMMON_COUNT    CHARGER_STATUS + 1
/**
 * Reserved to demark downstream custom properties - use this value as the actual value may
 * change over future versions of this API
 */
#define CHARGER_CUSTOM_BEGIN    CHARGER_COMMON_COUNT + 1

/** Reserved to demark end of valid enum properties */
#define CHARGER_PROP_MAX        UINT16_MAX

enum charger_health {
	CHARGER_HEALTH_UNKNOWN = 0,
	CHARGER_HEALTH_COLD,
	CHARGER_HEALTH_OVERHEAT,
	CHARGER_HEALTH_GOOD,
};

enum charger_online {
	CHARGER_ONLINE_OFFLINE = 0,
	CHARGER_ONLINE_FIXED,
	CHARGER_ONLINE_PROGRAMMABLE,
};

enum charger_status {
	CHARGER_STATUS_UNKNOWN = 0,
	CHARGER_STATUS_CHARGING,
	CHARGER_STATUS_DISCHARGING,
	CHARGER_STATUS_NOT_CHARGING,
	CHARGER_STATUS_FULL,
};

struct charger_get_property {
	/** Battery charger property to get */
	uint16_t property_type;

	/** Negative error status set by callee e.g. -ENOTSUP for an unsupported property */
	int err;

	/** Property field for getting */
	union {
		/* Fields have the format: */
		/* CHARGER_PROPERTY_FIELD */
		/* type property_field; */

		/* Dynamic Charger Info */
	/** CHARGER_HEALTH */
	enum charger_health health;
	/** CHARGER_ONLINE */
	enum charger_online online;
	/** CHARGER_PRESENT */
	bool present;
	/** CHARGER_STATUS */
	enum charger_status status;
	} value;
};

/**
 * @brief Fetch a battery charger property
 *
 * @param dev Pointer to the battery charger device
 * @param props pointer to array of charger_get_property struct where the property struct
 * field is set by the caller to determine what property is read from the
 * charger device into the charger_get_property struct's value field.
 * @param props_len number of properties in props array
 *
 * @return return=0 if successful, return < 0 if getting all properties failed, return > 0 if some
 * properties failed where return=number of failing properties.
 */
typedef int (*charger_get_property_t)(const struct device *dev,
					 struct charger_get_property *props, size_t props_len);

/* Caching is entirely on the onus of the client */

__subsystem struct charger_driver_api {
	charger_get_property_t get_property;
};

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* ZEPHYR_INCLUDE_DRIVERS_CHARGER_H_ */
