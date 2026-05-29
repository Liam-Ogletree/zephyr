/*
 * Copyright (c) 2025 Cirrus Logic, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/haptics.h>
#include <zephyr/drivers/haptics/cs40l5x.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device_runtime.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(main);

const struct device *cs40l5x[] = {
	DEVICE_DT_GET(DT_ALIAS(haptic0)),
	DEVICE_DT_GET(DT_ALIAS(haptic1)),
	DEVICE_DT_GET(DT_ALIAS(haptic2)),
	DEVICE_DT_GET(DT_ALIAS(haptic3)),
};

static void cs40l5x_dummy_callback(const struct device *const dev, const uint32_t errors,
				   void *const user_data)
{
	LOG_INF("fatal errors detected (0x%08X)", errors);
}

int main(void)
{
	int ret;

	for (int i = 0; i < ARRAY_SIZE(cs40l5x); i++) {
		if (!cs40l5x[i] || !device_is_ready(cs40l5x[i])) {
			LOG_ERR("device not available: %s", cs40l5x[i]->name);
			return -ENODEV;
		}

		if (IS_ENABLED(CONFIG_PM_DEVICE_RUNTIME)) {
			ret = pm_device_runtime_enable(cs40l5x[i]);
			if (ret < 0) {
				LOG_ERR("PM runtime disabled for %s (%d)", cs40l5x[i]->name, ret);
				return ret;
			}
		}

		(void)haptics_register_error_callback(cs40l5x[i], cs40l5x_dummy_callback, NULL);
	}

	return 0;
}
