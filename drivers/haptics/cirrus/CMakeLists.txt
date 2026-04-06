# Copyright (c) 2026 Cirrus Logic, Inc.
# SPDX-License-Identifier: Apache-2.0

zephyr_library_sources_ifdef(CONFIG_HAPTICS_CIRRUS cirrus-common.c cirrus-firmware.c cirrus-flash.c)
zephyr_library_sources_ifdef(CONFIG_HAPTICS_CIRRUS_I2C cirrus-i2c.c)
zephyr_library_sources_ifdef(CONFIG_HAPTICS_CIRRUS_SPI cirrus-spi.c)
zephyr_library_sources_ifdef(CONFIG_HAPTICS_CS40L26 cs40l26.c)
zephyr_library_sources_ifdef(CONFIG_HAPTICS_CS40L5X cs40l5x.c)
