# Copyright (c) 2020-2023 The ZMK Contributors
# SPDX-License-Identifier: MIT

# Zephyr v4.4 removed drivers/kscan, so we create our own library
# instead of amending the (now absent) Zephyr kscan library.
zephyr_library_named(drivers__kscan)

zephyr_library_sources_ifdef(CONFIG_ZMK_KSCAN_GPIO_DRIVER kscan_gpio.c)
zephyr_library_sources_ifdef(CONFIG_ZMK_KSCAN_GPIO_MATRIX kscan_gpio_matrix.c)
zephyr_library_sources_ifdef(CONFIG_ZMK_KSCAN_GPIO_CHARLIEPLEX kscan_gpio_charlieplex.c)
zephyr_library_sources_ifdef(CONFIG_ZMK_KSCAN_GPIO_DIRECT kscan_gpio_direct.c)
zephyr_library_sources_ifdef(CONFIG_ZMK_KSCAN_GPIO_DEMUX kscan_gpio_demux.c)
zephyr_library_sources_ifdef(CONFIG_ZMK_KSCAN_MOCK_DRIVER kscan_mock.c)
zephyr_library_sources_ifdef(CONFIG_ZMK_KSCAN_COMPOSITE_DRIVER kscan_composite.c)
