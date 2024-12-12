# Copyright (c) 2024 The ZMK Contributors
# SPDX-License-Identifier: MIT

zephyr_include_directories(${CMAKE_SOURCE_DIR}/include)

zephyr_library()

zephyr_library_sources(trackpoint_tpsg.c)
