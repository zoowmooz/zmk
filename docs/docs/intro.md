---
title: Introduction to ZMK
sidebar_label: Introduction
slug: /
---

ZMK Firmware is an open source (MIT) keyboard
firmware built on the [Zephyr™ Project](https://zephyrproject.org/) Real Time Operating System (RTOS). ZMK's goal is to provide a modern, wireless, and powerful firmware free of licensing issues.

## Features

ZMK is currently missing some features found in other popular firmware. This table compares the features supported by ZMK, BlueMicro and QMK:

| Legend: | ✅ Supported | 🚧 Under Development | 💡 Planned |
| :------ | :----------- | :------------------- | :--------- |

| **Feature**                                                                                                                                                | ZMK | BlueMicro | QMK |
| ---------------------------------------------------------------------------------------------------------------------------------------------------------- | :-: | :-------: | :-: |
| Low Latency BLE Support                                                                                                                                    | ✅  |    ✅     |     |
| Multi-Device BLE Support                                                                                                                                   | ✅  |           |     |
| [USB Connectivity](keymaps/behaviors/outputs.md)                                                                                                           | ✅  |    ✅     | ✅  |
| User Configuration Repositories                                                                                                                            | ✅  |           |     |
| Split Keyboard Support                                                                                                                                     | ✅  |    ✅     | ✅  |
| [Keymaps and Layers](keymaps/behaviors/layers.md)                                                                                                          | ✅  |    ✅     | ✅  |
| [Hold-Tap](keymaps/behaviors/hold-tap.mdx) (which includes [Mod-Tap](keymaps/behaviors/mod-tap.md) and [Layer-Tap](keymaps/behaviors/layers.md#layer-tap)) | ✅  |    ✅     | ✅  |
| [Tap-Dance](keymaps/behaviors/tap-dance.mdx)                                                                                                               | ✅  |  ✅[^2]   | ✅  |
| [Keyboard Keycodes](keymaps/list-of-keycodes.mdx#keyboard)                                                                                                 | ✅  |    ✅     | ✅  |
| [Media](keymaps/list-of-keycodes.mdx#media-controls) & [Consumer](keymaps/list-of-keycodes.mdx#consumer-controls) Codes                                    | ✅  |    ✅     | ✅  |
| [Encoders](features/encoders.md)                                                                                                                           | ✅  |    ✅     | ✅  |
| [Display Support](features/displays.md)[^1]                                                                                                                | 🚧  |    🚧     | ✅  |
| [RGB Underglow](features/underglow.md)                                                                                                                     | ✅  |    ✅     | ✅  |
| [Backlight](features/backlight.mdx)                                                                                                                        | ✅  |    ✅     | ✅  |
| One Shot Keys                                                                                                                                              | ✅  |    ✅     | ✅  |
| [Combo Keys](keymaps/combos.md)                                                                                                                            | ✅  |           | ✅  |
| [Macros](keymaps/behaviors/macros.md)                                                                                                                      | ✅  |    ✅     | ✅  |
| Mouse Keys                                                                                                                                                 | ✅  |    ✅     | ✅  |
| Low Active Power Usage                                                                                                                                     | ✅  |           |     |
| Low Power Sleep States                                                                                                                                     | ✅  |    ✅     |     |
| [Low Power Mode (VCC Shutoff)](keymaps/behaviors/power.md)                                                                                                 | ✅  |    ✅     |     |
| Battery Reporting                                                                                                                                          | ✅  |    ✅     |     |
| Shell over BLE                                                                                                                                             | 💡  |           |     |
| Realtime Keymap Updating                                                                                                                                   | 💡  |           | ✅  |
| AVR/8 Bit                                                                                                                                                  |     |           | ✅  |
| [Wide Range of ARM Chips Supported](https://docs.zephyrproject.org/3.5.0/boards/index.html)                                                                | ✅  |           |     |

[^2]: Tap-Dances are limited to single and double-tap on BlueMicro
[^1]: OLEDs are currently proof of concept in ZMK.

## Code of Conduct

Please note that this project is released with a
[Contributor Code of Conduct](https://www.contributor-covenant.org/version/2/0/code_of_conduct/).
By participating in this project you agree to abide by its terms.
