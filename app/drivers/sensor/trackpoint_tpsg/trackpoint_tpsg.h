/*
 * Copyright (c) 2022 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_trackpoint_strain_gauge
#include <device.h>
#include <drivers/gpio.h>
#include <drivers/adc.h>
#include <drivers/sensor.h>
#include <logging/log.h>
#include <stdlib.h>
#include <zmk/usb.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

struct io_channel_config {
    const char *label;
    uint8_t channel;
};

struct gpio_channel_config {
    const char *label;
    uint8_t pin;
    uint8_t flags;
};

struct tpsg_config {
    struct io_channel_config io_channel_x;
    struct io_channel_config io_channel_y;
    struct gpio_channel_config power_gpios;
};

struct tpsg_data {
    const struct device *adc_x;
    const struct device *adc_y;
    const struct device *gpio;
    struct adc_channel_cfg acc_x;
    struct adc_channel_cfg acc_y;
    struct adc_sequence as_x;
    struct adc_sequence as_y;
    uint16_t adc_raw_x;
    uint16_t adc_raw_y;
    int16_t voltage_x;
    int16_t voltage_y;
    uint8_t state_of_charge;
};

static int tpsg_init(const struct device *dev);
  
static int tpsg_sample_fetch(const struct device *dev, enum sensor_channel chan);

static int tpsg_channel_get(const struct device *dev, enum sensor_channel chan,
			    struct sensor_value *val);

static struct tpsg_data tpsg_data;

static const struct tpsg_config tpsg_cfg = {
    .io_channel_x =
        {
            DT_INST_IO_CHANNELS_LABEL(0),
            DT_INST_IO_CHANNELS_INPUT_BY_IDX(0, 0),
        },
    .io_channel_y =
        {
            DT_INST_IO_CHANNELS_LABEL(0),
            DT_INST_IO_CHANNELS_INPUT_BY_IDX(0, 1),
        },
#if DT_INST_NODE_HAS_PROP(0, power_gpios)
    .power_gpios =
        {
            DT_INST_GPIO_LABEL(0, power_gpios),
            DT_INST_GPIO_PIN(0, power_gpios),
            DT_INST_GPIO_FLAGS(0, power_gpios),
        },
#endif
};

static const struct sensor_driver_api tpsg_api = {
  .sample_fetch = tpsg_sample_fetch,
  .channel_get = tpsg_channel_get,
};

DEVICE_DT_INST_DEFINE(0, &tpsg_init, device_pm_control_nop, &tpsg_data, &tpsg_cfg, POST_KERNEL,
                      CONFIG_SENSOR_INIT_PRIORITY, &tpsg_api);
