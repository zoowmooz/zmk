/*
 * Copyright (c) 2022 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#ifdef CONFIG_ZMK_TRACK_POINT

#define DT_DRV_COMPAT zmk_track_point
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

struct tp_config {
    struct io_channel_config io_channel_x;
    struct io_channel_config io_channel_y;
    struct gpio_channel_config power_gpios;
};

struct tp_data {
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

#define tpbufflen 64 //32
#define tpbuffmsk 63 //31
#define tpbuffsft 6  //5
static int16_t tp_raw_filter_x(uint16_t raw_new) {
    static uint16_t raw_array[tpbufflen] = {0};
    static uint16_t point = 0;
    static uint32_t raw_sum = 0, sqr_sum = 0;
    static int32_t temp_average = 0;
    if (raw_new == 0) {
      temp_average = 0;
      return 0;
    }
    int16_t raw_old = raw_array[(point & tpbuffmsk)];
    int32_t average, variance;
    raw_sum -= raw_old;
    raw_sum += raw_new;
    sqr_sum -= (raw_old * raw_old);
    sqr_sum += (raw_new * raw_new);
    raw_array[(point++ & tpbuffmsk)] = raw_new;
    average = (raw_sum >> tpbuffsft);
    variance = abs((sqr_sum - raw_sum * average) >> tpbuffsft);

    if (variance < CONFIG_ZMK_TRACK_POINT_VARIANCE) temp_average = average;
    if ( temp_average != 0) return raw_new - temp_average;
    else return (point & 16) * ((point & 32) ? - 4:4);
}
static int16_t tp_raw_filter_y(uint16_t raw_new) {
    static uint16_t raw_array[tpbufflen] = {0};
    static uint16_t point = 0;
    static uint32_t raw_sum = 0, sqr_sum = 0;
    static int32_t temp_average = 0;
    if (raw_new == 0) {
      temp_average = 0;
      return 0;
    }
    int16_t raw_old = raw_array[(point & tpbuffmsk)];
    int32_t average, variance;
    raw_sum -= raw_old;
    raw_sum += raw_new;
    sqr_sum -= (raw_old * raw_old);
    sqr_sum += (raw_new * raw_new);
    raw_array[point++ & tpbuffmsk] = raw_new;
    average = raw_sum >> tpbuffsft;
    variance = abs((sqr_sum - raw_sum * average) >> tpbuffsft);

    if (variance < CONFIG_ZMK_TRACK_POINT_VARIANCE) temp_average = average;
    if ( temp_average != 0) return raw_new - temp_average;
    else return (point & 16) * ((point & 32) ? - 4:4);
}

#ifdef CONFIG_ZMK_TRACK_POINT_REDUCE_POWER_CONSUMPTION
static int tp_power_gpio(const struct device *dev, const int power) {
    int rc = 0;
#if DT_INST_NODE_HAS_PROP(0, power_gpios)
    const struct tp_data *drv_data = dev->data;
    const struct tp_config *drv_cfg = dev->config;
    static bool reduce_mode = true;
    int out = power;
    
    if (zmk_usb_is_powered()) {
      out = 1;
      if (reduce_mode) {
	reduce_mode = false;
	rc = tp_raw_filter_x(0);
	rc = tp_raw_filter_y(0);
      }
    } else {
      if (!reduce_mode) {
	reduce_mode = true;
	rc = tp_raw_filter_x(0);
	rc = tp_raw_filter_y(0);
      }
    }

    if (drv_data->gpio) {
        int rc = gpio_pin_set(drv_data->gpio, drv_cfg->power_gpios.pin, out);
        if (rc != 0) {
            LOG_DBG("Failed to %d ADC power GPIO: %d", out, rc);
        }
    }
#endif /* power_gpios */
    return rc;
}
#endif /* CONFIG_ZMK_TRACK_POINT_REDUCE_POWER_CONSUMPTION */

static int tp_sample_fetch(const struct device *dev, enum sensor_channel chan) {
    struct tp_data *drv_data = dev->data;
    struct adc_sequence *as_x = &drv_data->as_x;
    struct adc_sequence *as_y = &drv_data->as_y;
    int rc = 0;

    if (chan != SENSOR_CHAN_ACCEL_X && chan != SENSOR_CHAN_ACCEL_Y && chan != SENSOR_CHAN_ACCEL_Z &&
        chan != SENSOR_CHAN_ACCEL_XYZ) {
        LOG_DBG("Selected channel is not supported: %d.", chan);
        return -ENOTSUP;
    }

#ifdef CONFIG_ZMK_TRACK_POINT_REDUCE_POWER_CONSUMPTION
    rc = tp_power_gpio(dev, 1);
    if (rc != 0)
        return rc;
    // wait for any capacitance to charge up
    if (CONFIG_ZMK_TRACK_POINT_POWER_CHARGE_UP_TIME != 0)
      k_sleep(K_MSEC(CONFIG_ZMK_TRACK_POINT_POWER_CHARGE_UP_TIME));
#endif /* CONFIG_ZMK_TRACK_POINT_REDUCE_POWER_CONSUMPTION */

    // Read ADC
    rc = adc_read(drv_data->adc_x, as_x);
    if (rc != 0) {
        LOG_DBG("Failed to read X ADC: %d", rc);
        return rc;
    }
    as_x->calibrate = false;
    rc = adc_read(drv_data->adc_y, as_y);
    if (rc != 0) {
        LOG_DBG("Failed to read Y ADC: %d", rc);
        return rc;
    }
    as_y->calibrate = false;

#ifdef CONFIG_ZMK_TRACK_POINT_REDUCE_POWER_CONSUMPTION
    rc = tp_power_gpio(dev, 0);
    if (rc != 0)
        return rc;
#endif /* CONFIG_ZMK_TRACK_POINT_REDUCE_POWER_CONSUMPTION */
    drv_data->voltage_x = tp_raw_filter_x(drv_data->adc_raw_x);
    drv_data->voltage_y = tp_raw_filter_y(drv_data->adc_raw_y);

    return 0;
}

static int tp_channel_get(const struct device *dev, enum sensor_channel chan,
                          struct sensor_value *val) {
    struct tp_data *drv_data = dev->data;

    switch (chan) {
    case SENSOR_CHAN_ACCEL_X:
    case SENSOR_CHAN_ACCEL_Y:
    case SENSOR_CHAN_ACCEL_Z:
    case SENSOR_CHAN_ACCEL_XYZ:
        val->val1 = drv_data->voltage_x;
        val->val2 = drv_data->voltage_y;
        break;
    default:
        return -ENOTSUP;
    }

    return 0;
}

static const struct sensor_driver_api tp_api = {
    .sample_fetch = tp_sample_fetch,
    .channel_get = tp_channel_get,
};

static int tp_init(const struct device *dev) {
    struct tp_data *drv_data = dev->data;
    const struct tp_config *drv_cfg = dev->config;
    int rc = 0;

    drv_data->adc_x = device_get_binding(drv_cfg->io_channel_x.label);
    if (drv_data->adc_x == NULL) {
        LOG_ERR("ADC %s failed to retrieve", drv_cfg->io_channel_x.label);
        return -ENODEV;
    }

    drv_data->adc_y = device_get_binding(drv_cfg->io_channel_y.label);
    if (drv_data->adc_y == NULL) {
        LOG_ERR("ADC %s failed to retrieve", drv_cfg->io_channel_y.label);
        return -ENODEV;
    }

    drv_data->as_x = (struct adc_sequence){
        .channels = BIT(drv_cfg->io_channel_x.channel),
        .buffer = &drv_data->adc_raw_x,
        .buffer_size = sizeof(drv_data->adc_raw_x),
        .oversampling = 4,
        .calibrate = true,
    };

    drv_data->as_y = (struct adc_sequence){
        .channels = BIT(drv_cfg->io_channel_y.channel),
        .buffer = &drv_data->adc_raw_y,
        .buffer_size = sizeof(drv_data->adc_raw_y),
        .oversampling = 4,
        .calibrate = true,
    };

#ifdef CONFIG_ADC_NRFX_SAADC
    drv_data->acc_x = (struct adc_channel_cfg){
        .gain = ADC_GAIN_1_6,
        .reference = ADC_REF_INTERNAL,
        .acquisition_time = ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 40),
        .channel_id = drv_cfg->io_channel_x.channel,
        .input_positive = SAADC_CH_PSELP_PSELP_AnalogInput0 + drv_cfg->io_channel_x.channel,
    };
    drv_data->as_x.resolution = 12;

    drv_data->acc_y = (struct adc_channel_cfg){
        .gain = ADC_GAIN_1_6,
        .reference = ADC_REF_INTERNAL,
        .acquisition_time = ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 40),
        .channel_id = drv_cfg->io_channel_y.channel,
        .input_positive = SAADC_CH_PSELP_PSELP_AnalogInput0 + drv_cfg->io_channel_y.channel,
    };
    drv_data->as_y.resolution = 12;

#else
#error Unsupported ADC
#endif

    rc = adc_channel_setup(drv_data->adc_x, &drv_data->acc_x);
    LOG_DBG("AIN%u label:%s setup returned %d", drv_cfg->io_channel_x.channel,
            drv_cfg->io_channel_x.label, rc);
    if (rc != 0) {
        LOG_ERR("adc_channel_setup X err %d", rc);
        return rc;
    }
    rc = adc_channel_setup(drv_data->adc_y, &drv_data->acc_y);
    LOG_DBG("AIN%u label:%s setup returned %d", drv_cfg->io_channel_y.channel,
            drv_cfg->io_channel_y.label, rc);
    if (rc != 0) {
        LOG_ERR("adc_channel_setup Y err %d", rc);
        return rc;
    }

#if DT_INST_NODE_HAS_PROP(0, power_gpios)
    if (drv_cfg->power_gpios.label) {
        drv_data->gpio = device_get_binding(drv_cfg->power_gpios.label);
        if (drv_data->gpio == NULL) {
            LOG_ERR("Failed to get GPIO %s", drv_cfg->power_gpios.label);
            return -ENODEV;
        }
        rc = gpio_pin_configure(drv_data->gpio, drv_cfg->power_gpios.pin,
                                GPIO_OUTPUT_ACTIVE | drv_cfg->power_gpios.flags);
        if (rc != 0) {
            LOG_ERR("Failed to control feed %s.%u: %d", drv_cfg->power_gpios.label,
                    drv_cfg->power_gpios.pin, rc);
            return rc;
        }
        rc = gpio_pin_set(drv_data->gpio, drv_cfg->power_gpios.pin, 1);
        if (rc != 0) {
            LOG_DBG("Failed to enable ADC power GPIO: %d", rc);
            return rc;
        }
    }
#endif
    return rc;
}

static struct tp_data tp_data;
static const struct tp_config tp_cfg = {
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

DEVICE_DT_INST_DEFINE(0, &tp_init, device_pm_control_nop, &tp_data, &tp_cfg, POST_KERNEL,
                      CONFIG_SENSOR_INIT_PRIORITY, &tp_api);

#endif /* CONFIG_ZMK_TRACK_POINT */
