/*
 * Copyright (c) 2020 The ZMK Contributors
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
    struct io_channel_config io_channel;  
    struct io_channel_config io_channel_y;
    struct gpio_channel_config power_gpios;
    uint32_t output_ohm;
    uint32_t full_ohm;
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

static uint8_t lithium_ion_mv_to_pct(int16_t bat_mv) {
    // Simple linear approximation of a battery based off adafruit's discharge graph:
    // https://learn.adafruit.com/li-ion-and-lipoly-batteries/voltages

    if (bat_mv >= 4200) {
        return 100;
    } else if (bat_mv <= 3450) {
        return 0;
    }

    return bat_mv * 2 / 15 - 459;
}

    //LOG_HEXDUMP_DBG((uint32_t *)(0x50000000 + 0x504), 36,"OUT       :");
    //LOG_HEXDUMP_DBG((uint32_t *)(0x50000000 + 0x700), 128,"PN_CNF[0] :");
    //LOG_HEXDUMP_DBG((uint32_t *)(0x50000000 + 0x724),  4,"PN_CNF[ 9] :");
    //LOG_HEXDUMP_DBG((uint32_t *)(0x50000000 + 0x77C),  4,"PN_CNF[31] :");
    // Make sure selected channel is supported

static int16_t tp_raw_filter_x(uint16_t raw_new){
  static uint16_t raw_array[32];
  static  int16_t point = 0;
  static uint32_t raw_sum = 0, sqr_sum = 0;
  static  int32_t temp_average = 0;
  int16_t         raw_old = raw_array[point & 31];
  int32_t         average, variance;
  raw_sum -= raw_old;  raw_sum += raw_new;
  sqr_sum -= raw_old * raw_old;  sqr_sum += raw_new * raw_new;
  raw_array[point++ & 31] = raw_new;
  average  = raw_sum >>5; /* avarage = raw_sum / 2^sizeofbuffer */
  variance = (sqr_sum - raw_sum * average) >>5;

  if ( variance < 1000 && (3 < abs(temp_average - average) || temp_average == 0))
    temp_average = average;
  //LOG_DBG("X c%5d rab:%5d tab:%5d var:%d", raw_new, average, temp_average, variance);
  if ( 3 < abs(raw_new - temp_average) && temp_average != 0 )
    return raw_new - temp_average;
  else
    return 0;
}

static uint16_t tp_raw_filter_y(uint16_t raw_new){
  static uint16_t raw_array[32];
  static  int16_t point = 0;
  static uint32_t raw_sum = 0, sqr_sum = 0;
  static  int32_t temp_average = 0;
  int16_t         raw_old = raw_array[point & 31];
  int32_t         average, variance;
  raw_sum -= raw_old;  raw_sum += raw_new;
  sqr_sum -= raw_old * raw_old;  sqr_sum += raw_new * raw_new;
  raw_array[point++ & 31] = raw_new;
  average  = raw_sum >>5; /* avarage = raw_sum / 2^sizeofbuffer */
  variance = (sqr_sum - raw_sum * average) >>5;

  if ( variance < 1000 && (3 < abs(temp_average - average) || temp_average == 0))
    temp_average = average;
  //LOG_DBG("Y c%5d rab:%5d tab:%5d var:%d", raw_new, average, temp_average, variance);
  if ( 3 < abs(raw_new - temp_average) && temp_average != 0 )
    return raw_new - temp_average;
  else
    return 0;
}

static int tp_sample_fetch(const struct device *dev, enum sensor_channel chan) {
    struct tp_data *drv_data = dev->data;
    const struct tp_config *drv_cfg = dev->config;
    struct adc_sequence *as_x = &drv_data->as_x;
    struct adc_sequence *as_y = &drv_data->as_y;
    //int val_holdvref = 0;
    //int val_resetmaxmin = 0;

    if (chan != SENSOR_CHAN_ACCEL_X && chan != SENSOR_CHAN_ACCEL_Y &&
        chan != SENSOR_CHAN_ACCEL_Z && chan != SENSOR_CHAN_ACCEL_XYZ) {
        LOG_DBG("Selected channel is not supported: %d.", chan);
        return -ENOTSUP;
    }

    //val_holdvref = gpio_pin_get_raw(drv_data->gpio, 9);
    //val_resetmaxmin = gpio_pin_get_raw(drv_data->gpio,10);
    int rc = 0;
    int rc_y = 0;
    
    // Enable power GPIO if present
    if (drv_data->gpio) {
//        rc = gpio_pin_set(drv_data->gpio, drv_cfg->power_gpios.pin, 1);
        if (rc != 0) {
            LOG_DBG("Failed to enable ADC power GPIO: %d", rc);
            return rc;
        }
        // wait for any capacitance to charge up
	//k_sleep(K_MSEC(5));
    }

    // Read ADC
    rc = adc_read(drv_data->adc_x, as_x);
    as_x->calibrate = false;
    rc_y = adc_read(drv_data->adc_y, as_y);
    as_y->calibrate = false;

    // Disable power GPIO if present
    if (drv_data->gpio) {
      //LOG_DBG("power GPIO %d OFF",drv_cfg->power_gpios.pin);
      //int rc2 = gpio_pin_set(drv_data->gpio, drv_cfg->power_gpios.pin, val_holdvref);
//        int rc2 = gpio_pin_set(drv_data->gpio, drv_cfg->power_gpios.pin, 0);
//        if (rc2 != 0) {
//            LOG_DBG("Failed to disable ADC power GPIO: %d", rc2);
//            return rc2;
//        }
    }

    if (rc == 0) {
        drv_data->voltage_x = tp_raw_filter_x(drv_data->adc_raw_x);
    } else {
        LOG_DBG("Failed to read X ADC: %d", rc);
    }

    if (rc_y == 0) {
        drv_data->voltage_y = tp_raw_filter_y(drv_data->adc_raw_y);
    } else {
        LOG_DBG("Failed to read Y ADC: %d", rc_y);
    }
    /*
    LOG_DBG("gpio raw");
    for (int i=0;i<48;){
      LOG_DBG("+%2d:%d %d %d %d %d %d %d %d ",i,
	      gpio_pin_get_raw(drv_data->gpio,i++),
	      gpio_pin_get_raw(drv_data->gpio,i++),
	      gpio_pin_get_raw(drv_data->gpio,i++),
	      gpio_pin_get_raw(drv_data->gpio,i++),
	      gpio_pin_get_raw(drv_data->gpio,i++),
	      gpio_pin_get_raw(drv_data->gpio,i++),
	      gpio_pin_get_raw(drv_data->gpio,i++),
	      gpio_pin_get_raw(drv_data->gpio,i++)
	      );
    }
    */
    return rc;
}

static int tp_channel_get(const struct device *dev, enum sensor_channel chan,
                           struct sensor_value *val) {
    struct tp_data *drv_data = dev->data;
    //LOG_DBG("tp_channel_get:name=%s chan=%d",dev->name, chan);
    
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


    drv_data->adc_x = device_get_binding(drv_cfg->io_channel.label);

    if (drv_data->adc_x == NULL) {
        LOG_ERR("ADC %s failed to retrieve", drv_cfg->io_channel.label);
        return -ENODEV;
    }

    drv_data->adc_y = device_get_binding(drv_cfg->io_channel_y.label);

    if (drv_data->adc_y == NULL) {
        LOG_ERR("ADC %s failed to retrieve", drv_cfg->io_channel_y.label);
        return -ENODEV;
    }

    int rc = 0;

    if (drv_cfg->power_gpios.label) {
      //LOG_ERR(" power_gpios.label %s", drv_cfg->power_gpios.label);
      //LOG_ERR(" power_gpios.pin   %d", drv_cfg->power_gpios.pin);
      //LOG_ERR(" power_gpios.flag  %x", drv_cfg->power_gpios.flags);
      drv_data->gpio = device_get_binding(drv_cfg->power_gpios.label);
        if (drv_data->gpio == NULL) {
            LOG_ERR("Failed to get GPIO %s", drv_cfg->power_gpios.label);
            return -ENODEV;
        }
        rc = gpio_pin_configure(drv_data->gpio, drv_cfg->power_gpios.pin,
                                /* GPIO_OUTPUT_INACTIVE |*/
				GPIO_OUTPUT_ACTIVE |
				drv_cfg->power_gpios.flags);
	/*
				//0x0203);
				256 * 2 +
				4   * 0 +
				2   * 0 +
				1   * 1
				);
	  a = (uint32_t *)(0x50000000 + 0x700);
	  *a = 0xc;
	  */
	if (rc != 0) {
            LOG_ERR("Failed to control feed %s.%u: %d", drv_cfg->power_gpios.label,
                    drv_cfg->power_gpios.pin, rc);
            return rc;
        }
      /*
        rc = gpio_pin_configure(drv_data->gpio,10, GPIO_INPUT|GPIO_PULL_UP);
        if (rc != 0) {
            LOG_ERR("Failed to control test input %u: %d",10, rc);
            return rc;
        }
        rc = gpio_pin_configure(drv_data->gpio, 9, GPIO_INPUT|GPIO_PULL_UP);
        if (rc != 0) {
            LOG_ERR("Failed to control test input %u: %d", 9, rc);
            return rc;
        }

	for(int i=0;i<48;i++) {
	  rc = gpio_pin_configure(drv_data->gpio, i, GPIO_INPUT|GPIO_PULL_UP);
	  if (rc != 0) {
            LOG_ERR(" Failed to control test input %u: %d", i, rc);
            return rc;
	  }
	}
      */
    }

    drv_data->as_x = (struct adc_sequence){
      .channels = BIT(0),
        .buffer = &drv_data->adc_raw_x,
        .buffer_size = sizeof(drv_data->adc_raw_x),
        .oversampling = 4,
        .calibrate = true,
    };

    drv_data->as_y = (struct adc_sequence){
      .channels = BIT(5),
        .buffer = &drv_data->adc_raw_y,
        .buffer_size = sizeof(drv_data->adc_raw_y),
        .oversampling = 4,
        .calibrate = true,
    };

#ifdef CONFIG_ADC_NRFX_SAADC
    drv_data->acc_x = (struct adc_channel_cfg){
        .gain = ADC_GAIN_1_6, 
      //.gain = ADC_GAIN_1_6,
      //.gain = ADC_GAIN_1_5,   /* 2.1mV -> */
      //.gain = ADC_GAIN_1_4,   /* 2.1mV -> */
      //.gain = ADC_GAIN_1_3,   /* 2.1mV -> */
      //.gain = ADC_GAIN_1_2,   /* 2.1mV -> 4095*/
      //.gain = ADC_GAIN_2_3,   /* 2.1mV -> */
      //.gain = ADC_GAIN_1,     /* 2.1mV -> 4095*/
      //.gain = ADC_GAIN_2,     /* 2.1mV -> 4095 */
      //.reference = ADC_REF_VDD_1,     /* < VDD. */
      //.reference = ADC_REF_VDD_1_2,   /* < VDD/2. */
      //.reference = ADC_REF_VDD_1_3,   /* < VDD/3. */
      //.reference = ADC_REF_VDD_1_4,   /**< VDD/4. */
        .reference = ADC_REF_INTERNAL,  /* < Internal. */ /* default */
      //.reference = ADC_REF_EXTERNAL0, /* < External, input 0. */
      //.reference = ADC_REF_EXTERNAL1, /* < External, input 1. */
        .acquisition_time = ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 40),
	.channel_id = 0,
        .input_positive = SAADC_CH_PSELP_PSELP_AnalogInput0 + drv_cfg->io_channel.channel,
    };
    drv_data->as_x.resolution = 12;

    drv_data->acc_y = (struct adc_channel_cfg){
        .gain = ADC_GAIN_1_6, 
      //.gain = ADC_GAIN_1_6,
      //.gain = ADC_GAIN_1_5,
      //.gain = ADC_GAIN_1_4,
      //.gain = ADC_GAIN_1_3,
      //.gain = ADC_GAIN_1_2,
      //.gain = ADC_GAIN_2_3,
      //.gain = ADC_GAIN_1,
      //.gain = ADC_GAIN_2,
      //.reference = ADC_REF_VDD_1,     /* < VDD. */
      //.reference = ADC_REF_VDD_1_2,   /* < VDD/2. */
      //.reference = ADC_REF_VDD_1_3,   /* < VDD/3. */
      //.reference = ADC_REF_VDD_1_4,   /* < VDD/4. */
        .reference = ADC_REF_INTERNAL,  /* < Internal. */ /* default */
      //.reference = ADC_REF_EXTERNAL0, /* < External, input 0. */
      //.reference = ADC_REF_EXTERNAL1, /* < External, input 1. */
        .acquisition_time = ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 40),
	.channel_id = 5,
        .input_positive = SAADC_CH_PSELP_PSELP_AnalogInput0 + drv_cfg->io_channel_y.channel,
    };
    drv_data->as_y.resolution = 12;

#else
#error Unsupported ADC
#endif


    rc = adc_channel_setup(drv_data->adc_x,   &drv_data->acc_x  );
    LOG_DBG("AIN%u label:%s setup returned %d", drv_cfg->io_channel.channel, drv_cfg->io_channel.label, rc);
    if ( rc != 0 ) LOG_ERR("adc_channel_setup X err %d",rc);
    rc = adc_channel_setup(drv_data->adc_y, &drv_data->acc_y);
    LOG_DBG("AIN%u label:%s setup returned %d", drv_cfg->io_channel_y.channel, drv_cfg->io_channel_y.label, rc);
    if ( rc != 0 ) LOG_ERR("adc_channel_setup Y err %d",rc);

    return rc;
}

static struct tp_data tp_data;
static const struct tp_config tp_cfg = {
    .io_channel =
        {
	  DT_INST_IO_CHANNELS_LABEL(0),
	  DT_INST_IO_CHANNELS_INPUT(0),
        },
    .io_channel_y =
        {
            DT_INST_IO_CHANNELS_LABEL(0),
            5,
        },
#if DT_INST_NODE_HAS_PROP(0, power_gpios)
    .power_gpios =
        {
            DT_INST_GPIO_LABEL(0, power_gpios),
            DT_INST_GPIO_PIN(0, power_gpios),
            DT_INST_GPIO_FLAGS(0, power_gpios),
        },
#endif
    .output_ohm = DT_INST_PROP(0, output_ohms),
    .full_ohm = DT_INST_PROP(0, full_ohms),
};

DEVICE_AND_API_INIT(tp_dev, DT_INST_LABEL(0), &tp_init, &tp_data, &tp_cfg, POST_KERNEL,
                    CONFIG_SENSOR_INIT_PRIORITY, &tp_api);

#endif /* CONFIG_ZMK_TRACK_POINT */
