/*
 * Copyright (c) 2022 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include "trackpoint_tpsg.h"

#define TPSG_VAR_BUF_SFT 7 /* buffer length 7,6,5,4,,, */
#define TPSG_VAR_BUF_LEN (0x01 << TPSG_VAR_BUF_SFT)
#define TPSG_VAR_BUF_MSK (TPSG_VAR_BUF_LEN - 1)
#define TPSG_VARIANCE(monitor) ({												\
	static uint16_t raw_array[TPSG_VAR_BUF_LEN] = {0};							\
	static uint16_t point = 0;													\
	static uint32_t raw_sum = 0, sqr_sum = 0;									\
	static int32_t long_average = 0;											\
	int16_t raw_old = raw_array[(point & TPSG_VAR_BUF_MSK)];					\
	int32_t moving_average, variance, ret = 16384;								\
	if (raw_new == 0) return long_average = 0;									\
	raw_sum -= raw_old;															\
	raw_sum += raw_new;															\
	sqr_sum -= (raw_old * raw_old);												\
	sqr_sum += (raw_new * raw_new);												\
	raw_array[(point++ & TPSG_VAR_BUF_MSK)] = raw_new;							\
	moving_average = (raw_sum >> TPSG_VAR_BUF_SFT);								\
	variance = abs((sqr_sum - raw_sum * moving_average) >> TPSG_VAR_BUF_SFT);	\
	if ((long_average == 0) && (variance < CONFIG_ZMK_TPSG_VARIANCE))			\
		long_average = moving_average;											\
	if ( long_average != 0)	ret = raw_new - long_average;						\
	if (monitor) LOG_DBG("TPSG val:%8d mv_avg:%8d avg:%8d var:%10d",			\
				raw_new, moving_average, long_average, variance);				\
	return ret;																	\
})

int16_t tpsg_raw_filter_x(uint16_t raw_new) {
#ifdef    CONFIG_ZMK_TPSG_X_VARIANCE_MONITOR
	TPSG_VARIANCE(true);
#else  /* CONFIG_ZMK_TPSG_X_VARIANCE_MONITOR */
	TPSG_VARIANCE(false);
#endif /* CONFIG_ZMK_TPSG_X_VARIANCE_MONITOR */
}

int16_t tpsg_raw_filter_y(uint16_t raw_new) {
#ifdef    CONFIG_ZMK_TPSG_Y_VARIANCE_MONITOR
	TPSG_VARIANCE(true);
#else  /* CONFIG_ZMK_TPSG_Y_VARIANCE_MONITOR */
	TPSG_VARIANCE(false);
#endif /* CONFIG_ZMK_TPSG_Y_VARIANCE_MONITOR */
}

#ifdef CONFIG_ZMK_TPSG_REDUCE_POWER_CONSUMPTION
static int tpsg_power_gpio(const struct device *dev, const int power) {
	int rc = 0;
#if DT_INST_NODE_HAS_PROP(0, power_gpios)
	const struct tpsg_data *drv_data = dev->data;
	const struct tpsg_config *drv_cfg = dev->config;
	int out = power;

#ifndef CONFIG_ZMK_TPSG_REDUCE_POWER_CONSUMPTION_USB
	static bool reduce_mode = true;
	if (zmk_usb_is_powered()) {
		out = 1;
		if (reduce_mode) {
			reduce_mode = false;
			rc = tpsg_raw_filter_x(0);
			rc = tpsg_raw_filter_y(0);
		}
	} else {
		if (!reduce_mode) {
			reduce_mode = true;
			rc = tpsg_raw_filter_x(0);
			rc = tpsg_raw_filter_y(0);
		}
	}
#endif
	
	if (drv_data->gpio) {
		int rc = gpio_pin_set(drv_data->gpio, drv_cfg->power_gpios.pin, out);
		if (rc != 0) {
			LOG_DBG("Failed to %d ADC power GPIO: %d", out, rc);
		}
	}
#endif /* power_gpios */
	return rc;
}
#endif /* CONFIG_ZMK_TPSG_REDUCE_POWER_CONSUMPTION */

static int tpsg_sample_fetch(const struct device *dev, enum sensor_channel chan) {
	struct tpsg_data *drv_data = dev->data;
	struct adc_sequence *as_x = &drv_data->as_x;
	struct adc_sequence *as_y = &drv_data->as_y;
	int rc = 0;

	if (chan != SENSOR_CHAN_ACCEL_X && chan != SENSOR_CHAN_ACCEL_Y && chan != SENSOR_CHAN_ACCEL_Z &&
		chan != SENSOR_CHAN_ACCEL_XYZ) {
		LOG_DBG("Selected channel is not supported: %d.", chan);
		return -ENOTSUP;
	}

#ifdef CONFIG_ZMK_TPSG_REDUCE_POWER_CONSUMPTION
	rc = tpsg_power_gpio(dev, 1);
	if (rc != 0)
		return rc;
	// wait for any capacitance to charge up
	if (CONFIG_ZMK_TPSG_POWER_CHARGE_UP_TIME != 0)
	  k_sleep(K_MSEC(CONFIG_ZMK_TPSG_POWER_CHARGE_UP_TIME));
#endif /* CONFIG_ZMK_TPSG_REDUCE_POWER_CONSUMPTION */

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

#ifdef CONFIG_ZMK_TPSG_REDUCE_POWER_CONSUMPTION
	rc = tpsg_power_gpio(dev, 0);
	if (rc != 0)
		return rc;
#endif /* CONFIG_ZMK_TPSG_REDUCE_POWER_CONSUMPTION */

//#define CONFIG_ZMK_TPSG_WHEASTONE_BRIDGE_TUNING
#ifdef CONFIG_ZMK_TPSG_WHEASTONE_BRIDGE_TUNING
	{ // for tunning 
	  static int xmax =0,ymax=0,xmin =65535,ymin=65535,xdif=0,ydif=0,xtarget=0,ytarget=0;
	  int x=drv_data->adc_raw_x;
	  int y=drv_data->adc_raw_y;
	  if (xmax < x) xmax = x;
	  if (ymax < y) ymax = y;
	  if (x < xmin) xmin = x;
	  if (y < ymin) ymin = y;
	  xdif = xmax - xmin;
	  ydif = ymax - ymin;
	  xtarget = xmin + xdif /2;
	  ytarget = ymin + ydif /2;
	  LOG_DBG("TPSG:X=%4d (T=%4d)%4d<%4d Y=%4d (T=%4d)%4d<%4d", x,xtarget,xmin,xmax,y,ytarget,ymin,ymax);
	}
#endif /* CONFIG_ZMK_TPSG_WHEASTONE_BRIDGE_TUNING */
	drv_data->voltage_x = tpsg_raw_filter_x(drv_data->adc_raw_x);
	drv_data->voltage_y = tpsg_raw_filter_y(drv_data->adc_raw_y);
	//LOG_DBG("filtered xy %6d %6d", drv_data->voltage_x,drv_data->voltage_y);
	return 0;
}

static int tpsg_channel_get(const struct device *dev, enum sensor_channel chan,
						  struct sensor_value *val) {
	struct tpsg_data *drv_data = dev->data;

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

static int tpsg_init(const struct device *dev) {
	struct tpsg_data *drv_data = dev->data;
	const struct tpsg_config *drv_cfg = dev->config;
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
		//.gain = ADC_GAIN_1_6, // for wings42_tpsg
		.gain = ADC_GAIN_1_3, // for corne_tpst_sxb
		.reference = ADC_REF_INTERNAL,
		.acquisition_time = ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 40),
		.channel_id = drv_cfg->io_channel_x.channel,
		.input_positive = SAADC_CH_PSELP_PSELP_AnalogInput0 + drv_cfg->io_channel_x.channel,
	};
	drv_data->as_x.resolution = 12;

	drv_data->acc_y = (struct adc_channel_cfg){
		//.gain = ADC_GAIN_1_6, // for wings42_tpsg
		.gain = ADC_GAIN_1_3, // for corne_tpst_sxb
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
			LOG_DBG("Failed to enable Strain Gauge and OPAmp power GPIO: %d", rc);
			return rc;
		}
	}
#endif
	return rc;
}
