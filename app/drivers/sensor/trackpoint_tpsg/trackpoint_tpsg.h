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

#define TPSG_VAR_BUF_SFT 6 /* buffer length 6,5,4,,, */
#define TPSG_VAR_BUF_LEN (0x01 << TPSG_VAR_BUF_SFT)
#define TPSG_VAR_BUF_MSK (TPSG_VAR_BUF_LEN - 1)

#define TPSG_VARIANCE(raw_new, debug_x, debug_y) ({			\
      static uint16_t raw_array[TPSG_VAR_BUF_LEN] = {0};		\
      static uint16_t point = 0;					\
      static uint32_t raw_sum = 0, sqr_sum = 0;				\
      static int32_t long_average = 0;					\
      int16_t raw_old = raw_array[(point & TPSG_VAR_BUF_MSK)];		\
      int32_t moving_average, variance;					\
      if (raw_new == 0) {						\
	long_average = 0;						\
	return 0;							\
      }									\
      raw_sum -= raw_old;						\
      raw_sum += raw_new;						\
      sqr_sum -= (raw_old * raw_old);					\
      sqr_sum += (raw_new * raw_new);					\
      raw_array[(point++ & TPSG_VAR_BUF_MSK)] = raw_new;		\
      moving_average = (raw_sum >> TPSG_VAR_BUF_SFT);			\
      variance = abs((sqr_sum - raw_sum * moving_average) >> TPSG_VAR_BUF_SFT);	\
      if ((long_average == 0) && (variance < CONFIG_ZMK_TPSG_VARIANCE)	)			\
	{								\
	  if (debug_x)							\
	    LOG_DBG("TPSG x                                       %8d %8d %8d %10d", raw_new, moving_average, long_average, variance); \
	  if (debug_y)							\
	    LOG_DBG("TPSG y                                           %8d %8d %8d %10d", raw_new, moving_average, long_average, variance); \
	  long_average = moving_average;				\
	}								\
      if ( long_average != 0)						\
	return raw_new - long_average;					\
      else								\
	return 16384;							\
    })

      /* if ((variance < CONFIG_ZMK_TPSG_VARIANCE)	&&			 */
      /* 	  ((long_average == 0) ||					 */
      /* 	   ((long_average != 0) &&					 */
      /* 	    ((abs(long_average - moving_average) < (long_average /10))))))  */
      /* 	{								 */
      /* 	  if (debug_x)							 */
      /* 	    LOG_DBG("TPSG x                                       %8d %8d %8d %10d", raw_new, moving_average, long_average, variance);  */
      /* 	  if (debug_y)							 */
      /* 	    LOG_DBG("TPSG y                                           %8d %8d %8d %10d", raw_new, moving_average, long_average, variance);  */
      /* 	  long_average = moving_average;				 */
      /* 	}								 */



//	  && (long_average != 0) && (abs(long_average - moving_average) < (long_average /200)))
//	  &&								
//	  ((long_average == 0) ||				
//	   ((long_average != 0) &&				
//	    ((abs(long_average - moving_average) < (long_average /200)) || 
//	     (moving_average < long_average)))))			
//
// memo
// wings42_tpsg 1498 1547


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
