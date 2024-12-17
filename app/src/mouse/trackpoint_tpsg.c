/*
 * Copyright (c) 2024 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#ifdef CONFIG_ZMK_TPSG

#include <device.h>
#include <init.h>
#include <kernel.h>
#include <drivers/sensor.h>
#include <bluetooth/services/bas.h>

#include <logging/log.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#include <zmk/event_manager.h>
#include <zmk/hid.h>
#include <zmk/endpoints.h>
#include <zmk/usb.h>

const struct device *tpsg;
struct k_timer *tpsg_timer_p;

struct zmk_hid_mouse_report *zmk_tpsg_trans(int32_t x, int32_t y) {
  struct zmk_hid_mouse_report *mouse_report = zmk_hid_get_mouse_report();
  if ((x == 16384)||(y == -16384)) {
    static int loop = 0;
    switch((loop++>>3)&3){
    case 0:
      mouse_report->body.x =  1;
      mouse_report->body.y = -1;
      break;
    case 1:
      mouse_report->body.x = -1;
      mouse_report->body.y = -1;
      break;
    case 2:
      mouse_report->body.x = -1;
      mouse_report->body.y =  1;
      break;
    case 3:
      mouse_report->body.x =  1;
      mouse_report->body.y =  1;
      break;
    }
    return mouse_report;

  }
  
  if (0 < x) {
      mouse_report->body.x =
	      (int16_t)(((x * x) >> CONFIG_ZMK_TPSG_AXPR) + (x >> CONFIG_ZMK_TPSG_BXPR)); // Right
  } else {
	    x *= -1;
      mouse_report->body.x = -1 *
	      (int16_t)(((x * x) >> CONFIG_ZMK_TPSG_AXMR) + (x >> CONFIG_ZMK_TPSG_BXMR)); // Left
  }
    
  if (0 < y) {
      mouse_report->body.y =
	      (int16_t)(((y * y) >> CONFIG_ZMK_TPSG_AYPR) + (y >> CONFIG_ZMK_TPSG_BYPR)); // Down
  } else {
	    y *= -1;
      mouse_report->body.y = -1 *
	      (int16_t)(((y * y) >> CONFIG_ZMK_TPSG_AYMR) + (y >> CONFIG_ZMK_TPSG_BYMR)); // Up
  }

#ifdef CONFIG_ZMK_TPSG_GAIN_TUNING
  LOG_DBG("GAIN TUNING %6d %6d %6d %6d", x, y, mouse_report->body.x, mouse_report->body.y);
#endif /* CONFIG_ZMK_TPSG_GAIN_TUNING */

  return mouse_report;
}

static int zmk_tpsg_update(const struct device *tpsg) {
    struct sensor_value state_of_charge;

    int rc = sensor_sample_fetch_chan(tpsg, SENSOR_CHAN_ACCEL_XYZ);
    if (rc != 0) {
        LOG_DBG("Failed to fetch tpsg values: %d", rc);
        return rc;
    }

    rc = sensor_channel_get(tpsg, SENSOR_CHAN_ACCEL_XYZ, &state_of_charge);
    if (rc != 0) {
        LOG_DBG("Failed to get tpsg state of charge: %d", rc);
        return rc;
    }

    struct zmk_hid_mouse_report *mouse_report;
    mouse_report = zmk_tpsg_trans(state_of_charge.val1, state_of_charge.val2 * -1);

    {
      int x = mouse_report->body.x, y = mouse_report->body.y;;
      int z = x * x + y * y;
      if ( z != 0 ) zmk_endpoints_send_mouse_report();
    }
    return rc;
}

static void zmk_tpsg_work(struct k_work *work) {
    int rc = zmk_tpsg_update(tpsg);
    if (rc != 0) {
        LOG_DBG("Failed to update tpsg value: %d.", rc);
    }
}

K_WORK_DEFINE(tpsg_work, zmk_tpsg_work);

static void zmk_tpsg_timer(struct k_timer *timer) { k_work_submit(&tpsg_work); }

K_TIMER_DEFINE(tpsg_timer, zmk_tpsg_timer, NULL);

static int zmk_tpsg_init(const struct device *_arg) {
    tpsg = device_get_binding("TRACKPOINT_TPSG");
    if (tpsg == NULL) {
        LOG_WRN("No tpsg device labelled TRACKPOINT_TPSG found.");
        return -ENODEV;
    }
    zmk_hid_mouse_clear();

    int rc = zmk_tpsg_update(tpsg);

    if (rc != 0) {
        LOG_DBG("Failed to update tpsg value: %d.", rc);
        return rc;
    }

    k_timer_start(&tpsg_timer, K_MSEC(CONFIG_ZMK_TPSG_SCAN_TIME), K_MSEC(CONFIG_ZMK_TPSG_SCAN_TIME));
    tpsg_timer_p = &tpsg_timer;

    return 0;
}

SYS_INIT(zmk_tpsg_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

#endif /* CONFIG_ZMK_TPSG */
