/*
 * Copyright (c) 2022 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#ifdef CONFIG_ZMK_TRACK_POINT

#include <device.h>
#include <init.h>
#include <kernel.h>
#include <drivers/sensor.h>
#include <bluetooth/services/bas.h>

#include <logging/log.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#include <zmk/event_manager.h>
//#include <zmk/trackpoint.h>
//#include <zmk/events/trackpoint_state_changed.h>
#include <zmk/hid.h>
//#include <zmk/events/mouse_state_changed.h>
#include <zmk/endpoints.h>

const struct device *trackpoint;

// static uint8_t last_state_of_charge = 0;

// uint8_t zmk_trackpoint_state_of_charge() { return last_state_of_charge; }

static int zmk_trackpoint_update(const struct device *trackpoint) {
    struct sensor_value state_of_charge;
    // LOG_DBG("zmk_trackpoint_update:name=%s",trackpoint->name);
    
    int rc = sensor_sample_fetch_chan(trackpoint, SENSOR_CHAN_ACCEL_XYZ);
    if (rc != 0) {
        LOG_DBG("Failed to fetch trackpoint values: %d", rc);
        return rc;
    }

    rc = sensor_channel_get(trackpoint, SENSOR_CHAN_ACCEL_XYZ, &state_of_charge);
    if (rc != 0) {
        LOG_DBG("Failed to get trackpoint state of charge: %d", rc);
        return rc;
    }

    // LOG_DBG("X:%5d Y:%5d before by 10 times", state_of_charge.val1, state_of_charge.val2);
    if (state_of_charge.val1 != 0 || state_of_charge.val2 !=0){
      	struct zmk_hid_mouse_report *mouse_report = zmk_hid_get_mouse_report();
	mouse_report->body.y = state_of_charge.val1 > 0 ?
	  (int16_t)(state_of_charge.val1/12) * -1:
	  (int16_t)(state_of_charge.val1/30) * -1;
	mouse_report->body.x = state_of_charge.val2 > 0 ?
	  (int16_t)(state_of_charge.val2/20):
	  (int16_t)(state_of_charge.val2/25);
	zmk_endpoints_send_mouse_report();
    }
    return rc;
}

static void zmk_trackpoint_work(struct k_work *work) {
    int rc = zmk_trackpoint_update(trackpoint);

    if (rc != 0) {
        LOG_DBG("Failed to update trackpoint value: %d.", rc);
    }
}

K_WORK_DEFINE(trackpoint_work, zmk_trackpoint_work);

static void zmk_trackpoint_timer(struct k_timer *timer) { k_work_submit(&trackpoint_work); }

K_TIMER_DEFINE(trackpoint_timer, zmk_trackpoint_timer, NULL);

static int zmk_trackpoint_init(const struct device *_arg) {
    trackpoint = device_get_binding("TRACKPOINT");
    if (trackpoint == NULL) {
      	LOG_DBG("No trackpoint device labelled TRACKPOINT found.");
	return -ENODEV;
    }
    zmk_hid_mouse_clear();
    
    int rc = zmk_trackpoint_update(trackpoint);

    if (rc != 0) {
        LOG_DBG("Failed to update trackpoint value: %d.", rc);
        return rc;
    }

    //    k_timer_start(&trackpoint_timer, K_MINUTES(1), K_MINUTES(1));
    //    k_timer_start(&trackpoint_timer, K_SECONDS(1), K_SECONDS(1));
    k_timer_start(&trackpoint_timer, K_MSEC(50), K_MSEC(50));
    //    k_timer_start(&trackpoint_timer, K_MSEC(50), K_MSEC(50));
    return 0;
}

SYS_INIT(zmk_trackpoint_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

#endif /* CONFIG_ZMK_TRACK_POINT */
