/*
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */

/* Bluetooth: Mesh Generic OnOff, Generic Level, Lighting & Vendor Models
 *
 * Copyright (c) 2018 Vikrant More
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <sysinit/sysinit.h>                //  Contains all app settings consolidated from "apps/my_sensor_app/syscfg.yml" and "targets/bluepill_my_sensor/syscfg.yml"

#if MYNEWT_VAL(ACCELEROMETER_ADXL362)

#include "bsp/bsp.h"
#include <console/console.h>  //  For Mynewt console output. Actually points to libs/semihosting_console
#include <sensor/sensor.h>    //  For Mynewt Sensor Framework
#include <custom_coap/custom_coap.h>        //  For sensor_value
#include "adxl362/adxl362.h"
#include "collector.h"

static TX_REASON reason;

//Declare called function by notifier
static int motion_detected(struct sensor* sensor, void *arg, sensor_event_type_t evtype);

//  Define the notifier function to be called when detector is moving.
static struct sensor_notifier notifier = {
    /* The type of sensor event to be notified on, this is interpreted as a
     * mask, and this notifier is called for all event types on this
     * sensor that match the mask.
     */
    /*SENSOR_EVENT_TYPE_ORIENT_CHANGE*/
    .sn_sensor_event_type = (   SENSOR_EVENT_TYPE_SLEEP |
                                SENSOR_EVENT_TYPE_WAKEUP ),

    /* Sensor event notification handler function, called when a matching
     * event occurred.
     */
    .sn_func = motion_detected,

    /* Opaque argument for the sensor event notification handler function. */
    .sn_arg = NULL
};


int start_motion_detector(void) {
    int rc;
    //  Ask Mynewt to poll the position sensor every 10 seconds and call `handle_position_data()`.
    //  Return 0 if successful.
    if (strlen(ADXL362_DEVICE_NAME) == 0) { return 0; }  //  Sensor device not defined.

    console_printf("Motion Notif. by %s\n", ADXL362_DEVICE_NAME);

    //  Fetch the sensor by name, without locking the driver for exclusive access.
    struct sensor *motion_detector = sensor_mgr_find_next_bydevname(ADXL362_DEVICE_NAME, NULL);
    assert(motion_detector != NULL);

    rc = sensor_register_notifier(motion_detector, &notifier);
    assert(rc == 0);
   
    return 0;
}



static int motion_detected(struct sensor* sensor, void *arg, sensor_event_type_t evtype)
{
    console_printf("%s\n", (evtype==SENSOR_EVENT_TYPE_SLEEP)?"Motion stopped":"Motion detected");

    reason = (evtype==SENSOR_EVENT_TYPE_SLEEP)?TIMEOUT_HALTED:TIMEOUT_MOVING;

    struct os_event ev_md;
    ev_md.ev_arg = &reason;

    send_datacollector(&ev_md);

    return 0;

}
#endif
