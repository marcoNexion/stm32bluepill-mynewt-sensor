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
//  Poll the temperature sensor every 10 seconds. Transmit the sensor data to the CoAP server after polling.

//  Configure SENSOR_DEVICE and  SENSOR_POLL_TIME by editing `targets/bluepill_my_sensor/syscfg.yml`
//  Mynewt consolidates all app settings into "bin/targets/bluepill_my_sensor/generated/include/syscfg/syscfg.h"
#include <sysinit/sysinit.h>  //  Contains all app settings consolidated from "apps/my_sensor_app/syscfg.yml" and "targets/bluepill_my_sensor/syscfg.yml"
#include <os/os.h>            //  For Mynewt OS functions
#include <console/console.h>  //  For Mynewt console output. Actually points to libs/semihosting_console
#include <sensor/sensor.h>    //  For Mynewt Sensor Framework
#include <custom_coap/custom_coap.h>        //  For sensor_value
#include <custom_sensor/custom_sensor.h>    //  For sensor_temp_raw_data
#include "position.h"
#include "network.h"                        //  For send_sensor_data()
#include "gps_neo6m/gps_neo6m.h"

//  Defined later below
static int handle_position_data(struct sensor* sensor, void *arg, void *sensor_data, sensor_type_t type);

//  Define the listener function to be called after polling the temperature sensor.
static struct sensor_listener listener = {
    .sl_sensor_type = SENSOR_TYPE_GEOLOCATION,  //  Type of sensor: raw ambient temperature (integer)
    .sl_func        = handle_position_data,                   //  Listener function to be called with the sensor data
    .sl_arg         = NULL,
};

int start_position_listener(void) {
    //  Ask Mynewt to poll the position sensor every 10 seconds and call `handle_position_data()`.
    //  Return 0 if successful.
    if (strlen(MYNEWT_VAL(GPS_DEVICE)) == 0) { return 0; }  //  Sensor device not defined.
    console_printf("POS get %s\n", MYNEWT_VAL(GPS_DEVICE));

    //  Set the sensor polling time to 10 seconds.  SENSOR_DEVICE is "temp_stm32_0", SENSOR_POLL_TIME is 10,000.
    int rc = sensor_set_poll_rate_ms(MYNEWT_VAL(GPS_DEVICE), MYNEWT_VAL(GPS_POLL_TIME));
    assert(rc == 0);

    //  Fetch the sensor by name, without locking the driver for exclusive access.
    struct sensor *listen_position = sensor_mgr_find_next_bydevname(MYNEWT_VAL(GPS_DEVICE), NULL);
    assert(listen_position != NULL);

    //  Set the Listener Function to be called every 10 seconds, with the polled sensor data.
    rc = sensor_register_listener(listen_position, &listener);
    assert(rc == 0);

    rc = gps_neo6m_start();
    assert(rc == 0);
    
    return 0;
}

static int handle_position_data(struct sensor* sensor, void *arg, void *sensor_data, sensor_type_t type) {
    //  This listener function is called every 10 seconds by Mynewt to handle the polled sensor data.
    //  Return 0 if we have handled the sensor data successfully.
    if (sensor_data == NULL) { return SYS_EINVAL; }      //  Exit if data is missing
    assert(sensor && type == SENSOR_TYPE_GEOLOCATION);   //  We only support Geolocation sensor data

    //  Interpret the sensor data as a struct that contains geolocation.
    struct sensor_geolocation_data *geolocation = (struct sensor_geolocation_data *) sensor_data;
    if (
        !geolocation->sgd_latitude_is_valid  ||
        !geolocation->sgd_longitude_is_valid /*||
        !geolocation->sgd_altitude_is_valid*/
    ) { 
        console_printf("GPS not ready\n"); console_flush(); ////
        return SYS_EINVAL;  //  Exit if data is not valid
    }

    console_printf("handle_gps_data lat: "); console_printdouble(geolocation->sgd_latitude);
    console_printf(" / lng: ");  console_printdouble(geolocation->sgd_longitude);
    console_printf(" / alt: ");  console_printfloat(geolocation->sgd_altitude);
    console_printf("\n"); console_flush(); ////

    char raw_value[64]="";

    struct custom_value datas;

    datas.id = 0xDEADBEEF;
    datas.tx_reason = START;
    datas.timestamp = 1600081783;
    datas.mV_Bat = 3300;
    datas.position_type = GPS;
    datas.position.gps.latitude = geolocation->sgd_latitude;
    datas.position.gps.longitude = geolocation->sgd_longitude;

    int lat_int, long_int, lat_dec, long_dec;
    bool lat_sign, long_sign;

    //split float for latitude representation
    split_float(datas.position.gps.latitude, &lat_sign, &lat_int, &lat_dec);
    //split float for longitude representation
    split_float(datas.position.gps.longitude, &long_sign, &long_int, &long_dec);

    sprintf(raw_value, 
            "%4x,%1d,%d,%4d,%1d,%s%d.%06d,%s%d.%06d", 
            (unsigned int)datas.id,
            datas.tx_reason,
            (int)datas.timestamp,
            (int)datas.mV_Bat,
            datas.position_type,
            lat_sign?"-":"", lat_int, lat_dec,
            long_sign?"-":"", long_int, long_dec
            );

#if 0
    sprintf(raw_value, 
            "%4x,%1d,%d,%4d,%1d,%f,%f", 
            (unsigned int)datas.id,
            datas.tx_reason,
            (int)datas.timestamp,
            (int)datas.mV_Bat,
            datas.position_type,
            datas.position.gps.latitude,
            datas.position.gps.longitude
            );
#endif
    console_printf("Trying to tx : %s\n", raw_value);

    //  Compose a CoAP message with the temperature sensor data and send to the 
    //  CoAP server.  The message will be enqueued for transmission by the OIC 
    //  background task so this function will return without waiting for the message 
    //  to be transmitted.
    int rc = send_plain_text_data(raw_value);

    //  SYS_EAGAIN means that the network is still starting up. We send at the next poll.
    if (rc == SYS_EAGAIN) { return 0; }
    assert(rc == 0);

    return rc;
}
