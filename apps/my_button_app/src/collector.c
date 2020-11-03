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

//  Collect data for each registered sensor


#include <sysinit/sysinit.h>                //  Contains all app settings consolidated from "apps/my_sensor_app/syscfg.yml" and "targets/bluepill_my_sensor/syscfg.yml"
#include <os/os.h>                          //  For Mynewt OS functions
#include <console/console.h>                //  For Mynewt console output. Actually points to libs/semihosting_console
#include <sensor/sensor.h>                  //  For Mynewt Sensor Framework
#include <sensor/temperature.h>             //  For temperature sensor definitions
#include <sensor/accel.h>             //  For acceleration sensor definitions
#include <sensor_network/sensor_network.h>  //  For Sensor Network Library
#include <custom_coap/custom_coap.h>        //  For sensor_value
#include <custom_sensor/custom_sensor.h>    //  For sensor_temp_raw_data
#include "network.h"                        //  For send_sensor_data()
#include "gps_neo6m/gps_neo6m.h"    

#if MYNEWT_VAL(ACCELEROMETER_ADXL362)
#include "adxl362/adxl362.h"
#endif
#include "collector.h"

static struct os_mutex collect_mtx;

#define KEEP_ALIVE_PERIOD	((MYNEWT_VAL(BC95G_NBIOT_T3412) - 1) * 60 * OS_TICKS_PER_SEC) //every 10 min
static struct os_callout keepalive;
static TX_REASON reason;

static struct{
    
    struct{
        struct sensor *sensor;//GPS
        sensor_type_t sensor_type;
        struct sensor_geolocation_data geolocation;
    }GPS;

    struct{
        struct os_timeval time;
    }UTC;
    //UTC Time
    
    //VDD board Voltage
    struct{
        struct sensor *sensor;
        sensor_type_t sensor_type;
        struct sensor_voltage_data board;
    }VDD;

#if MYNEWT_VAL(ACCELEROMETER_ADXL362)
    //acceleration for motion detection
    struct{
        struct sensor *sensor;
        sensor_type_t sensor_type;
        struct sensor_accel_data accel;
    }Motion;
#endif
    
    struct os_timeval report_time;


}datacollection;


void
collect_lock(void)
{
    os_mutex_pend(&collect_mtx, 20*OS_TICKS_PER_SEC);
}

void
collect_unlock(void)
{
    os_mutex_release(&collect_mtx);
}



static int store_datacollector(struct sensor* sensor, void *arg, void *databuf, sensor_type_t type){

    int r = 0;

    struct sensor_geolocation_data *geolocation;
    struct sensor_voltage_data *voltage;
#if MYNEWT_VAL(ACCELEROMETER_ADXL362)
    struct sensor_accel_data *acceleration;
#endif

    switch(type){
        case SENSOR_TYPE_GEOLOCATION:
                geolocation = (struct sensor_geolocation_data *) databuf;
                datacollection.GPS.geolocation.sgd_latitude = geolocation->sgd_latitude;
                datacollection.GPS.geolocation.sgd_longitude = geolocation->sgd_longitude;
                break;
        
        case SENSOR_TYPE_VOLTAGE:
                voltage = (struct sensor_voltage_data *) databuf;
                datacollection.VDD.board.mV = voltage->mV;
                break;
#if MYNEWT_VAL(ACCELEROMETER_ADXL362)
        case SENSOR_TYPE_ACCELEROMETER:
                acceleration = (struct sensor_accel_data*) databuf;
                datacollection.Motion.accel = *acceleration;
                break;
#endif
        default:
                console_printf("unrecognized sensor type !\n");
                r = -1;
                break;

    }

    return r;
}


int start_datacollector(void){
    int rc;

    if (strlen(MYNEWT_VAL(GPS_DEVICE)) == 0) {
        console_printf("GPS device not defined\n");
        datacollection.GPS.sensor = NULL;

    }else{
        console_printf("GPS %s used in datacollection\n", MYNEWT_VAL(GPS_DEVICE));
        //  Fetch the gps sensor by name, without locking the driver for exclusive access.
        datacollection.GPS.sensor = sensor_mgr_find_next_bydevname(MYNEWT_VAL(GPS_DEVICE), NULL);
        datacollection.GPS.sensor_type = SENSOR_TYPE_GEOLOCATION;
        assert(datacollection.GPS.sensor != NULL);

        rc = gps_neo6m_start();
        assert(rc == 0);
    }
        
    if (strlen(MYNEWT_VAL(VDD_DEVICE)) == 0) {
        console_printf("Vdd sensor not defined\n");
        return -1;
    }else{
        console_printf("Vdd measure %s used in datacollection\n", MYNEWT_VAL(VDD_DEVICE));
        //  Fetch the vdd sensor by name, without locking the driver for exclusive access.
        datacollection.VDD.sensor = sensor_mgr_find_next_bydevname(MYNEWT_VAL(VDD_DEVICE), NULL);
        datacollection.VDD.sensor_type = SENSOR_TYPE_VOLTAGE;
        assert(datacollection.VDD.sensor != NULL);
    }
#if MYNEWT_VAL(ACCELEROMETER_ADXL362)
    if (strlen(ADXL362_DEVICE_NAME) == 0) {
        console_printf("Accel sensor not defined\n");
        return -1;
    }else{
        console_printf("Accel measure %s used in datacollection\n", ADXL362_DEVICE_NAME);
        //  Fetch the vdd sensor by name, without locking the driver for exclusive access.
        datacollection.Motion.sensor = sensor_mgr_find_next_bydevname(ADXL362_DEVICE_NAME, NULL);
        datacollection.Motion.sensor_type = SENSOR_TYPE_ACCELEROMETER;
        assert(datacollection.Motion.sensor != NULL);
    }
#endif
    //datacollector use a mutex to prevent concurrency calls
    os_mutex_init(&collect_mtx);

    //init a keepalive mechanism to uplink periodically
	os_callout_init(&keepalive, os_eventq_dflt_get(), send_datacollector, &reason);
	reason = KEEP_ALIVE;
    os_callout_reset(&keepalive, KEEP_ALIVE_PERIOD);

    //now start network services
    rc = start_server_transport();
    assert(rc == 0);

    return 0;

}

void send_datacollector(struct os_event *ev){
    
    int rc;
    TX_REASON *reason;
    reason = ev->ev_arg;

    collect_lock();

    os_callout_reset(&keepalive, KEEP_ALIVE_PERIOD);

    if(datacollection.GPS.sensor != NULL)
    {
        rc = sensor_read(datacollection.GPS.sensor, datacollection.GPS.sensor_type,
                        store_datacollector, (void *)SENSOR_IGN_LISTENER,
                        OS_TIMEOUT_NEVER);
    }else{
        rc= -1;
    }
    
    if (rc) {
        console_printf("Cannot read GPS sensor %s\n", MYNEWT_VAL(GPS_DEVICE));
        //return 0;
    }

    rc = sensor_read(datacollection.VDD.sensor, datacollection.VDD.sensor_type,
                     store_datacollector, (void *)SENSOR_IGN_LISTENER,
                     OS_TIMEOUT_NEVER);
    if (rc) {
        console_printf("Cannot read VDD sensor %s\n", MYNEWT_VAL(VDD_DEVICE));
        //return 0;
    }

#if MYNEWT_VAL(ACCELEROMETER_ADXL362)
    rc = sensor_read(datacollection.Motion.sensor, datacollection.Motion.sensor_type,
                     store_datacollector, (void *)SENSOR_IGN_LISTENER,
                     OS_TIMEOUT_NEVER);
    if (rc) {
        console_printf("Cannot read ACCEL sensor %s\n", ADXL362_DEVICE_NAME);
        //return 0;
    }
#endif
    os_gettimeofday(&datacollection.UTC.time, NULL);

    struct custom_value datas;

    //fill the output hex struct
    datas.id = 0xDEADBEEF;
    datas.tx_reason = *reason;
    datas.timestamp = datacollection.UTC.time.tv_sec;
    datas.mV_Bat = datacollection.VDD.board.mV;
    datas.position_type = GPS;
    datas.position.gps.latitude = datacollection.GPS.geolocation.sgd_latitude;
    datas.position.gps.longitude = datacollection.GPS.geolocation.sgd_longitude;

    int lat_int, long_int, lat_dec, long_dec;
    bool lat_sign, long_sign;

    //split float for latitude representation
    split_float(datas.position.gps.latitude, &lat_sign, &lat_int, &lat_dec);
    //split float for longitude representation
    split_float(datas.position.gps.longitude, &long_sign, &long_int, &long_dec);

    //convert the output struct from hex to ascii (must be in UTF-8 as required by server)
    static char str[64]="";
    sprintf(str, 
            "%4x,%1d,%d,%4d,%1d,%s%d.%06d,%s%d.%06d", 
            (unsigned int)datas.id,
            datas.tx_reason,
            (int)datas.timestamp,
            (int)datas.mV_Bat,
            datas.position_type,
            lat_sign?"-":"", lat_int, lat_dec,
            long_sign?"-":"", long_int, long_dec
            );

    console_printf("Trying to tx : %s\n", str);
    console_flush();

    //  Compose a CoAP message with the temperature sensor data and send to the 
    //  CoAP server.  The message will be enqueued for transmission by the OIC 
    //  background task so this function will return without waiting for the message 
    //  to be transmitted.
    rc = send_plain_text_data(str);

    //  SYS_EAGAIN means that the network is still starting up. We send at the next poll.
    if (rc == SYS_EAGAIN){
        console_printf("Network is still starting up\n");    
    }
    
    collect_unlock();
    return;
}
