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
//  Quectel L70-R GPS Driver for Apache Mynewt
//  More about Mynewt Drivers: https://mynewt.apache.org/latest/os/modules/drivers/driver.html
#ifndef __GPS_NEO6M_DRIVER_H__
#define __GPS_NEO6M_DRIVER_H__

#include <os/mynewt.h>
#include <uart/uart.h>
#include <sensor/sensor.h>

#ifdef __cplusplus
extern "C" {  //  Expose the types and functions below to C functions.
#endif

#define GPS_NEO6M_DEVICE "gps_neo6m"  //  Name of the device

//  Use static buffers to avoid dynamic memory allocation (new, delete)
#define GPS_NEO6M_TX_BUFFER_SIZE      32  
#define GPS_NEO6M_RX_BUFFER_SIZE      512
#define GPS_NEO6M_PARSER_BUFFER_SIZE  512


//  GPS Configuration
struct gps_neo6m_cfg {
    sensor_type_t bc_s_mask;   //  Sensor data types that will be returned, i.e. Geolocation only
    struct uart_dev uart;                  //  UART port: 0 for UART2, 1 for UART0, 2 for UART3
};

//  GPS Device Instance for Mynewt
struct gps_neo6m {
    struct os_dev dev;     //  Mynewt device
    struct sensor sensor;  //  Mynewt sensor
    struct gps_neo6m_cfg cfg;  //  Sensor configuration    
    int last_error;           //  Last error encountered
};

//  Start the GPS driver. Return 0 if successful.
int gps_neo6m_start(void);

//  Create the device instance and configure it.  Called by sysinit() during startup, defined in pkg.yml.
//  Implemented in creator.c as function DEVICE_CREATE().
void gps_neo6m_create(void);

//  Copy the default GPS_NEO6M config into cfg.  Returns 0.
int gps_neo6m_default_cfg(struct gps_neo6m_cfg *cfg);

//  Configure the GPS driver.  Called by os_dev_create().  Return 0 if successful.
int gps_neo6m_init(struct os_dev *dev0, void *arg);
    
//  Copy the GPS_NEO6M driver configuration from cfg into drv.  Return 0 if successful.
int gps_neo6m_config(struct gps_neo6m *drv, struct gps_neo6m_cfg *cfg);  

//  Connect to the GPS module.  Return 0 if successful.
int gps_neo6m_connect(struct gps_neo6m *dev);

//  Read position by polling
int gps_neo6m_poll_position(struct gps_neo6m *dev);

//  Internal Sensor Functions

//  Configure the GPS driver as a Mynewt Sensor.  Return 0 if successful.
int gps_neo6m_sensor_init(struct gps_neo6m *dev, void *arg);

//  Copy the default sensor config into cfg.  Returns 0.
int gps_neo6m_sensor_default_cfg(struct gps_neo6m_cfg *cfg);

//  Configure the GPS sensor.  Return 0 if successful.
int gps_neo6m_sensor_config(struct gps_neo6m *dev, struct gps_neo6m_cfg *cfg);

#ifdef __cplusplus
}
#endif

#endif /* __GPS_NEO6M_DRIVER_H__ */