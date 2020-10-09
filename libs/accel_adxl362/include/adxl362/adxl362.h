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

#ifndef __SENSOR_ADXL362_H__
#define __SENSOR_ADXL362_H__

#include "os/mynewt.h"
#include "sensor/sensor.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ADXL362_DEVICE_NAME     "adxl362_0"
    
enum adxl362_accel_range {
    ADXL362_ACCEL_RANGE_2   = 0, /* +/- 2g  */
    ADXL362_ACCEL_RANGE_4   = 1, /* +/- 4g  */
    ADXL362_ACCEL_RANGE_8   = 2, /* +/- 8g  */
};

enum adxl362_power_mode {
    ADXL362_POWER_STANDBY  = 0,
    ADXL362_POWER_MEASURE  = 2
};

enum adxl362_sample_rate {
    ADXL362_ODR_12_5_HZ   =  0, /* 12.5 Hz */
    ADXL362_ODR_25_HZ     =  1, /* 25 Hz */
    ADXL362_ODR_50_HZ     =  2, /* 50 Hz */
    ADXL362_ODR_100_HZ    =  3, /* 100 Hz */
    ADXL362_ODR_200_HZ    =  4, /* 200 Hz */
    ADXL362_ODR_400_HZ    =  5 /* 400 Hz */
};

    
struct adxl362_cfg {
    enum adxl362_power_mode power_mode;
    uint8_t low_power_enable;
    
    enum adxl362_accel_range accel_range;
    enum adxl362_sample_rate sample_rate; 

    uint16_t active_threshold;
    uint16_t inactive_threshold;

    sensor_type_t mask;
};

/* Device private data */
struct adxl362_private_driver_data {
    struct sensor_notify_ev_ctx notify_ctx;
    struct sensor_read_ev_ctx read_ctx;
    uint8_t registered_mask;

    uint8_t int_num;
    uint8_t int_route;
    uint8_t int_enable;
};

    
struct adxl362 {
    struct os_dev dev;
    struct sensor sensor;
    struct adxl362_cfg cfg;

    struct adxl362_private_driver_data pdd;  
};

int adxl362_default_cfg(struct adxl362_cfg *cfg);

int adxl362_set_power_mode(struct sensor_itf *itf, enum adxl362_power_mode state);
int adxl362_get_power_mode(struct sensor_itf *itf, enum adxl362_power_mode *state);

int adxl362_set_low_power_enable(struct sensor_itf *itf, uint8_t enable);
int adxl362_get_low_power_enable(struct sensor_itf *itf, uint8_t *enable);

int adxl362_set_accel_range(struct sensor_itf *itf,
                            enum adxl362_accel_range range);
int adxl362_get_accel_range(struct sensor_itf *itf,
                            enum adxl362_accel_range *range);

int adxl362_set_active_settings(struct sensor_itf *itf, bool enables, bool refMode,  
                                uint16_t threshold, uint16_t time);
int adxl362_get_active_settings(struct sensor_itf *itf, bool *enables, bool *refMode, 
                                uint16_t *threshold, uint16_t *time);

int adxl362_set_inactive_settings(struct sensor_itf *itf, bool enables, bool refMode,  
                                    uint16_t threshold, uint16_t time);
int adxl362_get_inactive_settings(struct sensor_itf *itf, bool *enables, bool *refMode, 
                                uint16_t *threshold, uint16_t *time);

int adxl362_set_sample_rate(struct sensor_itf *itf, enum adxl362_sample_rate rate);
int adxl362_get_sample_rate(struct sensor_itf *itf, enum adxl362_sample_rate *rate);

int adxl362_setup_interrupts(struct sensor_itf *itf, uint8_t enables, uint8_t mapping);
int adxl362_clear_interrupts(struct sensor_itf *itf, uint8_t *int_status); 

int adxl362_init(struct os_dev *, void *);
int adxl362_config(struct adxl362 *, struct adxl362_cfg *);

int adxl362_get_accel_data(struct sensor_itf *itf, struct sensor_accel_data *sad);

#if MYNEWT_VAL(ADXL362_CLI)
int adxl362_shell_init(void);
#endif

    
#ifdef __cplusplus
}
#endif

#endif /* __SENSOR_ADXL362_H__ */
