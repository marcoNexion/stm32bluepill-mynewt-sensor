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

//  Driver for STM32 Internal vdd voltage measurement. Have a look on the datasheet to know on which channel vdd is connected
//  This sensor is selected if VDD_BOARD=1 in syscfg.yml.



#ifndef __VDD_BOARD_H__
#define __VDD_BOARD_H__

#include "os/mynewt.h"
#include "sensor/sensor.h"

//  Define Sensor Type, Sensor Value Type and Sensor Key
#include "custom_sensor/custom_sensor.h"                       
#define VOLTAGE_SENSOR_TYPE       		SENSOR_TYPE_VOLTAGE  		//  Set to raw sensor type
#define VOLTAGE_SENSOR_VALUE_TYPE		SENSOR_VALUE_TYPE_OPAQUE	//	SENSOR_VALUE_TYPE_INT32         //  Return integer sensor values
#define VOLTAGE_SENSOR_KEY        		"mV"                        //  If necessary Use key (field name)

#ifdef __cplusplus
extern "C" {
#endif

struct adc_dev;  //  ADC device

//  Configuration for the STM32 Internal vdd voltage measurement.
struct vdd_board_cfg {
    sensor_type_t bc_s_mask;   //  Sensor data types that will be returned, i.e.voltage
    const char *adc_dev_name;  //  Name of the ADC device that will be opened to access the sensor. For STM32F1: "adc1"
    uint8_t adc_channel;       //  ADC channel that will be configured to access the sensor. For STM32F1: 16
    void *adc_open_arg;        //  Argument that will be passed to os_dev_open() when opening ADC device.
    void *adc_channel_cfg;     //  Argument that will be passed to adc_chan_config() when configuring ADC channel.
};

//  Device for the STM32 Internal vdd voltage measurement.
struct vdd_board {
    struct os_dev dev;     //  Mynewt device
    struct sensor sensor;  //  Mynewt sensor
    struct vdd_board_cfg cfg;  //  Sensor configuration
    os_time_t last_read_time;   //  Last time the sensor was read.
    struct adc_dev *adc;        //  ADC device that will be used to access the sensor.
};

/**
 * Create the internal voltage sensor instance.  Implemented in creator.c, function DEVICE_CREATE().
 */
void vdd_board_create(void);

/**
 * Return the default configuration for STM32 Internal vdd voltage measurement.
 *
 * @param cfg  Pointer to the vdd_board_cfg device config
 *
 * @return 0 on success, and non-zero error code on failure
 */
int vdd_board_default_cfg(struct vdd_board_cfg *cfg);

/**
 * Initialize the STM32 Internal vdd voltage measurement.
 *
 * @param dev  Pointer to the vdd_board_dev device descriptor
 *
 * @return 0 on success, and non-zero error code on failure
 */
int vdd_board_init(struct os_dev *dev, void *arg);

/**
 * Configure STM32 Internal vdd voltage measurement
 *
 * @param Sensor device vdd_board structure
 * @param Sensor device vdd_board_cfg config
 *
 * @return 0 on success, and non-zero error code on failure
 */
int vdd_board_config(struct vdd_board *vdd_board, struct vdd_board_cfg *cfg);

/**
 * Get raw voltage from STM32 Internal vdd reading from ADC. Will block until data is available.
 *
 * @param dev The vdd_board device
 * @param num_readings How many readings to take
 * @param vdd_sum Pointer to an int. Will store the sum of the readings. Each reading ranges from 0 to 4095.
 * @param vdd_diff An array of (num_readings / 2) uint8_t. If non-null, will store the array of voltage differences between each reading and the last one.  Each byte in the array consists of two difference values, 4 bits each.
 *
 * @return 0 on success, and non-zero error code on failure
 */
int vdd_board_get_raw_voltage(struct vdd_board *dev, int num_readings, int *voltage_sum, uint8_t *voltage_diff);

#ifdef __cplusplus
}
#endif

#endif /* __vdd_board_H__ */
