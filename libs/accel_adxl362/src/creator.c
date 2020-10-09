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

//  Create device
#include <os/mynewt.h>
#include <console/console.h>
#include <sensor/sensor.h>
#include "adxl362/adxl362.h"

//  Define the device specifics here so the device creation code below can be generic.
#define DEVICE_NAME        ADXL362_DEVICE_NAME    //  Name of device e.g "adxl362_0"
#define DEVICE_DEV         adxl362              //  Device type
#define DEVICE_INSTANCE    adxl362              //  Device instance
#define DEVICE_CFG         adxl362_cfg          //  Device config
#define DEVICE_CFG_DEFAULT adxl362_default_cfg  //  Device default config
#define DEVICE_CFG_FUNC    adxl362_config       //  Device config function
#define DEVICE_INIT        adxl362_init         //  Device init function
#define DEVICE_CREATE      adxl362_create       //  Device create function
#define DEVICE_ITF         spi_1_itf_adxl       //  Device interface

static struct DEVICE_DEV DEVICE_INSTANCE;  //  Global instance of the device

static struct sensor_itf spi_1_itf_adxl = {
    .si_type = SENSOR_ITF_SPI,
    .si_num  = 0,
    .si_cs_pin = MYNEWT_VAL(ADXL632_CS_PIN),
    .si_ints = {
       { MYNEWT_VAL(ADXL362_INT_PIN_HOST), MYNEWT_VAL(ADXL362_INT_PIN_DEVICE),
         MYNEWT_VAL(ADXL362_INT_CFG_ACTIVE)}}
};

///////////////////////////////////////////////////////////////////////////////
//  Generic Device Creator Code based on repos\apache-mynewt-core\hw\sensor\creator\src\sensor_creator.c
const char *_accel = "ACCEL. ";  //  Trailer for console output

static int config_adxl362_sensor(void);

//  Create the device instance and configure it.  Called by sysinit() during startup, defined in pkg.yml.
void DEVICE_CREATE(void) {
    int rc;
    
    console_printf("%s create " DEVICE_NAME "\n", _accel);

    //  Create the device.
    rc = os_dev_create((struct os_dev *) &adxl362, DEVICE_NAME,
      OS_DEV_INIT_PRIMARY, 0, adxl362_init, (void *)&spi_1_itf_adxl);
    assert(rc == 0);

    rc = config_adxl362_sensor();
    assert(rc == 0);
}

//  Device configuration
static int config_adxl362_sensor(void) {
    int rc;
    struct os_dev *dev;
    struct DEVICE_CFG cfg;

    //  Fetch the device.
    dev = (struct os_dev *) os_dev_open(DEVICE_NAME, OS_TIMEOUT_NEVER, NULL);
    assert(dev != NULL);

    //  Get the default config for the device.
    rc = DEVICE_CFG_DEFAULT(&cfg);
    assert(rc == 0);

    cfg.power_mode = ADXL362_POWER_MEASURE;
    cfg.low_power_enable = 1;
    cfg.accel_range = ADXL362_ACCEL_RANGE_4;
    cfg.sample_rate = ADXL362_ODR_12_5_HZ;

    cfg.mask = SENSOR_TYPE_ACCELEROMETER;

    //  Copy the default config into the device.
    rc = DEVICE_CFG_FUNC((struct DEVICE_DEV *)dev, &cfg);
    
    // keep device opened
    //os_dev_close(dev);
    return rc;
}






