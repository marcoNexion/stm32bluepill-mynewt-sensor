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
#include <string.h>
#include "os/mynewt.h"
#include "console/console.h"
#include "sensor/sensor.h"
#include "sensor/temperature.h"
#include "vdd_board/vdd_board.h"

//  Exports for the sensor API
static int vdd_board_sensor_read(struct sensor *, sensor_type_t, sensor_data_func_t, void *, uint32_t);
static int vdd_board_sensor_get_config(struct sensor *, sensor_type_t, struct sensor_cfg *);

//  Global instance of the sensor driver
static const struct sensor_driver g_vdd_board_sensor_driver = {
    vdd_board_sensor_read,
    vdd_board_sensor_get_config
};

#if defined(STM32L4R5xx)
#include "stm32l4xx_hal_dma.h"
#include "stm32l4xx_hal_adc.h"
#include "adc_stm32l4/adc_stm32l4.h"

//  Config for the temperature channel on ADC1.
static ADC_ChannelConfTypeDef vbat_channel_config = {
    .Channel      = ADC_CHANNEL_VREFINT,      //  Channel number of temperature sensor on ADC1.
    .Rank         = ADC_REGULAR_RANK_1,          //  Every ADC1 channel should be assigned a rank to indicate which channel gets converted first.  Rank 1 is the first to be converted.
    .SamplingTime = ADC_SAMPLETIME_640CYCLES_5,  //  Sampling time 640 ADC clock cycles.
};

int vdd_board_default_cfg(struct vdd_board_cfg *cfg) {
    //  Return the default sensor configuration.
    memset(cfg, 0, sizeof(struct vdd_board_cfg));  //  Zero the entire object.
    cfg->bc_s_mask       = SENSOR_TYPE_ALL;         //  Return all sensor values, i.e. temperature.
    cfg->adc_dev_name    = STM32L4_ADC1_DEVICE;     //  For STM32L4: adc1
    cfg->adc_channel     = MYNEWT_ADC_CHANNEL_VREFINT;
    cfg->adc_open_arg    = NULL;
    cfg->adc_channel_cfg = &vbat_channel_config;    //  Configure the temperature channel.
    return 0;
}
#endif  //(STM32L4R5xx)


static int vdd_board_open(struct os_dev *dev0, uint32_t timeout, void *arg) {
    //  Setup ADC channel configuration for temperature sensor.  Return 0 if successful.
    //  This locks the ADC channel until the sensor is closed.
    int rc = -1;
    struct vdd_board *dev;    
    struct vdd_board_cfg *cfg;
    dev = (struct vdd_board *) dev0;  assert(dev);  
    cfg = &dev->cfg; assert(cfg); /*assert(cfg->adc_channel);*/  assert(cfg->adc_channel_cfg);  assert(cfg->adc_dev_name);

    //  Open port ADC1.
    dev->adc = (struct adc_dev *) os_dev_open(cfg->adc_dev_name, timeout, cfg->adc_open_arg);
    assert(dev->adc);
    if (!dev->adc) { goto err; }
    console_printf("ADC open\n");  ////

    //  Configure port ADC1 channel 16 for temperature sensor.
    rc = adc_chan_config(dev->adc, cfg->adc_channel, cfg->adc_channel_cfg);
    if (rc) { 
        if (dev->adc) { os_dev_close((struct os_dev *) dev->adc); }
        goto err; 
    }
    return 0;
err:
    assert(rc == 0);
    return rc;
}

static int vdd_board_close(struct os_dev *dev0) {
    //  Close the sensor.  This unlocks the ADC channel.  Return 0 if successful.
    //  console_printf("ADC close\n");  ////
    struct vdd_board *dev;    
    dev = (struct vdd_board *) dev0;
    if (dev->adc) {
        //  Close port ADC1.
        os_dev_close((struct os_dev *) dev->adc);
        dev->adc = NULL;
    }
    return 0;
}

/**
 * Expects to be called back through os_dev_create().
 *
 * @param The device object associated with vdd_board
 * @param Argument passed to OS device init, unused
 *
 * @return 0 on success, non-zero error on failure.
 */
int vdd_board_init(struct os_dev *dev0, void *arg) {
    struct vdd_board *dev;
    struct sensor *sensor;
    int rc;
    if (!arg || !dev0) { rc = SYS_ENODEV; goto err; }
    dev = (struct vdd_board *) dev0;
    dev->adc = NULL;

    //  Get the default config.
    rc = vdd_board_default_cfg(&dev->cfg);
    if (rc) { goto err; }

    //  Init the sensor.
    sensor = &dev->sensor;
    rc = sensor_init(sensor, dev0);
    if (rc != 0) { goto err; }

    //  Add the driver with all the supported sensor data types.
    rc = sensor_set_driver(sensor, VOLTAGE_SENSOR_TYPE,
        (struct sensor_driver *) &g_vdd_board_sensor_driver);
    if (rc != 0) { goto err; }

    //  Set the interface.
    rc = sensor_set_interface(sensor, arg);
    if (rc) { goto err; }

    //  Register with the Sensor Manager.
    rc = sensor_mgr_register(sensor);
    if (rc != 0) { goto err; }

    //  Set the handlers for opening and closing the device.
    OS_DEV_SETHANDLERS(dev0, vdd_board_open, vdd_board_close);
    return (0);
err:
    return (rc);
}

static int vdd_board_sensor_read(struct sensor *sensor, sensor_type_t type,
    sensor_data_func_t data_func, void *data_arg, uint32_t timeout) {
    //  Read the sensor values depending on the sensor types specified in the sensor config.
	
	struct sensor_voltage_data databuf;
    struct vdd_board *dev;
    int rc = 0, rawvoltage;

    //  We only allow reading of voltage values.
    if (!(type & VOLTAGE_SENSOR_TYPE)) { rc = SYS_EINVAL; goto err; }
    dev = (struct vdd_board *) SENSOR_GET_DEVICE(sensor); assert(dev);
    rawvoltage = -1;
    {   //  Begin ADC Lock: Open and lock port ADC1, configure channel 16.
        rc = vdd_board_open((struct os_dev *) dev, 0, NULL);
        if (rc) { goto err; }

        //  Get a new voltage sample from voltage sensor (channel XXX of port ADC1).
        rc = vdd_board_get_raw_voltage(dev, 1, &rawvoltage, NULL);

        databuf.mV = (1212 * 4095) / rawvoltage;

        vdd_board_close((struct os_dev *) dev);
    }   //  End ADC Lock: Close and unlock port ADC1.
    if (rc) { goto err; }  //  console_printf("rawvoltage: %d\n", rawvoltage);  ////


    if (data_func) {  //  Call the Listener Function to process the sensor data.
        rc = data_func(sensor, data_arg, &databuf.mV, VOLTAGE_SENSOR_TYPE);
        if (rc) { goto err; }
    }
    return 0;
err:
    return rc;
}

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
int vdd_board_get_raw_voltage(struct vdd_board *dev, int num_readings, int *voltage_sum, uint8_t *voltage_diff) {
    //  If adc_read_channel() fails to return a value, check that
    //  ExternalTrigConv is set to ADC_SOFTWARE_START for STM32F1.
    //  Also the STM32 HAL should be called in this sequence:
    //    __HAL_RCC_ADC1_CLK_ENABLE();
    //    HAL_ADC_Init(hadc1);
    //    HAL_ADC_ConfigChannel(hadc1, &temp_config);
    //    HAL_ADC_Start(hadc1);
    //    HAL_ADC_PollForConversion(hadc1, 10 * 1000);
    //    HAL_ADC_Stop(hadc1);
    //  See https://github.com/cnoviello/mastering-stm32/blob/master/nucleo-f446RE/src/ch12/main-ex1.c
    //  and https://os.mbed.com/users/hudakz/code/Internal_Temperature_F103RB/file/f5c604b5eceb/main.cpp/
    
	console_printf("STM read int voltage on vdd\n");  ////
    assert(dev->adc);  assert(voltage_sum);
    int rc = 0, i;
    int rawvoltage;           //  Raw voltage read from the 12-bit ADC, i.e. 0 to 4095
    int lastvoltage = 0;      //  Previous raw voltage
    uint8_t lastdiff = 0;  //  Delta between current raw voltage and previous raw voltage

    //  When called with num_readings = 1:  This function returns a valid raw temperature value (0 to 4095)
    //  When called with num_readings = 64: This function is used to generate 32 noisy bytes as the entropy 
    //    seed for the pseudorandom number generator "hmac_prng": libs/hmac_prng/src/hmac_prng.c

    //  How do we generate random numbers on a simple microcontroller like Blue Pill, without connecting 
    //  to an external sensor to create the noisy seed?  The internal temperature sensor is actually 
    //  sufficient for generating the noisy seed.  But it needs some coaxing to make it sufficiently noisy...
    //
    //  (1) Take 64 (num_readings) integer samples from the internal temperature sensor. Each sample is 12 bits (0 to 4095)
    //
    //  (2) Compute the delta (difference) between successive samples. The deltas are usually very small: 
    //      mostly 0, some +/- 1, +/- 2, occasionally some odd ones like 88.
    //
    //  (3) To prevent the seed from becoming all zeros, keep only the lower 4 bits of each delta.  
    //      Combine 64 deltas of 4 bits each, and we get the 32-byte seed.  This is written into temp_diff.

    for (i = 0; i < num_readings; i++) {  //  For each sample to be read...
        //  Read the ADC value: rawvoltage will be in the range 0 to 4095.
        rawvoltage = -1;
        //  Block until the voltage is read from the ADC channel.
        rc = adc_read_channel(dev->adc, 0, &rawvoltage);  //  Channel number is not used
        assert(rc == 0);
        if (rc) { goto err; }
        assert(rawvoltage > 0);  //  If equals 0, it means we haven't sampled any values.  Check the above note.

        //  Populate the voltage_diff array with the deltas.
        uint8_t diff = (rawvoltage - lastvoltage) & 0xf;  //  Delta between this and last reading, keeping lower 4 bits.
        if (i % 2 == 1) {
            uint8_t i2 = i >> 1;  //  i2 is (i / 2)
            uint8_t b = diff + (lastdiff << 4);    //  Combine current delta (4 bits) and previous delta (4 bits) to make 8 bits.
            if (voltage_diff) { voltage_diff[i2] = b; }  //  Save the combined delta into voltage_diff as entropy.
        }
        *voltage_sum += rawvoltage;  //  Accumulate the raw voltage.
        lastvoltage = rawvoltage;    //  Remember the previous raw voltage and previous delta.
        lastdiff = diff;
    }
    return 0;
err:
    return rc;
}

static int vdd_board_sensor_get_config(struct sensor *sensor, sensor_type_t type,
    struct sensor_cfg *cfg) {
    //  Return the type of the sensor value returned by the sensor.
    int rc;
    if (!(type & VOLTAGE_SENSOR_TYPE)) {
        rc = SYS_EINVAL;
        goto err;
    }
    cfg->sc_valtype = VOLTAGE_SENSOR_VALUE_TYPE;  //  We return float (computed values) or int (raw values).
    return (0);
err:
    return (rc);
}

/**
 * Configure STM32 internal temperature sensor
 *
 * @param Sensor device vdd_board structure
 * @param Sensor device vdd_board_cfg config
 *
 * @return 0 on success, and non-zero error code on failure
 */
int vdd_board_config(struct vdd_board *dev, struct vdd_board_cfg *cfg) {
    struct sensor_itf *itf;
    int rc;
    itf = SENSOR_GET_ITF(&(dev->sensor)); assert(itf);
    rc = sensor_set_type_mask(&(dev->sensor),  cfg->bc_s_mask);
    if (rc) { goto err; }

    dev->cfg.bc_s_mask = cfg->bc_s_mask;
    return 0;
err:
    return (rc);
}
