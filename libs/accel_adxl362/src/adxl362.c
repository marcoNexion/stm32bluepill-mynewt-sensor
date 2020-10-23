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
#include <errno.h>
#include <assert.h>
#include <math.h>

#include "os/mynewt.h"
#include <console/console.h>
#include "hal/hal_spi.h"
#include "hal/hal_gpio.h"
#include "sensor/sensor.h"
#include "sensor/accel.h"
#include "adxl362/adxl362.h"
#include "adxl362/ADXL362Reg.h"
#include "adxl362_priv.h"
#include "modlog/modlog.h"
#include "stats/stats.h"
#include <syscfg/syscfg.h>

#if MYNEWT_VAL(ADXL_362_LPMODE)
#include "lowpower_mgnt/lowpower_mgnt.h"
LP_ID_t LPMgr_id = LP_UNITIALIZED_ID;
#endif


static struct hal_spi_settings spi_adxl362_settings = {
    .data_order = HAL_SPI_MSB_FIRST,
    .data_mode  = HAL_SPI_MODE0,
    .baudrate   = 4000,
    .word_size  = HAL_SPI_WORD_SIZE_8BIT,
};



///////////////////////////////////////////////////////////////////////////////
//  Logging Functions: Put common strings here to reduce space.


#define ADXL362_NOTIFY_MASK  0x01
#define ADXL362_READ_MASK    0x02

static int init_spi(struct adxl362 *adxl, struct hal_spi_settings *spi_settings);
static int deinit_spi(struct adxl362 *adxl);

#if MYNEWT_VAL(ADXL_362_LPMODE)
// Callback from low power manager about change of mode
//deal with : LP_RUN, LP_DOZE, LP_SLEEP, LP_DEEPSLEEP, LP_OFF
static void adxl362_lp_change(  LP_MODE_t prevmode, 
                                LP_MODE_t newmode,
                                void * arg) {

    struct adxl362 *adxl;
    adxl = arg;                                
        
    if (prevmode>=LP_DEEPSLEEP && newmode <LP_DEEPSLEEP) {
       // TURN ON : wake up & init its periphs

                      
    } else if (prevmode<LP_DEEPSLEEP && newmode >= LP_DEEPSLEEP) {
        // TURN OFF : go to sleep & deinit its periphs
        if (adxl->sensor.s_itf.si_type == SENSOR_ITF_SPI){
            deinit_spi(adxl);
        }
    }
}
#endif

#if MYNEWT_VAL(ADXL362_INT_ENABLE)
static void interrupt_handler(void * arg);
static int init_intpin(struct adxl362 * adxl362, hal_gpio_irq_handler_t handler,
                       void * arg);
static int enable_interrupt(struct sensor * sensor, uint8_t ints_to_enable);
static int disable_interrupt(struct sensor * sensor, uint8_t ints_to_disable);
#endif

/* Exports for the sensor API */
static int adxl362_sensor_read(struct sensor *, sensor_type_t,
                               sensor_data_func_t, void *, uint32_t);
static int adxl362_sensor_get_config(struct sensor *, sensor_type_t,
                                     struct sensor_cfg *);
static int adxl362_sensor_set_trigger_thresh(struct sensor * sensor,
                                             sensor_type_t sensor_type,
                                             struct sensor_type_traits * stt);
static int adxl362_sensor_set_notification(struct sensor * sensor,
                                           sensor_event_type_t sensor_event_type);
static int adxl362_sensor_unset_notification(struct sensor * sensor,
                                             sensor_event_type_t sensor_event_type);
static int adxl362_sensor_handle_interrupt(struct sensor * sensor);
static int adxl362_sensor_clear_low_thresh(struct sensor *sensor,
                                           sensor_type_t type);
static int adxl362_sensor_clear_high_thresh(struct sensor *sensor,
                                            sensor_type_t type);


static const struct sensor_driver adxl362_sensor_driver = {
     .sd_read                      = adxl362_sensor_read,
     .sd_get_config                = adxl362_sensor_get_config,
     .sd_set_trigger_thresh        = adxl362_sensor_set_trigger_thresh,
     .sd_set_notification          = adxl362_sensor_set_notification,
     .sd_unset_notification        = adxl362_sensor_unset_notification,
     .sd_handle_interrupt          = adxl362_sensor_handle_interrupt,
     .sd_clear_low_trigger_thresh  = adxl362_sensor_clear_low_thresh,
     .sd_clear_high_trigger_thresh = adxl362_sensor_clear_high_thresh
};

int adxl362_default_cfg(struct adxl362_cfg *cfg) {
    //  Return the default sensor configuration.
    memset(cfg, 0, sizeof(struct adxl362_cfg));  //  Zero the entire object.
    cfg->mask       = SENSOR_TYPE_ALL;      //  Return all sensor values, i.e. temperature.

    return 0;
}


/**
 * Writes a single byte to the specified register using SPI
 *
 * @param The sensor interface
 * @param The register address to write to
 * @param The value to write
 *
 * @return 0 on success, non-zero error on failure.
 */
int
adxl362_spi_write8(struct sensor_itf *itf, uint8_t reg, uint8_t value)
{
    int rc;

    /* Select the device */
    hal_gpio_write(itf->si_cs_pin, 0);

    uint8_t tx_buffer[3];
    tx_buffer[0] = ADXL362_WRITE_REG;
    tx_buffer[1] = reg;
    tx_buffer[2] = value;

    rc = hal_spi_txrx(itf->si_num, tx_buffer, NULL, sizeof(tx_buffer));
    if (rc == 0xFFFF) {
        rc = SYS_EINVAL;
        console_printf("SPI_%u write failed addr:0x%02X\n", itf->si_num, reg);
    }

    /* De-select the device */
    hal_gpio_write(itf->si_cs_pin, 1);

    return rc;
}


/**
 * Reads a single byte from the specified register using SPI
 *

 * @param The register address to read from
 * @param Pointer to where the register value should be written
 *
 * @return 0 on success, non-zero error on failure.
 */
int
adxl362_spi_read8(struct sensor_itf *itf, uint8_t reg, uint8_t *value)
{
    int rc = 0;

    /* Select the device */
    hal_gpio_write(itf->si_cs_pin, 0);

    uint8_t tx_buffer[3];
    tx_buffer[0] = ADXL362_READ_REG;
    tx_buffer[1] = reg;
    tx_buffer[2] = 0;

    rc = hal_spi_txrx(itf->si_num, tx_buffer, tx_buffer, sizeof(tx_buffer));

    if (rc == 0xFFFF) {
        rc = SYS_EINVAL;
        console_printf("SPI_%u write failed addr:0x%02X\n", itf->si_num, reg);
        goto err;
    }

    *value=tx_buffer[2];
        
err:
    /* De-select the device */
    hal_gpio_write(itf->si_cs_pin, 1);

    return rc;
}

/**
 * Read multiple bytes starting from specified register over SPI
 *
 * @param The sensor interface
 * @param The register address start reading from
 * @param Pointer to where the register value should be written
 * @param Number of bytes to read
 *
 * @return 0 on success, non-zero on failure
 */
int
adxl362_spi_readlen(struct sensor_itf *itf, uint8_t reg, uint8_t *buffer,
                    uint8_t len)
{
    uint8_t tx_buffer[8];
    int rc = 0;

    if(len>(sizeof(tx_buffer)-2)){
        /* len is too large */
        return -1;
    }

    /* Select the device */
    hal_gpio_write(itf->si_cs_pin, 0);
    
    tx_buffer[0] = ADXL362_READ_REG;
    tx_buffer[1] = reg;
    memset(&tx_buffer[2], 0x00, sizeof(tx_buffer)-2);

    rc = hal_spi_txrx(itf->si_num, tx_buffer, tx_buffer, sizeof(tx_buffer));

    if (rc == 0xFFFF) {
        rc = SYS_EINVAL;
        console_printf("SPI_%u readlen failed addr:0x%02X\n", itf->si_num, reg);
        goto err;
    }

    /* copy the retruned buffer */
    memcpy(buffer, &tx_buffer[2], len);

err:
    /* De-select the device */
    hal_gpio_write(itf->si_cs_pin, 1);

    return rc;
}


/**
 * Write byte to ADXL362 sensor
 *
 * @param The sensor interface
 * @param The register address to write to
 * @param The value to write
 *
 * @return 0 on success, non-zero on failure
 */
int
adxl362_write8(struct sensor_itf *itf, uint8_t reg, uint8_t value)
{
    int rc;

    rc = sensor_itf_lock(itf, MYNEWT_VAL(ADXL362_ITF_LOCK_TMO));
    if (rc) {
        return rc;
    }

    rc = adxl362_spi_write8(itf, reg, value);

    sensor_itf_unlock(itf);

    return rc;
}

/**
 * Read byte data from ADXL362 sensor
 *
 * @param The sensor interface
 * @param The register address to read from
 * @param Pointer to where the register value should be written
 *
 * @return 0 on success, non-zero on failure
 */
int
adxl362_read8(struct sensor_itf *itf, uint8_t reg, uint8_t *value)
{
    int rc;

    rc = sensor_itf_lock(itf, MYNEWT_VAL(ADXL362_ITF_LOCK_TMO));
    if (rc) {
        return rc;
    }
    
    rc = adxl362_spi_read8(itf, reg, value);

    sensor_itf_unlock(itf);

    return rc;
}

/**
 * Read multiple bytes starting from specified register
 *
 * @param The sensor interface
 * @param The register address start reading from
 * @param Pointer to where the register value should be written
 * @param Number of bytes to read
 *
 * @return 0 on success, non-zero on failure
 */
int
adxl362_readlen(struct sensor_itf *itf, uint8_t reg, uint8_t *buffer,
                uint8_t len)
{
    int rc;

    rc = sensor_itf_lock(itf, MYNEWT_VAL(ADXL362_ITF_LOCK_TMO));
    if (rc) {
        return rc;
    }

    rc = adxl362_spi_readlen(itf, reg, buffer, len);

    sensor_itf_unlock(itf);

    return rc;
}


static float adxl362_convert_reg_to_ms2(int16_t val, float lsb_mg)
{
    float tmp = val * lsb_mg * 0.001; /* convert to g */
    return tmp * STANDARD_ACCEL_GRAVITY; /* convert to ms2 */
}

static uint8_t adxl362_convert_ms2_to_reg(float ms2, float lsb_mg)
{
    float tmp = (ms2 * 1000) / STANDARD_ACCEL_GRAVITY; /* convert to mg */
    tmp /= lsb_mg; /* convert to reg format */   
    return (uint8_t) tmp;
}

/*
static uint16_t adxl362_convert_regtimer_to_ms(uint16_t time, uint8_t rate_regval)
{
    uint16_t sampleRate;
    
    switch(rate_regval){
        case ADXL362_ODR_12_5_HZ:
                                sampleRate = 12;
                                break;            
        case ADXL362_ODR_25_HZ:
                                sampleRate = 25;
                                break;
        case ADXL362_ODR_50_HZ:
                                sampleRate = 50;
                                break;
        default:
        case ADXL362_ODR_100_HZ:
                                sampleRate = 100;
                                break;
        case ADXL362_ODR_200_HZ:
                                sampleRate = 200;
                                break;
        case ADXL362_ODR_400_HZ:
                                sampleRate = 400;
                                break;
    }

    return ((time/sampleRate)*1000);
}
*/

static uint16_t adxl362_convert_ms_to_regtimer(uint16_t time, uint8_t rate_regval)
{
    uint16_t sampleRate;
    
    switch(rate_regval){
        case ADXL362_ODR_12_5_HZ:
                                sampleRate = 12;
                                break;            
        case ADXL362_ODR_25_HZ:
                                sampleRate = 25;
                                break;
        case ADXL362_ODR_50_HZ:
                                sampleRate = 50;
                                break;
        default:
        case ADXL362_ODR_100_HZ:
                                sampleRate = 100;
                                break;
        case ADXL362_ODR_200_HZ:
                                sampleRate = 200;
                                break;
        case ADXL362_ODR_400_HZ:
                                sampleRate = 400;
                                break;
    }

    return ((time/1000)*sampleRate);
}


/**
 * Sets ADXL362 into new power mode
 *
 * @param The sensor interface
 * @param Power mode to set
 *
 * @return 0 on success, non-zero error on failure.
 */
int
adxl362_set_power_mode(struct sensor_itf *itf, enum adxl362_power_mode state)
{
    int rc;
    unsigned char oldPowerCtl = 0;
    unsigned char newPowerCtl = 0;

    rc = adxl362_read8(itf, ADXL362_REG_POWER_CTL, &oldPowerCtl);

    if(rc){ 
        return rc;
    }
    newPowerCtl = oldPowerCtl & ~ADXL362_POWER_CTL_MEASURE(0x3);
    newPowerCtl = newPowerCtl | ADXL362_POWER_CTL_MEASURE(state);

    return adxl362_write8(itf, ADXL362_REG_POWER_CTL, newPowerCtl);
}

/**
 * Gets power mode ADXL362 is currently in
 *
 * @param The sensor interface
 * @param Pointer to store current power mode
 *
 * @return 0 on success, non-zero error on failure.
 */
int
adxl362_get_power_mode(struct sensor_itf *itf, enum adxl362_power_mode *state)
{
    uint8_t reg;
    int rc;

    rc = adxl362_read8(itf, ADXL362_REG_POWER_CTL, &reg);
    if (rc) {
        return rc;
    }

    *state = (enum adxl362_accel_range)ADXL362_POWER_CTL_MEASURE(reg);

    return 0;
}

/**
 * Sets whether low power mode is enabled. Low power mode acheives lower
 * power consumption at the expense of higher noise, but is only effective
 * at sample rates between 12.5Hz and 400Hz
 *
 * @param The sensor interface
 * @param Setting to use for low power mode
 *
 * @return 0 on success, non-zero error on failure.
 */
int
adxl362_set_low_power_enable(struct sensor_itf *itf, uint8_t enable)
{
    /* setup the device in wake-up mode */
    int rc;
    unsigned char oldPowerCtl = 0;
    unsigned char newPowerCtl = 0;

    rc = adxl362_read8(itf, ADXL362_REG_POWER_CTL, &oldPowerCtl);

    if(rc){ 
        return rc;
    }

    if(enable)
        newPowerCtl = oldPowerCtl | ADXL362_POWER_CTL_WAKEUP;
    else
        newPowerCtl = oldPowerCtl & ~ADXL362_POWER_CTL_WAKEUP;

    return adxl362_write8(itf, ADXL362_REG_POWER_CTL, newPowerCtl);
}

/**
 * Gets current setting of low power mode. Low power mode acheives lower
 * power consumption at the expense of higher noise, but is only effective
 * at sample rates between 12.5Hz and 400Hz
 *
 * @param The sensor interface
 * @param Pointer to store value
 *
 * @return 0 on success, non-zero error on failure.
 */
int
adxl362_get_low_power_enable(struct sensor_itf *itf, uint8_t *enable)
{
    uint8_t reg;
    int rc;

    rc = adxl362_read8(itf, ADXL362_REG_POWER_CTL, &reg);
    if (rc) {
        return rc;
    }

    *enable = ((reg & ADXL362_POWER_CTL_WAKEUP) != 0);

    return 0;
}

/**
 * Sets the accelerometer range to specified setting
 *
 * @param The sensor interface
 * @param gRange - Range option.
 *                  Example: ADXL362_RANGE_2G  -  +-2 g
 *                           ADXL362_RANGE_4G  -  +-4 g
 *                           ADXL362_RANGE_8G  -  +-8 g
 *
 * @return 0 on success, non-zero error on failure.
 */
int
adxl362_set_accel_range(struct sensor_itf *itf, enum adxl362_accel_range range)
{
    unsigned char oldFilterCtl = 0;
    unsigned char newFilterCtl = 0;

    adxl362_read8(itf, ADXL362_REG_FILTER_CTL, &oldFilterCtl);
    newFilterCtl = oldFilterCtl & ~ADXL362_FILTER_CTL_RANGE(0x3);
    newFilterCtl = newFilterCtl | ADXL362_FILTER_CTL_RANGE(range);
    adxl362_write8(itf, ADXL362_REG_FILTER_CTL, (uint8_t)newFilterCtl);

    return 0;
}

/**
 * Gets the accelerometer range currently set
 *
 * @param The sensor interface
 * @param Pointer to location to store accelerometer range
 *
 * @return 0 on success, non-zero error on failure.
 */
int
adxl362_get_accel_range(struct sensor_itf *itf, enum adxl362_accel_range *range)
{
    adxl362_read8(itf, ADXL362_REG_FILTER_CTL, range);

    *range = (*range & 0xC0)>>6;

    return 0;
}


/**
 * Sets new threshold and time for triggering activity interrupt
 *
 * @param The sensor interface
 * @param enables if triggering is enabled
 * @param refMode : Referenced/Absolute Activity Select.
 *                    Example: 0 - absolute mode.
 *                             1 - referenced mode.
 * @param threshold value
 * @param time value in miliseconds
 *
 * @return 0 on success, non-zero error on failure.
 */
int 
adxl362_set_active_settings(struct sensor_itf *itf, bool enables, bool refMode, 
                                uint16_t threshold, uint16_t time)
{
    uint8_t reg;
    int rc;

    /*read ctrl register*/
    rc = adxl362_read8(itf, ADXL362_REG_ACT_INACT_CTL, &reg);
    if(rc){ 
        return rc;
    }

    /* Set Linked mode */
    reg |= ADXL362_ACT_INACT_CTL_LINKLOOP(0x1);

    if(enables){
        /*set only inact enable bit*/
        reg |= ADXL362_ACT_INACT_CTL_ACT_EN;
    }else{
        /*clear only inact enable bit*/
        reg &= ~ADXL362_ACT_INACT_CTL_ACT_EN;
    }
    if(refMode){
        /*clear only inact enable bit*/
        reg |= ADXL362_ACT_INACT_CTL_ACT_REF;
    }else{
        /*clear only inact enable bit*/
        reg &= ~ADXL362_ACT_INACT_CTL_ACT_REF;
    }

    /* writes ctrl register*/
    rc = adxl362_write8(itf, ADXL362_REG_ACT_INACT_CTL, reg);
    if(rc){ 
        return rc;
    }
    
    /* writes active thresh registers*/
    rc = adxl362_write8(itf, ADXL362_REG_THRESH_ACT_L, (uint8_t)(threshold&0x00FF));
    if(rc){ 
        return rc;
    }
    rc = adxl362_write8(itf, ADXL362_REG_THRESH_ACT_H, (uint8_t)((threshold&0x0700)>>8));
    if(rc){ 
        return rc;
    }

    /* writes active time register*/
    rc = adxl362_write8(itf, ADXL362_REG_TIME_ACT, (uint8_t)time);
    if(rc){ 
        return rc;
    }
    return rc;
}

/**
 * Gets current threshold and time set for triggering activity interrupt
 *
 * @param The sensor interface
 * @param enables if triggering is enabled
 * @param refMode : Referenced/Absolute Activity Select.
 *                    Example: 0 - absolute mode.
 *                             1 - referenced mode.
 * @param threshold value
 * @param time value in miliseconds
 *
 * @return 0 on success, non-zero error on failure.
 */
int
adxl362_get_active_settings(struct sensor_itf *itf, bool *enables, bool *refMode, 
                                uint16_t *threshold, uint16_t *time)
{
    uint8_t thresholdL, thresholdH, reg;
    int rc;
  
    rc = adxl362_read8(itf, ADXL362_REG_THRESH_ACT_L, &thresholdL);
    if(rc){ 
        return rc;
    }
    rc = adxl362_read8(itf, ADXL362_REG_THRESH_ACT_H, &thresholdH);
    if(rc){ 
        return rc;
    }
    *threshold = (((uint16_t)thresholdH<<8)&0xFF00) | (uint16_t)thresholdL;

    rc = adxl362_read8(itf, ADXL362_REG_TIME_ACT, (uint8_t*)time);
    if(rc){ 
        return rc;
    }

    rc = adxl362_read8(itf, ADXL362_REG_ACT_INACT_CTL, &reg);
    if(rc){ 
        return rc;
    }
    *refMode = (reg & ADXL362_ACT_INACT_CTL_ACT_REF) != 0;
    *enables = (reg & ADXL362_ACT_INACT_CTL_ACT_EN) != 0;
    
    return rc;
}


/**
 * Sets new threshold and time for triggering inactivity interrupt
 *
 * @param The sensor interface
 * @param enables if triggering is enabled
 * @param refMode : Referenced/Absolute Activity Select.
 *                    Example: 0 - absolute mode.
 *                             1 - referenced mode.
 * @param threshold value
 * @param time value in miliseconds
 *
 * @return 0 on success, non-zero error on failure.
 */
int 
adxl362_set_inactive_settings(struct sensor_itf *itf, bool enables, bool refMode, 
                                uint16_t threshold, uint16_t time)
{
    uint8_t reg;
    int rc;

    /*read ctrl register*/
    rc = adxl362_read8(itf, ADXL362_REG_ACT_INACT_CTL, &reg);
    if(rc){ 
        return rc;
    }

    /* Set Linked mode */
    reg |= ADXL362_ACT_INACT_CTL_LINKLOOP(0x1);
    
    if(enables){
        /*set only inact enable bit*/
        reg |= ADXL362_ACT_INACT_CTL_INACT_EN;
    }else{
        /*clear only inact enable bit*/
        reg &= ~ADXL362_ACT_INACT_CTL_INACT_EN;
    }
    if(refMode){
        /*clear only inact enable bit*/
        reg |= ADXL362_ACT_INACT_CTL_INACT_REF;
    }else{
        /*clear only inact enable bit*/
        reg &= ~ADXL362_ACT_INACT_CTL_INACT_REF;
    }

    /* writes ctrl register*/
    rc = adxl362_write8(itf, ADXL362_REG_ACT_INACT_CTL, reg);
    if(rc){ 
        return rc;
    }
    
    /* writes inactive thresh registers*/
    rc = adxl362_write8(itf, ADXL362_REG_THRESH_INACT_L, (uint8_t)(threshold&0x00FF));
    if(rc){ 
        return rc;
    }
    rc = adxl362_write8(itf, ADXL362_REG_THRESH_INACT_H, (uint8_t)((threshold&0x0700)>>8));
    if(rc){ 
        return rc;
    }

    /* writes inactive time register*/
    rc = adxl362_write8(itf, ADXL362_REG_TIME_INACT_L, (uint8_t)(time&0x00FF));
    if(rc){ 
        return rc;
    }
    rc = adxl362_write8(itf, ADXL362_REG_TIME_INACT_H, (uint8_t)((time&0xFF00)>>8)); 

    return rc;
}

/**
 * Gets current threshold and time set for triggering inactivity interrupt
 *
 * @param The sensor interface
 * @param Pointer to store threshold value
 * @param Ponter to store time value in miliseconds
 *
 * @return 0 on success, non-zero error on failure.
 */
int
adxl362_get_inactive_settings(struct sensor_itf *itf, bool *enables, bool *refMode, 
                                uint16_t *threshold, uint16_t *time)
{
    uint8_t thresholdL, thresholdH, timeH, timeL, reg;
    int rc;
  
    rc = adxl362_read8(itf, ADXL362_REG_THRESH_INACT_L, &thresholdL);
    if(rc){ 
        return rc;
    }
    rc = adxl362_read8(itf, ADXL362_REG_THRESH_INACT_H, &thresholdH);
    if(rc){ 
        return rc;
    }
    *threshold = (((uint16_t)thresholdH<<8)&0xFF00) | (uint16_t)thresholdL;

    rc = adxl362_read8(itf, ADXL362_REG_TIME_INACT_L, &timeL);
    if(rc){ 
        return rc;
    }
    rc = adxl362_read8(itf, ADXL362_REG_TIME_INACT_H, &timeH);
    if(rc){ 
        return rc;
    }
    *time = (((uint16_t)timeH<<8)&0xFF00) | (uint16_t)timeL;

    rc = adxl362_read8(itf, ADXL362_REG_ACT_INACT_CTL, &reg);
    if(rc){ 
        return rc;
    }
    *refMode = (reg & ADXL362_ACT_INACT_CTL_ACT_REF) != 0;
    *enables = (reg & ADXL362_ACT_INACT_CTL_ACT_EN) != 0;
    
    return rc;
}

/**
 * Sets new sample rate for measurements
 *
 * @param The sensor interface
 * @param outRate - Output Data Rate option.
 *                  Example: ADXL362_ODR_12_5_HZ  -  12.5Hz
 *                           ADXL362_ODR_25_HZ    -  25Hz
 *                           ADXL362_ODR_50_HZ    -  50Hz
 *                           ADXL362_ODR_100_HZ   -  100Hz
 *                           ADXL362_ODR_200_HZ   -  200Hz
 *                           ADXL362_ODR_400_HZ   -  400Hz
 *
 * @return 0 on success, non-zero error on failure.
 */
int
adxl362_set_sample_rate(struct sensor_itf *itf, enum adxl362_sample_rate rate)
{
    int rc;
    unsigned char oldFilterCtl = 0;
    unsigned char newFilterCtl = 0;

    rc = adxl362_read8(itf, ADXL362_REG_FILTER_CTL, &oldFilterCtl);
    if (rc) {
        return rc;
    }
    newFilterCtl = oldFilterCtl & ~ADXL362_FILTER_CTL_ODR(0x7);
    newFilterCtl = newFilterCtl | ADXL362_FILTER_CTL_ODR(rate);

    return adxl362_write8(itf, ADXL362_REG_FILTER_CTL, (uint8_t) newFilterCtl);
}

/**
 * Gets current sample rate settings
 *
 * @param The sensor interface
 * @param Pointer to store rate value
 *
 * @return 0 on success, non-zero error on failure.
 */
int
adxl362_get_sample_rate(struct sensor_itf *itf, enum adxl362_sample_rate *rate)
{
    int rc;

    rc = adxl362_read8(itf, ADXL362_REG_FILTER_CTL, (uint8_t*)rate);
    if (rc) {
        return rc;
    }

    return ADXL362_FILTER_CTL_ODR(*rate);
}

/**
 * Configures which interrupts are enabled and which interrupt pins they are 
 * mapped to.
 *
 * @param The sensor interface
 * @param which interrupts are enabled
 * @param which interrupts are mapped to which pin
 *
 * @return 0 on success, non-zero error on failure
 */
int
adxl362_setup_interrupts(struct sensor_itf *itf, uint8_t enables, uint8_t mapping)
{
    int rc;
    uint8_t reg;

    if(mapping==1)
    {
        rc = adxl362_read8(itf, ADXL362_REG_INTMAP1, &reg);
        if (rc) {
            return rc;
        }
        reg = (reg & (~ADXL362_INTMAP1_INT_LOW)) | enables;

        return adxl362_write8(itf, ADXL362_REG_INTMAP1, reg);
    
    }

    rc = adxl362_read8(itf, ADXL362_REG_INTMAP2, &reg);
    if (rc) {
        return rc;
    }
    reg = (reg & (~ADXL362_INTMAP2_INT_LOW)) | enables;

    return adxl362_write8(itf, ADXL362_REG_INTMAP2, reg);
}

/**
 * Clears interrupts (except DATA_READY, Watermark & Overrun which need data to
 * be read to clear). Provides status as output to identify which interrupts have
 * triggered.
 * 
 * @param The sensor interface
 * @param Pointer to store interrupt status
 *
 * @return 0 on success, non-zero error on failure 
 */
int adxl362_clear_interrupts(struct sensor_itf *itf, uint8_t *int_status)
{
    return adxl362_read8(itf, ADXL362_REG_STATUS, int_status);
}

/**
 * Expects to be called back through os_dev_create().
 *
 * @param The device object associated with this accelerometer
 * @param Argument passed to OS device init, unused
 *
 * @return 0 on success, non-zero error on failure.
 */
int
adxl362_init(struct os_dev *dev, void *arg)
{
    struct adxl362 *adxl;
    struct sensor *sensor;
    int rc;

    if (!arg || !dev) {
        return SYS_ENODEV;
    }

    adxl = (struct adxl362 *) dev;

    adxl->cfg.mask = SENSOR_TYPE_ALL;

    sensor = &adxl->sensor;

    /* Initialise the stats entry */
    rc = stats_init(
        STATS_HDR(g_adxl362stats),
        STATS_SIZE_INIT_PARMS(g_adxl362stats, STATS_SIZE_32),
        STATS_NAME_INIT_PARMS(adxl362_stat_section));
    SYSINIT_PANIC_ASSERT(rc == 0);
    /* Register the entry with the stats registry */
    rc = stats_register(dev->od_name, STATS_HDR(g_adxl362stats));
    SYSINIT_PANIC_ASSERT(rc == 0);

    rc = sensor_init(sensor, dev);
    if (rc) {
        return rc;
    }

    /* Add the accelerometer/gyroscope driver */
    rc = sensor_set_driver(sensor, SENSOR_TYPE_ACCELEROMETER,
                           (struct sensor_driver *) &adxl362_sensor_driver);
    if (rc) {
        return rc;
    }

    rc = sensor_set_interface(sensor, arg);
    if (rc) {
        return rc;
    }

    rc = sensor_mgr_register(sensor);
    if (rc) {
        return rc;
    }

    if (sensor->s_itf.si_type == SENSOR_ITF_SPI) {

        rc = hal_spi_config(sensor->s_itf.si_num, &spi_adxl362_settings);
        if (rc == EINVAL) {
            return rc;
        }

        rc = hal_spi_enable(sensor->s_itf.si_num);
        if (rc) {
            return rc;
        }

        rc = hal_gpio_init_out(sensor->s_itf.si_cs_pin, 1);
        if (rc) {
            return rc;
        }

    }

#if MYNEWT_VAL(ADXL362_INT_ENABLE)
    adxl->pdd.read_ctx.srec_sensor = sensor;
    adxl->pdd.notify_ctx.snec_sensor = sensor;

    rc = init_intpin(adxl, interrupt_handler, sensor);
    if (rc != 0) {
        return rc;
    }

#endif

#if MYNEWT_VAL(ADXL_362_LPMODE)
    // register with lowpowermgr to know when to deinit/init the device
    LPMgr_id = LPMgr_register(adxl362_lp_change, adxl);
    // after init we are ok to go into DEEPSLEEP mode
    LPMgr_setLPMode(LPMgr_id, LP_DEEPSLEEP);

#endif
    
    return 0;
}

/**
 * Configure ADXL362 sensor
 *
 * @param Sensor device ADXL362 structure
 * @param Sensor device ADXL362 config
 *
 * @return 0 on success, non-zero on failure
 */
int
adxl362_config(struct adxl362 *dev, struct adxl362_cfg *cfg)
{
    int rc;
    uint8_t val=0;
    struct sensor_itf *itf;

    itf = SENSOR_GET_ITF(&(dev->sensor));

    /* Write to SOFT RESET */
    adxl362_write8(itf, ADXL362_REG_SOFT_RESET, ADXL362_RESET_KEY);
    /* wait a little before continuing */
    os_time_delay(10 * OS_TICKS_PER_SEC/1000);

    /* Check device id, mst, ad are correct */
    
    rc = adxl362_read8(itf, ADXL362_REG_DEVID_AD, &val);
    if (rc) {
        return rc;
    }
    if (val != ADXL362_DEVICE_AD) {
        return SYS_EINVAL;
    }

    rc = adxl362_read8(itf, ADXL362_REG_DEVID_MST, &val);
    if (rc) {
        return rc;
    }
    if (val != ADXL362_DEVICE_MST) {
        return SYS_EINVAL;
    }
    
    rc = adxl362_read8(itf, ADXL362_REG_PARTID, &val);
    if (rc) {
        return rc;
    }
    if (val != ADXL362_PART_ID) {
        return SYS_EINVAL;
    }

    /* Setup interrupt polarity */
    rc = adxl362_write8(itf, ADXL362_REG_INTMAP1, 
                        dev->sensor.s_itf.si_ints[dev->pdd.int_num].active ? 0x0 : ADXL362_INTMAP1_INT_LOW);
    if (rc) {
        return rc;
    }

    rc = adxl362_write8(itf, ADXL362_REG_INTMAP2, 
                        dev->sensor.s_itf.si_ints[dev->pdd.int_num].active ? 0x0 : ADXL362_INTMAP2_INT_LOW);
    if (rc) {
        return rc;
    }
    
    /* Setup range as per config */
    rc = adxl362_set_accel_range(itf, cfg->accel_range);
    if (rc) {
        return rc;
    }
    dev->cfg.accel_range = cfg->accel_range;

    /* setup sample rate */
    rc = adxl362_set_sample_rate(itf, cfg->sample_rate);
    if (rc) {
        return rc;
    }
    dev->cfg.sample_rate = cfg->sample_rate;

    /* setup activity detection settings */
    rc = adxl362_set_active_settings(itf, 1, 1, cfg->active_threshold, 
                                        adxl362_convert_ms_to_regtimer(cfg->active_time_ms, dev->cfg.sample_rate));
    if (rc) {
        return rc;
    }

    /* setup inactivity detection settings */
    rc = adxl362_set_inactive_settings(itf, 1, 1, cfg->inactive_threshold, 
                                        adxl362_convert_ms_to_regtimer(cfg->inactive_time_ms, dev->cfg.sample_rate));
    if (rc) {
        return rc;
    }
    
    /* setup low power mode */
    rc = adxl362_set_low_power_enable(itf, cfg->low_power_enable);
    if (rc) {
        return rc;
    }
    dev->cfg.low_power_enable = cfg->low_power_enable;
    
    /* setup default interrupt config */
    /* use only INTMAP1 for now */
    /* device's interrupt disabled by default */
    rc = adxl362_setup_interrupts(itf, 0x0, 1);
    if (rc) {
        return rc;
    }

    rc = adxl362_clear_interrupts(itf, &val);
    if (rc) {
        return rc;
    }
    
    /* Setup current power mode */
    rc = adxl362_set_power_mode(itf, cfg->power_mode);
    if (rc) {
        return rc;
    } 
    dev->cfg.power_mode = cfg->power_mode;

    
    rc = sensor_set_type_mask(&(dev->sensor), cfg->mask);
    if (rc) {
        return rc;
    }

    dev->cfg.mask = cfg->mask;

    return 0;
}

/**
 * Reads Accelerometer data from ADXL362 sensor
 *
 * @param Pointer to sensor interface
 * @param Pointer to structure to store result
 *
 * @return 0 on success, non-zero on failure
 */
int
adxl362_get_accel_data(struct sensor_itf *itf, struct sensor_accel_data *sad)
{
    int rc;
    uint8_t payload[6];
    int16_t x,y,z;

    /* Get a new accelerometer sample */
    rc = adxl362_readlen(itf, ADXL362_REG_XDATA_L, payload, 6);
    if (rc) {
        return rc;
    }

    x = (((int16_t)payload[1]) << 8) | payload[0];
    y = (((int16_t)payload[3]) << 8) | payload[2];
    z = (((int16_t)payload[5]) << 8) | payload[4];

    sad->sad_x = x;
    sad->sad_x_is_valid = 0;
    sad->sad_y = y;
    sad->sad_y_is_valid = 0;
    sad->sad_z = z;
    sad->sad_z_is_valid = 0;

    return 0;
}

/**
 * Read Accelerometer data from ADXL362 sensor
 *
 * @param Pointer to sensor structure
 * @param Type of sensor data to read
 * @param Pointer to function to call to output data
 * @param Pointer to data to pass to data_func
 * @param timeout value
 *
 * @return 0 on success, non-zero on failure
 */
static int
adxl362_sensor_read(struct sensor *sensor, sensor_type_t type,
                    sensor_data_func_t data_func, void *data_arg, uint32_t timeout)
{
    (void)timeout;
    int rc;

    struct sensor_itf *itf;
    struct sensor_accel_data sad;
    struct adxl362 *adxl362;
    float selectedRange;
  
    /* If the read isn't looking for accel don't do anything. */
    if (!(type & SENSOR_TYPE_ACCELEROMETER)) {
        return SYS_EINVAL;
    }
  
    itf = SENSOR_GET_ITF(sensor);
  
    /*get raw accel datas*/
    rc = adxl362_get_accel_data(itf, &sad);
    if (rc) {
        return rc;
    }

    /* calculate MS*2 values to provide to data_func */
    adxl362 = (struct adxl362 *)SENSOR_GET_DEVICE(sensor);

    selectedRange = (1<<(1 + adxl362->cfg.accel_range));

    sad.sad_x = adxl362_convert_reg_to_ms2(sad.sad_x, selectedRange/2);
    sad.sad_x_is_valid = 1;
    sad.sad_y = adxl362_convert_reg_to_ms2(sad.sad_y, selectedRange/2);
    sad.sad_y_is_valid = 1;
    sad.sad_z = adxl362_convert_reg_to_ms2(sad.sad_z, selectedRange/2);
    sad.sad_z_is_valid = 1;

    /* output data using data_func */
    rc = data_func(sensor, data_arg, &sad, SENSOR_TYPE_ACCELEROMETER);
    if (rc) {
        return rc;
    }

    return 0;
}

static int
adxl362_sensor_get_config(struct sensor *sensor, sensor_type_t type,
                          struct sensor_cfg *cfg)
{
    /* If the read isn't looking for accel, don't do anything. */
    if (!(type & SENSOR_TYPE_ACCELEROMETER)) {
        return SYS_EINVAL;
    }

    cfg->sc_valtype = SENSOR_VALUE_TYPE_FLOAT_TRIPLET;

    return 0;
}


/**
 * Expects to be called back through os_dev_create().
 *
 * @param The device object associated with this accelerometer
 * @param Argument passed to OS device init, unused
 *
 * @return 0 on success, non-zero error on failure.
 */
static int 
init_spi(struct adxl362 *adxl, struct hal_spi_settings *spi_settings)
{
    int rc = 0;

    if(adxl->pdd.itf_is_initialized == true){
        return rc;
    }

    rc = hal_spi_config(adxl->sensor.s_itf.si_num, spi_settings);
    if (rc == EINVAL) {
        return rc;
    }

    rc = hal_spi_enable(adxl->sensor.s_itf.si_num);
    if (rc) {
        return rc;
    }

    rc = hal_gpio_init_out(adxl->sensor.s_itf.si_cs_pin, 1);
    if (rc) {
        return rc;
    }

    adxl->pdd.itf_is_initialized = true;

    return rc;
}

static int 
deinit_spi(struct adxl362 *adxl){
    int rc = 0;

    if(adxl->pdd.itf_is_initialized == false){
        return rc;
    }

    adxl->pdd.itf_is_initialized = false;
    
    return hal_spi_disable(adxl->sensor.s_itf.si_num);
}


#if MYNEWT_VAL(ADXL362_INT_ENABLE)
static void
interrupt_handler(void * arg)
{
#if MYNEWT_VAL(ADXL_362_LPMODE)
    struct sensor *sensor;
    struct adxl362 *adxl362;
    sensor = arg;
    adxl362 = (struct adxl362 *)SENSOR_GET_DEVICE(sensor);

    if (adxl362->sensor.s_itf.si_type == SENSOR_ITF_SPI){
        init_spi(adxl362, &spi_adxl362_settings);
    }
#endif

    sensor_mgr_put_interrupt_evt(sensor);
}

/**
 * Initialises the local interrupt pin 
 *
 * @param Pointer to device structure
 * @param interrupt handler to setup
 * @param Pointer to data to pass to irq
 *
 * @return 0 on success, non-zero on failure
 */
static int
init_intpin(struct adxl362 * adxl362, hal_gpio_irq_handler_t handler,
            void * arg)
{
    struct adxl362_private_driver_data *pdd = &adxl362->pdd;
    hal_gpio_irq_trig_t trig;
    int pin = -1;
    int rc;
    int i;

    for (i = 0; i < MYNEWT_VAL(SENSOR_MAX_INTERRUPTS_PINS); i++){
        pin = adxl362->sensor.s_itf.si_ints[i].host_pin;
        if (pin >= 0) {
            break;
        }
    }

    if (pin < 0) {
        console_printf("Interrupt pin not configured\n");
        return SYS_EINVAL;
    }

    pdd->int_num = i;
/*    if (adxl362->sensor.s_itf.si_ints[pdd->int_num].active) {
        trig = HAL_GPIO_TRIG_RISING;
    } else {
        trig = HAL_GPIO_TRIG_FALLING;
    }
*/
    trig = HAL_GPIO_TRIG_BOTH;
  
    if (adxl362->sensor.s_itf.si_ints[pdd->int_num].device_pin > 2) {
        console_printf("Route not configured\n");
        return SYS_EINVAL;
    }else{
        pdd->int_route = adxl362->sensor.s_itf.si_ints[pdd->int_num].device_pin;
    }

    rc = hal_gpio_irq_init(pin,
                           handler,
                           arg,
                           trig,
                           HAL_GPIO_PULL_NONE);
    if (rc != 0) {
        console_printf("Failed to initialise interrupt pin %d\n", pin);
        return rc;
    }
    
    /* disable interrupt for now */
    hal_gpio_irq_disable(pin);
    
    return 0;
}

/**
 * Enables an interrupt in ADXL362 device
 *
 * @param Pointer to sensor structure
 * @param which interrupt(s) to enable
 *
 * @return 0 on success, non-zero on failure
 */
static int
enable_interrupt(struct sensor * sensor, uint8_t ints_to_enable)
{
    struct adxl362_private_driver_data *pdd;
    struct adxl362 *adxl362;
    struct sensor_itf *itf;

    if (ints_to_enable == 0) {
        return SYS_EINVAL;
    }

    adxl362 = (struct adxl362 *)SENSOR_GET_DEVICE(sensor);
    itf = SENSOR_GET_ITF(sensor);
    pdd = &adxl362->pdd;
 
    /* if no interrupts are currently in use enable int pin */
    if (pdd->int_enable == 0) {
        hal_gpio_irq_enable(adxl362->sensor.s_itf.si_ints[pdd->int_num].host_pin);
    }
    
    /* update which interrupts are enabled */
    pdd->int_enable |= ints_to_enable;

    /* enable interrupt in device */
    return adxl362_setup_interrupts(itf, pdd->int_enable, pdd->int_route);
}

/**
 * Disables an interrupt in ADXL362 device
 *
 * @param Pointer to sensor structure
 * @param which interrupt(s) to disable
 *
 * @return 0 on success, non-zero on failure
 */
static int
disable_interrupt(struct sensor * sensor, uint8_t ints_to_disable)
{
    struct adxl362_private_driver_data *pdd;
    struct adxl362 *adxl362;
    struct sensor_itf *itf;

    if (ints_to_disable == 0) {
        return SYS_EINVAL;
    }

    adxl362 = (struct adxl362 *)SENSOR_GET_DEVICE(sensor);
    itf = SENSOR_GET_ITF(sensor);
    pdd = &adxl362->pdd;
    
    /* update which interrupts are enabled */
    pdd->int_enable &= ~ints_to_disable;

    /* if no interrupts are now in use disable int pin */
    if (pdd->int_enable == 0) {
        hal_gpio_irq_disable(adxl362->sensor.s_itf.si_ints[pdd->int_num].host_pin);
    }

    /* update interrupt setup in device */
    return adxl362_setup_interrupts(itf, pdd->int_enable, pdd->int_route);
}
#endif

/**
 * Handles and interrupt, firing correct events
 *
 * @param Pointer to sensor structure
 *
 * @return 0 on success, non-zero on failure
 */
static int
adxl362_sensor_handle_interrupt(struct sensor * sensor)
{
#if MYNEWT_VAL(ADXL362_INT_ENABLE)
    struct adxl362 * adxl362;
    struct adxl362_private_driver_data *pdd;
    struct sensor_itf *itf;
    uint8_t int_status;
    
    int rc;

    adxl362 = (struct adxl362 *)SENSOR_GET_DEVICE(sensor);
    itf = SENSOR_GET_ITF(sensor);

    pdd = &adxl362->pdd;

    rc = adxl362_clear_interrupts(itf, &int_status);
    if (rc != 0) {
        console_printf("Cound not read int status err=0x%02x\n", rc);
        return rc;
    }

    if (pdd->registered_mask & ADXL362_NOTIFY_MASK) {
        sensor_mgr_put_notify_evt(&pdd->notify_ctx, 
                ((int_status & ADXL362_STATUS_INACT) != 0)?SENSOR_EVENT_TYPE_SLEEP : SENSOR_EVENT_TYPE_WAKEUP);
    }

    /* revert interrupt polarity on INT1 MAP !*/
    /*
    uint8_t reg;
    rc = adxl362_read8(itf, ADXL362_REG_INTMAP1, &reg);
    if (rc) {
        return rc;
    }
    if((reg & ADXL362_INTMAP1_INT_LOW)){
        reg &= ~ADXL362_INTMAP1_INT_LOW;
    }else{
        reg |=ADXL362_INTMAP1_INT_LOW;
    }
    rc = adxl362_write8(itf, ADXL362_REG_INTMAP1, reg);
    if (rc) {
        return rc;
    }*/

    /*TODO know what is it for ???*/
    if ((pdd->registered_mask & ADXL362_READ_MASK) &&
        ((int_status & ADXL362_STATUS_INACT) ||
         (int_status & ADXL362_STATUS_INACT))) {
        console_printf("READ EVT 0x%02x\n", int_status);
        sensor_mgr_put_read_evt(&pdd->read_ctx);
    }

    return 0;
#else
    return SYS_ENODEV;
#endif
}

/**
 * Sets up trigger thresholds and enables interrupts
 *
 * @param Pointer to sensor structure
 * @param type of sensor
 * @param threshold settings to configure
 *
 * @return 0 on success, non-zero on failure
 */
static int
adxl362_sensor_set_trigger_thresh(struct sensor * sensor,
                                  sensor_type_t sensor_type,
                                  struct sensor_type_traits * stt)
{
#if MYNEWT_VAL(ADXL362_INT_ENABLE)
    struct adxl362 * adxl362;
    struct sensor_itf *itf;
    int rc;
    const struct sensor_accel_data * low_thresh;
    const struct sensor_accel_data * high_thresh;
    uint8_t ints_to_enable = 0;
    float thresh, selectedRange;
    struct adxl362_private_driver_data *pdd;

    if (sensor_type != SENSOR_TYPE_ACCELEROMETER) {
        return SYS_EINVAL;
    }

    adxl362 = (struct adxl362 *)SENSOR_GET_DEVICE(sensor);
    itf = SENSOR_GET_ITF(sensor);
    pdd = &adxl362->pdd;

    low_thresh  = stt->stt_low_thresh.sad;
    high_thresh = stt->stt_high_thresh.sad;
    
    selectedRange = (1<<(1 + adxl362->cfg.accel_range));

    if (low_thresh->sad_x_is_valid |
        low_thresh->sad_y_is_valid |
        low_thresh->sad_z_is_valid) {
        thresh = INFINITY;

        if (low_thresh->sad_x_is_valid) {
            if (thresh > low_thresh->sad_x) {
                thresh = low_thresh->sad_x;
            }
        }
        if (low_thresh->sad_y_is_valid) {
            if (thresh > low_thresh->sad_y) {
                thresh = low_thresh->sad_y;
            }
        }
        if (low_thresh->sad_z_is_valid) {
            if (thresh > low_thresh->sad_z) {
                thresh = low_thresh->sad_z;
            }
        }

        rc = adxl362_set_inactive_settings(itf, 1, 1,
                adxl362_convert_ms2_to_reg(thresh, selectedRange/2), 
                adxl362_convert_ms_to_regtimer(2000, adxl362->cfg.sample_rate));
        
        if (rc) {
            return rc;
        }

        ints_to_enable |= ADXL362_STATUS_AWAKE;
    }

    if (high_thresh->sad_x_is_valid |
        high_thresh->sad_y_is_valid |
        high_thresh->sad_z_is_valid) {
        thresh = 0.0;

        if (high_thresh->sad_x_is_valid) {
            if (thresh < high_thresh->sad_x) {
                thresh = high_thresh->sad_x;
            }
        }
        if (high_thresh->sad_y_is_valid) {
            if (thresh < high_thresh->sad_y) {
                thresh = high_thresh->sad_y;
            }
        }
        if (high_thresh->sad_z_is_valid) {
            if (thresh < high_thresh->sad_z) {
                thresh = high_thresh->sad_z;
            }
        }

        rc = adxl362_set_active_settings(itf, 1, 1,
                adxl362_convert_ms2_to_reg(thresh, selectedRange/2), 
                adxl362_convert_ms_to_regtimer(2000, adxl362->cfg.sample_rate));
        if (rc) {
            return rc;
        }

        ints_to_enable |= ADXL362_STATUS_AWAKE;
       
    }

    rc = enable_interrupt(sensor, ints_to_enable);
    if (rc) {
        return rc;
    }
    
    pdd->read_ctx.srec_type |= sensor_type;
    pdd->registered_mask |= ADXL362_READ_MASK;

    return 0;
#else
    return SYS_ENODEV;
#endif
}

/**
 * Disable the low threshold interrupt
 *
 * @param ptr to sensor
 * @param the Sensor type
 *
 * @return 0 on success, non-zero on failure
 */
static int
adxl362_sensor_clear_low_thresh(struct sensor *sensor,
                                 sensor_type_t type)
{
    uint8_t ints_to_disable = ADXL362_STATUS_AWAKE;

    if (type != SENSOR_TYPE_ACCELEROMETER) {
        return SYS_EINVAL;
    }

    struct adxl362 *adxl362;
    
    adxl362 = (struct adxl362 *)SENSOR_GET_DEVICE(sensor);
    /* if neither high or low threshs are now set disable read mask */
    if((adxl362->pdd.int_enable & ADXL362_STATUS_AWAKE) == 0) {
        adxl362->pdd.read_ctx.srec_type &= ~type;
        adxl362->pdd.registered_mask &= ~ADXL362_READ_MASK;
    }

    return disable_interrupt(sensor, ints_to_disable);
}


/**
 * Disable the high threshold interrupt
 *
 * @param ptr to sensor
 * @param the Sensor type
 *
 * @return 0 on success, non-zero on failure
 */
static int
adxl362_sensor_clear_high_thresh(struct sensor *sensor,
                                  sensor_type_t type)
{

    uint8_t ints_to_disable = ADXL362_STATUS_AWAKE;

    if (type != SENSOR_TYPE_ACCELEROMETER) {
        return SYS_EINVAL;
    }

    struct adxl362 *adxl362;
    adxl362 = (struct adxl362 *)SENSOR_GET_DEVICE(sensor);

    /* if neither high or low threshs are now set disable read mask */
    if((adxl362->pdd.int_enable & ADXL362_STATUS_AWAKE) == 0) {
        adxl362->pdd.read_ctx.srec_type &= ~type;
        adxl362->pdd.registered_mask &= ~ADXL362_READ_MASK;
    }

    return disable_interrupt(sensor, ints_to_disable);
}

/**
 * Enables notifications on orientation changing
 *
 * @param Pointer to sensor structure
 * @param which event to get notifications for
 *
 * @return 0 on success, non-zero on failure
 */
static int
adxl362_sensor_set_notification(struct sensor * sensor,
                                sensor_event_type_t sensor_event_type)
{
#if MYNEWT_VAL(ADXL362_INT_ENABLE)
    struct adxl362 * adxl362;
    uint8_t ints_to_enable = 0;
    struct adxl362_private_driver_data *pdd;
    int rc;

    console_printf("Enabling notifications\n");
    
    /*XXX for now we do not support anything else */
    if ((sensor_event_type & SENSOR_EVENT_TYPE_WAKEUP) == 0 &&
        (sensor_event_type & SENSOR_EVENT_TYPE_SLEEP) == 0 
    ) {
        return SYS_EINVAL;
    }

    adxl362 = (struct adxl362 *)SENSOR_GET_DEVICE(sensor);
    pdd = &adxl362->pdd;

    if (pdd->registered_mask & ADXL362_NOTIFY_MASK) {
        return SYS_EBUSY;
    }

    /* Enable orientation changing event*/
    //if(sensor_event_type == SENSOR_EVENT_TYPE_ORIENT_CHANGE) {
        ints_to_enable |= ADXL362_STATUS_AWAKE;
    //}

    rc = enable_interrupt(sensor, ints_to_enable);
    if (rc) {
        return rc;
    }

    pdd->notify_ctx.snec_evtype |= sensor_event_type;
    pdd->registered_mask |= ADXL362_NOTIFY_MASK;

    console_printf("Enabled notifications\n");
    
    return 0;
#else
    return SYS_ENODEV;
#endif
}


/**
 * Disables notifications on orientation changing
 *
 * @param Pointer to sensor structure
 * @param which event to get notifications for
 *
 * @return 0 on success, non-zero on failure
 */
static int
adxl362_sensor_unset_notification(struct sensor * sensor,
                                  sensor_event_type_t sensor_event_type)
{
#if MYNEWT_VAL(ADXL362_INT_ENABLE)
    struct adxl362 * adxl362;
    uint8_t ints_to_disable = 0;

    /*XXX for now we do not support registering something else */
    if ((sensor_event_type & ~(SENSOR_EVENT_TYPE_ORIENT_CHANGE)) != 0) {
        return SYS_EINVAL;
    }

    adxl362 = (struct adxl362 *)SENSOR_GET_DEVICE(sensor);

    adxl362->pdd.notify_ctx.snec_evtype &= ~sensor_event_type;
    adxl362->pdd.registered_mask &= ~ADXL362_NOTIFY_MASK;

    ints_to_disable = ADXL362_STATUS_AWAKE;
    
    return disable_interrupt(sensor, ints_to_disable);
    
#else
    return SYS_ENODEV;
#endif
}



