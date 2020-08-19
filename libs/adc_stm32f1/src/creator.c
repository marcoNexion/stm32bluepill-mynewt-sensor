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
//  Create STM32F1 ADC1 port driver. Adapted from repos/apache-mynewt-core/hw/bsp/olimex_stm32-e407_devboard/src/hal_bsp.c
#include <stm32f1xx_hal.h>
#include <stm32f1xx_hal_dma.h>
#include <stm32f1xx_hal_adc.h>
#include <adc_stm32f1/adc_stm32f1.h>
#include <console/console.h>

//  BSP Definitions for ADC1.  Only ADC1 is supported.

#define STM32F1_ADC_DEFAULT_INIT_TD {\
    /* TODO: .ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV2, */ \
    /* TODO: .Resolution     = ADC_RESOLUTION12b, */ \
    .DataAlign             = ADC_DATAALIGN_RIGHT, /* Align the converted result right */ \
    .ScanConvMode          = DISABLE,             /* Disable scan conversion mode */ \
    /* TODO: .EOCSelection = DISABLE, */ \
    .ContinuousConvMode    = ENABLE,              /* Disable continuous conversion mode, because we start and stop the ADC at each poll */ \
    .NbrOfConversion       = 1,                   /* Number of channels of the regular group that will be converted in scan mode: Only 1 channel */ \
    .DiscontinuousConvMode = DISABLE,\
    .NbrOfDiscConversion   = 0,\
    .ExternalTrigConv      = ADC_SOFTWARE_START,  /* Trigger the conversion by software */ \
    /* TODO: .ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE, */ \
    /* TODO: .DMAContinuousRequests = ENABLE */ \
}

/*****************ADC1 Config ***************/
#define STM32F1_DEFAULT_ADC1_HANDLE {\
    .Init     = STM32F1_ADC_DEFAULT_INIT_TD,\
    .Instance = ADC1,\
    /* TODO: .NbrOfCurrentConversionRank = 0, */ \
    .DMA_Handle = NULL, /* TODO: &adc1_dma00_handle, */ \
    .Lock       = HAL_UNLOCKED,\
    .State      = 0,\
    .ErrorCode  = 0\
}

static ADC_HandleTypeDef adc1_handle = STM32F1_DEFAULT_ADC1_HANDLE;

//  We support total 18 channels on ADC1, including ADC_CHANNEL_TEMPSENSOR and ADC_CHANNEL_VREFINT.
#define STM32F1_ADC1_DEFAULT_CONFIG {\
    .sac_chan_count = 18,\
    .sac_chans = (struct adc_chan_config [18]){{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},/* TODO: STM32F1_ADC1_DEFAULT_SAC */ {0},{0},{0},{0},{0},{0},{0},{0} } ,\
    .sac_adc_handle = &adc1_handle,\
}
/*********************************************/

//  Define the device specifics here so the device creator code below can be generic.
#define DEVICE_NAME      STM32F1_ADC1_DEVICE  //  Name of device
#define DEVICE_DEV       adc_dev              //  Device type
#define DEVICE_INSTANCE  stm32f1_adc1_dev     //  Device instance
#define DEVICE_CFG       stm32f1_adc_dev_cfg  //  Device config
#define DEVICE_INIT      stm32f1_adc_dev_init //  Device init function
#define DEVICE_CREATE    stm32f1_adc_create   //  Device create function
#define DEVICE_ITF       adc1_config          //  Device interface
//  #define DEVICE_CFG_FUNC  stm32f1_adc1_config  //  Device config function

static struct DEVICE_DEV DEVICE_INSTANCE;

static struct DEVICE_CFG DEVICE_ITF = STM32F1_ADC1_DEFAULT_CONFIG;

///////////////////////////////////////////////////////////////////////////////
//  Generic Device Creator Code

//  Create the device instance and configure it. Called by sysinit() during startup, defined in pkg.yml.
void DEVICE_CREATE(void) {
    console_printf("ADC create " DEVICE_NAME "\n");

    int rc = os_dev_create((struct os_dev *) &DEVICE_INSTANCE, DEVICE_NAME,
        OS_DEV_INIT_PRIMARY, 0,  //  For BSP: OS_DEV_INIT_KERNEL, OS_DEV_INIT_PRIO_DEFAULT,
        DEVICE_INIT, (void *) &DEVICE_ITF);
    assert(rc == 0);
}
