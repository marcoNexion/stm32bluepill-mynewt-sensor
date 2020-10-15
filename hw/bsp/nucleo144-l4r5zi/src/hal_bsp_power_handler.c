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
#include <assert.h>

#include "os/mynewt.h"



#define NELEMS(x)  (sizeof(x) / sizeof((x)[0]))


#if 0//MYNEWT_VAL(BSP_POWER_SETUP)

#include "sysflash/sysflash.h"

#if MYNEWT_VAL(UART_0) || MYNEWT_VAL(UART_DBG)
#include <uart/uart.h>
#endif
#if MYNEWT_VAL(UART_0)
#include <uart_hal/uart_hal.h>
#endif
#if MYNEWT_VAL(UART_DBG)
#include <uart_bitbang/uart_bitbang.h>
#endif

#include <hal/hal_bsp.h>
#include <hal/hal_gpio.h>
#include <hal/hal_flash_int.h>
#include <hal/hal_timer.h>

#if MYNEWT_VAL(ADC) 
//#include <adc/adc.h>
#include "stm32l1xx_hal_adc.h"
#include "stm32l1xx_hal_rcc.h"
#include "stm32l1xx_hal.h"

#endif

#if MYNEWT_VAL(SPI_0_MASTER) || MYNEWT_VAL(SPI_0_SLAVE) || MYNEWT_VAL(SPI_1_MASTER) || MYNEWT_VAL(SPI_1_SLAVE)
#include <hal/hal_spi.h>
#endif
#if MYNEWT_VAL(I2C_0)
#if MYNEWT_VAL(USE_BUS_I2C)
#include "bus/drivers/i2c_common.h"
#include "bus/drivers/i2c_hal.h"
#else
#include "hal/hal_i2c.h"
#endif  /* USE_BUS_I2C */
#endif      /* I2C_0 */

#include <stm32l151xc.h>
#include <stm32l1xx_hal_rcc.h>
#include <stm32l1xx_hal_pwr.h>
#include <stm32l1xx_hal_flash.h>
#include <stm32l1xx_hal_gpio.h>
#include <stm32l1xx_hal_gpio_ex.h>
#include <mcu/stm32l1_bsp.h>
#include "mcu/stm32l1xx_mynewt_hal.h"
#include "mcu/stm32_hal.h"
#if MYNEWT_VAL(I2S)
#include "stm32l1xx_hal_i2s.h"
#endif
#endif //#if MYNEWT_VAL(BSP_POWER_SETUP)

#include <hal/hal_bsp.h>
#include "bsp/bsp.h"
#include "bsp/lowpowermgr.h"
#include "bsp/hal_power_mgnt.h"


#if MYNEWT_VAL(BSP_POWER_SETUP)

static LP_HOOK_t _hook_get_mode_cb=NULL;
static LP_HOOK_t _hook_exit_cb=NULL;
static LP_HOOK_t _hook_enter_cb=NULL;

/* hook idle enter/exit phases. 
 * Note the get_mode call is made with irq disabled in OS critical section - so don't hang about
 * enter/exit are called outside of the critical region
 * */
void hal_bsp_power_hooks(LP_HOOK_t getMode, LP_HOOK_t enter, LP_HOOK_t exit)
{
    // Should only have 1 hook of sleeping in the code, so assert if called twice
    assert(_hook_enter_cb==NULL);
    _hook_get_mode_cb = getMode;
    _hook_enter_cb = enter;
    _hook_exit_cb = exit;
}

/* the 2 following functions are called from hal_os_tick.c iff BSP_POWER_SETUP is set */
/* get the required power sleep mode that the application wants to be in */
int hal_bsp_power_handler_get_mode(os_time_t ticks)
{
    // ask to BSP for the appropriate power mode
    return (_hook_get_mode_cb!=NULL)?(*_hook_get_mode_cb)():HAL_BSP_POWER_WFI;
}

/* enter sleep - called before entering critical region */
void hal_bsp_power_handler_sleep_enter(int nextMode)
{
    if (_hook_enter_cb!=NULL) {
        (*_hook_enter_cb)();
    }

    /* Now BSP deinit                      */
    /* shared bus interfaces               */
    switch(nextMode) {
        
        case HAL_BSP_POWER_OFF:
        case HAL_BSP_POWER_DEEP_SLEEP:
        case HAL_BSP_POWER_SLEEP:
          
            /* I2S */
            //bsp_deinit_i2s();

            /* I2C */
            //hal_bsp_deinit_i2c();

            /* SPI */
            //hal_bsp_deinit_spi();

            /*UART */
            //hal_bsp_uart_deinit();

            break;
        case HAL_BSP_POWER_WFI: 

        case HAL_BSP_POWER_ON:
        default:             
           break;
    }

}

/* exit sleep - called after exiting critical region */
void hal_bsp_power_handler_sleep_exit(int lastMode)
{
    /* and tell hook */
    if (_hook_exit_cb!=NULL) {
        (*_hook_exit_cb)();
    }

    /* Now BSP must reinit                 */
    /* shared bus interfaces               */
    switch(lastMode) {
        
        case HAL_BSP_POWER_OFF:
        case HAL_BSP_POWER_DEEP_SLEEP:
        case HAL_BSP_POWER_SLEEP:
     
            /* I2S */
            //bsp_init_i2s();

            /* I2C */         
            //hal_bsp_init_i2c();

            /* SPI */
            //hal_bsp_init_spi();

            /*UART */
            //hal_bsp_uart_init();

            break;
        case HAL_BSP_POWER_WFI: 

        case HAL_BSP_POWER_ON:
        default:             
           break;
    }
}
#else 
void hal_bsp_power_hooks(LP_HOOK_t getMode, LP_HOOK_t enter, LP_HOOK_t exit) {
    // noop
    (void)getMode;
    (void)enter;
    (void)exit;
}
int hal_bsp_power_handler_get_mode(os_time_t ticks) {
    return HAL_BSP_POWER_WFI;
}
void hal_bsp_power_handler_sleep_enter(int nextMode){

}
void hal_bsp_power_handler_sleep_exit(int lastMode){
    
}

#endif


/** enter a MCU stop mode, with all periphs off or lowest possible power, and never return */
void hal_bsp_halt() {
      
    //tell lowpowermgr to deinit stuff
    hal_bsp_power_handler_sleep_enter(HAL_BSP_POWER_DEEP_SLEEP);

    // ask MCU HAL to stop it
    hal_mcu_halt();
}

