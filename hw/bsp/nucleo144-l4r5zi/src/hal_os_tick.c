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
#include <hal/hal_os_tick.h>
#include <hal/hal_bsp.h>

/* implementation of different power modes available on STM32 processors */
#if defined STM32L151xC || defined STM32L4R5xx
#include "bsp/hal_bsp_power_handler.h"
extern void stm32_power_enter(int power_mode, uint32_t durationMS);
extern void stm32_tick_init(uint32_t os_ticks_per_sec, int prio);
#else
static void 
stm32_tick_init(uint32_t os_ticks_per_sec, int prio) 
{
    /*nb of ticks per seconds is hardcoded in HAL_InitTick(..) to have 1ms/tick */
    assert(os_ticks_per_sec == OS_TICKS_PER_SEC);
    
    uint32_t reload_val;

    reload_val = ((uint64_t)SystemCoreClock / os_ticks_per_sec) - 1;

    /* Set the system time ticker up */
    SysTick->LOAD = reload_val;
    SysTick->VAL = 0;
    SysTick->CTRL = 0x0007;

    /* Set the system tick priority */
    NVIC_SetPriority(SysTick_IRQn, prio);

    /*
     * Keep clocking debug even when CPU is sleeping, stopped or in standby.
     */

#if !MYNEWT_VAL(MCU_STM32F0)
    DBGMCU->CR |= (DBGMCU_CR_DBG_SLEEP | DBGMCU_CR_DBG_STOP | DBGMCU_CR_DBG_STANDBY);
#else
    DBGMCU->CR |= (DBGMCU_CR_DBG_STOP | DBGMCU_CR_DBG_STANDBY);
#endif

}

static void 
stm32_power_enter(int power_mode, uint32_t durationMS)
{
    __DSB();
    __WFI();
}

#endif


void
__wrap_os_tick_idle(os_time_t ticks)
{
    /* default mode will enter basic sleep mode, in WFI */
    int power_mode = HAL_BSP_POWER_WFI; //HAL_BSP_POWER_SLEEP;//

    OS_ASSERT_CRITICAL();

    /* this means the required sleep time was < OS_IDLE_TICKLESS_MS_MIN, so just leave standard SYSTICK and WFI to and wakeup in 1ms */
    if (ticks == 0) { 
        stm32_power_enter(power_mode, 0);
        return;
    }

    /* Convert to ms */
    uint32_t timeMS = os_time_ticks_to_ms32(ticks);

#if MYNEWT_VAL(BSP_POWER_SETUP)
    /* ask bsp for lowest power mode that is possible */
    power_mode = hal_bsp_power_handler_get_mode(timeMS);

    /* Tell BSP we enter sleep, so it should shut down any board periphs it can for the mode it wants */
    hal_bsp_power_handler_sleep_enter(power_mode);

#endif

    /* setup the appropriate mcu power mode during timeMs asked by os scheduling */
    stm32_power_enter(power_mode, timeMS);

#if MYNEWT_VAL(BSP_POWER_SETUP)
    /* bsp exits the actual power mode */
    hal_bsp_power_handler_sleep_exit(power_mode);
#endif


}

void
__wrap_os_tick_init(uint32_t os_ticks_per_sec, int prio)
{
    stm32_tick_init(os_ticks_per_sec, prio);
}