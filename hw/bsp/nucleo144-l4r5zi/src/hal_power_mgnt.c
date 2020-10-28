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
#include <assert.h>
#include <os/mynewt.h>
#include <syscfg/syscfg.h>
#include <mcu/stm32_hal.h>
#include <hal/hal_bsp.h>
#include "hal/hal_system.h"
#include "stm32l4xx_hal_pwr.h"
#include "bsp/hal_power_mgnt.h"
#include <mcu/cmsis_nvic.h>
#include "hal/hal_watchdog.h"

#define EXT_GPIO_FOR_SLEEP_TESTING
#define EXT_GPIO_FOR_WFI_TESTING

extern void IRQset_get_status(void);
extern void IRQset_disable_all(void);

#ifdef EXT_GPIO_FOR_SLEEP_TESTING 
#include "bsp/bsp.h"
#include "hal/hal_gpio.h"
#define EXT_SLEEP_OUTPUT  MCU_GPIO_PORTC(6)
#endif


#ifdef EXT_GPIO_FOR_WFI_TESTING 
#include "bsp/bsp.h"
#include "hal/hal_gpio.h"
#define EXT_WFI_OUTPUT      MCU_GPIO_PORTC(5)
#endif

/* maximum time that can be requested from the wakeup timer (and still get the elapsed rtc time correctly) */
#define MAX_WAKEUP_TIMER_MS MYNEWT_VAL(OS_IDLE_TICKLESS_MS_MAX) //to avoid 16-bit timer overflow

#if MYNEWT_VAL(OS_TICKLESS)
#include "hal_lptimer.h"
static void stm32_tickless_start(uint32_t timeMS);
static void stm32_tickless_stop(uint32_t timeMS);
#endif

/* Put MCU  in lowest power stop state, exit only via POR or reset pin */
void 
hal_mcu_halt(void) 
{

    /* all interupts and exceptions off */
    /* PVD off */
    /* power watchdog off */
    /* Be in lowest power mode forever */

    /* ensure RTC not gonna wake us */
    //hal_rtc_disable_wakeup();

    /* Stop SYSTICK */
    NVIC_DisableIRQ(SysTick_IRQn);
    /* Suspend SysTick Interrupt */
    CLEAR_BIT(SysTick->CTRL,SysTick_CTRL_TICKINT_Msk);

    while (1) {

        /*Disables the Power Voltage Detector(PVD) */                
        HAL_PWR_DisablePVD( );
        /* Enable Ultra low power mode */
        //HAL_PWREx_EnableUltraLowPower( );
        /* Enable the fast wake up from Ultra low power mode */
        //HAL_PWREx_EnableFastWakeUp( );
        HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN1);
        HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN2);
        /* System clock down to MSI */
//        SystemClock_StopPLL();
        /* Enters Stop mode */
//        HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
        HAL_PWR_EnterSTANDBYMode();
        // Shouldn't return
        hal_system_reset();
    }
}

void 
stm32_tick_init(uint32_t os_ticks_per_sec, int prio)
{
    /* Even for tickless we use SYSTICK for normal tick.*/
    /* nb of ticks per seconds is hardcoded in HAL_InitTick(..) to have 1ms/tick */
    assert(os_ticks_per_sec == OS_TICKS_PER_SEC);
    
    volatile uint32_t reload_val;

    /*Reload Value = SysTick Counter Clock (Hz) x  Desired Time base (s) */
    reload_val = ((uint64_t)SystemCoreClock / os_ticks_per_sec) - 1;
    /* Set the system time ticker up */
    SysTick->LOAD = reload_val;
    SysTick->VAL = 0;

    /* CLKSOURCE : 1 -> HCLK, 0-> AHB Clock (which is HCLK/8). Use HCLK, as this is the value of SystemCoreClock as used above */
    SysTick->CTRL = (SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk);  
        
    /* Set the system tick priority */
    NVIC_SetPriority(SysTick_IRQn, prio);

#ifdef RELEASE_BUILD
    HAL_DBGMCU_DisableDBGSleepMode();
    HAL_DBGMCU_DisableDBGStopMode();
    HAL_DBGMCU_DisableDBGStandbyMode();
#else
    /* Keep clocking debug even when CPU is sleeping, stopped or in standby.*/
    HAL_DBGMCU_EnableDBGSleepMode();
    HAL_DBGMCU_EnableDBGStopMode();
    HAL_DBGMCU_EnableDBGStandbyMode();
#endif

#if MYNEWT_VAL(OS_TICKLESS)
	hal_lptimer_init();
#endif

#ifdef EXT_GPIO_FOR_SLEEP_TESTING
    hal_gpio_init_out(EXT_SLEEP_OUTPUT, 1);
#endif

#ifdef EXT_GPIO_FOR_WFI_TESTING
    hal_gpio_init_out(EXT_WFI_OUTPUT, 1);
#endif

}


#if MYNEWT_VAL(OS_TICKLESS)
static void 
stm32_tickless_start(uint32_t timeMS)
{
    /* Start RTC alarm for in this amount of time if not 0 (note: case of timeMS==0 is used for HALT ie never coming back) */
    assert(timeMS > 0);

    hal_lptimer_start(timeMS);
    
    /* Suspend SysTick Interrupt */
    HAL_SuspendTick();
}

static void 
stm32_tickless_stop(uint32_t timeMS)
{
    /* add asleep duration to tick counter */
    uint32_t asleep_ms = (uint32_t)hal_lptimer_get_elapsed_time();

    if(asleep_ms > 0)
    {
        /*little adjustement*/
        asleep_ms -= 1;
    }

    assert(asleep_ms <= timeMS);

    int asleep_ticks = os_time_ms_to_ticks32(asleep_ms);
    
    /* Since ISR is actually suspended : clean pending IRQs */
    /* It's mandatory due to PendSV routine awaiting for  */
    /* no exception is active */
    hal_lptimer_clear_pending_irq();

    /* disable LPtimer wakeup */
    hal_lptimer_stop();

    /* Now update Systick counter and scheduling        */
    /* PendSV interrupt pending bit will be set         */
    /* to force context switch         */
    os_time_advance(asleep_ticks);

    /* reenable SysTick Interrupt */
    HAL_ResumeTick();

    hal_watchdog_tickle();

}
#endif //#if MYNEWT_VAL(OS_TICKLESS)

void 
stm32_power_enter(int power_mode, uint32_t durationMS)
{
    /* if sleep time was less than MIN_TICKS, it is 0. Just do usual WFI and systick will wake us in 1ms */
    if (durationMS == 0) {
        HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
        return;
    }
    /* limit sleep to largest value of wakeuptimer that is supported by RTC - ensure we wake up before RTC timer wraps */
    if (durationMS > MAX_WAKEUP_TIMER_MS) {
        durationMS = MAX_WAKEUP_TIMER_MS; 
    }

#if MYNEWT_VAL(TIMER_0)
    //NVIC_DisableIRQ(TIM2_IRQn);
//    hal_timer_deinit(0);
#endif

    /* begin tickless */
#if MYNEWT_VAL(OS_TICKLESS)
    stm32_tickless_start(durationMS);
#endif

//    IRQset_get_status();

    switch (power_mode) {

        case HAL_BSP_POWER_OFF:
        case HAL_BSP_POWER_DEEP_SLEEP: {
            /*Disables the Power Voltage Detector(PVD) */                
            HAL_PWR_DisablePVD( );
            /* Enters StandBy mode */
            HAL_PWR_EnterSTANDBYMode();
            /* If exit standby mode then the RAM has been lost. Reboot cleanly. */
            hal_system_reset();
            break;
        }
        case HAL_BSP_POWER_SLEEP: {
#ifdef EXT_GPIO_FOR_SLEEP_TESTING
            hal_gpio_write(EXT_SLEEP_OUTPUT, 0);
#endif
            /*Disables the Power Voltage Detector(PVD) */                 
            //HAL_PWR_DisablePVD( );
            HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);

            /* Enable the Internal High Speed oscillator (HSI). */
            __HAL_RCC_HSI_ENABLE();
            break;
        }
        case HAL_BSP_POWER_WFI: {
#ifdef EXT_GPIO_FOR_WFI_TESTING
            hal_gpio_write(EXT_WFI_OUTPUT, 0);
#endif
            HAL_PWREx_EnableLowPowerRunMode();
            HAL_PWR_EnterSLEEPMode(PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFI);
            HAL_PWREx_DisableLowPowerRunMode();
            /* Clock is not interuppted in SLEEP mode, no need to restart it */
            break;
        }
        case HAL_BSP_POWER_ON:
        default: {
            
            break;
        }
    }

#ifdef EXT_GPIO_FOR_SLEEP_TESTING
    hal_gpio_write(EXT_SLEEP_OUTPUT, 1);
#endif
    
#ifdef EXT_GPIO_FOR_WFI_TESTING
    hal_gpio_write(EXT_WFI_OUTPUT, 1);
#endif

#if MYNEWT_VAL(OS_TICKLESS)
    /* exit tickless low power mode */
    stm32_tickless_stop(durationMS);
#endif

#if MYNEWT_VAL(TIMER_0)
    //NVIC_EnableIRQ(TIM2_IRQn);
    //hal_timer_init(0, TIM2);
#endif


}