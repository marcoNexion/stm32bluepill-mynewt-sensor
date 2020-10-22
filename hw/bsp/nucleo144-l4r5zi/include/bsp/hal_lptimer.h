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
#ifndef H_HAL_LPTIMER_H
#define H_HAL_LPTIMER_H

#include <inttypes.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialise internal LPTIMER1
 *
 * @param none
 *
 * @return none
 * 
 * As prerequisite, initialise proper clock
 * in hal_system_clock_start() like that :
 * 
 * RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI ... ;
 * RCC_OscInitStruct.LSIState = RCC_LSI_ON;
 * ...
 *  PeriphClkInit.PeriphClockSelection =  RCC_PERIPHCLK_LPTIM1 ...
 * ...
 * PeriphClkInit.Lptim1ClockSelection = RCC_LPTIM1CLKSOURCE_LSI;
 * ...
 * 
 */
void hal_lptimer_init(void);


/**
 * Start internal LPTIMER1
 *
 * @param timeMs : timeout
 *
 * @return none
 * 
 * **/
void hal_lptimer_start(uint16_t timeMs);


/**
 * Stop internal LPTIMER1
 *
 * @param timeMs : none
 *
 * @return none
 * 
 * **/
void hal_lptimer_stop(void);


/**
 * Clear only LPTIM_IT_CMPM bit 
 * specificaly to this LP timer 
 * operating mode
 *
 * @param timeMs : none
 *
 * @return none
 * 
 * **/
void hal_lptimer_clear_pending_irq(void);

/**
 * Get the actual counter value
 *
 * @param none
 *
 * @return counter value (lsb = 1ms)
 * 
 * **/
uint16_t hal_lptimer_get_elapsed_time(void);

#ifdef __cplusplus
}
#endif

#endif  /* H_HAL_LPTIMER_H */