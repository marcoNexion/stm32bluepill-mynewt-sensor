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
#ifndef H_HAL_BSP_POWER_HANDLER_H
#define H_HAL_BSP_POWER_HANDLER_H

#include <inttypes.h>

#if MYNEWT_VAL(BSP_POWER_SETUP)
#include "lowpower_mgnt/lowpower_mgnt.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

#if MYNEWT_VAL(BSP_POWER_SETUP)
void hal_bsp_power_hooks(LP_HOOK_t getMode, LP_HOOK_t enter, LP_HOOK_t exit);
#else
void hal_bsp_power_hooks(void* getMode, void* enter, void* exit);
#endif

// Halt board and MCU in lowest power mode possible. Never returns
void hal_bsp_halt();

/** functions called from OS (os.c and hal_os_tick.c) */
int hal_bsp_power_handler_get_mode(os_time_t ticks);
void hal_bsp_power_handler_sleep_enter(int nextMode);
void hal_bsp_power_handler_sleep_exit(int lastMode);


#ifdef __cplusplus
}
#endif

#endif