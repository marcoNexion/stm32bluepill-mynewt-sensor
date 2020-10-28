/**
 * Copyright 2019 Wyres
 * Licensed under the Apache License, Version 2.0 (the "License"); 
 * you may not use this file except in compliance with the License. 
 * You may obtain a copy of the License at
 *    http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, 
 * software distributed under the License is distributed on 
 * an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, 
 * either express or implied. See the License for the specific 
 * language governing permissions and limitations under the License.
*/
/**
 * Low power manager. Used to change power modes
 * Can register callbacks to inform of changes to mode, eg GPIO mgr will do this to disable GPIOS etc
 */
#include "os/os.h"
#include <assert.h>
#include <hal/hal_bsp.h>
#include "lowpower_mgnt/lowpower_mgnt.h"
#include "bsp/hal_power_mgnt.h"
#include "bsp/hal_bsp_power_handler.h"

#define MAX_LPCBFNS             MYNEWT_VAL(MAX_LPCBFNS)

/* wrapped definitions for different power mode     */
/* on this specific bsp                             */
#define HAL_BSP_LP_RUN          HAL_BSP_POWER_ON
#define HAL_BSP_LP_OFF          HAL_BSP_POWER_OFF
#define HAL_BSP_LP_DEEPSLEEP    HAL_BSP_POWER_SLEEP
#define HAL_BSP_LP_SLEEP        HAL_BSP_POWER_WFI
#define HAL_BSP_LP_DOZE         HAL_BSP_POWER_WFI


// Registered callbacks fns
static struct lp_ctx {
    struct {
        LP_MODE_t desiredMode;  // Current sleep mode required
        LP_CBFN_t cb;           // callback to be informed of changes
        void *cb_arg;
    } lpUsers[MAX_LPCBFNS];
    uint8_t deviceCnt;
    LP_MODE_t sleepMode;
} _ctx = {
    .deviceCnt=0,
    .sleepMode=LP_SLEEP,
};
static LP_MODE_t calcNextSleepMode();

// Initialise low power manager
void LPMgr_init(void) {
    // This is a generic low power manager. So it delegates this stuff to BSP/MCU code...
#if MYNEWT_VAL(BSP_POWER_SETUP)
    assert((MAX_LPCBFNS > 0) && (MAX_LPCBFNS < LP_UNITIALIZED_ID));

    hal_bsp_power_hooks(LPMgr_getMode, LPMgr_entersleep, LPMgr_exitsleep);
#endif
}

// Register to be told when we change mode
LP_ID_t LPMgr_register(LP_CBFN_t cb, void *cb_arg) {
    assert(_ctx.deviceCnt < MAX_LPCBFNS);
    uint8_t id = _ctx.deviceCnt;
    _ctx.lpUsers[_ctx.deviceCnt].cb=cb;     // May be NULL if user only changes the desired LP mode...
    _ctx.lpUsers[_ctx.deviceCnt].cb_arg=cb_arg;                 //callback may carry arg
    _ctx.lpUsers[_ctx.deviceCnt].desiredMode=LP_DEEPSLEEP;      // start by assuming everyone is ok with deep sleep
    _ctx.deviceCnt++;
    return id;
}
// THe level of sleeping when someone (the OS) asks to enter 'low power mode'
void LPMgr_setLPMode(LP_ID_t id, LP_MODE_t m) {
    assert(id>=0 && id < MAX_LPCBFNS);

    /* in case of re-activating, set new mode               */
    /* and generates callback to wake it up immediatley     */
//    LP_MODE_t prevmode = _ctx.lpUsers[id].desiredMode;
/*
    if((m <= LP_RUN) && (_ctx.lpUsers[id].desiredMode > LP_RUN))
    {
        if (_ctx.lpUsers[id].cb!=NULL) {
            (*_ctx.lpUsers[id].cb)(_ctx.sleepMode, LP_RUN);
        }
    }
*/
    // Set this guy's desired mode
    _ctx.lpUsers[id].desiredMode = m;
    _ctx.sleepMode = calcNextSleepMode();

    // new mode taken into account next time we sleep as BSP should call LPMgr_getMode() to get a HAL related sleep level
/* for debug only and be careful
    if (prevmode != m) {
        log_warn("LP::%d:%d", m, _ctx.sleepMode);
    }
*/


}
LP_MODE_t LPMgr_getNextLPMode() {
    _ctx.sleepMode = calcNextSleepMode();
    return _ctx.sleepMode;
}

/* Hook functions round the 'idle' method. These allow the code to pause/resume external hw or other tasks.
 * These functions are called with IRQs disabled, and with a minimal stack size (mynewt idle stack). 
 * DO NOT LOG, OR SLEEP, OR DO TOO MUCH STUFF IN THEM OR BAD STUFF WILL HAPPEN
 */

/* getMode returns indication of level of sleep to use eg light sleep (1) or a deep sleep (2) or even a OFF mode (3)
 * These should be the HAL_BSP_POWER_XXX defines from hap_power.h. The value is passed to the MUC specific 'sleep' 
 * method hal_bsp_power_state() to setup the actual sleep.
 */
int LPMgr_getMode() {
    if (_ctx.sleepMode == LP_OFF) {
        return HAL_BSP_LP_OFF;       // Restart on RTC
    } else if (_ctx.sleepMode == LP_DEEPSLEEP) {
        return HAL_BSP_LP_DEEPSLEEP;     // NOTE : we don't use HAL_BSP_POWER_DEEP_SLEEP as this implies RAM loss
    } else if (_ctx.sleepMode == LP_SLEEP) {
        return HAL_BSP_LP_SLEEP;       // Only CPU pause bus still run
    } else {
        // LP_DOZE normally
        return HAL_BSP_LP_DOZE;       // Only CPU pause
    }
}
/** signal sleep entry (outside critical region). Returns anticipated sleep level.*/
int LPMgr_entersleep() {
    // tell all registered CBs we sleep (and at what level)
    for(int i=0;i<_ctx.deviceCnt;i++) {
        if (_ctx.lpUsers[i].cb!=NULL) {
            (*_ctx.lpUsers[i].cb)(LP_RUN, _ctx.sleepMode, _ctx.lpUsers[i].cb_arg);
        }
    }
    return LPMgr_getMode();
}
/** signale sleep exit (outside critical region) */
int LPMgr_exitsleep() {
    // Tell everyone who cares
    for(int i=0;i<_ctx.deviceCnt;i++) {
        if (_ctx.lpUsers[i].cb!=NULL) {
            (*_ctx.lpUsers[i].cb)(_ctx.sleepMode, LP_RUN, _ctx.lpUsers[i].cb_arg);
        }
    }
    return HAL_BSP_LP_RUN;
}

// Internals
static LP_MODE_t calcNextSleepMode() {
    LP_MODE_t ret = LP_OFF;     // the deepest mode, man...
    for(int i=0;i<_ctx.deviceCnt;i++) {
        // check each user, anyone needing 'less' sleep is priority
        if (_ctx.lpUsers[i].desiredMode<ret) {
            ret = _ctx.lpUsers[i].desiredMode;
        }
    }
    return ret;
}