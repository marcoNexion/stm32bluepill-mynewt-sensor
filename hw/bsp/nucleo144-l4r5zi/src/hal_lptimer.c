#include <mcu/cmsis_nvic.h>
#include "console/console.h"
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_lptim.h"
#include "stm32l4xx_hal_rcc.h"


#ifdef EXT_GPIO_FOR_IRQ_TESTING
#include "bsp/bsp.h"
#include "hal/hal_gpio.h"
#define EXT_OUTPUT  MCU_GPIO_PORTC(9)
#endif

static LPTIM_HandleTypeDef hlptimer;

void LPTIM_IRQ_Handler(void){
#ifdef HAL_IRQ_LPTIMER_HANDLER
	HAL_LPTIM_IRQHandler(&hlptimer);
#endif
#ifdef EXT_GPIO_FOR_IRQ_TESTING
	hal_gpio_toggle(EXT_OUTPUT);
#endif
}

/*HAL_LPTIM_MspInit is called by HAL into HAL_LPTIM_Init() */
void HAL_LPTIM_MspInit(LPTIM_HandleTypeDef *hlptim){

    /*Stop the timers at debugger.*/
	//__HAL_DBGMCU_FREEZE_LPTIM1();
	/*LPTIM used as tickless may have the same priority than Systick */
	NVIC_SetPriority(LPTIM1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0)); //
	/* Note : IRQ handler is not configured into hal. Do it here.*/
	NVIC_SetVector(LPTIM1_IRQn, (uint32_t)LPTIM_IRQ_Handler);
	/*Enable IRQ now forever */
	NVIC_EnableIRQ(LPTIM1_IRQn);
}

#ifdef HAL_IRQ_LPTIMER_HANDLER
void HAL_LPTIM_CompareMatchCallback(LPTIM_HandleTypeDef *hlptim){
	
}
#endif



void hal_lptimer_init(void)
{
    __HAL_RCC_LPTIM1_CLK_ENABLE();

    /* Clk_in : LSI @32KHz */
    /* Prescaler : 32 */
    hlptimer.Instance = LPTIM1;
    hlptimer.Init.Clock.Source    = LPTIM_CLOCKSOURCE_APBCLOCK_LPOSC;
    hlptimer.Init.Clock.Prescaler = LPTIM_PRESCALER_DIV32; 
    hlptimer.Init.Trigger.Source = LPTIM_TRIGSOURCE_SOFTWARE;
    hlptimer.Init.OutputPolarity = LPTIM_OUTPUTPOLARITY_HIGH;
    hlptimer.Init.UpdateMode = LPTIM_UPDATE_IMMEDIATE;
    hlptimer.Init.CounterSource = LPTIM_COUNTERSOURCE_INTERNAL;

    /* Initialize LPTIM peripheral according to the passed parameters */
    if (HAL_LPTIM_Init(&hlptimer) != HAL_OK){
        assert(0);
    }

#ifdef EXT_GPIO_FOR_IRQ_TESTING
    hal_gpio_init_out(EXT_OUTPUT, 0);
#endif

}

void hal_lptimer_start(uint16_t timeMs)
{
    assert(timeMs<0xFFFF);

    if (HAL_LPTIM_TimeOut_Start_IT(&hlptimer, 0xFFFF, timeMs) != HAL_OK)
    {
        assert(0);
    }
}

void hal_lptimer_stop(void)
{
    if (HAL_LPTIM_TimeOut_Stop_IT(&hlptimer) != HAL_OK)
    {
        assert(0);
    }
}

uint16_t hal_lptimer_get_elapsed_time(void){

    return HAL_LPTIM_ReadCounter(&hlptimer);
}