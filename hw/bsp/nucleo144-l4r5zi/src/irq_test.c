#include <inttypes.h>
#include <mcu/cmsis_nvic.h>
#include "stm32l4xx_hal.h"

#define NELEMS(x)  (sizeof(x) / sizeof((x)[0]))

typedef struct _IRQset{
	IRQn_Type type;
	uint32_t priority_grouping;
	uint32_t priority;			
	uint32_t is_enabled;			
	uint32_t is_active;				
	uint32_t is_pending;			
}_IRQset;


_IRQset IRQ_Causes[94] = {

	{ NonMaskableInt_IRQn         , 0, 0, 0, 0, 0 },
	{ HardFault_IRQn              , 0, 0, 0, 0, 0 },
	{ MemoryManagement_IRQn       , 0, 0, 0, 0, 0 },
	{ BusFault_IRQn               , 0, 0, 0, 0, 0 },
	{ UsageFault_IRQn             , 0, 0, 0, 0, 0 },
	{ SVCall_IRQn                 , 0, 0, 0, 0, 0 },
	{ DebugMonitor_IRQn           , 0, 0, 0, 0, 0 },
	{ PendSV_IRQn                 , 0, 0, 0, 0, 0 },
	{ SysTick_IRQn                , 0, 0, 0, 0, 0 },
	{ WWDG_IRQn                   , 0, 0, 0, 0, 0 },
	{ PVD_PVM_IRQn                , 0, 0, 0, 0, 0 },
	{ TAMP_STAMP_IRQn             , 0, 0, 0, 0, 0 },
	{ RTC_WKUP_IRQn               , 0, 0, 0, 0, 0 },
	{ FLASH_IRQn                  , 0, 0, 0, 0, 0 },
	{ RCC_IRQn                    , 0, 0, 0, 0, 0 },
	{ EXTI0_IRQn                  , 0, 0, 0, 0, 0 },
	{ EXTI1_IRQn                  , 0, 0, 0, 0, 0 },
	{ EXTI2_IRQn                  , 0, 0, 0, 0, 0 },
	{ EXTI3_IRQn                  , 0, 0, 0, 0, 0 },
	{ EXTI4_IRQn                  , 0, 0, 0, 0, 0 },
	{ DMA1_Channel1_IRQn          , 0, 0, 0, 0, 0 },
	{ DMA1_Channel2_IRQn          , 0, 0, 0, 0, 0 },
	{ DMA1_Channel3_IRQn          , 0, 0, 0, 0, 0 },
	{ DMA1_Channel4_IRQn          , 0, 0, 0, 0, 0 },
	{ DMA1_Channel5_IRQn          , 0, 0, 0, 0, 0 },
	{ DMA1_Channel6_IRQn          , 0, 0, 0, 0, 0 },
	{ DMA1_Channel7_IRQn          , 0, 0, 0, 0, 0 },
	{ ADC1_IRQn                   , 0, 0, 0, 0, 0 },
	{ CAN1_TX_IRQn                , 0, 0, 0, 0, 0 },
	{ CAN1_RX0_IRQn               , 0, 0, 0, 0, 0 },
	{ CAN1_RX1_IRQn               , 0, 0, 0, 0, 0 },
	{ CAN1_SCE_IRQn               , 0, 0, 0, 0, 0 },
	{ EXTI9_5_IRQn                , 0, 0, 0, 0, 0 },
	{ TIM1_BRK_TIM15_IRQn         , 0, 0, 0, 0, 0 },
	{ TIM1_UP_TIM16_IRQn          , 0, 0, 0, 0, 0 },
	{ TIM1_TRG_COM_TIM17_IRQn     , 0, 0, 0, 0, 0 },
	{ TIM1_CC_IRQn                , 0, 0, 0, 0, 0 },
	{ TIM2_IRQn                   , 0, 0, 0, 0, 0 },
	{ TIM3_IRQn                   , 0, 0, 0, 0, 0 },
	{ TIM4_IRQn                   , 0, 0, 0, 0, 0 },
	{ I2C1_EV_IRQn                , 0, 0, 0, 0, 0 },
	{ I2C1_ER_IRQn                , 0, 0, 0, 0, 0 },
	{ I2C2_EV_IRQn                , 0, 0, 0, 0, 0 },
	{ I2C2_ER_IRQn                , 0, 0, 0, 0, 0 },
	{ SPI1_IRQn                   , 0, 0, 0, 0, 0 },
	{ SPI2_IRQn                   , 0, 0, 0, 0, 0 },
	{ USART1_IRQn                 , 0, 0, 0, 0, 0 },
	{ USART2_IRQn                 , 0, 0, 0, 0, 0 },
	{ USART3_IRQn                 , 0, 0, 0, 0, 0 },
	{ EXTI15_10_IRQn              , 0, 0, 0, 0, 0 },
	{ RTC_Alarm_IRQn              , 0, 0, 0, 0, 0 },
	{ DFSDM1_FLT3_IRQn            , 0, 0, 0, 0, 0 },
	{ TIM8_BRK_IRQn               , 0, 0, 0, 0, 0 },
	{ TIM8_UP_IRQn                , 0, 0, 0, 0, 0 },
	{ TIM8_TRG_COM_IRQn           , 0, 0, 0, 0, 0 },
	{ TIM8_CC_IRQn                , 0, 0, 0, 0, 0 },
	{ FMC_IRQn                    , 0, 0, 0, 0, 0 },
	{ SDMMC1_IRQn                 , 0, 0, 0, 0, 0 },
	{ TIM5_IRQn                   , 0, 0, 0, 0, 0 },
	{ SPI3_IRQn                   , 0, 0, 0, 0, 0 },
	{ UART4_IRQn                  , 0, 0, 0, 0, 0 },
	{ UART5_IRQn                  , 0, 0, 0, 0, 0 },
	{ TIM6_DAC_IRQn               , 0, 0, 0, 0, 0 },
	{ TIM7_IRQn                   , 0, 0, 0, 0, 0 },
	{ DMA2_Channel1_IRQn          , 0, 0, 0, 0, 0 },
	{ DMA2_Channel2_IRQn          , 0, 0, 0, 0, 0 },
	{ DMA2_Channel3_IRQn          , 0, 0, 0, 0, 0 },
	{ DMA2_Channel4_IRQn          , 0, 0, 0, 0, 0 },
	{ DMA2_Channel5_IRQn          , 0, 0, 0, 0, 0 },
	{ DFSDM1_FLT0_IRQn            , 0, 0, 0, 0, 0 },
	{ DFSDM1_FLT1_IRQn            , 0, 0, 0, 0, 0 },
	{ DFSDM1_FLT2_IRQn            , 0, 0, 0, 0, 0 },
	{ COMP_IRQn                   , 0, 0, 0, 0, 0 },
	{ LPTIM1_IRQn                 , 0, 0, 0, 0, 0 },
	{ LPTIM2_IRQn                 , 0, 0, 0, 0, 0 },
	{ OTG_FS_IRQn                 , 0, 0, 0, 0, 0 },
	{ DMA2_Channel6_IRQn          , 0, 0, 0, 0, 0 },
	{ DMA2_Channel7_IRQn          , 0, 0, 0, 0, 0 },
	{ LPUART1_IRQn                , 0, 0, 0, 0, 0 },
	{ OCTOSPI1_IRQn               , 0, 0, 0, 0, 0 },
	{ I2C3_EV_IRQn                , 0, 0, 0, 0, 0 },
	{ I2C3_ER_IRQn                , 0, 0, 0, 0, 0 },
	{ SAI1_IRQn                   , 0, 0, 0, 0, 0 },
	{ SAI2_IRQn                   , 0, 0, 0, 0, 0 },
	{ OCTOSPI2_IRQn               , 0, 0, 0, 0, 0 },
	{ TSC_IRQn                    , 0, 0, 0, 0, 0 },
	{ RNG_IRQn                    , 0, 0, 0, 0, 0 },
	{ FPU_IRQn                    , 0, 0, 0, 0, 0 },
	{ CRS_IRQn                    , 0, 0, 0, 0, 0 },
	{ I2C4_EV_IRQn                , 0, 0, 0, 0, 0 },
	{ I2C4_ER_IRQn                , 0, 0, 0, 0, 0 },
	{ DCMI_IRQn                   , 0, 0, 0, 0, 0 },
	{ DMA2D_IRQn                  , 0, 0, 0, 0, 0 },
	{ DMAMUX1_OVR_IRQn            , 0, 0, 0, 0, 0 }
};


void IRQset_get_status(void){
	_IRQset *currentIRQ;

	for(int i=0; i<NELEMS(IRQ_Causes); i++){
		currentIRQ = &IRQ_Causes[i];
		currentIRQ->priority_grouping	= NVIC_GetPriorityGrouping();
		currentIRQ->priority			= (NVIC_GetPriority(currentIRQ->type));
		currentIRQ->is_enabled			= (NVIC_GetEnableIRQ(currentIRQ->type));
		currentIRQ->is_active			= (NVIC_GetActive(currentIRQ->type));
		currentIRQ->is_pending			= (NVIC_GetPendingIRQ(currentIRQ->type));
	
		if(currentIRQ->is_enabled){
			asm("nop");
			asm("nop");
			asm("nop");
			//NVIC_DisableIRQ(currentIRQ->type);
		}
	}
}

void IRQset_disable_all(void){
	_IRQset *currentIRQ;

	for(int i=8; i<NELEMS(IRQ_Causes); i++){
		currentIRQ = &IRQ_Causes[i];
		currentIRQ->priority_grouping	= NVIC_GetPriorityGrouping();
		currentIRQ->priority			= (NVIC_GetPriority(currentIRQ->type));
		currentIRQ->is_enabled			= (NVIC_GetEnableIRQ(currentIRQ->type));
		currentIRQ->is_active			= (NVIC_GetActive(currentIRQ->type));
		currentIRQ->is_pending			= (NVIC_GetPendingIRQ(currentIRQ->type));

		if(currentIRQ->is_enabled){
			NVIC_DisableIRQ(currentIRQ->type);
		}

	}
}