#include <inttypes.h>
#include <assert.h>
#include "syscfg/syscfg.h"
#include "sysflash/sysflash.h"
#include "hal/hal_system.h"
#include "hal/hal_flash_int.h"
#include "hal/hal_timer.h"
#include "hal/hal_bsp.h"
#include "os/os.h"
#include "bsp/bsp.h"

#include <stm32l4r5xx.h>
#include <stm32l4xx_hal_rcc.h>
#include <stm32l4xx_hal_pwr.h>
#include <stm32l4xx_hal_flash.h>
#include <stm32l4xx_hal_gpio_ex.h>
#include <mcu/stm32l4_bsp.h>
#include "mcu/stm32l4xx_mynewt_hal.h"
#include "mcu/stm32_hal.h"
#include "hal/hal_i2c.h"
#include "hal/hal_spi.h"
#include "hal/hal_gpio.h"

extern uint32_t __HeapBase;
extern uint32_t __HeapLimit;

#if MYNEWT_VAL(SPI_0_MASTER) || MYNEWT_VAL(SPI_0_SLAVE)
#include <hal/hal_spi.h>
#endif

#if MYNEWT_VAL(UART_0) || MYNEWT_VAL(UART_1) || MYNEWT_VAL(UART_2)
#include <uart/uart.h>
#include <uart_hal/uart_hal.h>
#endif

/* UartBitbang is a software uart on a gpio - initialised by the bitbang package in sysinit */
#if MYNEWT_VAL(UART_BITBANG)
#include <uart/uart.h>
#include <uart_bitbang/uart_bitbang.h>

static struct uart_dev hal_uartbitbang;
static const struct uart_bitbang_conf uartbitbang_cfg = {
    .ubc_rxpin = BSP_UART_BITBANG_RX,
    .ubc_txpin = BSP_UART_BITBANG_TX,
    .ubc_cputimer_freq = MYNEWT_VAL(OS_CPUTIME_FREQ),
};
#endif


#if MYNEWT_VAL(UART_1)
static struct uart_dev hal_uart_dbg;

static const struct stm32_uart_cfg uart_dbg_cfg = {
    .suc_uart = LPUART1,
    .suc_rcc_reg = &RCC->APB1ENR2,
    .suc_rcc_dev = RCC_APB1ENR2_LPUART1EN,
    .suc_pin_tx = MCU_GPIO_PORTC(4),
    .suc_pin_rx = MCU_GPIO_PORTC(5),
    .suc_pin_rts = -1,
    .suc_pin_cts = -1,
    .suc_pin_af = GPIO_AF8_LPUART1,
    .suc_irqn = LPUART1_IRQn
};
#endif

#if MYNEWT_VAL(UART_0)

static struct uart_dev hal_uart_bc95;

static const struct stm32_uart_cfg uart_bc95_cfg = {
#if 0
    //doesn't work on USART1 channel...
    //TODO : fix it !!!
    .suc_uart = USART1,
    .suc_rcc_reg = &RCC->APB2ENR,
    .suc_rcc_dev = RCC_APB2ENR_USART1EN,
    .suc_pin_tx = MCU_GPIO_PORTG(9),
    .suc_pin_rx = MCU_GPIO_PORTG(10),
    .suc_pin_rts = -1,
    .suc_pin_cts = -1,
    .suc_pin_af = GPIO_AF7_USART1,
    .suc_irqn = USART1_IRQn
#else
    .suc_uart = USART2,
    .suc_rcc_reg = &RCC->APB1ENR1,
    .suc_rcc_dev = RCC_APB1ENR1_USART2EN,
    .suc_pin_tx = MCU_GPIO_PORTA(2),
    .suc_pin_rx = MCU_GPIO_PORTA(3),
    .suc_pin_rts = -1,
    .suc_pin_cts = -1,
    .suc_pin_af = GPIO_AF7_USART2,
    .suc_irqn = USART2_IRQn
#endif

};
#endif

#if MYNEWT_VAL(UART_2)
static struct uart_dev hal_uart_gps_cfg;

static const struct stm32_uart_cfg uart_gps_cfg = {
#if 0    
    .suc_uart = USART3,
    .suc_rcc_reg = &RCC->APB1ENR1,
    .suc_rcc_dev = RCC_APB1ENR1_USART3EN,
    .suc_pin_tx = MCU_GPIO_PORTD(8),
    .suc_pin_rx = MCU_GPIO_PORTD(9),
    .suc_pin_rts = -1,
    .suc_pin_cts = -1,
    .suc_pin_af = GPIO_AF7_USART3,
    .suc_irqn = USART3_IRQn
#else
    .suc_uart = USART3,
    .suc_rcc_reg = &RCC->APB1ENR1,
    .suc_rcc_dev = RCC_APB1ENR1_USART3EN,
    .suc_pin_tx = MCU_GPIO_PORTD(8),
    .suc_pin_rx = MCU_GPIO_PORTD(9),
    .suc_pin_rts = -1,
    .suc_pin_cts = -1,
    .suc_pin_af = GPIO_AF7_USART3,
    .suc_irqn = USART3_IRQn
    
#endif
};
#endif


/* NOTE: ACCELEROMETER ADXL362 */
/* The numbers in the switch below are offset by 1, */
/* because the HALs index SPI ports from 0.*/
#if MYNEWT_VAL(SPI_1)
struct stm32_hal_spi_cfg spi1_cfg = {
    .sck_pin  = MCU_GPIO_PORTA(5),
    .miso_pin = MCU_GPIO_PORTA(11),
    .mosi_pin = MCU_GPIO_PORTA(12),
    .irq_prio = 2,
};
#endif

/** What memory to include in coredump. */
static const struct hal_bsp_mem_dump dump_cfg[] = {
    [0] = {
        .hbmd_start = &_ram_start,
        .hbmd_size = RAM_SIZE,
    }
};

const struct hal_bsp_mem_dump *
hal_bsp_core_dump(int *area_cnt)
{
    *area_cnt = sizeof(dump_cfg) / sizeof(dump_cfg[0]);
    return dump_cfg;
}

/**
 * Retrieves the flash device with the specified ID.  Returns NULL if no such
 * device exists.
 */
const struct hal_flash *
hal_bsp_flash_dev(uint8_t id)
{
    switch (id) {
    case 0:
        /* MCU internal flash. */
        /* XXX: Return pointer to MCU's flash object. */
        return NULL;

    default:
        /* External flash.  Assume not present in this BSP. */
        return NULL;
    }
}

/**
 * Retrieves the configured priority for the given interrupt. If no priority
 * is configured, returns the priority passed in.
 *
 * @param irq_num               The IRQ being queried.
 * @param pri                   The default priority if none is configured.
 *
 * @return uint32_t             The specified IRQ's priority.
 */
uint32_t
hal_bsp_get_nvic_priority(int irq_num, uint32_t pri)
{
    return pri;
}






void hal_system_clock_start(void){

    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Configure the main internal regulator output voltage
     */
    if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) != HAL_OK) {
        assert(0);
    }

    /**Configure LSE Drive Capability
     */
    HAL_PWR_EnableBkUpAccess();

    __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

    /**Initializes the CPU, AHB and APB busses clocks
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_MSI | RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.LSEState = RCC_LSE_OFF;
    RCC_OscInitStruct.LSIState = RCC_LSI_ON;
    RCC_OscInitStruct.MSIState = RCC_MSI_ON;
    RCC_OscInitStruct.MSICalibrationValue = 0;
    RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
    RCC_OscInitStruct.PLL.PLLM = 1;
    RCC_OscInitStruct.PLL.PLLN = 60;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        assert(0);
    }

    /**Initializes the CPU, AHB and APB busses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                    | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
        assert(0);
    }

    PeriphClkInit.PeriphClockSelection =    RCC_PERIPHCLK_USART1 | 
                                            RCC_PERIPHCLK_USART2 | 
                                            RCC_PERIPHCLK_USART3 | 
                                            RCC_PERIPHCLK_LPUART1 | 
                                            RCC_PERIPHCLK_ADC |
    #if MYNEWT_VAL(OS_TICKLESS)
                                            RCC_PERIPHCLK_LPTIM1 |
    #endif
                                            RCC_PERIPHCLK_USB;

    PeriphClkInit.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_PCLK1;
    PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
    PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_HSI;
    PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
    PeriphClkInit.AdcClockSelection    = RCC_ADCCLKSOURCE_PLLSAI1;
    #if MYNEWT_VAL(OS_TICKLESS)
    PeriphClkInit.Lptim1ClockSelection = RCC_LPTIM1CLKSOURCE_LSI;
    #endif
    PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLLSAI1;
    PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
    PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
    PeriphClkInit.PLLSAI1.PLLSAI1N = 24;
    PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV2;
    PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
    PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
    PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
    
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
        assert(0);
    }

    /**Enable MSI Auto calibration
     */
    HAL_RCCEx_EnableMSIPLLMode();

    HAL_RCCEx_WakeUpStopCLKConfig(RCC_STOP_WAKEUPCLOCK_MSI);

    HAL_RCC_MCOConfig( RCC_MCO1, RCC_MCO1SOURCE_HSI, RCC_MCODIV_1);


}





void
hal_bsp_init(void)
{
    int rc;

    (void)rc;

    //TODO : do heap limt intialization into startup_XXX.s
    _sbrkInit((char *)&__HeapBase, (char *)&__HeapLimit);

    /* Make sure system clocks have started. */
    hal_system_clock_start();

#if MYNEWT_VAL(UART_1)
    rc = os_dev_create((struct os_dev *) &hal_uart_dbg, "uart1",
      OS_DEV_INIT_PRIMARY, 0, uart_hal_init, (void *)&uart_dbg_cfg);
    assert(rc == 0);
#endif

#if MYNEWT_VAL(UART_0)
    rc = os_dev_create((struct os_dev *) &hal_uart_bc95, "uart0",
      OS_DEV_INIT_PRIMARY, 0, uart_hal_init, (void *)&uart_bc95_cfg);
    assert(rc == 0);
#endif

#if MYNEWT_VAL(UART_2)
    rc = os_dev_create((struct os_dev *) &hal_uart_gps_cfg, "uart2",
      OS_DEV_INIT_PRIMARY, 0, uart_hal_init, (void *)&uart_gps_cfg);
    assert(rc == 0);
#else
    hal_gpio_init_in(MCU_GPIO_PORTD(8), HAL_GPIO_PULL_NONE);
    hal_gpio_init_in(MCU_GPIO_PORTD(9), HAL_GPIO_PULL_NONE);
#endif

    /* Initialised by bitbang package in sysinit */
#if MYNEWT_VAL(UART_BITBANG)
    rc = os_dev_create((struct os_dev *) &hal_uartbitbang, "uart3",
      OS_DEV_INIT_PRIMARY, 0, uart_bitbang_init, (void *)&uartbitbang_cfg);
    assert(rc == 0);
#else
    hal_gpio_init_in(BSP_UART_BITBANG_RX, HAL_GPIO_PULL_NONE);
    hal_gpio_init_in(BSP_UART_BITBANG_TX, HAL_GPIO_PULL_NONE);
#endif

#if MYNEWT_VAL(OS_CPUTIME_TIMER_REF)

#if MYNEWT_VAL(TIMER_0)
    rc = hal_timer_init(0, TIM2);
    assert(rc == 0);
#endif

#if (MYNEWT_VAL(OS_CPUTIME_TIMER_NUM) >= 0)
    rc = os_cputime_init(MYNEWT_VAL(OS_CPUTIME_FREQ));
    assert(rc == 0);
#endif

#endif

#if MYNEWT_VAL(SPI_1)
    rc = hal_spi_init(0, &spi1_cfg, HAL_SPI_TYPE_MASTER);
    assert(rc == 0);
#endif

}

