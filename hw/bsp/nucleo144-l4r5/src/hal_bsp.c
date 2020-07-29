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


#if MYNEWT_VAL(UART_0)
#include <uart/uart.h>
#include <uart_hal/uart_hal.h>
#endif


#if MYNEWT_VAL(UART_0)
static struct uart_dev hal_uart0;

static const struct stm32_uart_cfg uart_cfg[UART_CNT] = {
    [0] = {
        .suc_uart = USART2,
        .suc_rcc_reg = &RCC->APB1ENR1,
        .suc_rcc_dev = RCC_APB1ENR1_USART2EN,
        .suc_pin_tx = MCU_GPIO_PORTA(2),
        .suc_pin_rx = MCU_GPIO_PORTA(3),
        .suc_pin_rts = -1,
        .suc_pin_cts = -1,
        .suc_pin_af = GPIO_AF7_USART2,
        .suc_irqn = USART2_IRQn
    }
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

void
hal_bsp_init(void)
{
    int rc;

    (void)rc;

    /* Make sure system clocks have started. */
    hal_system_clock_start();

#if MYNEWT_VAL(UART_0)
    rc = os_dev_create((struct os_dev *) &hal_uart0, "uart0",
      OS_DEV_INIT_PRIMARY, 0, uart_hal_init, (void *)&uart_cfg[0]);
    assert(rc == 0);
#endif

#if MYNEWT_VAL(TIMER_0)
    rc = hal_timer_init(0, NULL);
    assert(rc == 0);
#endif

#if (MYNEWT_VAL(OS_CPUTIME_TIMER_NUM) >= 0)
    rc = os_cputime_init(MYNEWT_VAL(OS_CPUTIME_FREQ));
    assert(rc == 0);
#endif

    /* Initialize additional BSP peripherals here. */
}
