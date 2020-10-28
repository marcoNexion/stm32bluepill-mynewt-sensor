//  Ported from https://github.com/ARMmbed/ATParser//blob/269f14532b98442669c50383782cbce1c67aced5/BufferedSerial/BufferedSerial.cpp
/**
 * @file    BufferedSerial.cpp
 * @brief   Software Buffer - Extends mbed Serial functionallity adding irq driven TX and RX
 * @author  sam grove
 * @version 1.0
 * @see
 *
 * Copyright (c) 2013
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stdarg.h>
#include <assert.h>
#include <os/os.h>
#include <console/console.h>
#include "buffered_serial/buffered_serial.h"


extern "C" int BufferedPrintfC(void *stream, int size, const char* format, va_list arg);

#if MYNEWT_VAL(SERIAL_LPMODE)
// Callback from low power manager about change of mode
//deal with : LP_RUN, LP_DOZE, LP_SLEEP, LP_DEEPSLEEP, LP_OFF
static void serial_lp_change(  LP_MODE_t prevmode, 
                               LP_MODE_t newmode,
                               void * arg) {

    BufferedSerial *serial = (BufferedSerial *) arg;

    assert(serial->_LPMgr_id != LP_UNITIALIZED_ID);

    if (prevmode>=LP_DEEPSLEEP && newmode <LP_DEEPSLEEP) {
       // TURN ON : wake up & init its periphs
        os_dev_resume((struct os_dev *)&serial->_uartdev);
                      
    } else if (prevmode<LP_DEEPSLEEP && newmode >= LP_DEEPSLEEP) {
        // TURN OFF : go to sleep & deinit its periphs
        os_dev_suspend((struct os_dev *)&serial->_uartdev, os_time_get(), true);
    }
}
#endif

static int serial_tx_char(void *arg) {    
    //  UART driver asks for more data to send. Return -1 if no more data is available for TX.
    assert(arg != NULL);
    BufferedSerial *serial = (BufferedSerial *) arg;
    int byte = serial->txIrq();
    //  { char buf[1]; buf[0] = (char) byte; console_buffer(buf, 1); console_printf("["); console_printhex(byte); console_printf("] "); } ////
    return byte;
}

static int serial_rx_char(void *arg, uint8_t byte) {
    //  UART driver reports incoming byte of data. Return -1 if data was dropped.
    assert(arg != NULL);
    BufferedSerial *serial = (BufferedSerial *) arg;
    int rc = serial->rxIrq(byte);
    //  { char buf[1]; buf[0] = (char) byte; console_printf("("); console_buffer(buf, 1); console_printf(") "); } ////
    return rc;
}

static void serial_tx_done(void *arg) {
    //  UART driver reports that transmission is complete.
}

int setup_uart(BufferedSerial *serial) {
    //open uart device selected for GPS_NEO6M
    //todo map DEV_NAME with MYNEWT_VAL for "uart3" 
    //struct uart_dev *uartdev = (struct uart_dev *) os_dev_lookup("uart3");

    uint32_t baud = serial->_baud;
    const char *uartdevname = serial->_uart_devname;

    struct uart_conf uc = {
        .uc_databits = 8,
        .uc_stopbits = 1,
        .uc_parity = UART_PARITY_NONE_,
        .uc_flow_ctl = UART_FLOW_CTL_NONE,
        .uc_tx_char = serial_tx_char,
        .uc_rx_char = serial_rx_char,
        .uc_tx_done = serial_tx_done,
        .uc_cb_arg = serial
    };

    uc.uc_speed = baud;

    struct uart_dev *dev = (struct uart_dev *) os_dev_open(uartdevname, OS_TIMEOUT_NEVER, &uc);
    if (dev == NULL) {
        console_printf("failed to open %s\n", uartdevname);
        return -1;
    }
    serial->_uartdev = dev;
    
    //// TODO
#ifdef NOTUSED    
    if (uart == 2) {
        // Swap TX/RX so that USART3 behaves like LPUART1
        console_printf("USART3->CR2: %lx | %lx ", USART3->CR2, USART_CR2_SWAP);

        //  u->u_regs->CR1 |= (USART_CR1_RXNEIE | USART_CR1_UE);
        _CLEAR_BIT(USART3->CR1, USART_CR1_UE);
        (void) RXDR(USART3);
        (void) STATUS(USART3);

        _SET_BIT(USART3->CR2, USART_CR2_SWAP);
        (void) RXDR(USART3);
        (void) STATUS(USART3);

        _SET_BIT(USART3->CR1, USART_CR1_UE);
        (void) RXDR(USART3);
        (void) STATUS(USART3);

        console_printf("-> %lx\n", USART3->CR2); console_flush();
    }
#endif  //  NOTUSED

    return 0;
}

void BufferedSerial::init(char *txbuf, uint32_t txbuf_size, char *rxbuf, uint32_t rxbuf_size, const char* name)
{
    _initialised = 0;
    _baud = 0;
    _txbuf_size = txbuf_size;
    _rxbuf_size = rxbuf_size;
    _txbuf.init(txbuf, txbuf_size);
    _rxbuf.init(rxbuf, rxbuf_size);

#if MYNEWT_VAL(SERIAL_LPMODE)
    // register with lowpowermgr to know when to deinit/init the device
    _LPMgr_id = LPMgr_register(serial_lp_change, this);
    // after init we are ok to wait datas while LP_DOZE
    LPMgr_setLPMode(_LPMgr_id, LP_DEEPSLEEP);
#endif

    os_error_t rc = os_sem_init(&_rx_sem, 0);  //  Init to 0 tokens, so caller will block until data is available.
    assert(rc == OS_OK);
}

void BufferedSerial::configure(const char *devname) {
    _uart_devname = devname;

}

int BufferedSerial::readable(void)
{
    return _rxbuf.available();  // note: look if things are in the buffer
}

int BufferedSerial::writeable(void)
{
    return 1;   // buffer allows overwriting by design, always true
}

int BufferedSerial::getc(int timeout)
{
    //  If no data available, wait until the timeout for data.
#if MYNEWT_VAL(SERIAL_LPMODE)
    LPMgr_setLPMode(_LPMgr_id, LP_DOZE);
#endif
    os_sem_pend(&_rx_sem, timeout * OS_TICKS_PER_SEC / 1000);
    if (_rxbuf.available()) { 
        return _rxbuf.get(); 
    }
    return -1;
}

int BufferedSerial::putc(int c)
{
    _txbuf.put(c);    
    BufferedSerial::prime();
    return c;
}

#ifdef NOTUSED
int BufferedSerial::puts(const char *s)
{
    if (s != NULL) {
        const char* ptr = s;    
        while(*(ptr) != 0) {
            _txbuf = *(ptr++);
        }
        _txbuf = '\n';  // done per puts definition
        BufferedSerial::prime();
        return (ptr - s) + 1;
    }
    return 0;
}
#endif  //  NOTUSED

extern "C" size_t BufferedSerialThunk(void *buf_serial, const void *s, size_t length)
{
    BufferedSerial *buffered_serial = (BufferedSerial *)buf_serial;
    return buffered_serial->write(s, length);
}

size_t BufferedSerial::write(const void *s, size_t length)
{
    if (s != NULL && length > 0) {
        const char* ptr = (const char*)s;
        const char* end = ptr + length;    
        while (ptr != end) {
            _txbuf = *(ptr++);
        }
        BufferedSerial::prime();    
        return ptr - (const char*)s;
    }
    return 0;
}

int BufferedSerial::rxIrq(uint8_t byte)
{
    //  UART driver reports incoming byte of data. Return -1 if data was dropped.
    _rxbuf.put(byte);  //  Add to TX buffer.
    os_error_t rc = os_sem_release(&_rx_sem);  //  Signal to semaphore that data is available.
    assert(rc == OS_OK);
    //  Trigger callback if necessary
    if (_cbs[RxIrq]) { _cbs[RxIrq](_cbs_arg[RxIrq]); }
    return 0;
}

int BufferedSerial::txIrq(void)
{
    //  UART driver asks for more data to send. Return -1 if no more data is available for TX.
    if(_txbuf.available()) {
        uint8_t byte = _txbuf.get();  //  Get data from TX buffer.
        return byte;
    }
    //  Trigger callback if no more data to send.
    if (_cbs[TxIrq]) { _cbs[TxIrq](_cbs_arg[TxIrq]); }
    return -1;
}

void BufferedSerial::prime(void)
{
    if (!_initialised) {
        //  Configure the UART port on first use.
        _initialised = 1;
        int rc = setup_uart(this);
        assert(rc == 0);
        _uartdev->ud_funcs.uf_start_rx(_uartdev); //  Start receiving UART data
    }
    _uartdev->ud_funcs.uf_start_tx(_uartdev);     //  Start transmitting UART data in the buffer.  txIrq will retrieve the data from the buffer.

#if MYNEWT_VAL(SERIAL_LPMODE)
    LPMgr_setLPMode(_LPMgr_id, LP_DOZE);
#endif
}

void BufferedSerial::attach(void (*func)(void *), void *arg, IrqType type)
{
    _cbs[type] = func;
    _cbs_arg[type] = arg;
}

void BufferedSerial::baud(uint32_t baud0)
{
    _baud = baud0;
}

void BufferedSerial::halt(void){
#if MYNEWT_VAL(SERIAL_LPMODE)
    LPMgr_setLPMode(_LPMgr_id, LP_DEEPSLEEP);
#endif
}
