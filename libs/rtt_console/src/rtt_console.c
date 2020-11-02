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

#include <os/mynewt.h>

#if MYNEWT_VAL(CONSOLE_RTT)
#include <os/os_mbuf.h>
#include <ctype.h>

#include "rtt/SEGGER_RTT.h"
#include "console/console.h"
#include "console_priv.h"

#if MYNEWT_VAL(CONSOLE_INPUT)
static struct hal_timer rtt_timer;
#endif

#define OUTPUT_BUFFER_SIZE 2048  //  Use a larger buffer size so that we don't affect interrupt processing.
static bool log_enabled = true;     //  Logging is on by default.
static bool buffer_enabled = true;  //  Buffering is on by default.

void enable_log(void)  { log_enabled = true; }
void disable_log(void) { log_enabled = false; }
void enable_buffer(void) { buffer_enabled = true; }  //  Enable buffering.
void disable_buffer(void) { buffer_enabled = false; console_flush(); }  //  Disable buffering.
static void split_float(float f, bool *neg, int *i, int *d);
static void split_double(double f, bool *neg, int *i, int *d);

#ifndef DISABLE_RTT  //  If RTT is enabled...


static int jlink_connected(void) {
    //  Return non-zero if debugger is connected. From repos/apache-mynewt-core/hw/mcu/ambiq/apollo2/src/hal_system.c
    return 1;//CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk;
}
#endif  //  !DISABLE_RTT

// We normally set the file handle to 0 to write to the rtt terminal.
#define RTT_FILE_HANDLE 0

static int rtt_write(uint32_t fh, const unsigned char *buffer, unsigned int length) {
    //  Write "length" number of bytes from "buffer" to the debugger's file handle fh.
    //  We normally set fh=2 to write to the debugger's stderr output.
#ifdef DISABLE_RTT  //  If RTT is disabled...
    return 0;               //  Don't write debug messages.
#else                       //  If RTT is enabled...
    if (!jlink_connected()) { return 0; }  //  If debugger is not connected, quit.
    if (length == 0) { return 0; }
    
    os_sr_t sr;

    OS_ENTER_CRITICAL(sr);
    SEGGER_RTT_WriteNoLock(0, buffer, length);
    OS_EXIT_CRITICAL(sr);

    return 0;

#endif  //  DISABLE_RTT
}

static struct os_mbuf *rtt_mbuf = NULL;

void console_flush(void) {
    //  Flush output buffer to the console log.  This will be slow.
    if (!log_enabled) { return; }       //  Skip if log not enabled.
    if (!rtt_mbuf) { return; }     //  Buffer is empty, nothing to write.
    if (os_arch_in_isr()) { return; }   //  Don't flush if we are called during an interrupt.

    //  Swap mbufs first to prevent concurrency problems.
    struct os_mbuf *old = rtt_mbuf;
    rtt_mbuf = NULL;

    struct os_mbuf *m = old;
    while (m) {  //  For each mbuf in the chain...
        const unsigned char *data = OS_MBUF_DATA(m, const unsigned char *);  //  Fetch the data.
        int size = m->om_len;                         //  Fetch the size.
        rtt_write(RTT_FILE_HANDLE, data, size);  //  Write the data to RTT output.
        m = m->om_next.sle_next;                      //  Fetch next mbuf in the chain.
    }
    if (old) { os_mbuf_free_chain(old); }  //  Deallocate the old chain.
}

void console_buffer(const char *buffer, unsigned int length) {
    //  Append "length" number of bytes from "buffer" to the output buffer.
#ifdef DISABLE_RTT  //  If RTT is disabled...
    return;                 //  Don't write debug messages.
#else                       //  If RTT is enabled...
    int rc;
    if (!log_enabled) { return; }           //  Skip if log not enabled.
    if (!jlink_connected()) { return; }  //  If debugger is not connected, quit.
    if (!rtt_mbuf) {                   //  Allocate mbuf if not already allocated.
        rtt_mbuf = os_msys_get_pkthdr(length, 0);
        if (!rtt_mbuf) { return; }  //  If out of memory, quit.
    }
    //  Limit the buffer size.  Quit if too big.
    if (os_mbuf_len(rtt_mbuf) + length >= OUTPUT_BUFFER_SIZE) { return; }
    //  Append the data to the mbuf chain.  This may increase the numbere of mbufs in the chain.
    rc = os_mbuf_append(rtt_mbuf, buffer, length);
    if (rc) { return; }  //  If out of memory, quit.
#endif  //  DISABLE_RTT
}

void console_printhex(uint8_t v) {
    //  Write a char in hexadecimal to the output buffer.
    #define MAX_BYTE_LENGTH 2
    char buffer[MAX_BYTE_LENGTH + 1];
    int size = MAX_BYTE_LENGTH + 1;
    bool prefixByZero = true;
    int length = 0;
    for(uint8_t divisor = 16; divisor >= 1; divisor = divisor / 16) {
        char digit = '0' + (char)(v / divisor);
        if (digit > '9') { digit = digit - 10 - '0' + 'a'; }
        if (digit > '0' || length > 0 || prefixByZero) {
            if (length < size) {
                buffer[length++] = digit;
            }
        }
        v = v % divisor;
    }
    if (length == 0) { buffer[length++] = '0'; };
    if (length < size) buffer[length] = 0;
    buffer[size - 1] = 0;  //  Terminate in case of overflow.
    console_buffer(buffer, strlen(buffer));
}

void console_printint(int i) {
    //  Write an int i the output buffer.
    console_printf("%d", i);
}

void console_printfloat(float f) {
    //  Write a float to the output buffer, with 2 decimal places.
    bool neg; int i, d;
    split_float(f, &neg, &i, &d);  //  Split the float into neg, integer and decimal parts to 2 decimal places
    console_printf("%s%d.%02d", neg ? "-" : "", i, d);   //  Combine the sign, integer and decimal parts
}

void console_printdouble(double f) {
    //  Write a double to the output buffer, with 6 decimal places.
    bool neg; int i, d;
    split_double(f, &neg, &i, &d);  //  Split the double into neg, integer and decimal parts to 6 decimal places
    console_printf("%s%d.%06d", neg ? "-" : "", i, d);   //  Combine the sign, integer and decimal parts
}

void console_dump(const uint8_t *buffer, unsigned int len) {
	//  Append "length" number of bytes from "buffer" to the output buffer in hex format.
    if (buffer == NULL || len == 0) { return; }
	for (int i = 0; i < len; i++) { console_printhex(buffer[i]); console_buffer(" ", 1); } 
}

static void split_float(float f, bool *neg, int *i, int *d) {
    //  Split the float f into 3 parts: neg is true if negative, the absolute integer part i, and the decimal part d, with 2 decimal places.
    *neg = (f < 0.0f);                    //  True if f is negative
    float f_abs = *neg ? -f : f;          //  Absolute value of f
    *i = (int) f_abs;                     //  Integer part
    *d = ((int) (100.0f * f_abs)) % 100;  //  Two decimal places
}

static void split_double(double f, bool *neg, int *i, int *d) {
    //  Split the double f into 3 parts: neg is true if negative, the absolute integer part i, and the decimal part d, with 6 decimal places.
    *neg = (f < 0.0f);                    //  True if f is negative
    float f_abs = *neg ? -f : f;          //  Absolute value of f
    *i = (int) f_abs;                     //  Integer part
    *d = ((int) (1000000.0f * f_abs)) % 1000000;  //  6 decimal places
}

static void rtt_console_write_ch(char c) {
    if (c == '\r') { return; }  //  Don't display \r.
    console_buffer(&c, 1);  //  Append the char to the output buffer.
    //  if (c == '\n') { console_flush(); }  //  If we see a newline, flush the buffer.
}

int console_out_nolock(int character) {
    char c = (char)character;
    if (g_silence_console) { return c; }
    if ('\n' == c) {
        rtt_console_write_ch('\r');
        console_is_midline = 0;
    } else {
        console_is_midline = 1;
    }
    rtt_console_write_ch(c);
    return character;
}

void console_rx_restart(void) {}




#if MYNEWT_VAL(CONSOLE_INPUT)

#define RTT_INPUT_POLL_INTERVAL_MIN     10 /* ms */
#define RTT_INPUT_POLL_INTERVAL_STEP    10 /* ms */
#define RTT_INPUT_POLL_INTERVAL_MAX     MYNEWT_VAL(CONSOLE_RTT_INPUT_POLL_INTERVAL_MAX)

static void
rtt_console_poll_func(void *arg)
{
    static uint32_t itvl_ms = RTT_INPUT_POLL_INTERVAL_MIN;
    static int key = -1;
    int ret;

    if (key < 0) {
        key = SEGGER_RTT_GetKey();
    }

    if (key < 0) {
        itvl_ms += RTT_INPUT_POLL_INTERVAL_STEP;
        itvl_ms = min(itvl_ms, RTT_INPUT_POLL_INTERVAL_MAX);
    } else {
        while (key >= 0) {
            ret = console_handle_char((char)key);
            if (ret < 0) {
                return;
            }
            key = SEGGER_RTT_GetKey();
        }
        itvl_ms = RTT_INPUT_POLL_INTERVAL_MIN;
    }

    os_cputime_timer_relative(&rtt_timer, itvl_ms * 1000);
}
#endif

int
rtt_console_is_init(void)
{
    return 1;
}

int
rtt_console_init(void)
{
#if MYNEWT_VAL(CONSOLE_INPUT)
    os_cputime_timer_init(&rtt_timer, rtt_console_poll_func, NULL);
    /* start after a second */
    os_cputime_timer_relative(&rtt_timer, 1000000);
#endif
    return 0;
}



#endif /* MYNEWT_VAL(CONSOLE_RTT) */
