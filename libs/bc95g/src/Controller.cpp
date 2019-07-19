/* 
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

#include <assert.h>
#include <console/console.h>
#include "util.h"
#include "Controller.h"

extern "C" int debug_bc95g;

//  Refer to https://medium.com/@ly.lee/get-started-with-nb-iot-and-quectel-modules-6e7c581e0d61

static const char *commands[] = {
    //  Sequence MUST match CommandID.
    //  [0] Prepare to transmit
    "NCONFIG=AUTOCONNECT,FALSE",  //  NCONFIG: configure
    "QREGSWT=2",    //  QREGSWT: huawei
    "NRB",          //  NRB: reboot

    //  [1] Attach to network
    "NBAND=8",  //  NBAND: select band
    "CFUN=1",   //  CFUN: enable functions
    "CGATT=1",  //  CGATT: attach network
    "CGATT?",   //  CGATT_QUERY: query attach
    "CEREG?",   //  CEREG_QUERY: query registration

    //  [2] Transmit message
    "NSOCR=DGRAM,17,0,1",  //  NSOCR: allocate port
    "NSOST=%d,104.199.85.211,5683,%d,%s,%d",  //  NSOST: transmit

    //  [3] Receive response
    "NSORF=1,%d",  //  NSORF: receive msg
    "NSOCL=1,%d",  //  NSOCL: close port

    //  [4] Diagnostics
    "CGPADDR",   //  CGPADDR: IP address
    "NUESTATS",  //  NUESTATS: network stats
};

static char buf[1024];  //  TODO: Check size.
static char buf2[64];   //  TODO: Check size.

const char *Controller::getCommand(enum CommandID cmdID) {
    assert(cmdID >= 0);
    assert(cmdID < (sizeof(commands) / sizeof(commands[0]));
    const char *cmd = commands[cmdID];
    return cmd;
}

bool Controller::sendCommandInternal(const char *cmd) {
    //  Send the AT command.
    assert(strlen(cmd) + 4 <= sizeof(buf));  //  Sufficient space for "AT+"
    strcpy(buf, "AT+");
    strncat(buf, sizeof(buf) - 4, cmd);
    return _parser.send(buf) && _parser.recv("OK");
}

bool Controller::sendCommand(enum CommandID cmdID) {
    //  Send an AT command with no parameters.
    const char *cmd = getCommand(cmdID);
    return sendCommandInternal(cmd);
}

bool Controller::sendCommandInt(enum CommandID cmdID, int arg) {
    //  Send an AT command with 1 int parameter.
    const char *cmd = getCommand(cmdID);
    //  Assume cmd contains "...%d..."
    assert(strlen(cmd) + 5 <= sizeof(buf2));  //  Sufficient space for "&d"
    sprintf(buf2, cmd, arg);
    return sendCommandInternal(buf2);
}

bool Controller::sendQuery(enum CommandID cmdID, char *result, uint8_t size) {
    //  Send an AT query like "AT+CGATT?". Return the result.
    const char *cmd = getCommand(cmdID);
    return sendCommandInternal(cmd);
}

bool Controller::sendCommand(enum CommandID cmdID) {
    assert(cmdID >= 0);
    assert(cmdID < (sizeof(commands) / sizeof(commands[0]));
    const char *cmd = commands[cmdID];
    sprintf(buf, "AT+%s", cmd);
    return _parser.send(buf) && _parser.recv("OK");
}

bool Controller::transmit(uint16_t port, uint16_t seq, const char *data, uint16_t size) {
    //  AT+NSOST=1,104.199.85.211,5683,147,(data),100
    assert(strlen(data) + 50 < sizeof(buf));
    sprintf(buf, "AT+NSOST=%d,104.199.85.211,5683,%d,%s,%d", port, size, data, seq);
    return _parser.send(buf) && _parser.recv("OK");

    ////  TODO
    const char *_f = "tx";
    console_printf("%s%s %u...\n", _esp, _f, (unsigned) amount);  console_flush();
    //  May take a second try if device is busy
    for (unsigned i = 0; i < 2; i++) {
        if (_parser.send("AT+CIPSEND=%d,%d", id, amount)
            && _parser.recv(">")
            && _parser.write((char*)data, (int)amount) >= 0 
            && _parser.recv("SEND OK")) {
            _log(_f, true);
            return true;
        }
    }
    _log(_f, false);
    return false;
}

bool Controller::transmitMbuf(int id,  struct os_mbuf *m0) {
    //  Send the chain of mbufs.
    //  AT+NSOST=1,104.199.85.211,5683,147,(data),100
    const char *cmd = getCommand(cmdID);

    uint32_t amount = OS_MBUF_PKTLEN(m0);  //  Length of the mbuf chain.
    const char *_f = "tx mbuf";
    console_printf("%s%s %u...\n", _esp, _f, (unsigned) amount);  console_flush();
    //  May take a second try if device is busy
    for (unsigned i = 0; i < 2; i++) {
        if (_parser.send("AT+CIPSEND=%d,%d", id, amount)
            && _parser.recv(">")) {
            struct os_mbuf *m = m0;
            bool failed = false;
            while (m) {  //  For each mbuf in the list...
                const char *data = OS_MBUF_DATA(m, const char *);  //  Fetch the data.
                int size = m->om_len;  //  Fetch the size.
                console_dump((const uint8_t *) data, size); console_printf("\n");
                if (_parser.write(data, size) < 0) {   //  If the writing failed, retry.
                    failed = true;
                    break;
                }
                m = m->om_next.sle_next;   //  Fetch next mbuf in the list.
            }
            if (failed) { break; }
            if (!_parser.recv("SEND OK")) { break; }
            _log(_f, true);  console_flush();
            return true;
        }
    }
    _log(_f, false);
    return false;
}

void Controller::init(char *txbuf, uint32_t txbuf_size, char *rxbuf, uint32_t rxbuf_size, 
    char *parserbuf, uint32_t parserbuf_size, bool debug)
{
    _uart = 0;
    _serial.init(txbuf, txbuf_size, rxbuf, rxbuf_size);
    _parser.init(_serial, parserbuf, parserbuf_size);
    _packets = 0;
    _packets_end = &_packets;
    _serial.baud(115200);
    _parser.debugOn(debug);
}

void Controller::configure(int uart) {
    _uart = uart;
    _serial.configure(uart);
}

void packet_handler(void *arg) {
    assert(arg != NULL);
    ((ESP8266 *)arg)->_packet_handler();
}

bool Controller::startup(int mode)
{
    //  only 3 valid modes
    if(mode < 1 || mode > 3) {
        return false;
    }
    bool success = 
        reset()            //  Restart the ESP8266 module.
        && setEcho(false)  //  Disable command echo to speed up response processing.
        && _parser.send("AT+CWMODE=%d", mode)  //  Set the mode to WiFi Client, WiFi Access Point, or both.
        && _parser.recv("OK")                  //  Wait for response.
        && _parser.send("AT+CIPMUX=1")         //  Allow multiple TCP/UDP connections.
        && _parser.recv("OK");                 //  Wait for response.
    _parser.oob("+IPD", packet_handler, this); //  Call the packet handler when network data is received.
    return success;
}

bool Controller::reset(void)
{
    //  debug_bc95g = 1;  ////
    const char *_f = "reset";    
    bool ret = false;
    console_printf("%s%s...\n", _esp, _f); console_flush(); 
    for (int i = 0; i < 2; i++) {
        if (
            _parser.send("\r\nAT+RST")
            && _parser.recv("OK")  //  Wait for response.
#ifndef TOOO
            && _parser.recv("ready")  //  TODO: Wait for response: "ready" or "[System Ready, Vendor:www.ai-thinker.com]"
            && _parser.recv("\r\n")   //  Wait for end of the line
#else
            && _parser.recv("jump")  //  Wait for last line of response: "jump to run user1 @ 1000"
            && _parser.recv("\r\n")  //  Wait for end of the line
#endif  //  TODO
        ) {
            _parser.flush();  //  Discard the rest of the response before sending next command.
            ret = true;
            break;
        }
    }
    _log(_f, ret);
    //  debug_bc95g = 0;  ////    
    return true;
}

const char *Controller::getIPAddress(void)
{
    if (!(_parser.send("AT+CIFSR")
        && _parser.recv("+CIFSR:STAIP,\"%15[^\"]\"", _ip_buffer)
        && _parser.recv("OK"))) {
        return 0;
    }

    return _ip_buffer;
}

bool Controller::isConnected(void)
{
    return getIPAddress() != 0;
}

bool Controller::open(const char *type, int id, const char* addr, int port)
{
    const char *_f = "open";
    //IDs only 0-4
    if(id > 4) {
        return false;
    }
    console_printf("%s%s...\n", _esp, _f);  console_flush();
    bool ret = _parser.send("AT+CIPSTART=%d,\"%s\",\"%s\",%d", id, type, addr, port)
        && _parser.recv("OK");
    _log(_f, ret);
    return ret;
}

void Controller::_packet_handler()
{
    int id;
    uint32_t amount;

    // parse out the packet
    if (!_parser.recv(",%d,%d:", &id, &amount)) {
        return;
    }

    struct packet *packet = (struct packet*)malloc(
            sizeof(struct packet) + amount);
    if (!packet) {
        return;
    }

    packet->id = id;
    packet->len = amount;
    packet->next = 0;

    if (!(_parser.read((char*)(packet + 1), amount))) {
        free(packet);
        return;
    }

    // append to packet list
    *_packets_end = packet;
    _packets_end = &packet->next;
}

int32_t Controller::recv(int id, void *data, uint32_t amount)
{
    while (true) {
        // check if any packets are ready for us
        for (struct packet **p = &_packets; *p; p = &(*p)->next) {
            if ((*p)->id == id) {
                struct packet *q = *p;

                if (q->len <= amount) { // Return and remove full packet
                    memcpy(data, q+1, q->len);

                    if (_packets_end == &(*p)->next) {
                        _packets_end = p;
                    }
                    *p = (*p)->next;

                    uint32_t len = q->len;
                    free(q);
                    return len;
                } else { // return only partial packet
                    memcpy(data, q+1, amount);

                    q->len -= amount;
                    memmove(q+1, (uint8_t*)(q+1) + amount, q->len);

                    return amount;
                }
            }
        }

        // Wait for inbound packet
        if (!_parser.recv("OK")) {
            return -1;
        }
    }
}

bool Controller::close(int id)
{
    //May take a second try if device is busy
    for (unsigned i = 0; i < 2; i++) {
        if (_parser.send("AT+CIPCLOSE=%d", id)
            && _parser.recv("OK")) {
            return true;
        }
    }

    return false;
}

void Controller::setTimeout(uint32_t timeout_ms)
{
    _parser.setTimeout(timeout_ms);
}

bool Controller::readable()
{
    return _serial.readable();
}

bool Controller::writeable()
{
    return _serial.writeable();
}

void Controller::attach(void (*func)(void *), void *arg)
{
    _serial.attach(func, arg);
}
