/*
// Copyright (c) 2016 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
*/
//  Post custom data to CoAP Server.  We call the Mynewt OIC
//  interface to encode and transmit CoAP messages.  For ESP8266 and nRF24L01, the OIC interface
//  is implemented by esp8266/transport.h and nrf24l01/transport.h.  This is a simpler version of oc_client_api 
//  that supports sensors and JSON.  Original version: repos\apache-mynewt-core\net\oic\src\api\oc_client_api.c

#include <os/mynewt.h>
#include <oic/port/mynewt/config.h>
#include <oic/messaging/coap/coap.h>
#include <oic/oc_buffer.h>
#include <oic/oc_client_state.h>
#include <console/console.h>
#include "custom_coap/custom_coap.h"

#define OC_CLIENT_CB_TIMEOUT_SECS COAP_RESPONSE_TIMEOUT

///  Contains the CoAP headers.
static struct os_mbuf *oc_c_message;  
///  Contains the CoAP payload body.
static struct os_mbuf *oc_c_rsp;      
///  CoAP request
static coap_packet_t oc_c_request[1]; 
///  Because the CoAP JSON / CBOR buffers are shared, use this semaphore to prevent two CoAP requests from being composed at the same time.
static struct os_sem oc_sem;          
///  True if the Sensor CoAP is ready for sending sensor data.
static bool oc_custom_coap_ready = false;  
///  CoAP Payload encoding format: APPLICATION_JSON or APPLICATION_CBOR. If 0, let Sensor Network decide.
int oc_content_format = 0;            


int custom_writes_mbuf(void *buf, char *data, int len);

///  Close the current custom payload.
void custom_writes_reset(void);

///  Prepare to write a new JSON CoAP payload into the mbuf.
void custom_writes_new(struct os_mbuf *m);

///  Finalise the payload and return the payload size.
int custom_writes_finalize(void);


///////////////////////////////////////////////////////////////////////////////
//  CoAP Functions

///  Init the Sensor CoAP module. Called by sysinit() during startup, defined in pkg.yml.
void init_custom_coap(void) {
    os_error_t rc = os_sem_init(&oc_sem, 1);  //  Init to 1 token, so only 1 caller will be allowed.
    assert(rc == OS_OK);
    oc_custom_coap_ready = true;
}
   
///  Return true if the Sensor CoAP is ready for sending sensor data.
bool custom_coap_ready(void) {
    return oc_custom_coap_ready;
}

///  Handle CoAP response.
static void handle_coap_response(oc_client_response_t *data) {
    console_printf("handle_coap\n");
}

//  Serialise the CoAP request and payload into the final mbuf format for transmitting.
//  Forward the serialised mbuf to the background transmit task for transmitting.
static bool
dispatch_coap_request(void)
{
    bool ret = false;
    //  TODO : CoAP Content Format not specified
    //assert(oc_content_format);
    
    int response_length = custom_writes_finalize();   

    if (response_length) {
        oc_c_request->payload_m = oc_c_rsp;
        oc_c_request->payload_len = response_length;
        coap_set_header_content_format(oc_c_request, oc_content_format);  //  Either JSON or CBOR.
    } else {
        os_mbuf_free_chain(oc_c_rsp);
    }
    oc_c_rsp = NULL;

    if (oc_c_message) {
        if (!coap_serialize_message(oc_c_request, oc_c_message)) {
            coap_send_message(oc_c_message, 0);
        } else {
            os_mbuf_free_chain(oc_c_message);
        }

        //  Deallocate the client callback for the message ID.  We won't be processing the response from server.  
        //  TODO: Handle errors from server.
        oc_ri_remove_client_cb_by_mid(oc_c_request->mid);

        oc_c_message = NULL;
        ret = true;
    }
    os_error_t rc = os_sem_release(&oc_sem);  //  Request completed.  Release the semaphore for another request.
    assert(rc == OS_OK);
    return ret;
}

///  Prepare a new CoAP request for transmitting sensor data.
static bool
prepare_coap_request(oc_client_cb_t *cb, oc_string_t *query)
{
    coap_message_type_t type = COAP_TYPE_NON;

    oc_c_rsp = os_msys_get_pkthdr(0, 0);
    if (!oc_c_rsp) {
        return false;
    }
    oc_c_message = oc_allocate_mbuf(&cb->server.endpoint);
    if (!oc_c_message) {
        goto free_rsp;
    }
    
    if (oc_content_format == TEXT_PLAIN) { //only text_plain for
        custom_writes_new(oc_c_rsp);
    }    
    else { assert(0); }  //  Unknown CoAP content format.

    coap_init_message(oc_c_request, type, cb->method, cb->mid);
    coap_set_header_accept(oc_c_request, oc_content_format);  //  Either JSON or CBOR.
    coap_set_token(oc_c_request, cb->token, cb->token_len);
    coap_set_header_uri_path(oc_c_request, oc_string(cb->uri));
    if (cb->observe_seq != -1) {
        coap_set_header_observe(oc_c_request, cb->observe_seq);
    }
    if (query && oc_string_len(*query)) {
        coap_set_header_uri_query(oc_c_request, oc_string(*query));
    }
    if (cb->observe_seq == -1 && cb->qos == LOW_QOS) {
        os_callout_reset(&cb->callout,
          OC_CLIENT_CB_TIMEOUT_SECS * OS_TICKS_PER_SEC);
    }

    return true;
free_rsp:
    os_mbuf_free_chain(oc_c_rsp);
    oc_c_rsp = NULL;
    return false;
}

///  Create a new sensor post request to send to CoAP server.
bool
init_custom_post(struct oc_server_handle *server, const char *uri, int coap_content_format)
{
    assert(oc_custom_coap_ready);  assert(server);  assert(uri);
    
    //  TODO : CoAP Content Format not specified
    // assert(coap_content_format != 0);  

    //  Lock the semaphore for preparing the CoAP request.
    os_error_t rc = os_sem_pend(&oc_sem, OS_TIMEOUT_NEVER);  //  Allow only 1 task to be creating a sensor request at any time.
    assert(rc == OS_OK);

    oc_content_format = coap_content_format;
    oc_qos_t qos = LOW_QOS;  //  Default to low QoS, no transactions.
    oc_response_handler_t handler = handle_coap_response;
    oc_client_cb_t *cb;
    bool status = false;

    cb = oc_ri_alloc_client_cb(uri, server, OC_POST, handler, qos);
    if (!cb) {
        rc = os_sem_release(&oc_sem);  //  Failed.  Release the semaphore.
        assert(rc == OS_OK);
        return false;
    }
    status = prepare_coap_request(cb, NULL);
    return status;
}

///  Send the sensor post request to CoAP server.
bool
do_custom_post(void)
{
    return dispatch_coap_request();
}



///  The mbuf that contains the outgoing CoAP payload.
static struct os_mbuf *coap_custom_mbuf;

void do_custom_fill_buffer(char *data, int len) {

    custom_writes_mbuf(NULL, data, len);
}

int custom_writes_mbuf(void *buf, char *data, int len) {
    
    if (oc_content_format != TEXT_PLAIN) { return 0; }  //  Exit if we are something else than TEXT_PLAIN
    assert(coap_custom_mbuf);
    assert(data);
    //  console_printf("custom writes "); console_buffer(data, len); console_printf("\n");  ////
    int rc = os_mbuf_append(coap_custom_mbuf, data, len);  assert(rc == 0);
    if (rc) { return -1; }
    return 0;
}

///  Close the current custom payload.
void custom_writes_reset(void) {
    coap_custom_mbuf = NULL;
}

///  Prepare to write a new JSON CoAP payload into the mbuf.
void custom_writes_new(struct os_mbuf *m) {
    assert(m);
    custom_writes_reset();  //  Erase all last writes
    coap_custom_mbuf = m;
}

///  Finalise the payload and return the payload size.
int custom_writes_finalize(void) {
    assert(coap_custom_mbuf);
    int size = OS_MBUF_PKTLEN(coap_custom_mbuf);

#define DUMP_COAP
#ifdef DUMP_COAP
    console_printf("NET payload size %d\n", size); struct os_mbuf *m = coap_custom_mbuf;
    while (m) {
        console_buffer((const char *) (m->om_databuf + m->om_pkthdr_len), m->om_len);
        m = m->om_next.sle_next;
    } console_printf("\n");
#endif  //  DUMP_COAP

    custom_writes_reset();
    return size;
}


///  Split the float f into 3 parts: neg is true if negative, the absolute integer part i, and the decimal part d, with 6 decimal places.
void split_float(float f, bool *sign, int *i, int *d) {
    *sign = (f < 0) ? true : false;

    float f_abs = (f < 0) ? -f : f;          //  Absolute value of f
    *i = (int) f_abs;                     //  Integer part
    *d = ((int) (1000000.0f * f_abs)) % 1000000;  //  Two decimal places
}