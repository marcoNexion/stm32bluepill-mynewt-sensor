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
//  Transmit sensor data to a CoAP server like thethings.io.  The CoAP payload will be encoded as JSON.
//  The sensor data will be transmitted over NB-IoT or WiFi.
//  Note that we are using a patched version of apps/my_sensor_app/src/vsscanf.c that
//  fixes response parsing bugs.  The patched file must be present in that location.

//  Mynewt consolidates all app settings into "bin/targets/bluepill_my_sensor/generated/include/syscfg/syscfg.h"
#include <sysinit/sysinit.h>  //  Contains all app settings consolidated from "apps/my_sensor_app/syscfg.yml" and "targets/bluepill_my_sensor/syscfg.yml"
#include <console/console.h>  //  For Mynewt console output. Actually points to libs/semihosting_console
#include <sensor_network/sensor_network.h>  //  For Sensor Network library

#include <custom_coap/custom_coap.h>        //  For Sensor CoAP library
#include "network.h"

int send_plain_text_data(char *str) {

    //  Compose a CoAP JSON message with the Sensor Key (field name) and Value in val 
    //  and send to the CoAP server.  The message will be enqueued for transmission by the CoAP / OIC 
    //  Background Task so this function will return without waiting for the message 
    //  to be transmitted.  Return 0 if successful, SYS_EAGAIN if network is not ready yet.

    //  For the CoAP server hosted at thethings.io, the CoAP payload should be encoded in JSON like this:
    //  {"values":[
    //    {"key":"device", "value":"0102030405060708090a0b0c0d0e0f10"},
    //    {"key":"t",      "value":1715}
    //  ]}
/*
    assert(lat);
    assert(lng);
    assert(alt);
*/

    assert(str);

    //  Start composing the CoAP Server message with the sensor data in the payload.  This will 
    //  block other tasks from composing and posting CoAP messages (through a semaphore).
    //  We only have 1 memory buffer for composing CoAP messages so it needs to be locked.
    bool rc = init_server_post(NULL);
    if (!rc) { return SYS_EAGAIN; }  //  If network transport not ready, tell caller (Sensor Listener) to try again later.

    do_custom_fill_buffer(str, strlen(str));

    //  Post the CoAP Server message to the CoAP Background Task for transmission.  After posting the
    //  message to the background task, we release a semaphore that unblocks other requests
    //  to compose and post CoAP messages.
    rc = do_server_post();  assert(rc);
    
    

    //  The CoAP Background Task will call oc_tx_ucast() in the network driver to transmit the message.
    return 0;
}


/*
Compose the CoAP Payload in JSON using the CP macros.
    CP_ROOT({                     //  Create the payload root
            <json_rep_start_root_object>
            <json_encode_array_name>
            <json_encode_array_start>
            <json_encode_object_start>
            <json_encode_object_entry>
            <json_encode_object_entry>
            <json_encode_object_finish>
            <json_encode_object_start>
            <json_encode_object_entry>
            <json_encode_object_entry>
            <json_encode_object_finish>
            <json_encode_array_finish>
            <json_rep_end_root_object>
        });                           //  End CP_ROOT:  Close the payload root

*/