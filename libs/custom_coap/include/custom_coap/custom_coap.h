//  Post custom data to CoAP server TEXT_PLAIN encoding.  We call the Mynewt OIC
//  interface to encode and transmit CoAP messages.  For ESP8266, the OIC interface
//  is implemented by esp8266/transport.h.
#ifndef __SENSOR_COAP_H__
#define __SENSOR_COAP_H__

#include <os/mynewt.h>

#ifdef __cplusplus
extern "C" {  //  Expose the types and functions below to C functions.
#endif

struct GPS{
    float latitude;
    float longitude; //dddmm.mmmm
};
struct GNSS{
    char capture[32];
};


typedef enum{
    STOP = 0x00,
    START = 0x01,
    TIMEOUT_MOVING = 0x02,
    TIMEOUT_HALTED = 0x03,
    KEEP_ALIVE = 0x04

}TX_REASON;

typedef enum{
    GPS = 0x00,
    GNSS = 0x01

}POSITION_TYPE;


//  custom_value represents a decoded custom data value of utf-8 strings with null terminated char (+1)
struct custom_value {

    uint32_t id;                         //  32 bit id
    TX_REASON tx_reason;          //  8 bit value (arrêt (0), départ (1), timeout device mouvant (2), timeout device à l'arrêt (3))
    uint32_t timestamp;                 //  32 bit unix timestamp
    uint32_t mV_Bat;                    //  Battery voltage in mV
    POSITION_TYPE position_type;      //  flag for positionning value type. 0: GPS, 1:GNSS
    union {
        struct GPS gps;
        struct GNSS gnss;
    } position;
};

///////////////////////////////////////////////////////////////////////////////
//  Custom CoAP Functions

#define COAP_PORT_UNSECURED (5683)  //  Port number for CoAP Unsecured

struct oc_server_handle;

//  Init the Sensor CoAP module. Called by sysinit() during startup, defined in pkg.yml.
void init_custom_coap(void);

//  Return true if the Sensor CoAP is ready for sending custom data.
bool custom_coap_ready(void);

//  Create a new custom post request to send to CoAP server.  coap_content_format is 
//  TEXT_PLAIN. If coap_content_format is 0, use the default format.
bool init_custom_post(struct oc_server_handle *server, const char *uri, int coap_content_format);

// fill local buffer with *data
void do_custom_fill_buffer(char *data, int len);

//  Send the custom post request to CoAP server.
bool do_custom_post(void);

void split_float(float f, bool *sign, int *i, int *d);

#ifdef __cplusplus
}
#endif

#endif  //  __SENSOR_COAP_H__
