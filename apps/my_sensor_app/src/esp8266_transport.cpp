//  ESP8266 Network Transport for Apache Mynewt.  This provides the OIC (Open Interconnect Consortium)
//  interface for the ESP8266 Driver, so that we may compose and transmit CoAP requests using Mynewt's
//  OIC implementation.  More about Mynewt OIC: https://mynewt.apache.org/latest/os/modules/devmgmt/newtmgr.html
#include <os/os.h>
#include <console/console.h>
#include "esp8266_driver.h"
#include "esp8266_transport.h"

static void oc_tx_ucast(struct os_mbuf *m);
static uint8_t oc_ep_size(const struct oc_endpoint *oe);
static int oc_ep_has_conn(const struct oc_endpoint *);
static char *oc_ep_str(char *ptr, int maxlen, const struct oc_endpoint *);
static int oc_init(void);
static void oc_shutdown(void);
//  static void oc_event(struct os_event *ev);

static struct esp8266_server *server;  //  CoAP Server host and port.  We only support 1 server.
static void *socket;                   //  Reusable UDP socket connection to the CoAP server.  Never closed.
static struct sensor_itf *driver;      //  Driver interface for ESP8266.  Will be called to perform ESP8266 functions.
static uint8_t transport_id = -1;      //  Will contain the Transport ID allocated by Mynewt OIC.

//  Definition of ESP8266 driver as a transport for CoAP.  Only 1 ESP8266 driver instance supported.
static const struct oc_transport transport = {
    0,               //  uint8_t ot_flags;
    oc_ep_size,      //  uint8_t (*ot_ep_size)(const struct oc_endpoint *);
    oc_ep_has_conn,  //  int (*ot_ep_has_conn)(const struct oc_endpoint *);
    oc_tx_ucast,     //  void (*ot_tx_ucast)(struct os_mbuf *);
    NULL,  //  void (*ot_tx_mcast)(struct os_mbuf *);
    NULL,  //  enum oc_resource_properties *ot_get_trans_security)(const struct oc_endpoint *);
    oc_ep_str,    //  char *(*ot_ep_str)(char *ptr, int maxlen, const struct oc_endpoint *);
    oc_init,      //  int (*ot_init)(void);
    oc_shutdown,  //  void (*ot_shutdown)(void);
};

int esp8266_register_transport(struct sensor_itf *itf, struct esp8266_server *server0) {
    //  Register the ESP8266 driver (itf) as the transport for the specifed CoAP server.
    assert(itf);  assert(server0);
    transport_id = oc_transport_register(&transport);
    driver = itf;
    server = server0;

    //  Init the server endpoint before use.
    int rc = init_esp8266_server(server);  assert(rc == 0);

    //  Allocate a new UDP socket for the CoAP server.  The socket will be always connected to the server and cannot be changed or closed.
    rc = esp8266_socket_open(driver, &socket, NSAPI_UDP);  assert(rc == 0);

    //  Connect the socket to the UDP address and port.  Command looks like: AT+CIPSTART=0,"UDP","coap.thethings.io",5683
    rc = esp8266_socket_connect(driver, socket, server->endpoint.host, server->endpoint.port);  assert(rc == 0);
    return 0;
}

int init_esp8266_server(struct esp8266_server *server) {
    //  Init the server endpoint before use.
    int rc = init_esp8266_endpoint(&server->endpoint);  assert(rc == 0);
    server->handle = (struct oc_server_handle *) server;
    return 0;
}

int init_esp8266_endpoint(struct esp8266_endpoint *endpoint) {
    //  Init the endpoint before use.
    assert(transport_id >= 0);  //  Transport ID must be allocated.
    endpoint->ep.oe_type = transport_id;
    endpoint->ep.oe_flags = 0;
    return 0;
}

static char esp8266_mbuf_buffer[ESP8266_TX_BUFFER_SIZE];  //  Buffer that contains the consolidated UDP packet.
static int esp8266_mbuf_index;  //  Next index for writing into esp8266_mbuf_buffer. 

static void oc_tx_ucast(struct os_mbuf *m0) {
    //  Transmit the mbuf to the network over UDP.  First mbuf is CoAP header, second mbuf is CoAP payload.
    //  Dump out each mbuf in the linked list.
    console_printf("  > send UDP packet\n");
    struct os_mbuf *m = m0;
    ////struct esp8266_endpoint *endpoint = NULL;
    ////int ep_size = oc_ep_size(NULL);  assert(ep_size > 0);

    //  Find the endpoint header.  Should be the end of the packet header of the first packet.
    assert(OS_MBUF_USRHDR_LEN(m) >= sizeof(struct esp8266_endpoint));
    struct esp8266_endpoint *endpoint = (struct esp8266_endpoint *) OC_MBUF_ENDPOINT(m);

    esp8266_mbuf_index = 0;
    while (m) {  //  For each mbuf in the list...
        assert(m->om_data);
        if (m->om_pkthdr_len) { console_printf("Header: %d\n", m->om_pkthdr_len); console_dump(m->om_databuf, m->om_pkthdr_len); console_printf("\n"); }
        if (m->om_len) { console_printf("Data: %d\n", m->om_len); console_dump(m->om_data, m->om_len); console_printf("\n"); }

        //  Find the endpoint header.  Should be the end of the packet header of the first packet.
        ////if (m->om_pkthdr_len >= ep_size) { endpoint = (esp8266_endpoint *) &m->om_databuf[m->om_pkthdr_len - ep_size]; }

        //  Consolidate the CoAP header and payload for sending.
        assert(esp8266_mbuf_index + m->om_len <= ESP8266_TX_BUFFER_SIZE);  //  Buffer too small.
        memcpy(&esp8266_mbuf_buffer[esp8266_mbuf_index], m->om_data, m->om_len);
        esp8266_mbuf_index += m->om_len;

        m = m->om_next.sle_next;  //  Fetch next mbuf in the list.
    }
    //  After consolidating, free the chain of mbufs.
    int rc = os_mbuf_free_chain(m0);  assert(rc == 0);
    assert(endpoint);  assert(endpoint->host);  assert(endpoint->port);  //  Host and endpoint should be in the endpoint.
    assert(server);  assert(endpoint->host == server->endpoint.host);  assert(endpoint->port == server->endpoint.port);  //  We only support 1 server connection. Must match the message endpoint.
    assert(driver);  assert(socket);  assert(esp8266_mbuf_index > 0);

    //  Send the consolidated buffer via UDP.
    rc = esp8266_socket_send(driver, socket, esp8266_mbuf_buffer, esp8266_mbuf_index);  
    assert(rc > 0);
}

static uint8_t oc_ep_size(const struct oc_endpoint *oe) {
    //  Return the size of the endpoint.  OIC will allocate space to store this endpoint in the transmitted mbuf.
    return sizeof(struct esp8266_endpoint);
}

static int oc_ep_has_conn(const struct oc_endpoint *oe) {
    console_printf("oc_ep_has_conn\n");
    return 0;
}

static char *oc_ep_str(char *ptr, int maxlen, const struct oc_endpoint *oe) {
    console_printf("oc_ep_str\n");
#ifdef NOTUSED
    const struct oc_endpoint_ip *oe_ip = (const struct oc_endpoint_ip *)oe;
    int len;
    mn_inet_ntop(MN_PF_INET, oe_ip->v4.address, ptr, maxlen);
    len = strlen(ptr);
    snprintf(ptr + len, maxlen - len, "-%u", oe_ip->port);
    return ptr;
#endif  //  NOTUSED
    strcpy(ptr, "TODO:oc_ep_str");
    return ptr;
}

static int oc_init(void) {
    console_printf("oc_init\n");
    return 0;
}

static void oc_shutdown(void) {
    console_printf("oc_shutdown\n");
}