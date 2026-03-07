/*
 * network_utils.h
 *
 *  Created on: May 7, 2024
 *      Author: rtsang
 */

#ifndef UTILS_NETWORK_UTILS_H_
#define UTILS_NETWORK_UTILS_H_

// Simplelink includes
#include "simplelink.h"

#include "utils.h"
#include "common.h"
#include <stdbool.h>

#define MAX_URI_SIZE 128
#define URI_SIZE MAX_URI_SIZE + 1

// when flashing, these must be loaded as user files, not system files.
#define SL_SSL_CA_CERT "/cert/rootCA.der" //starfield class2 rootca (from firefox) // <-- this one works
#define SL_SSL_PRIVATE "/cert/private.der"
#define SL_SSL_CLIENT  "/cert/client.der"

// globals
extern volatile unsigned long  g_ulStatus;//SimpleLink Status
extern unsigned long  g_ulPingPacketsRecv; //Number of Ping Packets received
extern unsigned long  g_ulGatewayIP; //Network Gateway IP address
extern unsigned char  g_ucConnectionSSID[SSID_LEN_MAX+1]; //Connection SSID
extern unsigned char  g_ucConnectionBSSID[BSSID_LEN_MAX]; //Connection BSSID

extern signed char * g_Host;
extern int g_port;

typedef struct SlDateTime {
   /* time */
   unsigned long tm_sec;
   unsigned long tm_min;
   unsigned long tm_hour;
   /* date */
   unsigned long tm_day;
   unsigned long tm_mon;
   unsigned long tm_year;
   unsigned long tm_week_day; //not required
   unsigned long tm_year_day; //not required
   unsigned long reserved[3];
} SlDateTime;

extern SlDateTime g_time;

typedef struct SlAppConfig {
    signed char * host;
    int port;
} SlAppConfig_t;

extern SlAppConfig_t g_app_config;

// Application specific status/error codes
typedef enum {
    // Choosing -0x7D0 to avoid overlap w/ host-driver's error codes
    LAN_CONNECTION_FAILED = -0x7D0,
    INTERNET_CONNECTION_FAILED = LAN_CONNECTION_FAILED - 1,
    DEVICE_NOT_IN_STATION_MODE = INTERNET_CONNECTION_FAILED - 1,

    STATUS_CODE_MAX = -0xBB8
} e_AppStatusCodes;


void SimpleLinkWlanEventHandler(SlWlanEvent_t *pWlanEvent);

void SimpleLinkNetAppEventHandler(SlNetAppEvent_t *pNetAppEvent);

void SimpleLinkHttpServerCallback(SlHttpServerEvent_t *pHttpEvent, SlHttpServerResponse_t *pHttpResponse);

void SimpleLinkGeneralEventHandler(SlDeviceEvent_t *pDevEvent);

void SimpleLinkSockEventHandler(SlSockEvent_t *pSock);

static long ConfigureSimpleLinkToDefaultState();

int tls_connect(bool mtls);

int connectToAccessPoint();

static long printErrConvenience(char * msg, long retVal);

static long InitializeAppVariables();

#endif /* UTILS_NETWORK_UTILS_H_ */
