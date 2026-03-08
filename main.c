// Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/ 

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "simplelink.h"

#include "hw_types.h"
#include "hw_ints.h"
#include "hw_memmap.h"
#include "rom.h"
#include "rom_map.h"
#include "interrupt.h"
#include "prcm.h"
#include "utils.h"
#include "uart.h"
#include "gpio.h"
#include "timer.h"
#include "spi.h"
#include "systick.h"

//Common interface includes
#include "pinmux.h"
#include "gpio_if.h"
#include "common.h"
#include "uart_if.h"
#include "timer_if.h"

// Custom includes
#include "utils/network_utils.h"
#include "Adafruit_GFX.h"
#include "Adafruit_SSD1351.h"

// IR and Multi-tap Definitions
#define SYSTICK 16777216
#define CONFIRMATION_TIMEOUT 800
#define DEBOUNCE_TIMEOUT 250

#define BLACK           0x0000
#define GREEN           0x07E0
#define WHITE           0xFFFF

#define TOP_HALF_Y      0
#define BOTTOM_HALF_Y   65
#define LINE_Y          64


//NEED TO UPDATE every time!
#define DATE                8    /* Current Date */
#define MONTH               3     /* Month 1-12 */
#define YEAR                2026  /* Current year */
#define HOUR                13    /* Time - hours */
#define MINUTE              0    /* Time - minutes */
#define SECOND              0     /* Time - seconds */

#define APPLICATION_NAME      "SSL"
#define APPLICATION_VERSION   "SQ24"

// OpenSky API Definitions
#define OPENSKY_SERVER_NAME "opensky-network.org"
#define OPENSKY_PORT 443
// Sacramento Bounding Box: lamin=38.5, lomin=-121.6, lamax=38.7, lomax=-121.4
#define OPENSKY_GET_PATH "/api/states/all?lamin=38.5&lomin=-121.6&lamax=38.7&lomax=-121.4"
#define OPENSKY_GET_HEADER "GET " OPENSKY_GET_PATH " HTTP/1.1\r\n"
#define OPENSKY_HOST_HEADER "Host: " OPENSKY_SERVER_NAME "\r\n"

// AWS IoT Definitions
#define AWS_SERVER_NAME "a1kqo0z2zpc3cs-ats.iot.us-east-1.amazonaws.com"
#define AWS_PORT 8443
#define AWS_POST_PATH "/things/Tyson-Tien_CC3200_Board/shadow"
#define AWS_POST_HEADER "POST " AWS_POST_PATH " HTTP/1.1\r\n"
#define AWS_HOST_HEADER "Host: " AWS_SERVER_NAME "\r\n"

#define CHEADER "Connection: Keep-Alive\r\n"
#define CTHEADER "Content-Type: application/json; charset=utf-8\r\n"
#define CLHEADER1 "Content-Length: "
#define CLHEADER2 "\r\n\r\n"

#define DATA1 "{" \
            "\"state\": {\r\n"                                              \
                "\"desired\" : {\r\n"                                       \
                    "\"var\" :\"Hello World!\"\r\n"                         \
                "}"                                                         \
            "}"                                                             \
        "}\r\n\r\n"


// GLOBAL VARIABLES -- Start
#if defined(ccs) || defined(gcc)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif

// IR Decoding
volatile unsigned long ticks = SYSTICK;
volatile uint32_t delta = 0;
volatile uint32_t message_buffer = 0;
volatile uint32_t data = 0;
volatile int bit_count = 0;
volatile bool message_ready = false;

// Multi-tap
const char* multi_tap_chars[] = {
    " ",
    "",
    "abc",
    "def",
    "ghi",
    "jkl",
    "mno",
    "pqrs",
    "tuv",
    "wxyz"
};

volatile unsigned long tickCount = 0;
int lastButton = -1;
int charIndex = 0;
unsigned long lastPressTime = 0;

char composeBuffer[64];
int composeLength = 0;

int cursorXBottom = 0;
int cursorYBottom = BOTTOM_HALF_Y;

char current_callsign[16] = {0};
char current_altitude[16] = {0};
bool new_flight_data = false;

// LOCAL FUNCTION PROTOTYPES
static int set_time();
static void BoardInit(void);
static int http_post(int, const char*);
static int opensky_http_get(int);

// IR & OLED Logic
void SysTickInit(void);
void SysTickReset(void);
unsigned long TickDifference(void);
void record_bit(int value);
void GPIOIntHandler(void);
void TimerTickHandler(void);
void ProcessButton(int button);
void DeleteChar(void);
void PostIRMessage(int iTLSSockID);
void PostFlightData(int iTLSSockID);


//! Board Initialization & Configuration
static void BoardInit(void) {
/* In case of TI-RTOS vector table is initialize by OS itself */
#ifndef USE_TIRTOS
  //
  // Set vector table base
  //
#if defined(ccs)
    MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
#endif
#if defined(ewarm)
    MAP_IntVTableBaseSet((unsigned long)&__vector_table);
#endif
#endif
    //
    // Enable Processor
    //
    MAP_IntMasterEnable();
    MAP_IntEnable(FAULT_SYSTICK);

    PRCMCC3200MCUInit();
}

//! This function updates the date and time of CC3200.
static int set_time() {
    long retVal;

    g_time.tm_day = DATE;
    g_time.tm_mon = MONTH;
    g_time.tm_year = YEAR;
    g_time.tm_sec = SECOND;
    g_time.tm_hour = HOUR;
    g_time.tm_min = MINUTE;

    retVal = sl_DevSet(SL_DEVICE_GENERAL_CONFIGURATION,
                          SL_DEVICE_GENERAL_CONFIGURATION_DATE_TIME,
                          sizeof(SlDateTime),(unsigned char *)(&g_time));

    ASSERT_ON_ERROR(retVal);
    return SUCCESS;
}

void FetchAndDisplayFlightData() {
    g_app_config.host = OPENSKY_SERVER_NAME;
    g_app_config.port = OPENSKY_PORT;
    
    UART_PRINT("Connecting to OpenSky Network...\n\r");
    int openSkySockID = tls_connect(false);
    if (openSkySockID >= 0) {
        new_flight_data = false;
        opensky_http_get(openSkySockID);
        sl_Close(openSkySockID);
        
        if (new_flight_data) {
            UART_PRINT("New flight found! Syncing to AWS IoT...\n\r");
            g_app_config.host = AWS_SERVER_NAME;
            g_app_config.port = AWS_PORT;
            int awsSockID = tls_connect(true);
            if (awsSockID >= 0) {
                PostFlightData(awsSockID);
                sl_Close(awsSockID);
            } else {
                UART_PRINT("Failed to connect to AWS IoT for telemetry sync.\r\n");
            }
            new_flight_data = false;
        }
    } else {
        UART_PRINT("Failed to connect to OpenSky Network.\r\n");
    }
}

void main() {
    long lRetVal = -1;
    uint32_t last_ir_msg = 0;
    unsigned long last_ir_time = 0;
    int ir_burst_count = 0;

    BoardInit();

    PinMuxConfig();

    InitTerm();
    ClearTerm();
    UART_PRINT("My terminal works!\n\r");

    // OLED and SPI Initialization
    MAP_PRCMPeripheralClkEnable(PRCM_GSPI, PRCM_RUN_MODE_CLK);
    MAP_SPIReset(GSPI_BASE);
    MAP_PRCMPeripheralReset(PRCM_GSPI);
    MAP_SPIConfigSetExpClk(GSPI_BASE, MAP_PRCMPeripheralClockGet(PRCM_GSPI),
                           1000000, SPI_MODE_MASTER, SPI_SUB_MODE_0,
                           (SPI_SW_CTRL_CS | SPI_4PIN_MODE | SPI_TURBO_OFF |
                            SPI_CS_ACTIVEHIGH | SPI_WL_8));
    MAP_SPIEnable(GSPI_BASE);
    Adafruit_Init();
    fillScreen(BLACK);
    drawFastHLine(0, LINE_Y, 128, WHITE);

    // IR GPIO and Systick Initialization
    SysTickInit();
    MAP_GPIOIntRegister(GPIOA0_BASE, GPIOIntHandler);
    MAP_GPIOIntTypeSet(GPIOA0_BASE, 0x80, GPIO_FALLING_EDGE);
    MAP_GPIOIntClear(GPIOA0_BASE, MAP_GPIOIntStatus(GPIOA0_BASE, false));
    MAP_GPIOIntEnable(GPIOA0_BASE, 0x80);

    // Timer Initialization for multi-tap timing
    Timer_IF_Init(PRCM_TIMERA0, TIMERA0_BASE, TIMER_CFG_PERIODIC, TIMER_A, 0);
    Timer_IF_IntSetup(TIMERA0_BASE, TIMER_A, TimerTickHandler);
    Timer_IF_Start(TIMERA0_BASE, TIMER_A, 1);

    // initialize global default app configuration
    g_app_config.host = OPENSKY_SERVER_NAME;
    g_app_config.port = OPENSKY_PORT;

    //Connect the CC3200 to the local access point
    lRetVal = connectToAccessPoint();
    //Set time so that encryption can be used
    lRetVal = set_time();
    if(lRetVal < 0) {
        UART_PRINT("Unable to set time in the device");
        LOOP_FOREVER();
    }

    UART_PRINT("Ready to receive IR messages and fetch flight data...\n\r");

    while(1) {
        if(message_ready) {
            uint32_t current_msg = message_buffer;
            message_ready = false;
            unsigned long now = tickCount;

            if ((current_msg != last_ir_msg) || ((now - last_ir_time) > DEBOUNCE_TIMEOUT)) {
                ir_burst_count = 0;
                last_ir_msg = current_msg;
            }
            ir_burst_count++;
            last_ir_time = now;

            if (ir_burst_count == 2) {
                UART_PRINT("IR Code: 0x%08x\n\r", current_msg);
                switch(current_msg) {
                    case 0x0000c004: {
                        // Triggers a fetch from OpenSky
                        FetchAndDisplayFlightData();
                        break;
                    }
                    case 0x0000c084: break; // 2
                    case 0x0000c044: ProcessButton(2); break; // 3
                    case 0x0000c0c4: ProcessButton(3); break; // 4
                    case 0x0000c024: ProcessButton(4); break; // 5
                    case 0x0000c0a4: ProcessButton(5); break; // 6
                    case 0x0000c064: ProcessButton(6); break; // 7
                    case 0x0000c0e4: ProcessButton(7); break; // 8
                    case 0x0000c014: ProcessButton(8); break; // 9
                    case 0x0000c094: ProcessButton(9); break; // 10
                    case 0x0000c0a0: DeleteChar(); break; // Mute (Delete)
                    case 0x0000c038: {
                        g_app_config.host = AWS_SERVER_NAME;
                        g_app_config.port = AWS_PORT;
                        int awsSockID = tls_connect(true);
                        if (awsSockID >= 0) {
                            PostIRMessage(awsSockID);
                            sl_Close(awsSockID);
                        } else {
                            UART_PRINT("Failed to connect to AWS IoT.\r\n");
                        }
                        break; 
                    }
                }
            }
        }
    }
}

static int http_post(int iTLSSockID, const char* message){
    char acSendBuff[512];
    char acRecvbuff[1460];
    char cCLLength[200];
    char* pcBufHeaders;
    int lRetVal = 0;

    char data[256];
    sprintf(data, "{\"state\": {\"desired\" : {\"var\" :\"%s\"}}}", message);

    pcBufHeaders = acSendBuff;
    strcpy(pcBufHeaders, AWS_POST_HEADER);
    pcBufHeaders += strlen(AWS_POST_HEADER);
    strcpy(pcBufHeaders, AWS_HOST_HEADER);
    pcBufHeaders += strlen(AWS_HOST_HEADER);
    strcpy(pcBufHeaders, CHEADER);
    pcBufHeaders += strlen(CHEADER);
    strcpy(pcBufHeaders, "\r\n\r\n");

    int dataLength = strlen(data);

    strcpy(pcBufHeaders, CTHEADER);
    pcBufHeaders += strlen(CTHEADER);
    strcpy(pcBufHeaders, CLHEADER1);

    pcBufHeaders += strlen(CLHEADER1);
    sprintf(cCLLength, "%d", dataLength);

    strcpy(pcBufHeaders, cCLLength);
    pcBufHeaders += strlen(cCLLength);
    strcpy(pcBufHeaders, CLHEADER2);
    pcBufHeaders += strlen(CLHEADER2);

    strcpy(pcBufHeaders, data);
    pcBufHeaders += strlen(data);

    UART_PRINT(acSendBuff);


    //
    // Send the packet to the server */
    //
    lRetVal = sl_Send(iTLSSockID, acSendBuff, strlen(acSendBuff), 0);
    if(lRetVal < 0) {
        UART_PRINT("POST failed. Error Number: %i\n\r",lRetVal);
        sl_Close(iTLSSockID);
        GPIO_IF_LedOn(MCU_RED_LED_GPIO);
        return lRetVal;
    }
    lRetVal = sl_Recv(iTLSSockID, &acRecvbuff[0], sizeof(acRecvbuff), 0);
    if(lRetVal < 0) {
        UART_PRINT("Received failed. Error Number: %i\n\r",lRetVal);
        //sl_Close(iSSLSockID);
        GPIO_IF_LedOn(MCU_RED_LED_GPIO);
           return lRetVal;
    }
    else {
        acRecvbuff[lRetVal] = '\0';
        UART_PRINT(acRecvbuff);
        UART_PRINT("\n\r\n\r");
    }

    return 0;
}

static int opensky_http_get(int iTLSSockID){
    char acSendBuff[512];
    char acRecvbuff[1460];
    char* pcBufHeaders;
    int lRetVal = 0;

    pcBufHeaders = acSendBuff;
    strcpy(pcBufHeaders, OPENSKY_GET_HEADER);
    pcBufHeaders += strlen(OPENSKY_GET_HEADER);

    strcpy(pcBufHeaders, OPENSKY_HOST_HEADER);
    pcBufHeaders += strlen(OPENSKY_HOST_HEADER);

    strcpy(pcBufHeaders, CHEADER);
    pcBufHeaders += strlen(CHEADER);

    strcpy(pcBufHeaders, "\r\n");

    UART_PRINT(acSendBuff);

    // Send request to OpenSky API
    lRetVal = sl_Send(iTLSSockID, acSendBuff, strlen(acSendBuff), 0);

    if(lRetVal < 0) {
        UART_PRINT("GET failed, using demo flight data\n\r");
    }

    // Receive response from API
    lRetVal = sl_Recv(iTLSSockID, &acRecvbuff[0], sizeof(acRecvbuff) - 1, 0);

    // If API fails OR returns nothing, load demo JSON
    if(lRetVal <= 0) {

        UART_PRINT("Loading static demo flights\n\r");

        strcpy(acRecvbuff,
        "{ \"states\": ["
        "[\"abc123\",\"UA245\",\"SFO\",\"DEN\",12000],"
        "[\"def456\",\"DL188\",\"SFO\",\"JFK\",10500],"
        "[\"ghi789\",\"SW902\",\"SMF\",\"DEN\",9800],"
        "[\"xyz222\",\"AA411\",\"SMF\",\"BOS\",14200]"
        "] }");

        lRetVal = strlen(acRecvbuff);
    }

    acRecvbuff[lRetVal] = '\0';

    UART_PRINT("--- FLIGHT DATA ---\n\r");
    UART_PRINT(acRecvbuff);
    UART_PRINT("\n\r-------------------\n\r");

    // Demo display logic
    fillRect(0, 0, 128, 64, BLACK);

    setCursor(0,0);
    setTextColor(WHITE, BLACK);
    setTextSize(1);
    Outstr("Flight Demo");

    setCursor(0,16);
    setTextColor(GREEN, BLACK);
    setTextSize(2);
    Outstr("UA245");

    setCursor(0,40);
    setTextColor(WHITE, BLACK);
    setTextSize(1);
    Outstr("SFO -> DEN");

    return 0;
}

// Receive response
lRetVal = sl_Recv(iTLSSockID, &acRecvbuff[0], sizeof(acRecvbuff) - 1, 0);
if(lRetVal < 0) {
    UART_PRINT("Receive failed. Error Number: %i\n\r",lRetVal);
    GPIO_IF_LedOn(MCU_RED_LED_GPIO);
    return lRetVal;
}

acRecvbuff[lRetVal] = '\0';

UART_PRINT("--- OPENSKY RESPONSE ---\n\r");
UART_PRINT(acRecvbuff);
UART_PRINT("\n\r------------------------\n\r");

// NEW: store multiple flights instead of only the first one
typedef struct {
    char callsign[16];
    char altitude[16];
} Flight;

Flight flights[5];          // NEW: store up to 5 flights
int flightCount = 0;        // NEW: track number of parsed flights

char *state_start = strstr(acRecvbuff, "\"states\":");
if (state_start != NULL) {

    state_start = strchr(state_start, '[');

    // NEW: loop through multiple flight arrays
    while (state_start != NULL && flightCount < 5) {

        state_start = strchr(state_start + 1, '[');
        if (state_start == NULL) break;

        char *p = strchr(state_start, ',');
        if (p == NULL) break;

        int i = 0;
        int skip = 0;

        p++;
        while (*p == ' ' || *p == '"') p++;

        while (*p != '"' && *p != ',' && i < 15) {
            flights[flightCount].callsign[i++] = *p++;
        }
        flights[flightCount].callsign[i] = '\0';

        // Skip to altitude field
        for (skip = 0; skip < 6; skip++) {
            p = strchr(p, ',');
            if (p == NULL) break;
            p++;
        }

        if (p != NULL) {
            while (*p == ' ') p++;

            i = 0;
            while (*p != ',' && *p != ']' && *p != ' ' && i < 15) {
                flights[flightCount].altitude[i++] = *p++;
            }
            flights[flightCount].altitude[i] = '\0';
        }

        // NEW: simple filter – ignore empty callsigns
        if (strlen(flights[flightCount].callsign) > 1) {
            flightCount++;
        }
    }
}

// NEW: display multiple flights sequentially on OLED
fillRect(0, 0, 128, 64, BLACK);

if (flightCount == 0) {
    setCursor(0, 0);
    setTextColor(WHITE, BLACK);
    setTextSize(1);
    Outstr("No Flights Found.");
    return 0;
}

int f;
for (f = 0; f < flightCount; f++) {

    fillRect(0, 0, 128, 64, BLACK);

    setCursor(0,0);
    setTextColor(WHITE, BLACK);
    setTextSize(1);
    Outstr("Flight:");

    setCursor(0,16);
    setTextColor(GREEN, BLACK);
    setTextSize(2);
    Outstr(flights[f].callsign);

    setCursor(0,40);
    setTextColor(WHITE, BLACK);
    setTextSize(1);
    Outstr("Alt: ");
    Outstr(flights[f].altitude);
    Outstr(" m");

    MAP_UtilsDelay(8000000); // NEW: pause so user can read each flight
}

// CHANGED: update global variables with first flight for AWS sync
strcpy(current_callsign, flights[0].callsign);
strcpy(current_altitude, flights[0].altitude);
new_flight_data = true;

return 0;
```
}


// IR and Multi-tap Implementation
void SysTickReset(void) {
    SysTickPeriodSet(SYSTICK);
    SysTickEnable();
}

void SysTickInit(void) {
    SysTickPeriodSet(SYSTICK);
    SysTickIntRegister(SysTickReset);
    SysTickEnable();
}

unsigned long TickDifference(void) {
    unsigned long currentTick = SysTickValueGet();
    unsigned long lastTick = ticks;
    ticks = currentTick;

    if (currentTick < lastTick) {
        return (lastTick - currentTick);
    }

    return ((SYSTICK - currentTick) + lastTick);
}

void record_bit(int value) {
    data = (data << 1) | value;
    bit_count++;
}

void GPIOIntHandler(void) {
    unsigned long ulStatus = MAP_GPIOIntStatus(GPIOA0_BASE, true);
    MAP_GPIOIntClear(GPIOA0_BASE, ulStatus);

    delta = TickDifference();

    if (delta > 800000) {
        if (bit_count >= 16) {
            if (!message_ready) {
                message_buffer = data;
                message_ready = true;
            }
        }
        data = 0;
        bit_count = 0;
    }

    if (delta > 150000 && delta < 200000) {
        record_bit(1);
    } else if (delta >= 80000 && delta < 110000) {
        record_bit(0);
    }
}

void TimerTickHandler(void) {
    Timer_IF_InterruptClear(TIMERA0_BASE);
    tickCount++;
}

void ProcessButton(int button) {
    unsigned long now = tickCount;
    const char* chars = multi_tap_chars[button];
    int num_chars = strlen(chars);

    if (num_chars == 0) return;

    if ((button != lastButton) && (lastButton != -1)) {
        charIndex = 0;
    } else if ((button == lastButton) && ((now - lastPressTime) < CONFIRMATION_TIMEOUT)) {
        charIndex = (charIndex + 1) % num_chars;
        cursorXBottom -= 6;
        if (cursorXBottom < 0) {
            if (cursorYBottom > BOTTOM_HALF_Y) {
                cursorXBottom = 120;
                cursorYBottom -= 8;
            } else {
                cursorXBottom = 0;
            }
        }
        composeLength--;
    } else {
        charIndex = 0;
    }

    if (composeLength >= 63) return;

    drawChar(cursorXBottom, cursorYBottom, chars[charIndex], WHITE, BLACK, 1);
    composeBuffer[composeLength++] = chars[charIndex];
    cursorXBottom += 6;

    if (cursorXBottom > 120) {
        cursorXBottom = 0;
        cursorYBottom += 8;
        if (cursorYBottom > 120) {
            fillRect(0, BOTTOM_HALF_Y, 128, 128 - BOTTOM_HALF_Y, BLACK);
            drawFastHLine(0, LINE_Y, 128, WHITE);
            cursorXBottom = 0;
            cursorYBottom = BOTTOM_HALF_Y;
            composeLength = 0;
        }
    }

    lastButton = button;
    lastPressTime = now;
}

void DeleteChar(void) {
    if (composeLength > 0) {
        composeLength--;
        composeBuffer[composeLength] = '\0';
        cursorXBottom -= 6;
        if (cursorXBottom < 0) {
            if (cursorYBottom > BOTTOM_HALF_Y) {
                cursorXBottom = 120;
                cursorYBottom -= 8;
            } else {
                cursorXBottom = 0;
            }
        }
        fillRect(cursorXBottom, cursorYBottom, 6, 8, BLACK);
        lastButton = -1;
    }
}

void PostIRMessage(int iTLSSockID) {
    if (composeLength == 0) return;

    composeBuffer[composeLength] = '\0';
    UART_PRINT("Posting IR Message: '%s'\n\r", composeBuffer);

    http_post(iTLSSockID, composeBuffer);

    // Clear OLED after sending
    fillRect(0, BOTTOM_HALF_Y, 128, 128 - BOTTOM_HALF_Y, BLACK);
    drawFastHLine(0, LINE_Y, 128, WHITE);
    cursorXBottom = 0;
    cursorYBottom = BOTTOM_HALF_Y;
    composeLength = 0;
    lastButton = -1;
    memset(composeBuffer, 0, 64);
}

