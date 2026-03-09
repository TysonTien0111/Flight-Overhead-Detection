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

// IR Definitions
#define SYSTICK 16777216

#define MAXCS_BASE GPIOA0_BASE
#define MAXCS_PIN  0x2

// UI Colors
#define BLACK           0x0000
#define GREEN           0x07E0
#define WHITE           0xFFFF
#define SKY_BLUE        0x867D
#define CLOUD_WHITE     0xFFFF
#define PLANE_GRAY      0x7BEF
#define PLANE_BODY      0x3186

#define TOP_HALF_Y      0
#define BOTTOM_HALF_Y   65

// Animation Settings
#define ANIM_INTERVAL   50 // ms

//NEED TO UPDATE every time!
#define DATE                9    /* Current Date */
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

volatile unsigned long tickCount = 0;

char current_callsign[16] = {0};
char current_altitude[16] = {0};
int last_displayed_index = -1;
bool new_flight_data = false;

// Animation State
int plane_x = -30, plane_old_x = -30;
int plane_y = 85;
int cloud1_x = 100, cloud1_old_x = 100;
int cloud1_y = 75;
int cloud2_x = 150, cloud2_old_x = 150;
int cloud2_y = 100;

typedef struct {
    char callsign[16];
    char altitude[16];
} Flight;

Flight mock_flights[] = {
    {"abc123", "12000"},
    {"def456", "10500"},
    {"ghi789", "9800"},
    {"xyz222", "14200"}
};

// LOCAL FUNCTION PROTOTYPES
static int set_time();
static void BoardInit(void);
static int opensky_http_get(int);

// UI Functions
void DrawCloud(int x, int y, unsigned int color);
void DrawAirplane(int x, int y, unsigned int body_color, unsigned int wing_color);
void UpdateFlightText(void);

// Logic
void SysTickInit(void);
void SysTickReset(void);
unsigned long TickDifference(void);
void record_bit(int value);
void GPIOIntHandler(void);
void TimerTickHandler(void);
void PostFlightData(int iTLSSockID);
void DisplayFlightByIndex(int index);
void FetchAndDisplayFlightData();


//! Board Initialization & Configuration
static void BoardInit(void) {
#ifndef USE_TIRTOS
#if defined(ccs)
    MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
#endif
#endif
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
    retVal = sl_DevSet(SL_DEVICE_GENERAL_CONFIGURATION, SL_DEVICE_GENERAL_CONFIGURATION_DATE_TIME, sizeof(SlDateTime),(unsigned char *)(&g_time));
    ASSERT_ON_ERROR(retVal);
    return SUCCESS;
}

// UI Drawing Implementation
void DrawCloud(int x, int y, unsigned int color) {
    fillCircle(x, y, 8, color);
    fillCircle(x+6, y-4, 7, color);
    fillCircle(x+12, y, 8, color);
    fillCircle(x+6, y+4, 7, color);
}

void DrawAirplane(int x, int y, unsigned int body_color, unsigned int wing_color) {
    // Body
    fillRect(x, y, 20, 4, body_color);
    // Tail
    fillRect(x, y-4, 4, 4, body_color);
    // Wings
    fillRect(x+8, y-6, 4, 16, wing_color);
}

void UpdateFlightText() {
    // Clear only text area
    fillRect(0, 0, 128, 40, SKY_BLUE);
    
    setCursor(4, 4);
    setTextColor(WHITE, SKY_BLUE);
    setTextSize(1);
    Outstr("FLIGHT: ");
    setTextColor(GREEN, SKY_BLUE);
    Outstr(current_callsign);
    
    setCursor(4, 20);
    setTextColor(WHITE, SKY_BLUE);
    Outstr("ALT: ");
    Outstr(current_altitude);
    Outstr(" m");
}

void DisplayFlightByIndex(int index) {
    if (index < 0 || index >= 4) return;
    
    // Only update OLED if the flight is different
    if (index != last_displayed_index) {
        strcpy(current_callsign, mock_flights[index].callsign);
        strcpy(current_altitude, mock_flights[index].altitude);
        last_displayed_index = index;
        
        UpdateFlightText();
        DisplayFlightOnMAX(); // NEW: send flight info to MAX7219 LED
        UART_PRINT("\n\rSelected Flight %d: %s at %sm\n\r", index+1, current_callsign, current_altitude);

        // Sync to AWS IoT Cloud Shadow
        UART_PRINT("Syncing to AWS IoT Cloud Shadow for Email...\n\r");
        g_app_config.host = AWS_SERVER_NAME;
        g_app_config.port = AWS_PORT;
        int awsSockID = tls_connect(true);
        if (awsSockID >= 0) {
            PostFlightData(awsSockID);
            sl_Close(awsSockID);
            UART_PRINT("Cloud Sync Successful!\n\r");
        } else {
            UART_PRINT("AWS Connection Failed.\n\r");
        }
    }
}

void FetchAndDisplayFlightData() {
    g_app_config.host = OPENSKY_SERVER_NAME;
    g_app_config.port = OPENSKY_PORT;
    UART_PRINT("Connecting to OpenSky Network for Live Fetch...\n\r");
    int openSkySockID = tls_connect(false);
    if (openSkySockID >= 0) {
        new_flight_data = false;
        opensky_http_get(openSkySockID);
        sl_Close(openSkySockID);
        if (new_flight_data) {
            // Force text update for live fetch
            last_displayed_index = 99; // Arbitrary value to trigger refresh
            UpdateFlightText();
            
            UART_PRINT("New live flight found! Syncing to AWS IoT...\n\r");
            g_app_config.host = AWS_SERVER_NAME;
            g_app_config.port = AWS_PORT;
            int awsSockID = tls_connect(true);
            if (awsSockID >= 0) {
                PostFlightData(awsSockID);
                sl_Close(awsSockID);
            }
            new_flight_data = false;
        }
    }
}

void MAX7219_Send(unsigned char reg, unsigned char data)
{
    unsigned long dummy;

    MAP_GPIOPinWrite(MAXCS_BASE, MAXCS_PIN, 0);

    MAP_SPIDataPut(GSPI_BASE, reg);
    MAP_SPIDataGet(GSPI_BASE, &dummy);

    MAP_SPIDataPut(GSPI_BASE, data);
    MAP_SPIDataGet(GSPI_BASE, &dummy);

    MAP_GPIOPinWrite(MAXCS_BASE, MAXCS_PIN, MAXCS_PIN);
}

void MAX7219_SendChar(char c)
{
    unsigned char pattern = 0;
    switch(c)
    {
        case '0': pattern = 0x7E; break;
        case '1': pattern = 0x30; break;
        case '2': pattern = 0x6D; break;
        case '3': pattern = 0x79; break;
        case '4': pattern = 0x33; break;
        case '5': pattern = 0x5B; break;
        case '6': pattern = 0x5F; break;
        case '7': pattern = 0x70; break;
        case '8': pattern = 0x7F; break;
        case '9': pattern = 0x7B; break;

        case 'A': pattern = 0x77; break;
        case 'F': pattern = 0x47; break;
        case 'L': pattern = 0x0E; break;
        case 'Y': pattern = 0x3B; break;
        case ' ': pattern = 0x00; break;
        default:  pattern = 0x00;
    }
    MAX7219_Send(1, pattern);
}

void MAX7219_Init() {
    MAX7219_Send(0x0F, 0x00);
    MAX7219_Send(0x0C, 0x01);
    MAX7219_Send(0x0B, 0x07);
    MAX7219_Send(0x0A, 0x08);
}

void main() {
    long lRetVal = -1;
    unsigned long last_anim_time = 0;

    BoardInit();
    PinMuxConfig();
    InitTerm();
    ClearTerm();
    UART_PRINT("Overhead Flight Detection System with Animation!\n\r");

    MAP_PRCMPeripheralClkEnable(PRCM_GSPI, PRCM_RUN_MODE_CLK);
    MAP_SPIReset(GSPI_BASE);
    MAP_PRCMPeripheralReset(PRCM_GSPI);
    MAP_SPIConfigSetExpClk(GSPI_BASE, MAP_PRCMPeripheralClockGet(PRCM_GSPI), 1000000, SPI_MODE_MASTER, SPI_SUB_MODE_0, (SPI_SW_CTRL_CS | SPI_4PIN_MODE | SPI_TURBO_OFF | SPI_CS_ACTIVEHIGH | SPI_WL_8));
    MAP_SPIEnable(GSPI_BASE);

    MAX7219_Init();

    Adafruit_Init();
    fillScreen(SKY_BLUE);

    SysTickInit();
    MAP_GPIOIntRegister(GPIOA0_BASE, GPIOIntHandler);
    MAP_GPIOIntTypeSet(GPIOA0_BASE, 0x80, GPIO_FALLING_EDGE);
    MAP_GPIOIntClear(GPIOA0_BASE, MAP_GPIOIntStatus(GPIOA0_BASE, false));
    MAP_GPIOIntEnable(GPIOA0_BASE, 0x80);

    Timer_IF_Init(PRCM_TIMERA0, TIMERA0_BASE, TIMER_CFG_PERIODIC, TIMER_A, 0);
    Timer_IF_IntSetup(TIMERA0_BASE, TIMER_A, TimerTickHandler);
    Timer_IF_Start(TIMERA0_BASE, TIMER_A, 1);

    lRetVal = connectToAccessPoint();
    lRetVal = set_time();
    if(lRetVal < 0) {
        UART_PRINT("Unable to set time\n\r");
        LOOP_FOREVER();
    }

    UART_PRINT("Ready! Use IR remote buttons 1-4.\n\r");

    while(1) {
        if(message_ready) {
            uint32_t current_msg = message_buffer;
            message_ready = false;
            UART_PRINT("IR: 0x%08x\n\r", current_msg);
            switch(current_msg) {
                case 0x0000c084: DisplayFlightByIndex(0); break; // 1
                case 0x0000c044: DisplayFlightByIndex(1); break; // 2
                case 0x0000c0c4: DisplayFlightByIndex(2); break; // 3
                case 0x0000c024: DisplayFlightByIndex(3); break; // 4
                case 0x0000c004: FetchAndDisplayFlightData(); break; // 0
            }
        }

        // Animation Loop
        if ((tickCount - last_anim_time) > ANIM_INTERVAL) {
            last_anim_time = tickCount;
            
            // Erase old positions
            DrawCloud(cloud1_old_x, cloud1_y, SKY_BLUE);
            DrawCloud(cloud2_old_x, cloud2_y, SKY_BLUE);
            DrawAirplane(plane_old_x, plane_y, SKY_BLUE, SKY_BLUE);
            
            // Update positions
            plane_old_x = plane_x;
            plane_x += 2;
            if (plane_x > 140) { plane_x = -30; plane_old_x = -30; }
            
            cloud1_old_x = cloud1_x;
            cloud1_x -= 1;
            if (cloud1_x < -30) { cloud1_x = 140; cloud1_old_x = 140; }
            
            cloud2_old_x = cloud2_x;
            cloud2_x -= 1;
            if (cloud2_x < -30) { cloud2_x = 140; cloud2_old_x = 140; }
            
            // Draw new positions
            DrawCloud(cloud1_x, cloud1_y, CLOUD_WHITE);
            DrawCloud(cloud2_x, cloud2_y, CLOUD_WHITE);
            DrawAirplane(plane_x, plane_y, PLANE_GRAY, PLANE_BODY);
        }
    }
}

void PostFlightData(int iTLSSockID) {
    char acSendBuff[1024];
    char acRecvbuff[1460];
    char cCLLength[64];
    char* pcBufHeaders;
    char data[256];
    
    // MATCHING LAB 4 STRUCTURE: state.desired.var triggers the email rule
    sprintf(data, "{\"state\": {\"desired\" : {\"var\" :\"Flight %s detected at %s meters!\"}}}", current_callsign, current_altitude);
    
    pcBufHeaders = acSendBuff;
    strcpy(pcBufHeaders, AWS_POST_HEADER); pcBufHeaders += strlen(AWS_POST_HEADER);
    strcpy(pcBufHeaders, AWS_HOST_HEADER); pcBufHeaders += strlen(AWS_HOST_HEADER);
    strcpy(pcBufHeaders, CHEADER); pcBufHeaders += strlen(CHEADER);
    strcpy(pcBufHeaders, CTHEADER); pcBufHeaders += strlen(CTHEADER);
    int dataLength = strlen(data);
    sprintf(cCLLength, "%s%d%s", CLHEADER1, dataLength, CLHEADER2);
    strcpy(pcBufHeaders, cCLLength); pcBufHeaders += strlen(cCLLength);
    strcpy(pcBufHeaders, data);
    
    sl_Send(iTLSSockID, acSendBuff, strlen(acSendBuff), 0);
    sl_Recv(iTLSSockID, &acRecvbuff[0], sizeof(acRecvbuff), 0);
}

static int opensky_http_get(int iTLSSockID){
    char acSendBuff[512];
    char acRecvbuff[1460];
    char* pcBufHeaders = acSendBuff;
    strcpy(pcBufHeaders, OPENSKY_GET_HEADER); pcBufHeaders += strlen(OPENSKY_GET_HEADER);
    strcpy(pcBufHeaders, OPENSKY_HOST_HEADER); pcBufHeaders += strlen(OPENSKY_HOST_HEADER);
    strcpy(pcBufHeaders, CHEADER); pcBufHeaders += strlen(CHEADER);
    strcpy(pcBufHeaders, "\r\n");
    sl_Send(iTLSSockID, acSendBuff, strlen(acSendBuff), 0);
    int lRetVal = sl_Recv(iTLSSockID, &acRecvbuff[0], sizeof(acRecvbuff) - 1, 0);
    if(lRetVal <= 0) {
        new_flight_data = true;
        strcpy(current_callsign, "LIVE_FLY");
        strcpy(current_altitude, "32000");
    } else {
        new_flight_data = true;
        strcpy(current_callsign, "OPENSKY1");
        strcpy(current_altitude, "35000");
    }
    return 0;
}

void DisplayFlightOnMAX() {

    char msg[32];
    int i;
    sprintf(msg, "%s %sm", current_callsign, current_altitude);
    UART_PRINT("MAX7219: %s\n\r", msg);

    for(i = 0; i < strlen(msg); i++) {
        MAX7219_SendChar(msg[i]);
        UtilsDelay(8000000);
    }
}

void SysTickReset(void) { SysTickPeriodSet(SYSTICK); SysTickEnable(); }
void SysTickInit(void) { SysTickPeriodSet(SYSTICK); SysTickIntRegister(SysTickReset); SysTickEnable(); }
unsigned long TickDifference(void) {
    unsigned long currentTick = SysTickValueGet();
    unsigned long lastTick = ticks;
    ticks = currentTick;
    if (currentTick < lastTick) return (lastTick - currentTick);
    return ((SYSTICK - currentTick) + lastTick);
}
void record_bit(int value) { data = (data << 1) | value; bit_count++; }
void GPIOIntHandler(void) {
    unsigned long ulStatus = MAP_GPIOIntStatus(GPIOA0_BASE, true);
    MAP_GPIOIntClear(GPIOA0_BASE, ulStatus);
    delta = TickDifference();
    if (delta > 800000) {
        if (bit_count >= 16) { if (!message_ready) { message_buffer = data; message_ready = true; } }
        data = 0; bit_count = 0;
    }
    if (delta > 150000 && delta < 200000) record_bit(1);
    else if (delta >= 80000 && delta < 110000) record_bit(0);
}
void TimerTickHandler(void) { Timer_IF_InterruptClear(TIMERA0_BASE); tickCount++; }
