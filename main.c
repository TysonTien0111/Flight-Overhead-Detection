// Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/ 

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

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
#include "i2c.h"

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
#include "i2c_if.h"
#include "world_map.h"

#define PI 3.14159265

// IR Definitions
#define SYSTICK 16777216

// UI Colors
#define BLACK           0x0000
#define GREEN           0x07E0
#define WHITE           0xFFFF
#define YELLOW          0xFFE0
#define CYAN            0x07FF
#define MAP_OCEAN       0x001F
#define MAP_LAND        0x07E0
#define RED             0xF800
#define GRAY            0x7BCF

// BMA222 I2C Definitions
#define BMA222_ADDR     0x18
#define BMA222_X_REG    0x03

//NEED TO UPDATE every time!
#define DATE                8    /* Current Date */
#define MONTH               3     /* Month 1-12 */
#define YEAR                2026  /* Current year */
#define HOUR                13    /* Time - hours */
#define MINUTE              0    /* Time - minutes */
#define SECOND              0     /* Time - seconds */

#define APPLICATION_NAME      "SSL"
#define APPLICATION_VERSION   "SQ24"

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
unsigned long last_interaction_time = 0;
unsigned long last_tilt_time = 0;

int last_displayed_index = -1;
bool new_flight_data = false;
bool sync_needed = false;

// View Mode State
typedef enum {
    VIEW_TEXT = 0,
    VIEW_MAP,
    VIEW_COMPASS,
    VIEW_PROGRESS,
    VIEW_RADAR
} CurrentView;

CurrentView current_view = VIEW_TEXT;
bool view_drawn = false;
unsigned long last_blink_time = 0;
bool blink_state = false;
float radar_angle = 0.0;
unsigned long last_radar_update = 0;

volatile bool interrupt_draw = false; // Priority bailout flag
int current_group = 0; // 0, 1, 2
int current_button_idx = 0; // 0-8

typedef struct {
    char callsign[16];
    char country[24];
    char category[20];
    char lat[12];
    char lon[12];
    char timestamp[16];
    int heading;
    char origin_code[4];
    char dest_code[4];
    int percent_complete;
} Flight;

// 27 Globally dispersed mock flights
Flight mock_flights[27] = {
    // GROUP 0: Western Hemisphere
    {"SWA1234", "USA",           "Large", "38.512",  "-121.501", "1710162000", 270, "SMF", "HNL", 45}, // 1
    {"UAL456",  "USA",           "Large", "40.712",  "-74.006",  "1710162005", 90,  "LAX", "JFK", 80}, // 2
    {"TAM789",  "Brazil",        "Large", "-23.550", "-46.633",  "1710162010", 180, "MIA", "GRU", 65}, // 3
    {"ACA098",  "Canada",        "Heavy", "43.677",  "-79.624",  "1710162011", 45,  "YVR", "LHR", 15}, // 4
    {"AMX43",   "Mexico",        "Medium","19.432",  "-99.133",  "1710162012", 220, "MEX", "BOG", 55}, // 5
    {"LAN12",   "Chile",         "Heavy", "-33.448", "-70.669",  "1710162013", 160, "LIM", "SCL", 90}, // 6
    {"DAL90",   "USA",           "Large", "33.640",  "-84.427",  "1710162014", 315, "ATL", "SEA", 30}, // 7
    {"AAL50",   "USA",           "Large", "32.899",  "-97.040",  "1710162015", 0,   "DFW", "ORD", 70}, // 8
    {"JBU11",   "USA",           "Medium","42.360",  "-71.002",  "1710162016", 180, "BOS", "FLL", 50}, // 9

    // GROUP 1: Europe & Africa
    {"DLH222",  "Germany",       "Heavy", "51.165",  "10.451",   "1710162015", 45,  "FRA", "PEK", 20}, // 1
    {"BAW99",   "UK",            "Heavy", "51.507",  "-0.127",   "1710162020", 315, "LHR", "JFK", 30}, // 2
    {"AFR101",  "France",        "Heavy", "48.856",  "2.352",    "1710162025", 225, "CDG", "EZE", 50}, // 3
    {"KLM44",   "Netherlands",   "Large", "52.308",  "4.768",    "1710162026", 90,  "AMS", "DXB", 60}, // 4
    {"RYR12",   "Ireland",       "Medium","53.349",  "-6.260",   "1710162027", 135, "DUB", "MAD", 85}, // 5
    {"SWR55",   "Switzerland",   "Heavy", "47.458",  "8.555",    "1710162028", 270, "ZRH", "SFO", 40}, // 6
    {"EZY98",   "UK",            "Medium","41.902",  "12.496",   "1710162029", 180, "LGW", "FCO", 75}, // 7
    {"MSR33",   "Egypt",         "Large", "30.044",  "31.235",   "1710162030", 90,  "CAI", "LHR", 25}, // 8
    {"SAA11",   "South Africa",  "Heavy", "-26.204", "28.047",   "1710162031", 315, "JNB", "JFK", 10}, // 9

    // GROUP 2: Asia & Oceania
    {"ANA52",   "Japan",         "Heavy", "35.676",  "139.650",  "1710162030", 0,   "HND", "SFO", 15}, // 1
    {"CES202",  "China",         "Heavy", "39.904",  "116.407",  "1710162035", 135, "PEK", "SYD", 75}, // 2
    {"QFA88",   "Australia",     "Medium","-33.868", "151.209",  "1710162040", 45,  "SYD", "LAX", 90}, // 3
    {"SIA33",   "Singapore",     "Heavy", "1.352",   "103.819",  "1710162041", 270, "SIN", "LHR", 50}, // 4
    {"UAE10",   "UAE",           "Heavy", "25.204",  "55.270",   "1710162042", 315, "DXB", "JFK", 65}, // 5
    {"CPA55",   "Hong Kong",     "Heavy", "22.308",  "113.914",  "1710162043", 90,  "HKG", "YVR", 35}, // 6
    {"AIC40",   "India",         "Large", "28.613",  "77.209",   "1710162044", 225, "DEL", "CDG", 80}, // 7
    {"KAL70",   "South Korea",   "Heavy", "37.566",  "126.978",  "1710162045", 180, "ICN", "AKL", 20}, // 8
    {"ANZ22",   "New Zealand",   "Large", "-36.848", "174.763",  "1710162046", 45,  "AKL", "SFO", 10}  // 9
};

// LOCAL FUNCTION PROTOTYPES
static int set_time();
static void BoardInit(void);

// UI Functions
void UpdateFlightDisplay(const char* status, int index);
void DrawMonochromeBitmap(int x, int y, const unsigned char *bitmap, int w, int h, unsigned int color);
void UpdateMapDisplay(void);
void UpdateCompassDisplay(void);
void UpdateProgressDisplay(void);
void UpdateRadarDisplay(void);
void SwitchViewMode(CurrentView view);

// Math Helpers
int get_x_offset(int degrees, int radius);
int get_y_offset(int degrees, int radius);

// Logic
void SysTickInit(void);
void SysTickReset(void);
unsigned long TickDifference(void);
void record_bit(int value);
void GPIOIntHandler(void);
void TimerTickHandler(void);
void PostFlightData(int iTLSSockID, int index);
void DisplayFlightByIndex(int index);
void ExecuteAWSSync();
void ProcessTilt();


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

int get_x_offset(int degrees, int radius) {
    return (int)(radius * sin(degrees * PI / 180.0));
}

int get_y_offset(int degrees, int radius) {
    return (int)(-radius * cos(degrees * PI / 180.0));
}

void DrawMonochromeBitmap(int x, int y, const unsigned char *bitmap, int w, int h, unsigned int color) {
    int i, j, byteWidth = (w + 7) / 8;
    for(j = 0; j < h; j++) {
        for(i = 0; i < w; i++) {
            if (interrupt_draw) return; 
            if(bitmap[j * byteWidth + i / 8] & (128 >> (i & 7))) {
                drawPixel(x + i, y + j, color);
            }
        }
    }
}

void UpdateFlightDisplay(const char* status, int index) {
    fillScreen(BLACK);
    if (index >= 0 && index < 27) {
        Flight f = mock_flights[index];
        setCursor(0, 5); setTextColor(YELLOW, BLACK); setTextSize(1);
        Outstr("FLIGHT TRACKING");
        setCursor(95, 5); setTextColor(CYAN, BLACK);
        char grp[8]; sprintf(grp, "GRP %d", current_group); Outstr(grp);
        setCursor(0, 20); setTextColor(GREEN, BLACK); setTextSize(2);
        Outstr(f.callsign);
        setTextSize(1); setTextColor(WHITE, BLACK);
        setCursor(0, 45); Outstr("Origin: "); Outstr(f.country);
        setCursor(0, 58); Outstr("Type:   "); Outstr(f.category);
        setCursor(0, 75); setTextColor(CYAN, BLACK);
        Outstr("LAT: "); Outstr(f.lat);
        setCursor(0, 88); Outstr("LON: "); Outstr(f.lon);
        setCursor(0, 101); setTextColor(WHITE, BLACK);
        Outstr("Time: "); Outstr(f.timestamp);
    } else {
        setCursor(10, 50); setTextColor(WHITE, BLACK);
        Outstr("Awaiting Input...");
    }
    fillRect(0, 115, 128, 13, BLACK); 
    setCursor(0, 118); setTextColor(WHITE, BLACK);
    Outstr("Status: "); Outstr(status);
}

void UpdateMapDisplay() {
    if (!view_drawn) {
        fillScreen(MAP_OCEAN);
        if (interrupt_draw) return;
        DrawMonochromeBitmap(0, 0, world_map, 128, 128, MAP_LAND);
        if (interrupt_draw) return;
        setCursor(0, 0); setTextColor(WHITE, MAP_OCEAN);
        Outstr("MAP: "); Outstr(mock_flights[last_displayed_index].callsign);
        view_drawn = true;
    }
    if (last_displayed_index >= 0 && last_displayed_index < 27) {
        Flight f = mock_flights[last_displayed_index];
        int x = (int)((atof(f.lon) + 180.0) * (128.0 / 360.0));
        int y = (int)((90.0 - atof(f.lat)) * (128.0 / 180.0));
        if ((tickCount - last_blink_time) > 500) {
            blink_state = !blink_state; last_blink_time = tickCount;
            if (interrupt_draw) return;
            int i; for(i = 1; i <= 4; i++) drawPixel(x - i*4, y + i*4, RED);
            if (blink_state) fillCircle(x, y, 3, RED);
            else fillCircle(x, y, 3, MAP_OCEAN); 
        }
    }
}

void UpdateCompassDisplay() {
    if (!view_drawn) {
        fillScreen(BLACK);
        if (interrupt_draw) return;
        setCursor(0, 0); setTextColor(YELLOW, BLACK); Outstr("COMPASS: ");
        setTextColor(WHITE, BLACK); Outstr(mock_flights[last_displayed_index].callsign);
        fillCircle(64, 68, 42, 0x0125); 
        if (interrupt_draw) return;
        fillCircle(64, 68, 40, BLACK);  
        int i;
        for(i = 0; i < 360; i += 45) {
            if (interrupt_draw) return;
            drawLine(64+get_x_offset(i,35), 68+get_y_offset(i,35), 64+get_x_offset(i,40), 68+get_y_offset(i,40), CYAN);
        }
        setCursor(61, 20); setTextColor(YELLOW, BLACK); Outstr("N");
        setCursor(108, 64); Outstr("E");
        setCursor(61, 109); Outstr("S");
        setCursor(16, 64); Outstr("W");
        int heading = mock_flights[last_displayed_index].heading;
        if (interrupt_draw) return;
        int tip_x = 64 + get_x_offset(heading, 38);
        int tip_y = 68 + get_y_offset(heading, 38);
        int tail_x = 64 + get_x_offset(heading + 180, 38);
        int tail_y = 68 + get_y_offset(heading + 180, 38);
        int bl_x = 64 + get_x_offset(heading - 90, 4);
        int bl_y = 68 + get_y_offset(heading - 90, 4);
        int br_x = 64 + get_x_offset(heading + 90, 4);
        int br_y = 68 + get_y_offset(heading + 90, 4);
        fillTriangle(tip_x, tip_y, bl_x, bl_y, br_x, br_y, RED);
        if (interrupt_draw) return;
        fillTriangle(tail_x, tail_y, bl_x, bl_y, br_x, br_y, WHITE);
        fillCircle(64, 68, 3, GRAY); 
        char buf[16]; sprintf(buf, "Hdg: %03d DEG", heading);
        setCursor(30, 118); setTextColor(GREEN, BLACK); Outstr(buf);
        view_drawn = true;
    }
}

const unsigned char airplane_bmp[] = { 0x18, 0x3C, 0x7E, 0xFF, 0x3C, 0x3C, 0x7E, 0x00 };

void UpdateProgressDisplay() {
    static int anim_x = 10;
    if (!view_drawn) {
        fillScreen(BLACK);
        if (interrupt_draw) return;
        Flight f = mock_flights[last_displayed_index];
        setCursor(0, 5); setTextColor(YELLOW, BLACK); Outstr("EN ROUTE");
        setCursor(10, 35); setTextColor(CYAN, BLACK); setTextSize(2); Outstr(f.origin_code);
        setCursor(94, 35); Outstr(f.dest_code); setTextSize(1);
        fillRect(10, 65, 108, 16, 0x2104);
        drawRect(10, 65, 108, 16, WHITE);
        if (interrupt_draw) return;
        char buf[20]; sprintf(buf, "Status: %d%%", f.percent_complete);
        setCursor(30, 95); setTextColor(GREEN, BLACK); Outstr(buf);
        anim_x = 10; view_drawn = true;
    }
    int target_x = 10 + (108 * mock_flights[last_displayed_index].percent_complete) / 100;
    if ((tickCount - last_blink_time) > 20) {
        last_blink_time = tickCount;
        if (anim_x < target_x) {
            if (interrupt_draw) return;
            fillRect(anim_x - 4, 52, 8, 8, BLACK);
            drawFastVLine(anim_x, 66, 14, GREEN);
            anim_x++;
            if (interrupt_draw) return;
            DrawMonochromeBitmap(anim_x - 4, 52, airplane_bmp, 8, 8, WHITE);
        }
    }
}

void UpdateRadarDisplay() {
    if (!view_drawn) {
        fillScreen(BLACK);
        if (interrupt_draw) return;
        fillCircle(64, 68, 60, 0x0100); 
        if (interrupt_draw) return;
        drawCircle(64, 68, 20, GREEN); drawCircle(64, 68, 40, GREEN); drawCircle(64, 68, 60, GREEN);
        drawFastHLine(4, 68, 120, GREEN); drawFastVLine(64, 8, 120, GREEN);
        setCursor(0, 0); setTextColor(YELLOW, BLACK); Outstr("RADAR: ");
        setTextColor(WHITE, BLACK); Outstr(mock_flights[last_displayed_index].callsign);
        view_drawn = true; radar_angle = 0.0;
    }
    if ((tickCount - last_radar_update) > 150) {
        last_radar_update = tickCount;
        if (interrupt_draw) return;
        drawLine(64, 68, 64+get_x_offset((int)radar_angle, 58), 68+get_y_offset((int)radar_angle, 58), 0x0100);
        radar_angle = (float)((int)(radar_angle + 10) % 360);
        if (interrupt_draw) return;
        drawLine(64, 68, 64+get_x_offset((int)radar_angle, 58), 68+get_y_offset((int)radar_angle, 58), GREEN);
        if (interrupt_draw) return;
        drawCircle(64, 68, 20, GREEN); drawCircle(64, 68, 40, GREEN); drawCircle(64, 68, 60, GREEN);
        drawFastHLine(4, 68, 120, GREEN); drawFastVLine(64, 8, 120, GREEN);
        char angle_buf[16]; sprintf(angle_buf, "%03d DEG", (int)radar_angle);
        setCursor(85, 118); setTextColor(GREEN, BLACK); Outstr(angle_buf);
        int target_angle = (last_displayed_index * 40) % 360;
        int target_radius = 15 + (last_displayed_index * 5) % 40;
        int tx = 64 + get_x_offset(target_angle, target_radius);
        int ty = 68 + get_y_offset(target_angle, target_radius);
        float diff = radar_angle - (float)target_angle; if (diff < 0) diff += 360.0;
        if (interrupt_draw) return;
        if (diff < 20.0) fillCircle(tx, ty, 3, WHITE);
        else if (diff < 50.0) fillCircle(tx, ty, 3, RED);
        else if (diff < 100.0) fillCircle(tx, ty, 2, 0x7800);
        else fillCircle(tx, ty, 3, 0x0100);
    }
}

void SwitchViewMode(CurrentView view) {
    if (last_displayed_index == -1) return;
    if (current_view == view) return;
    UART_PRINT("UI: Switching View %d\n\r", (int)view);
    current_view = view; view_drawn = false;
}

void DisplayFlightByIndex(int index) {
    if (index < 0) index = 26; if (index >= 27) index = 0;
    last_displayed_index = index; current_view = VIEW_TEXT; view_drawn = false; 
    UART_PRINT("UI: Displaying Flight Index %d (Mode: TEXT)\n\r", index);
    UpdateFlightDisplay("Selected", index);
}

void ExecuteAWSSync() {
    if (current_view == VIEW_TEXT) UpdateFlightDisplay("Syncing Cloud...", last_displayed_index);
    UART_PRINT("AWS: Manual Sync for %s\n\r", mock_flights[last_displayed_index].callsign);
    g_app_config.host = AWS_SERVER_NAME; g_app_config.port = AWS_PORT;
    int awsSockID = tls_connect(true);
    if (awsSockID >= 0) { PostFlightData(awsSockID, last_displayed_index); sl_Close(awsSockID); if (current_view == VIEW_TEXT) UpdateFlightDisplay("Tracked", last_displayed_index); }
    else if (current_view == VIEW_TEXT) UpdateFlightDisplay("Sync Failed", last_displayed_index);
}

void ProcessTilt() {
    signed char x = 0; unsigned char devAddr = 0x18, startingReg = 0x03, buffer[1];
    if (I2C_IF_ReadFrom(devAddr, &startingReg, 1, buffer, 1) == 0) {
        x = (signed char)buffer[0];
        static unsigned long last_log = 0; if ((tickCount - last_log) > 1000) { UART_PRINT("ACCEL X: %d\n\r", x); last_log = tickCount; }
        if ((tickCount - last_tilt_time) > 1000) {
            if (x > 50) { last_tilt_time = tickCount; DisplayFlightByIndex(last_displayed_index - 1); }
            else if (x < -50) { last_tilt_time = tickCount; DisplayFlightByIndex(last_displayed_index + 1); }
        }
    }
}

void main() {
    BoardInit(); PinMuxConfig(); InitTerm(); ClearTerm();
    UART_PRINT("Standalone Flight Tracker (Priority IR Booted)\n\r");
    MAP_PRCMPeripheralClkEnable(PRCM_GSPI, PRCM_RUN_MODE_CLK);
    MAP_SPIReset(GSPI_BASE); MAP_PRCMPeripheralReset(PRCM_GSPI);
    MAP_SPIConfigSetExpClk(GSPI_BASE, MAP_PRCMPeripheralClockGet(PRCM_GSPI), 1000000, SPI_MODE_MASTER, SPI_SUB_MODE_0, (SPI_SW_CTRL_CS | SPI_4PIN_MODE | SPI_TURBO_OFF | SPI_CS_ACTIVEHIGH | SPI_WL_8));
    MAP_SPIEnable(GSPI_BASE); Adafruit_Init(); I2C_IF_Open(I2C_MASTER_MODE_FST);
    fillScreen(BLACK); UpdateFlightDisplay("Idle", -1);
    SysTickInit(); MAP_GPIOIntRegister(GPIOA0_BASE, GPIOIntHandler);
    MAP_GPIOIntTypeSet(GPIOA0_BASE, 0x80, GPIO_FALLING_EDGE);
    MAP_GPIOIntClear(GPIOA0_BASE, MAP_GPIOIntStatus(GPIOA0_BASE, false));
    MAP_GPIOIntEnable(GPIOA0_BASE, 0x80);
    Timer_IF_Init(PRCM_TIMERA0, TIMERA0_BASE, TIMER_CFG_PERIODIC, TIMER_A, 0);
    Timer_IF_IntSetup(TIMERA0_BASE, TIMER_A, TimerTickHandler);
    Timer_IF_Start(TIMERA0_BASE, TIMER_A, 1);
    connectToAccessPoint(); set_time();
    while(1) {
        interrupt_draw = false; 
        if(message_ready) {
            uint32_t current_msg = message_buffer; message_ready = false;
            UART_PRINT("IR RECEIVED: 0x%08x\n\r", current_msg);
            switch(current_msg) {
                case 0x0000c084: current_button_idx = 0; DisplayFlightByIndex((current_group * 9) + 0); break;
                case 0x0000c044: current_button_idx = 1; DisplayFlightByIndex((current_group * 9) + 1); break;
                case 0x0000c0c4: current_button_idx = 2; DisplayFlightByIndex((current_group * 9) + 2); break;
                case 0x0000c024: current_button_idx = 3; DisplayFlightByIndex((current_group * 9) + 3); break;
                case 0x0000c0a4: current_button_idx = 4; DisplayFlightByIndex((current_group * 9) + 4); break;
                case 0x0000c064: current_button_idx = 5; DisplayFlightByIndex((current_group * 9) + 5); break;
                case 0x0000c0e4: current_button_idx = 6; DisplayFlightByIndex((current_group * 9) + 6); break;
                case 0x0000c014: current_button_idx = 7; DisplayFlightByIndex((current_group * 9) + 7); break;
                case 0x0000c094: current_button_idx = 8; DisplayFlightByIndex((current_group * 9) + 8); break;
                case 0x0000c004: if (current_view == VIEW_TEXT) { current_group = (current_group + 1) % 3; DisplayFlightByIndex((current_group * 9) + current_button_idx); } break;
                case 0x0000c098: SwitchViewMode(VIEW_MAP); break; 
                case 0x0000c0f8: 
                case 0xc0f8c0f8: SwitchViewMode(VIEW_COMPASS); break;
                case 0x0000c078: SwitchViewMode(VIEW_PROGRESS); break;
                case 0x0000c018: SwitchViewMode(VIEW_RADAR); break;
                case 0x0000c050: if (last_displayed_index != -1) ExecuteAWSSync(); break;
            }
        }
        if (!interrupt_draw) {
            if (current_view == VIEW_MAP) UpdateMapDisplay();
            else if (current_view == VIEW_COMPASS) UpdateCompassDisplay();
            else if (current_view == VIEW_PROGRESS) UpdateProgressDisplay();
            else if (current_view == VIEW_RADAR) UpdateRadarDisplay();
            ProcessTilt();
        }
    }
}

void PostFlightData(int iTLSSockID, int index) {
    char acSendBuff[1024], acRecvbuff[1460], cCLLength[64], data[512], *pcBufHeaders = acSendBuff;
    Flight f = mock_flights[index];
    if (current_view == VIEW_MAP) sprintf(data, "{\"state\": {\"desired\" : {\"var\" :\"Map triggered for Flight %s!\\nView on map: https://www.google.com/maps/search/?api=1&query=%s,%s\"}}}", f.callsign, f.lat, f.lon);
    else if (current_view == VIEW_COMPASS) sprintf(data, "{\"state\": {\"desired\" : {\"var\" :\"Compass Check: Flight %s\\nHeading: %03d Degrees\"}}}", f.callsign, f.heading);
    else if (current_view == VIEW_PROGRESS) sprintf(data, "{\"state\": {\"desired\" : {\"var\" :\"Progress Update for Flight %s!\\nRoute: %s to %s\\nCompletion: %d%%\"}}}", f.callsign, f.origin_code, f.dest_code, f.percent_complete);
    else if (current_view == VIEW_RADAR) sprintf(data, "{\"state\": {\"desired\" : {\"var\" :\"Radar sweeping Flight %s!\\nSweep Angle: %d\"}}}", f.callsign, (int)radar_angle);
    else sprintf(data, "{\"state\": {\"desired\" : {\"var\" :\"Flight: %s\\nOrigin: %s\\nType: %s\\nLat: %s\\nLon: %s\\nUnix: %s\"}}}", f.callsign, f.country, f.category, f.lat, f.lon, f.timestamp);
    strcpy(pcBufHeaders, AWS_POST_HEADER); pcBufHeaders += strlen(AWS_POST_HEADER);
    strcpy(pcBufHeaders, AWS_HOST_HEADER); pcBufHeaders += strlen(AWS_HOST_HEADER);
    strcpy(pcBufHeaders, CHEADER); pcBufHeaders += strlen(CHEADER);
    strcpy(pcBufHeaders, CTHEADER); pcBufHeaders += strlen(CTHEADER);
    sprintf(cCLLength, "%s%d%s", CLHEADER1, (int)strlen(data), CLHEADER2);
    strcpy(pcBufHeaders, cCLLength); pcBufHeaders += strlen(cCLLength);
    strcpy(pcBufHeaders, data);
    sl_Send(iTLSSockID, acSendBuff, strlen(acSendBuff), 0);
    sl_Recv(iTLSSockID, &acRecvbuff[0], sizeof(acRecvbuff), 0);
}

void SysTickReset(void) { SysTickPeriodSet(SYSTICK); SysTickEnable(); }
void SysTickInit(void) { SysTickPeriodSet(SYSTICK); SysTickIntRegister(SysTickReset); SysTickEnable(); }
unsigned long TickDifference(void) {
    unsigned long currentTick = SysTickValueGet();
    unsigned long lastTick = ticks; ticks = currentTick;
    if (currentTick < lastTick) return (lastTick - currentTick);
    return ((SYSTICK - currentTick) + lastTick);
}
void record_bit(int value) { data = (data << 1) | value; bit_count++; }
volatile unsigned long last_ir_intr_time = 0;
volatile int ir_burst_count = 0;
void GPIOIntHandler(void) {
    unsigned long ulStatus = MAP_GPIOIntStatus(GPIOA0_BASE, true);
    MAP_GPIOIntClear(GPIOA0_BASE, ulStatus);
    delta = TickDifference();
    if (delta > 800000) {
        if (bit_count >= 16) { 
            if ((tickCount - last_ir_intr_time) > 300) ir_burst_count = 1;
            else ir_burst_count++;
            last_ir_intr_time = tickCount;
            if (ir_burst_count == 2) { message_buffer = data; message_ready = true; interrupt_draw = true; }
        }
        data = 0; bit_count = 0;
    }
    if (delta > 150000 && delta < 200000) record_bit(1);
    else if (delta >= 80000 && delta < 110000) record_bit(0);
}
void TimerTickHandler(void) { Timer_IF_InterruptClear(TIMERA0_BASE); tickCount++; }
