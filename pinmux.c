//*****************************************************************************
// pinmux.c
//
// configure the device pins for different signals
//
//*****************************************************************************

#include "pinmux.h"
#include "hw_types.h"
#include "hw_memmap.h"
#include "hw_gpio.h"
#include "pin.h"
#include "gpio.h"
#include "prcm.h"

//*****************************************************************************
void PinMuxConfig(void)
{
    //
    // Enable Peripheral Clocks
    //
    PRCMPeripheralClkEnable(PRCM_GPIOA0, PRCM_RUN_MODE_CLK); // IR (PIN_62/GPIO7)
    PRCMPeripheralClkEnable(PRCM_GPIOA1, PRCM_RUN_MODE_CLK); // OLED CS (PIN_06/GPIO15)
    PRCMPeripheralClkEnable(PRCM_GPIOA2, PRCM_RUN_MODE_CLK); // OLED DC (PIN_08/GPIO17), RST (PIN_15/GPIO22)
    PRCMPeripheralClkEnable(PRCM_GSPI, PRCM_RUN_MODE_CLK);   // OLED SPI
    PRCMPeripheralClkEnable(PRCM_UARTA0, PRCM_RUN_MODE_CLK); // Console
    PRCMPeripheralClkEnable(PRCM_UARTA1, PRCM_RUN_MODE_CLK); // Board-to-Board
    PRCMPeripheralClkEnable(PRCM_I2CA0, PRCM_RUN_MODE_CLK);  // I2C for Accelerometer

    //
    // Configure PIN_06 for GPIO Output (OLED CS)
    //
    PinTypeGPIO(PIN_06, PIN_MODE_0, false);
    GPIODirModeSet(GPIOA1_BASE, 0x80, GPIO_DIR_MODE_OUT);

    //
    // Configure PIN_08 for GPIO Output (OLED DC)
    //
    PinTypeGPIO(PIN_08, PIN_MODE_0, false);
    GPIODirModeSet(GPIOA2_BASE, 0x2, GPIO_DIR_MODE_OUT);

    //
    // Configure PIN_15 for GPIO Output (OLED RST)
    //
    PinTypeGPIO(PIN_15, PIN_MODE_0, false);
    GPIODirModeSet(GPIOA2_BASE, 0x40, GPIO_DIR_MODE_OUT);

    //
    // Configure PIN_05 for SPI0 GSPI_CLK
    //
    PinTypeSPI(PIN_05, PIN_MODE_7);

    //
    // Configure PIN_07 for SPI0 GSPI_MOSI
    //
    PinTypeSPI(PIN_07, PIN_MODE_7);

    //
    // Configure PIN_62 for GPIO Input (IR Receiver)
    //
    PinTypeGPIO(PIN_62, PIN_MODE_0, false);
    GPIODirModeSet(GPIOA0_BASE, 0x80, GPIO_DIR_MODE_IN);

    //
    // Configure PIN_55 for UART0 UART0_TX
    //
    PinTypeUART(PIN_55, PIN_MODE_3);

    //
    // Configure PIN_57 for UART0 UART0_RX
    //
    PinTypeUART(PIN_57, PIN_MODE_3);

    //
    // Configure PIN_58 for UART1 UART1_TX
    //
    PinTypeUART(PIN_58, PIN_MODE_6);

    //
    // Configure PIN_59 for UART1 UART1_RX
    //
    PinTypeUART(PIN_59, PIN_MODE_6);

    //
    // Configure PIN_01 for I2C0 I2C_SCL
    //
    PinTypeI2C(PIN_01, PIN_MODE_1);

    //
    // Configure PIN_02 for I2C0 I2C_SDA
    //
    PinTypeI2C(PIN_02, PIN_MODE_1);

    // Set unused pins to PIN_MODE_0
    PinModeSet(PIN_03, PIN_MODE_0);
    PinModeSet(PIN_04, PIN_MODE_0);
    PinModeSet(PIN_18, PIN_MODE_0);
    PinModeSet(PIN_21, PIN_MODE_0);
    PinModeSet(PIN_45, PIN_MODE_0);
    PinModeSet(PIN_50, PIN_MODE_0);
    PinModeSet(PIN_52, PIN_MODE_0);
    PinModeSet(PIN_53, PIN_MODE_0);
    PinModeSet(PIN_60, PIN_MODE_0);
    PinModeSet(PIN_61, PIN_MODE_0);
    PinModeSet(PIN_63, PIN_MODE_0);
    PinModeSet(PIN_64, PIN_MODE_0);
}
