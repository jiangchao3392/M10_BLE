/**
 ****************************************************************************************
 *
 * @file periph_setup.h
 *
 * @brief Peripherals setup header file. 
 *
 * Copyright (C) 2012. Dialog Semiconductor Ltd, unpublished work. This computer 
 * program includes Confidential, Proprietary Information and is a Trade Secret of 
 * Dialog Semiconductor Ltd.  All use, disclosure, and/or reproduction is prohibited 
 * unless authorized in writing. All Rights Reserved.
 *
 * <bluetooth.support@diasemi.com> and contributors.
 *
 ****************************************************************************************
 */
/*
 * INCLUDE FILES
 ****************************************************************************************
 */
 
#include "global_io.h"
#include "arch.h"


///****************************************************************************************/ 
///* i2c eeprom configuration                                                             */
///****************************************************************************************/ 
//#define I2C_EEPROM_SIZE   0x20000         // EEPROM size in bytes
//#define I2C_EEPROM_PAGE   256             // EEPROM's page size in bytes
//#define I2C_SPEED_MODE    I2C_FAST        // 1: standard mode (100 kbits/s), 2: fast mode (400 kbits/s)
//#define I2C_ADDRESS_MODE  I2C_7BIT_ADDR   // 0: 7-bit addressing, 1: 10-bit addressing
//#define I2C_ADDRESS_SIZE  I2C_2BYTES_ADDR // 0: 8-bit memory address, 1: 16-bit memory address, 3: 24-bit memory address
/****************************************************************************************/ 
/* SPI FLASH configuration                                                             */
/****************************************************************************************/


// SPI Flash Manufacturer and ID

#define W25X10CL_MANF_DEV_ID (0xEF10)     // W25X10CL Manufacturer and ID
#define W25X20CL_MANF_DEV_ID (0xEF11)     // W25X10CL Manufacturer and ID

// SPI Flash options
#define W25X10CL_SIZE 131072              // SPI Flash memory size in bytes
#define W25X20CL_SIZE 262144              // SPI Flash memory size in bytes
#define W25X10CL_PAGE 256                 // SPI Flash memory page size in bytes
#define W25X20CL_PAGE 256                 // SPI Flash memory page size in bytes
#define SPI_FLASH_DEFAULT_SIZE  65536    // SPI Flash memory size in bytes   //M10 W25X05
#define SPI_FLASH_DEFAULT_PAGE  256
#define SPI_SECTOR_SIZE         4096

/*
 * DEFINES
 ****************************************************************************************
 */

#ifdef CFG_WKUP_EXT_PROCESSOR
/****************************************************************************************/ 
/* external MCU wake-up pin configuration                                               */
/****************************************************************************************/ 
    #define EXT_WAKEUP_PORT GPIO_PORT_1
    #define EXT_WAKEUP_PIN  GPIO_PIN_2 
#endif // #ifdef CFG_WKUP_EXT_PROCESSOR

#define LED_PORT GPIO_PORT_1
#define LED_PIN  GPIO_PIN_2   //ÐÞ¸ÄLED1 P1_2

//#define ADV_LED_PORT GPIO_PORT_0
//#define ADV_LED_PIN  GPIO_PIN_2

#define BUTTON_PORT GPIO_PORT_2
#define BUTTON1_PIN GPIO_PIN_2
//#define BUTTON2_PIN GPIO_PIN_4

#define SPI_PORT     GPIO_PORT_0
#define SPI_PIN_CLK  GPIO_PIN_0
#define SPI_PIN_CS   GPIO_PIN_3
#define SPI_PIN_DI   GPIO_PIN_5
#define SPI_PIN_DO   GPIO_PIN_6

#if BLE_INTEGRATED_HOST_GTL

/****************************************************************************************/ 
/* uart pin configuration                                                               */
/****************************************************************************************/ 
#if (defined(HW_CONFIG_BASIC_DK) || defined(HW_CONFIG_PRO_DK))

    #define UART1_TX_GPIO_PORT  GPIO_PORT_0
    #define UART1_TX_GPIO_PIN   GPIO_PIN_4

    #define UART1_RX_GPIO_PORT  GPIO_PORT_0
    #define UART1_RX_GPIO_PIN   GPIO_PIN_5

    #define UART1_RTSN_GPIO_PORT GPIO_PORT_0
    #define UART1_RTSN_GPIO_PIN  GPIO_PIN_6

    #define UART1_CTSN_GPIO_PORT GPIO_PORT_0
    #define UART1_CTSN_GPIO_PIN  GPIO_PIN_7

#else

    #define UART1_TX_GPIO_PORT  GPIO_PORT_0
    #define UART1_TX_GPIO_PIN   GPIO_PIN_4

    #define UART1_RX_GPIO_PORT  GPIO_PORT_0
    #define UART1_RX_GPIO_PIN   GPIO_PIN_5

    #ifdef PROGRAM_ALTERNATE_UART_PINS
        #define UART1_RTSN_GPIO_PORT GPIO_PORT_0
        #define UART1_RTSN_GPIO_PIN  GPIO_PIN_7

        #define UART1_CTSN_GPIO_PORT GPIO_PORT_0
        #define UART1_CTSN_GPIO_PIN  GPIO_PIN_6
    #else
        #define UART1_RTSN_GPIO_PORT GPIO_PORT_0
        #define UART1_RTSN_GPIO_PIN  GPIO_PIN_3

        #define UART1_CTSN_GPIO_PORT GPIO_PORT_0
        #define UART1_CTSN_GPIO_PIN  GPIO_PIN_2
    #endif // PROGRAM_ALTERNATE_UART_PINS

#endif // #if (defined(HW_CONFIG_BASIC_DK) || defined(HW_CONFIG_PRO_DK))

#endif //#if BLE_INTEGRATED_HOST_GTL


/****************************************************************************************/ 
/* i2c eeprom configuration                                                             */
/****************************************************************************************/ 
#define DEVICE_ADDR   0x50  // Set slave device address
#define I2C_10BITADDR 0     // 0: 7-bit addressing, 1: 10-bit addressing
#define EEPROM_SIZE   256   // EEPROM size in bytes
#define PAGE_SIZE     8     // EEPROM's page size in bytes
#define SPEED         2     // 1: standard mode (100 kbits/s), 2: fast mode (400 kbits/s)


/****************************************************************************************/ 
/* SPI Flash configuration                                                             */
/****************************************************************************************/  
#define SPI_FLASH_SIZE 0x40000 // SPI Flash memory size in bytes
#define SPI_FLASH_PAGE 256     // SPI Flash memory page size in bytes
#define SPI_WORD_MODE  0       // 0: 8-bit, 1: 16-bit, 2: 32-bit, 3: 9-bit
#define SPI_SMN_MODE   0       // 0: Master mode, 1: Slave mode
#define SPI_POL_MODE   1       // 0: SPI clk initially low, 1: SPI clk initially high
#define SPI_PHA_MODE   1       // If same with SPI_POL_MODE, data are valid on clk high edge, else on low
#define SPI_MINT_EN    0       // (SPI interrupt to the ICU) 0: Disabled, 1: Enabled
#define SPI_CLK_DIV    2       // (SPI clock divider) 0: 8, 1: 4, 2: 2, 3: 14
#define SPI_CS_PIN     3       // Define Chip Select pin


/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */
extern uint8_t timer[10];

void periph_init(void);

void GPIO_reservations(void);
void SetBeep(void);
char fliter(uint8_t* buffer,uint8_t iNoOfPoint);
void delay_us(uint16_t num);
void beep(void);
void delay_ms(uint16_t num);
void ZigBee_Sleep(void);
void ZigBee_Wake(void);
void ZigBee_Setting(void);

#define FUNC_DROP 0xAA
#define FUNC_KEY  0xBB
#define FUNC_ALARM 0xCC
#define WARNING_NORMAL 0x00
#define WARNING_PREALARM 0x33
#define WARNING_END 0x55
#define WARNING_TERMINAL 0xDD
#define WARNING_UNCOVER 0xEE
#define SEND_NORMAL 0xAA
#define SEND_END 0x55
