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
#define SPI_FLASH_SIZE 131072  // SPI Flash memory size in bytes
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
 
void periph_init(void);

void GPIO_reservations(void);

