/**
 ****************************************************************************************
 *
 * @file periph_setup.c
 *
 * @brief Peripherals setup and initialization. 
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
#include "rwip_config.h"             // SW configuration
#include "periph_setup.h"            // periphera configuration
#include "global_io.h"
#include "gpio.h"
#include "uart.h"                    // UART initialization

/*
 * DEFINES
 ****************************************************************************************
 */

/****************************************************************************************/
/* UART pin configuration                                                               */
/****************************************************************************************/
// Only one of the following lines must be uncommented.
#define UART_P04_P05
//#define UART_P00_P01
//#define UART_P14_P15


/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */
void init_TXEN_RXEN_irqs(void);

#if DEVELOPMENT_DEBUG

/**
 ****************************************************************************************
 * @brief GPIO_reservations. Globally reserved GPIOs
 *
 * @return void 
 ****************************************************************************************
*/
void GPIO_reservations(void)
{
#if defined(UART_P04_P05)
    RESERVE_GPIO( UART1_TX, GPIO_PORT_0,  GPIO_PIN_4, PID_UART1_TX);
    RESERVE_GPIO( UART1_RX, GPIO_PORT_0,  GPIO_PIN_5, PID_UART1_RX);
#elif defined(UART_P00_P01)
    RESERVE_GPIO( UART1_TX, GPIO_PORT_0,  GPIO_PIN_0, PID_UART1_TX);
    RESERVE_GPIO( UART1_RX, GPIO_PORT_0,  GPIO_PIN_1, PID_UART1_RX);
#elif defined(UART_P14_P15)
    RESERVE_GPIO( UART1_TX, GPIO_PORT_1,  GPIO_PIN_4, PID_UART1_TX);
    RESERVE_GPIO( UART1_RX, GPIO_PORT_1,  GPIO_PIN_5, PID_UART1_RX);
#else
    #error "=== No UART pin configuration selected in periph_setup.h ==="
#endif // defined(UART_P04_P05)
  
#ifdef SEND_RX_DATA_BACK_BY_SPI
    RESERVE_GPIO( SPI_CLK, GPIO_PORT_0, GPIO_PIN_6, PID_SPI_CLK);
    RESERVE_GPIO( SPI_DO, GPIO_PORT_0, GPIO_PIN_4, PID_SPI_DO);	
#endif    
}

#endif

/**
 ****************************************************************************************
 * @brief Map port pins
 *
 * The Uart and SPI port pins and GPIO ports(for debugging) are mapped
 ****************************************************************************************
 */
void set_pad_functions(void)
{
#if defined(UART_P04_P05)
    GPIO_ConfigurePin( GPIO_PORT_0, GPIO_PIN_4, OUTPUT, PID_UART1_TX, false );
    GPIO_ConfigurePin( GPIO_PORT_0, GPIO_PIN_5, INPUT, PID_UART1_RX, false );
#elif defined(UART_P00_P01)
    GPIO_ConfigurePin( GPIO_PORT_0, GPIO_PIN_0, OUTPUT, PID_UART1_TX, false );
    GPIO_ConfigurePin( GPIO_PORT_0, GPIO_PIN_1, INPUT, PID_UART1_RX, false );
#elif defined(UART_P14_P15)
    // wait until debugger is disconnected and then disable debuging
    while ((GetWord16(SYS_STAT_REG) & DBG_IS_UP) == DBG_IS_UP) {};
    SetBits16(SYS_CTRL_REG, DEBUGGER_ENABLE, 0);    // close debugger
    GPIO_ConfigurePin( GPIO_PORT_1, GPIO_PIN_4, OUTPUT, PID_UART1_TX, false );
    GPIO_ConfigurePin( GPIO_PORT_1, GPIO_PIN_5, INPUT, PID_UART1_RX, false );
#else
    #error "=== No UART pin configuration selected in periph_setup.h ==="
#endif // defined(UART_P04_P05)
		
	
#ifdef SEND_RX_DATA_BACK_BY_SPI
//ALLOCATE SPI SIGNALS
#if 0 //for testing with PXI system
    RESERVE_GPIO( SPI_CLK, GPIO_PORT_0, GPIO_PIN_6, PID_SPI_CLK);
    RESERVE_GPIO( SPI_DO, GPIO_PORT_0, GPIO_PIN_4, PID_SPI_DO);	
#endif
	
#endif //SEND_RX_DATA_BACK_BY_SPI	
  
}


/**
 ****************************************************************************************
 * @brief Enable pad's and peripheral clocks assuming that peripherals' power domain is down.
 *
 * The Uart and SPi clocks are set. 
 ****************************************************************************************
 */
void periph_init(void)  // set i2c, spi, uart, uart2 serial clks
{
	// Power up peripherals' power domain
    SetBits16(PMU_CTRL_REG, PERIPH_SLEEP, 0);
    while (!(GetWord16(SYS_STAT_REG) & PER_IS_UP)) ; 
    
#if ES4_CODE
    SetBits16(CLK_16M_REG,XTAL16_BIAS_SH_DISABLE, 1);
#endif    
	
	// TODO: Application specific - Modify accordingly!
	// Example: Activate UART and SPI.
	
    // Initialize UART component

    SetBits16(CLK_PER_REG, UART1_ENABLE, 1);    // enable clock - always @16MHz
	
    // baudr=9-> 115k2
    // mode=3-> no parity, 1 stop bit 8 data length
#ifdef UART_MEGABIT
    uart_init(UART_BAUDRATE_1M, 3);
#else
    uart_init(UART_BAUDRATE_115K2, 3);
#endif // UART_MEGABIT
	
	//rom patch
	patch_func();
	
	//Init pads
	set_pad_functions();

    // Enable the pads
	SetBits16(SYS_CTRL_REG, PAD_LATCH_EN, 1);
}

void init_TXEN_RXEN_irqs(void)
{
//init for TXEN	
	SetBits32(BLE_RF_DIAGIRQ_REG, DIAGIRQ_WSEL_0, 2); //SO SELECT RADIO_DIAG0
	SetBits32(BLE_RF_DIAGIRQ_REG, DIAGIRQ_MASK_0, 1); //ENABLE IRQ
	SetBits32(BLE_RF_DIAGIRQ_REG, DIAGIRQ_BSEL_0, 7); //BIT7 OF DIAG0 BUS, SO TXEN
	SetBits32(BLE_RF_DIAGIRQ_REG, DIAGIRQ_EDGE_0, 0); //SELECT POS EDGE

//init for RXEN	
	SetBits32(BLE_RF_DIAGIRQ_REG, DIAGIRQ_WSEL_1, 3); //SO SELECT RADIO_DIAG1
	SetBits32(BLE_RF_DIAGIRQ_REG, DIAGIRQ_MASK_1, 1); //ENABLE IRQ
	SetBits32(BLE_RF_DIAGIRQ_REG, DIAGIRQ_BSEL_1, 7); //BIT7 OF DIAG1 BUS, SO RXEN
	SetBits32(BLE_RF_DIAGIRQ_REG, DIAGIRQ_EDGE_1, 0); //SELECT POS EDGE
	
	NVIC_EnableIRQ(BLE_RF_DIAG_IRQn); 
	NVIC_SetPriority(BLE_RF_DIAG_IRQn,4);     
    NVIC_ClearPendingIRQ(BLE_RF_DIAG_IRQn); //clear eventual pending bit, but not necessary becasuse this is already cleared automatically in HW

}		
