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
#include "periph_setup.h"            // peripheral configuration
#include "global_io.h"
#include "gpio.h"
#include "uart.h"
#include "attm_db.h"
#include "prf_utils.h"
#include "streamdatad.h"
#include "app.h"
#include "pwm.h"
#include "adc.h"
#include "spi_flash.h"
#include "spi.h"

extern void prf_cleanup_func(uint8_t conidx, uint16_t conhdl, uint8_t reason);
uint8_t alarm_temp;
uint8_t alarm_sta=0x11;
extern uint8_t USART_RX_STA,js;
int8_t detected_spi_flash_device_index;
uint8_t LacalAddress[2]={0};
uint8_t RestoreSetting[9]={0xAB,0xBC,0xCD,0xDA,0x20,0x01,0x00,0x01,0x30};//恢复出厂设置指令
uint8_t ResSetting[9]={0xAB,0xBC,0xCD,0xD9,0x20,0x01,0x00,0x01,0x2f};   //复位指令
float Adc_div = 0.000;
uint16_t Adc_vbat;
SPI_Pad_t spi_FLASH_CS_Pad;
extern char rxBuffer[74];
/**
 ****************************************************************************************
 * @brief Each application reserves its own GPIOs here.
 *
 * @return void
 ****************************************************************************************
 */

#if DEVELOPMENT_DEBUG

void timer_init(void)
{
  set_tmr_enable(CLK_PER_REG_TMR_ENABLED);    //Enables TIMER0,TIMER2 clock
	set_tmr_div(CLK_PER_REG_TMR_DIV_8);	           //Sets TIMER0,TIMER2 clock division factor to 8
	timer0_init(TIM0_CLK_FAST, PWM_MODE_ONE, TIM0_CLK_DIV_BY_10);   
	timer2_init(HW_CAN_NOT_PAUSE_PWM_2_3_4, PWM_2_3_4_SW_PAUSE_ENABLED, 42);	// 初始化硬件不能停止PWM 软件可以使能PWM ，频率
	timer2_set_pwm2_duty_cycle(20); 
//	timer2_set_pwm3_duty_cycle(20);
	timer2_set_sw_pause(PWM_2_3_4_SW_PAUSE_DISABLED);      // release sw pause to let pwm2,pwm3,pwm4 run
	timer0_set(100, 0, 0);	  // 0.0005s
//	timer0_set(65530, 0, 0);	  // 0.0005s
	timer0_enable_irq();
	timer0_start();
}
void timer_callback(void)
{
//  static unsigned char flag = 0;
//	if(flag)
//	{
////    GPIO_SetActive(GPIO_PORT_2, GPIO_PIN_1);
////		//GPIO_SetActive(GPIO_PORT_2, GPIO_PIN_2);
//		flag = 0;
//  }
//	else
//	{
////    GPIO_SetInactive(GPIO_PORT_2, GPIO_PIN_1);
////		//GPIO_SetInactive(GPIO_PORT_2, GPIO_PIN_2);
//		flag = 1;
//  }
}

static void spi_flash_peripheral_init()
{
		spi_FLASH_CS_Pad.pin = SPI_PIN_CS;
		spi_FLASH_CS_Pad.port = SPI_PORT;
		// Enable SPI & SPI FLASH
	
        spi_init(&spi_FLASH_CS_Pad, SPI_MODE_8BIT, SPI_ROLE_MASTER, SPI_CLK_IDLE_POL_LOW, SPI_PHA_MODE_0, SPI_MINT_DISABLE, SPI_XTAL_DIV_8);
    
        detected_spi_flash_device_index = spi_flash_auto_detect();
    
	
		if (detected_spi_flash_device_index == SPI_FLASH_AUTO_DETECT_NOT_DETECTED)
		{
			// The device was not identified.
			// The default parameters are used (SPI_FLASH_SIZE, SPI_FLASH_PAGE)
			// Alternatively, an error can be asserted here.
			spi_flash_init(SPI_FLASH_SIZE, SPI_FLASH_PAGE);
//			printf_string("\n\r spi_flash_init");
		}
}	

void delay_us(uint16_t num)   //36us
{
  uint16_t i = 80*num;
	while(i--);
}
void delay_ms(uint16_t num)   
{
  uint16_t i,j;	
		for(j=0;j<num;j++)					 //
	{
		for(i=0;i<1000;i++);
	}
}

void GPIO_reservations(void)
{  
    RESERVE_GPIO( GPIO, LED_PORT,  LED_PIN, PID_GPIO);   //已使用GPIO 1-2   //M10未使用休眠功能 P1-2为默认休眠唤醒口
//	  RESERVE_GPIO( GPIO, ADV_LED_PORT,  ADV_LED_PIN, PID_GPIO);
    RESERVE_GPIO( GPIO, BUTTON_PORT,  BUTTON1_PIN, PID_GPIO); 
//    RESERVE_GPIO( GPIO, BUTTON_PORT,  BUTTON2_PIN, PID_GPIO);  
    	  	//////串口引脚定义/////
	  RESERVE_GPIO( UART1_TX, GPIO_PORT_2,  GPIO_PIN_3, PID_UART1_TX);
	  RESERVE_GPIO( UART1_RX, GPIO_PORT_2,  GPIO_PIN_4, PID_UART1_RX);	
	
	  RESERVE_GPIO( SPI_CLK, SPI_PORT, SPI_PIN_CLK, PID_SPI_CLK);
	  RESERVE_GPIO( SPI_DO, SPI_PORT, SPI_PIN_DO, PID_SPI_DO);
	  RESERVE_GPIO( SPI_EN, SPI_PORT, SPI_PIN_CS, PID_SPI_EN); 
	  RESERVE_GPIO( SPI_DI, SPI_PORT, SPI_PIN_DI, PID_SPI_DI);
//		RESERVE_GPIO( GPIO, GPIO_PORT_1, GPIO_PIN_5, PID_GPIO);
	
	
		RESERVE_GPIO( GPIO, GPIO_PORT_0, GPIO_PIN_1, PID_ADC);//
		RESERVE_GPIO( GPIO, GPIO_PORT_0, GPIO_PIN_2, PID_ADC);//P02为电流采样ADC输入；
		RESERVE_GPIO( GPIO, GPIO_PORT_0, GPIO_PIN_7, PID_PWM2); 

	  RESERVE_GPIO( GPIO, GPIO_PORT_0, GPIO_PIN_4, PID_GPIO);
//		RESERVE_GPIO( GPIO, GPIO_PORT_0, GPIO_PIN_5, PID_GPIO);

		RESERVE_GPIO( GPIO, GPIO_PORT_1, GPIO_PIN_0, PID_GPIO);
		RESERVE_GPIO( GPIO, GPIO_PORT_1, GPIO_PIN_1, PID_GPIO);
		//RESERVE_GPIO( GPIO, GPIO_PORT_1, GPIO_PIN_2, PID_GPIO);	
		RESERVE_GPIO( GPIO, GPIO_PORT_1, GPIO_PIN_3, PID_GPIO);
		RESERVE_GPIO( GPIO, GPIO_PORT_2, GPIO_PIN_0, PID_GPIO);
		RESERVE_GPIO( GPIO, GPIO_PORT_2, GPIO_PIN_1, PID_GPIO);
//		RESERVE_GPIO( GPIO, GPIO_PORT_2, GPIO_PIN_2, PID_GPIO);
		RESERVE_GPIO( GPIO, GPIO_PORT_2, GPIO_PIN_5, PID_GPIO);
		RESERVE_GPIO( GPIO, GPIO_PORT_2, GPIO_PIN_6, PID_GPIO);
		RESERVE_GPIO( GPIO, GPIO_PORT_2, GPIO_PIN_7, PID_GPIO);
		RESERVE_GPIO( GPIO, GPIO_PORT_2, GPIO_PIN_8, PID_GPIO);
		RESERVE_GPIO( GPIO, GPIO_PORT_2, GPIO_PIN_9, PID_GPIO);
}
#endif //DEVELOPMENT__NO_OTP

/**
 ****************************************************************************************
 * @brief Map port pins
 *
 * The Uart and SPI port pins and GPIO ports are mapped
 ****************************************************************************************
 */
void set_pad_functions(void)        // set gpio port function mode
{
    GPIO_ConfigurePin( LED_PORT, LED_PIN, OUTPUT, PID_GPIO, true );
//	  GPIO_ConfigurePin( ADV_LED_PORT, ADV_LED_PIN, OUTPUT, PID_GPIO, false );
    GPIO_ConfigurePin( BUTTON_PORT, BUTTON1_PIN, INPUT, PID_GPIO, false );	
//	  GPIO_ConfigurePin( BUTTON_PORT, BUTTON2_PIN, INPUT, PID_GPIO, false );
	
	  GPIO_ConfigurePin( SPI_PORT, SPI_PIN_CLK, OUTPUT, PID_SPI_CLK, false );
		GPIO_ConfigurePin( SPI_PORT, SPI_PIN_DO, OUTPUT, PID_SPI_DO, false ); 
		GPIO_ConfigurePin( SPI_PORT, SPI_PIN_CS, OUTPUT, PID_SPI_EN, true );
		GPIO_ConfigurePin( SPI_PORT, SPI_PIN_DI, INPUT, PID_SPI_DI, true );
//    	SetBits16(SYS_CTRL_REG, DEBUGGER_ENABLE, 0); 
   	//////////串口 OUTPUT////////////////
		GPIO_ConfigurePin( GPIO_PORT_2, GPIO_PIN_3, OUTPUT, PID_UART1_TX, false );
		GPIO_ConfigurePin( GPIO_PORT_2, GPIO_PIN_4, INPUT, PID_UART1_RX, false );
//	  GPIO_ConfigurePin(GPIO_PORT_0, GPIO_PIN_0, OUTPUT, PID_GPIO, false);  //power_ctrl
	

	//  GPIO_ConfigurePin(GPIO_PORT_1, GPIO_PIN_2, OUTPUT, PID_GPIO, false);  //Power_ctrl  10.11  将power_ctrl从
	                                                                        //将power_ctrl从P0.0移动至P1.2；
	  GPIO_ConfigurePin(GPIO_PORT_0, GPIO_PIN_4, OUTPUT, PID_GPIO, true);	
//	  GPIO_ConfigurePin(GPIO_PORT_0, GPIO_PIN_5, INPUT, PID_GPIO, false);   //ACK
//		GPIO_ConfigurePin(GPIO_PORT_1, GPIO_PIN_5, OUTPUT, PID_GPIO, true); //RESETZM	 /SWD	
		GPIO_ConfigurePin(GPIO_PORT_0, GPIO_PIN_1, INPUT, PID_ADC, false);   	 //PID_ADC
		GPIO_ConfigurePin(GPIO_PORT_0, GPIO_PIN_2, INPUT, PID_ADC, false);  // P0.2修改为电流采样ADC  10.25
  	GPIO_ConfigurePin(GPIO_PORT_0, GPIO_PIN_7, OUTPUT, PID_PWM2, true);  //DETSYS
	  GPIO_ConfigurePin(GPIO_PORT_1, GPIO_PIN_0, OUTPUT, PID_GPIO, false);  //MOTOR_IN1
	  GPIO_ConfigurePin(GPIO_PORT_1, GPIO_PIN_1, OUTPUT, PID_GPIO, false);  //MOTOR_IN2
   // GPIO_ConfigurePin(GPIO_PORT_1, GPIO_PIN_2, OUTPUT, PID_PWM3, false); //IR_MODU
	 // GPIO_ConfigurePin(GPIO_PORT_1, GPIO_PIN_2, OUTPUT, PID_GPIO, true); //IR_MODU??  老原理图上未用   //10.25 P1.2用于LED  
		GPIO_ConfigurePin(GPIO_PORT_1, GPIO_PIN_3, OUTPUT, PID_GPIO, false); // POWERON
	  GPIO_ConfigurePin(GPIO_PORT_2, GPIO_PIN_0, OUTPUT, PID_GPIO, false);  //LED2
	  GPIO_ConfigurePin(GPIO_PORT_2, GPIO_PIN_1, INPUT, PID_GPIO, false);   //DET_INT
//	  GPIO_ConfigurePin(GPIO_PORT_2, GPIO_PIN_2, INPUT, PID_GPIO, false);   //KEY
	  GPIO_ConfigurePin(GPIO_PORT_2, GPIO_PIN_5, INPUT, PID_GPIO, false);  //CLAMP
	  GPIO_ConfigurePin(GPIO_PORT_2, GPIO_PIN_6, OUTPUT, PID_GPIO, false); //BEEP
	  GPIO_ConfigurePin(GPIO_PORT_2, GPIO_PIN_7, OUTPUT, PID_GPIO, false);  //5VCTL
	  GPIO_ConfigurePin(GPIO_PORT_2, GPIO_PIN_8, OUTPUT, PID_GPIO, true); //SLEEP
		GPIO_ConfigurePin(GPIO_PORT_2, GPIO_PIN_9, OUTPUT, PID_GPIO, true); //WAKE
}

void gpio0_int_callback(void)
{
	unsigned char data_ptr[] = {0xA1,0xA2,0xA3};
	attmdb_att_set_value(STREAMDATAD_DIR_VAL_HANDLE(0), 3, data_ptr);
	prf_server_send_event((prf_env_struct *)&(streamdatad_env.con_info), false, STREAMDATAD_DIR_VAL_HANDLE(0));
  //printf_string("GWI_UART_button1\n\r");

  prf_cleanup_func(0x00,0x0000,0x08);
  app_timer_set(GAPC_DISCONNECT_IND,TASK_APP,1); 
}

void gpio1_int_callback(void)
{
		   	
//	unsigned char data_ptr[] = {0xA4,0xA5,0xA6};
//	attmdb_att_set_value(STREAMDATAD_DIR_VAL_HANDLE(0), 3, data_ptr);
//	prf_server_send_event((prf_env_struct *)&(streamdatad_env.con_info), false, STREAMDATAD_DIR_VAL_HANDLE(0));
//  //app_timer_set(GAPC_DISCONNECT_IND,TASK_APP,1); 	
//  printf_string("GWI_UART_button2\n\r");
}

/**
 ****************************************************************************************
 * @brief Enable pad's and peripheral clocks assuming that peripherals' power domain is down. The Uart and SPi clocks are set.
 *
 * @return void
 ****************************************************************************************
 */
void periph_init(void) 
{
	// Power up peripherals' power domain
    SetBits16(PMU_CTRL_REG, PERIPH_SLEEP, 0);
    while (!(GetWord16(SYS_STAT_REG) & PER_IS_UP)) ; 
    
#if ES4_CODE
    SetBits16(CLK_16M_REG,XTAL16_BIAS_SH_DISABLE, 1);
#endif     

	//rom patch
	patch_func();

	//Init pads
	set_pad_functions();
		adc_calibrate();
	uart_init();	//串口初始化

	timer_init();
//	timer0_register_callback(timer_callback);
	
	delay_us(5);
	adc_init(GP_ADC_SE, 0,0);
//	adc_enable_channel(ADC_CHANNEL_VBAT3V);	
//	Adc_vbat = adc_get_sample();
//	Adc_div = 3300.000/Adc_vbat;
	
	adc_enable_channel(ADC_CHANNEL_P01);
	
//	GPIO_RegisterCallback(GPIO0_IRQn, gpio0_int_callback);	
//	GPIO_RegisterCallback(GPIO1_IRQn, gpio1_int_callback);
	
	
	GPIO_EnableIRQ( GPIO_PORT_2, GPIO_PIN_1, GPIO1_IRQn, 0, 1, 20 );
//	GPIO_EnableIRQ( BUTTON_PORT, BUTTON1_PIN, GPIO0_IRQn, 0, 1, 20 );
//	GPIO_EnableIRQ( BUTTON_PORT, BUTTON2_PIN, GPIO1_IRQn, 0, 1, 20 );
  	
		// Enable FLASH and SPI
    spi_flash_peripheral_init(); 
		spi_cs_low();   
    spi_cs_high();
  
//	printf_string("GWI_UART_init\n\r");
#if (BLE_APP_PRESENT)
/*
* (Re)Initialize peripherals
i.e.    
    uart_init(UART_BAUDRATE_115K2, 3);
*/        
#endif
    
    // Enable the pads
	SetBits16(SYS_CTRL_REG, PAD_LATCH_EN, 1);
}


void SetBeep(void)
{
	int i;
	for(i=0;i<200;i++)
	{
		GPIO_SetActive(GPIO_PORT_2, GPIO_PIN_6);
		delay_us(5);
		GPIO_SetInactive(GPIO_PORT_2 , GPIO_PIN_6);	
		delay_us(5);
	}
}

char fliter(uint8_t* buffer,uint8_t iNoOfPoint)
{
	  long sum;
	  int i;
	  sum = 0;
	  for(i=0;i<iNoOfPoint;i++)
	  {
	  	sum+=buffer[i];
	  }
	  return sum/iNoOfPoint;
}

void beep(void)
{

	if(alarm_sta==0x22||alarm_sta==0x44||alarm_sta==0x66||alarm_sta==0x88)
	SetBeep();	
	if(timer[2]==0)	  			
	{

		if(alarm_sta==0x88)
		{
			timer[2]=100; 	
			alarm_sta=0x11;
			alarm_temp=1;
		}               //60ms  		       
		else 					
		{
			timer[2]=5;		   	
			alarm_sta +=0x11;	
		}				   
	}	 
}

 

void ZigBee_Sleep(void)
{
  GPIO_SetInactive(GPIO_PORT_2 , GPIO_PIN_8);	//给ZigBee休眠脚一个低电平
  delay_us(36);		//低电平持续时长 
  GPIO_SetActive(GPIO_PORT_2, GPIO_PIN_8);
}

void ZigBee_Wake(void)
{
  GPIO_SetInactive(GPIO_PORT_2 , GPIO_PIN_9);	//给ZigBee唤醒脚一个低电平
  delay_us(36);		//低电平持续时长 
  GPIO_SetActive(GPIO_PORT_2, GPIO_PIN_9);

}
//void ZigBee_Setting(void)
//{
//	GPIO_SetInactive(GPIO_PORT_2 , GPIO_PIN_0);//DEF
//		delay_us(200);
//	GPIO_SetInactive(GPIO_PORT_1 , GPIO_PIN_2);//RESET
//		delay_us(200);	
//	GPIO_SetActive(GPIO_PORT_1, GPIO_PIN_2);
//		delay_us(100);	
//	GPIO_SetActive(GPIO_PORT_2 , GPIO_PIN_0);	
//	delay_us(60);	
////	GPIO_SetInactive(GPIO_PORT_1 , GPIO_PIN_5);
////	delay_us(60);
////	GPIO_SetActive(GPIO_PORT_1, GPIO_PIN_5);
//	
//}

void ZigBee_Setting(void)
{	  uint8_t i; 
		 
	if(USART_RX_STA==1)
		  {
		  	USART_RX_STA=0;
	
			 switch(rxBuffer[3])
						 {
						 
						case 0xD1:   
									LacalAddress[0]=rxBuffer[40];
								LacalAddress[1]=rxBuffer[41]; 
								RestoreSetting[4]=LacalAddress[0];//USART_RX_BUFT[40];
								RestoreSetting[5]=LacalAddress[1];//USART_RX_BUFT[41];

								 
								for(i=0;i<9;i++)
									{
									uart_send_byte(RestoreSetting[i]);
									}									
								 break;
							case 0xDA:  
								if(rxBuffer[8]==0x00)		   //复位
									 {// 
											 ResSetting[4]=LacalAddress[0];
										 ResSetting[5]=LacalAddress[1]; 
											for(i=0;i<9;i++)
									{
									uart_send_byte(ResSetting[i]);
									}
											delay_ms(1000); 			
									
										}			 
								 break;			 

						 default:	                //
											break;
						}					
					js=0;
		  }	
		 
}	

