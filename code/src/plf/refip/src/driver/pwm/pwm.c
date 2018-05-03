/**
 ****************************************************************************************
 *
 * @file pwm.c
 *
 * @brief PWM driver source file.
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

#include "global_io.h"
#include "datasheet.h"
#include "pwm.h"
#include "periph_setup.h"
#include "gpio.h"
#include <stdint.h>     // standard integer definition

timer0_handler_function_t* TIMER0_callback;

volatile struct __CLK_PER_REG clk_per_reg __attribute__((at(CLK_PER_REG)));
volatile struct __TIMER0_CTRL_REG timer0_ctrl_reg __attribute__((at(TIMER0_CTRL_REG)));
volatile struct __TRIPLE_PWM_CTRL_REG triple_pwm_ctrl_reg __attribute__((at(TRIPLE_PWM_CTRL_REG)));
uint8_t timer[10];
uint16_t DropCnt = 0,opencount=0;
uint16_t iSecCount = 0,LEDCount=0,SystemCount=0,iSecCnt2=0,iseccnt3=0,iseccnt4=0,iseccnt4_flag,iseccnt5=0,iseccnt5_flag;
uint16_t ADCcount = 0;   //��ȡ������ʱ����
volatile uint16_t resetTime =0, setTime = 0;     //��λʱ�����
extern uint16_t BeSecTmp[2];
extern uint8_t RFOFF,kai;
/**
 ****************************************************************************************
 * @brief  Enables/Disables TIMER0,TIMER2 clock
 *
 * @param[in] TMR_ENABLE status: CLK_PER_REG_TMR_DISABLED or CLK_PER_REG_TMR_ENABLED
 ****************************************************************************************
 */
 inline void set_tmr_enable(CLK_PER_REG_TMR_ENABLE_t clk_per_reg_tmr_enable)
 {
     clk_per_reg.BITFLD_TMR_ENABLE = clk_per_reg_tmr_enable;    // Enable/Disable TIMER0/TIMER2 clock        
 }
 
 /**
 ****************************************************************************************
 * @brief  Sets TIMER0,TIMER2 clock division factor
 *
 * @param[in] TMR_ENABLE status: CLK_PER_REG_TMR_DISABLED or CLK_PER_REG_TMR_ENABLED
 ****************************************************************************************
 */
 inline void set_tmr_div(CLK_PER_REG_TMR_DIV_t per_tmr_div)
 {
     clk_per_reg.BITFLD_TMR_DIV = per_tmr_div;  // set TIMER0,TIMER2 clock division factor    
 }
 
 
/**
 ****************************************************************************************
 * @brief  Initializes TIMER0 
 *
 * @param[in] tim0_clk_sel: Timer0 uses 16MHz (fast) or 32kHz (slow) clock frequency
 * @param[in]     pwm_mode: (during high time) '1' or (fast) clock divided by 2
 * @param[in] tim0_clk_div: Division factor for TIMER0 "on" time in peripheral divider register
 *
 * @return 
 ****************************************************************************************
 */
void timer0_init(TIM0_CLK_SEL_t tim0_clk_sel, PWM_MODE_t pwm_mode, TIM0_CLK_DIV_t tim0_clk_div)
{
    timer0_ctrl_reg.BITFLD_PWM_MODE = pwm_mode;                  
    timer0_ctrl_reg.BITFLD_TIM0_CLK_DIV = tim0_clk_div; // NOTE: divider option is used only for the "ON" counter           
    timer0_ctrl_reg.BITFLD_TIM0_CLK_SEL = tim0_clk_sel;  
    NVIC_SetPriority (SWTIM_IRQn, 254);                 /* set Priority for TIM0 Interrupt to be the lowest */
}


/**
 ****************************************************************************************
 * @brief  starts TIMER0
 ****************************************************************************************
 */

inline void timer0_start(void)
{
    timer0_ctrl_reg.BITFLD_TIM0_CTRL = TIM0_CTRL_RUNNING;       // Timer0 is running
}


/**
 ****************************************************************************************
 * @brief  stops TIMER0
 ****************************************************************************************
 */

inline void timer0_stop(void)
{
    timer0_ctrl_reg.BITFLD_TIM0_CTRL = TIM0_CTRL_OFF_RESET;     // Timer0 is off and in reset state
}


/**
 ****************************************************************************************
 * @brief  Releases TIMER0. Does not disable the TIM clock, as it is shared with TIMER2
 *
 * @param[in]  timer2_duty_cycle: PWM2 duty cycle
 ****************************************************************************************
 */
inline void timer0_release(void)
{
    timer0_stop();
}


/**
 ****************************************************************************************
 * @brief  configures TIMER0 ON counter reload value
 * @param[in]    pwm_on: ON COUNTER reload value
 ****************************************************************************************
 */
inline void timer0_set_pwm_on_counter(uint16_t pwm_on)
{
    SetWord16(TIMER0_ON_REG, pwm_on);    
}


/**
 ****************************************************************************************
 * @brief  configures TIMER0 M counter reload value (duration of PWM0 high part)
 * @param[in]   pwm_high: M COUNTER reload value
 ****************************************************************************************
 */
inline void timer0_set_pwm_high_counter(uint16_t pwm_high)
{
    SetWord16(TIMER0_RELOAD_M_REG, pwm_high);    
}


/**
 ****************************************************************************************
 * @brief  configures TIMER0 N counter reload value (duration of PWM0 low part)
 * @param[in]   pwm_low: N COUNTER reload value
 ****************************************************************************************
 */

inline void timer0_set_pwm_low_counter(uint16_t pwm_low)
{
    SetWord16(TIMER0_RELOAD_N_REG, pwm_low);    
}


/**
 ****************************************************************************************
 * @brief  configures TIMER0 pwm timing parameters
 * @param[in]    pwm_on: ON COUNTER reload value
 * @param[in]  pwm_high: M COUNTER reload value
 * @param[in]   pwm_low: N COUNTER reload value
 ****************************************************************************************
 */

void timer0_set(uint16_t pwm_on, uint16_t pwm_high, uint16_t pwm_low)
{
    SetWord16(TIMER0_ON_REG, pwm_on);    
    SetWord16(TIMER0_RELOAD_M_REG, pwm_high);
    SetWord16(TIMER0_RELOAD_N_REG, pwm_low);
}


/**
 ****************************************************************************************
 * @brief  enables SWTIM_IRQn 
 ****************************************************************************************
 */

void timer0_enable_irq(void)
{    
    NVIC_EnableIRQ(SWTIM_IRQn);
}


/**
 ****************************************************************************************
 * @brief  disables SWTIM_IRQn 
 ****************************************************************************************
 */

void timer0_disable_irq(void)
{    
    NVIC_DisableIRQ(SWTIM_IRQn);
}


/**
 ****************************************************************************************
 * @brief  registers a callback function to be called from within SWTIM_Handler
           when SWTIM_IRQn has triggered
 ****************************************************************************************
 */

inline void timer0_register_callback(timer0_handler_function_t* callback)
{
   TIMER0_callback = callback;
}


 /**
 ****************************************************************************************
  @brief SWTIM_Handler. The SWTIM_IRQn interrupt handler
  ****************************************************************************************
 */

void SWTIM_Handler(void) 
{
//	    static unsigned char flag = 0;
	  static unsigned char CNT = 0;
//	  static unsigned char LEDCount = 0;
//	if (TIMER0_callback != NULL)
//        TIMER0_callback();
	 
	 DropCnt++;    //��Һ�μ��ʱ���ʱ 500us
	 CNT++;
	 if(CNT>=20)   //10ms
	   {
             
			 CNT=0;			 
			 timer[0]++;
       setTime++;   

       resetTime++;		
			 
			 timer[6]++;
//			 iWaringSend--;			
			if(timer[2]>0)
			{
				timer[2]--;   
			}
			
			if(iseccnt4_flag==1)
			{
			 iseccnt4++;   //ѹ���󱣳�����ʱ  10ms
			}
			if(iseccnt5_flag==1)
			{
			 iseccnt5++;   //ѹ���󱣳�����ʱ  10ms
			}
			 
	   }
	//	if(timer[0]%5==0&&kai)iseccnt3++;		   
		if(timer[0]==50)	//500ms			   
		{
            ADCcount++;
			opencount++;
			timer[0]=0;
			iSecCnt2++;		
			iSecCount++;
			LEDCount++;
			SystemCount++;	
			timer[3]--;
			if(timer[4] > 1)timer[4]--;			
			if(BeSecTmp[1]>2) 	BeSecTmp[1]--;	
			if(RFOFF > 0)RFOFF--;	
            timer[5]--; 
           
            
		}
		
//		if(setTime==5)
//		{
//			
//			setTime=0;
//			
//			iseccnt3++;
//			
//		}
//		if(iWaringSend<1)	iWaringSend = 1;  		 
//	 					if(SystemCount>65)	SystemCount = 65;
//		if(LEDCount>42)		LEDCount=40;
//		if(opencount>42)	opencount=0;  //��ֹopencount���
		 
		 
}


/**
 ****************************************************************************************
 * @brief  Enables/Disables TIMER2 
 * @param[in]  triple_pwm_enable:  TRIPLE_PWM_DISABLED or TRIPLE_PWM_ENABLED
 ****************************************************************************************
 */

inline void timer2_enable(TRIPLE_PWM_ENABLE_t triple_pwm_enable)
{
    triple_pwm_ctrl_reg.BITFLD_TRIPLE_PWM_ENABLE = triple_pwm_enable;     
}


/**
 ****************************************************************************************
 * @brief  Enables/Disables TIMER2 pause by hw
 * @param[in]  hw_pause_en:  HW_CAN_NOT_PAUSE_PWM_2_3_4 or HW_CAN_PAUSE_PWM_2_3_4
 ****************************************************************************************
 */

inline void timer2_set_hw_pause(TRIPLE_PWM_HW_PAUSE_EN_t hw_pause_en)
{
    triple_pwm_ctrl_reg.BITFLD_HW_PAUSE_EN = hw_pause_en;
}


/**
 ****************************************************************************************
 * @brief  Pauses by sw / releases sw pause for TIMER2
 * @param[in]  sw_pause_en:  PWM_2_3_4_SW_PAUSE_DISABLED or PWM_2_3_4_SW_PAUSE_ENABLED
 ****************************************************************************************
 */

inline void timer2_set_sw_pause(TRIPLE_PWM_SW_PAUSE_EN_t sw_pause_en)
{
    triple_pwm_ctrl_reg.BITFLD_SW_PAUSE_EN = sw_pause_en;
}


/**
 ****************************************************************************************
 * @brief  Sets TIMER2 frequency
 * @param[in]  triple_pwm_frequency:  TIMER2 frequency
 ****************************************************************************************
 */

inline void timer2_set_pwm_frequency(uint16_t triple_pwm_frequency)
{
    SetWord16(TRIPLE_PWM_FREQUENCY, triple_pwm_frequency);
}


/**
 ****************************************************************************************
 * @brief  Initializes TIMER2 
 *
 * @param[in]           hw_pause_en: HW_CAN_NOT_PAUSE_PWM_2_3_4 or HW_CAN_PAUSE_PWM_2_3_4
 * @param[in]           sw_pause_en: PWM_2_3_4_SW_PAUSE_DISABLED or PWM_2_3_4_SW_PAUSE_ENABLED
 * @param[in]  triple_pwm_frequency: TIMER2 frequency
 ****************************************************************************************
 */

 void timer2_init(TRIPLE_PWM_HW_PAUSE_EN_t hw_pause_en, TRIPLE_PWM_SW_PAUSE_EN_t sw_pause_en, uint16_t triple_pwm_frequency)
{
    timer2_enable(TRIPLE_PWM_ENABLED);      
    timer2_set_hw_pause(hw_pause_en);
    timer2_set_sw_pause(sw_pause_en);
    timer2_set_pwm_frequency(triple_pwm_frequency);
}


/**
 ****************************************************************************************
 * @brief  Stops timer2. Does not disable the TIM clock, as it is shared with TIMER0
 *
 * @param[in]  timer2_duty_cycle: PWM2 duty cycle
 ****************************************************************************************
 */
inline void timer2_stop(void)
{
    timer2_enable(TRIPLE_PWM_DISABLED);
}

/**
 ****************************************************************************************
 * @brief  Sets PWM2 duty cycle
 *
 * @param[in]  timer2_duty_cycle: PWM2 duty cycle
 ****************************************************************************************
 */

inline void timer2_set_pwm2_duty_cycle(uint16_t pwm2_duty_cycle)
{
    SetWord16(PWM2_DUTY_CYCLE, pwm2_duty_cycle);
}


/**
 ****************************************************************************************
 * @brief  Sets PWM3 duty cycle
 *
 * @param[in]  timer3_duty_cycle: PWM3 duty cycle
 ****************************************************************************************
 */

inline void timer2_set_pwm3_duty_cycle(uint16_t pwm3_duty_cycle)
{
    SetWord16(PWM3_DUTY_CYCLE, pwm3_duty_cycle);
}


/**
 ****************************************************************************************
 * @brief  Sets PWM4 duty cycle
 *
 * @param[in]  timer3_duty_cycle: PWM4 duty cycle
 ****************************************************************************************
 */

inline void timer2_set_pwm4_duty_cycle(uint16_t pwm4_duty_cycle)
{
    SetWord16(PWM4_DUTY_CYCLE, pwm4_duty_cycle);
}
