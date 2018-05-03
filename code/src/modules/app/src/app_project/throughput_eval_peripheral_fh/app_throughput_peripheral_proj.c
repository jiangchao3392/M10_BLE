/**
****************************************************************************************
*
* @file app_throughput_peripheral_proj.c
*
* @brief Throughput evaluation peripheral project source code .
*
* Copyright (C) 2012. Dialog Semiconductor Ltd, unpublished work. This computer 
 * program includes Confidential, Proprietary Information and is a Trade Secret of 
 * Dialog Semiconductor Ltd.  All use, disclosure, and/or reproduction is prohibited 
 * unless authorized in writing. All Rights Reserved.
*
* <bluetooth.support@diasemi.com> and contributors.
*
OTOR_RES
≥ı ºªØ:
gapm_device_ready_ind_handler
gapm_device_ready_ind_handler
app_configuration_func
gapm_device_ready_ind_handler
app_set_dev_config_complete_func
app_db_init_func
app_module_init_cmp_evt_handler
app_db_init_func
app_db_init_complete_func
app_adv_start
app_adv_func

gapc_connection_req_ind_handler //÷˜…Ë±∏¡¨»Î¥¶¿Ì
app_connection_func
app_connect_confirm
gapm_device_ready_ind_handler


prf_cleanup_func  //÷˜…Ë±∏∂œø™¥¶¿Ì
gapc_cmp_evt_handler            
app_disconnect_func
app_adv_start
app_adv_func


gattc_write_cmd_ind_handler//–ﬁ∏ƒ¿∂—¿MACµÿ÷∑

//œÍœ∏ 
GWI_UART_init

prf_init_func
gapm_device_ready_ind_handler
prf_init_func
gapm_device_ready_ind_handler
app_configuration_func
gapm_device_ready_ind_handler
app_set_dev_config_complete_func
app_db_init_func
streamdatad_create_db_req_handler
app_module_init_cmp_evt_handler
app_db_init_func
app_db_init_complete_func
app_adv_start
app_adv_func
gapc_connection_req_ind_handler
app_connection_func
app_connect_confirm
gapm_device_ready_ind_handler
int streamdatad_enable_req_handler
gattc_write_cmd_ind_handler
DDEEFF gattc_write_cmd_ind_handler
AABBCC  gattc_write_cmd_ind_handler
DDEEFF  gattc_write_cmd_ind_handler
AABBCC prf_server_send_event
prf_server_send_event
prf_server_send_event
prf_server_send_event


        #if (BLE_SPOTA_RECEIVER)               //M10 OTA
        app_spotar_enable();
        #endif //BLE_SPOTA_RECEIVER			

Ω” ‹macµÿ÷∑–ﬁ∏ƒ gattc_write_cmd_ind_handler

motor_handle
motor_reset_handle
****************************************************************************************
*/

/**
 ****************************************************************************************
 * @addtogroup APP
 * @{
 ****************************************************************************************
 */
#include "rwip_config.h"             // SW configuration

#if (BLE_APP_PRESENT)

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "app_sec.h"
#include "app_throughput_peripheral_proj.h"

#include "periph_setup.h"
#include "gpio.h"
#include "uart.h"
#include "pwm.h"
#include "adc.h"
#include "co_math.h"                 // Common Maths Definition
#include "gapc.h"
#if (NVDS_SUPPORT)
#include "nvds.h"                    // NVDS Definitions
#endif //(NVDS_SUPPORT)

#include "l2cc_task.h"

#if (STREAMDATA_QUEUE)
#include "app_stream_queue.h"
#endif

//–≠“È’ª∑¢ÀÕ÷ß≥÷ µ˜ ‘µÁ¡˜ƒ£ Ω//
#include "attm_db.h"
#include "prf_utils.h"
#include "streamdatad.h"

#include "app.h"
///
uint8_t test_state __attribute__((section("retention_mem_area0"),zero_init)); //@RETENTION MEMORY

#define MAX_TX_BUFS (32)
#define STREAMDATAD_PACKET_SIZE (20)
uint8 test_pkt [MAX_TX_BUFS][STREAMDATAD_PACKET_SIZE];

#ifdef METRICS
extern struct metrics global_metrics_tosend;
#endif

unsigned char adv_timer_flag = 0;
uint8_t uBatRemainPercent = 100,BatBuf[10],uBatRemainPercentMA=0,adbattery_status=0,BatCnt = 0,kai=0;
		float AD_value1 = 0.000;
	static	uint8_t MotorTimCnt=0,MotorFlag=0,LowPower=0,PowerOff=0;
extern volatile uint8_t	Power,MotorOn,ZigbFlg,BLE_MAC_Edit;
extern uint16_t LEDCount,Adc_vbat,iseccnt3,iseccnt4, iseccnt4_flag,iseccnt5,iseccnt5_flag;//,lkeyDownCount,iSecCount,iSecCnt2,SystemCount,count_present,BeSecTmp[2];
//extern unsigned char cIptWarning,cFunctionType;
//uint8_t LacalSetting[5]={0xAB,0xBC,0xCD,0xD1,0x05};		//"ABBCCDD105" ≤È—Ø±æµÿµÿ÷∑
extern float Adc_div ;
unsigned char  VIsenseCount;
unsigned short  VIsense[5],VIsense_Avg;
unsigned char  VIsense_Avg_send[2];
extern volatile uint8_t Motor;
extern volatile uint16_t resetTime;
extern volatile uint16_t setTime;

bool needStop;
bool needRelease_flag;
bool needPress_flag;

int adc_timer_handle(ke_msg_id_t const msgid, 
                           void const *param, 
													 ke_task_id_t const dest_id, 
													 ke_task_id_t const src_id)
{
//    for(uint16_t i = 0 ; i< 10; i++)
//    {
	    adc_enable_channel(ADC_CHANNEL_P01);//Ω´ADCÕ®µ¿«–ªª÷¡Õ®µ¿0  2017.10.13
	    uint16_t adc_data = adc_get_sample();
	     uint8_t i=0;	
      if(adc_data!=0 && MotorFlag!=1&&ZigbFlg!=1)
    	uBatRemainPercent = (char)(((adc_data-611.000)/(747.000 - 611.000))*100);   	                                                                                  			
	    if((adc_data-611.000)<=0 && adc_data!=0)
	      uBatRemainPercent = 0;			
	    BatBuf[BatCnt] =  uBatRemainPercent;
			BatCnt++;		
//     }	
//	    if( adbattery_status==0)	   
//		{	 
//					 for(i=BatCnt;i<10;i++)
//					 { 
//                        BatBuf[i] = 0x40;
//                       }
			uBatRemainPercentMA = fliter(BatBuf,BatCnt); //µÁ≥ÿµÁ¡øæ˘÷µ¬À≤®Œ¥ÕÍ≥… ±∑¢ÀÕ µ ± ˝æ›
//		}
//		else 	 
//				   uBatRemainPercentMA = fliter(BatBuf,10);
			
		if(BatCnt>=10)
		{	 
			adbattery_status=1;
            MotorFlag = 1;
			BatCnt = 0;							   	
		}			
			
			
			if(uBatRemainPercentMA>100)
					uBatRemainPercentMA=100;

			if(uBatRemainPercentMA<=0x14) //738.2--3.5(3.48)
			{
//			if((adc_data-602.000)<=0)  //3.4v
				if(uBatRemainPercentMA<=0) //3.45(3.34) 611
					{				
						uBatRemainPercentMA = 0;	  			
					}
			 
				if(LEDCount%3==0)	 						
					{						   
						if(LEDCount%6==0)
						GPIO_SetActive(GPIO_PORT_2, GPIO_PIN_0);  //LED
					}
				else  
					{
						GPIO_SetInactive(GPIO_PORT_2, GPIO_PIN_0); 
					}
							if(PowerOff==1)		
								Power = 0;										 
//				if((adc_data-602.000)< 0.000)		//3.4
					if(uBatRemainPercentMA<=0 && Motor == 0)
						{   		
                            delay_ms(2000);
							//app_timer_set(MOTOR_TIMER, TASK_APP, 20);
                             motor_handleEx();
                            LowPower = 1;
                            Motor = 1;
													
						}

			 }
           
			 if(MotorFlag == 1)
			ke_timer_clear(ADC_TIMER,TASK_APP);
		  else
			app_timer_set(ADC_TIMER, TASK_APP, 20);  //MOTO FLAGŒ™1
			
			 
			 
			
//			printf_string("\n\radc_data:");
//			print_hword(adc_data);
	
			return 0;
}

/////////¥Æø⁄handle  ∫Ø ˝ µÃÂ////////////////////////
 uint8_t flag_uart = 0;
int uart_handle(ke_msg_id_t const msgid, 
                           void const *param, 
													 ke_task_id_t const dest_id, 
													 ke_task_id_t const src_id)
{
//	printf_string("GWI_UART_DEMO\n\r");
	if(flag_uart)		
	  app_timer_set(UART_TIMER, TASK_APP, 100);
	else
		ke_timer_clear(UART_TIMER,TASK_APP);
	
	return 0;
}
uint8_t j=0;


void motor_handleEx(void)
{
    setTime=0;         //—πÀıº∆ ˝«Â¡„£¨ø™ ºº∆ ±
    VIsense_Avg = 0;	
     VIsense[0]=0;   
    VIsense[1]=0;
    VIsense[2]=0;
    VIsense[3]=0;
    VIsense[4]=0;
    VIsense_Avg=0;
    VIsenseCount=0;
    iseccnt4=0;
    iseccnt4_flag=0;  //µΩ¥Ô—πÀı¡¶º´œﬁ±Í÷æŒª
    needStop = false;
	
	  needPress_flag =true ;//–Ë“™÷¥––—πÀı±Í ∂
	
//    while(!needStop)
//    {
//        adc_enable_channel(ADC_CHANNEL_P02);        //µÁª˙∂¬◊™µÁ¡˜≤…—˘£¨   ≤…—˘µÁ◊Ë 5.1≈∑ƒ∑
//        VIsense[VIsenseCount] = adc_get_sample(); 
//	    VIsenseCount++;
//		VIsense_Avg= (VIsense[0]+VIsense[1]+VIsense[2])/3;    //3¥Œ≤…—˘µƒ∆Ωæ˘÷µ
//		if(VIsenseCount == 3)					     					   						
//		{VIsenseCount = 0;}		
//        GPIO_SetInactive(GPIO_PORT_1, GPIO_PIN_1);		// MOTOR_IN2----P1_1∏≥µÕµÁ∆Ω
//        GPIO_SetActive(GPIO_PORT_1, GPIO_PIN_0);      // MOTOR_IN1----P1_0∏≥∏ﬂµÁ∆Ω													  //MOTOR_IN2µÕ£¨ MOTOR_IN1∏ﬂ  µÁª˙’˝œÚ◊™∂Ø		
//        GPIO_SetActive(GPIO_PORT_2, GPIO_PIN_7);	   //µÁª˙5VµÁ‘¥π©µÁ πƒ‹£¨  øÿ÷∆AAT1218 EN∂À
//        GPIO_SetActive(GPIO_PORT_0, GPIO_PIN_0);	   //µÁª˙π©µÁø™πÿ Q6ø™πÿ πƒ‹£¨ øÿ÷∆8050µƒBº´øÿ÷∆ PMOS
//        if(setTime > 80)
//        {
//            needStop = true;
//        }
//        if(VIsense_Avg>=0x0220)
//        {
//            
//            iseccnt4_flag=1;  //µΩ¥Ô—πÀı¡¶º´œﬁ£¨ø™ ºº∆ ±£¨±£≥÷¡¶500ms
//           
//        }
//        if(iseccnt4>60)  //º´œﬁ—πÀı¡¶±£≥÷ ±º‰
//        {             
//              needStop = true;
//        }
//    }
//  
//    MotorOn = 0; 					
//    GPIO_SetInactive(GPIO_PORT_2, GPIO_PIN_7);     //πÿ±’ µÁª˙5VµÁ‘¥π©µÁ πƒ‹£¨  øÿ÷∆AAT1218 EN∂À
//    GPIO_SetInactive(GPIO_PORT_0, GPIO_PIN_0);    //πÿ±’ µÁª˙π©µÁø™πÿ Q6ø™πÿ πƒ‹£¨ øÿ÷∆8050µƒBº´øÿ÷∆ PMOS
//    MotorFlag = 0;    
}

void Press_EXE(void)
{
      if(needPress_flag == true)
			{
				adc_enable_channel(ADC_CHANNEL_P02);        //µÁª˙∂¬◊™µÁ¡˜≤…—˘£¨   ≤…—˘µÁ◊Ë 5.1≈∑ƒ∑
				VIsense[VIsenseCount] = adc_get_sample(); 
				VIsenseCount++;
				VIsense_Avg= (VIsense[0]+VIsense[1]+VIsense[2])/3;    //3¥Œ≤…—˘µƒ∆Ωæ˘÷µ
				if(VIsenseCount == 3)					     					   						
				{VIsenseCount = 0;}		
				GPIO_SetInactive(GPIO_PORT_1, GPIO_PIN_1);		// MOTOR_IN2----P1_1∏≥µÕµÁ∆Ω
				GPIO_SetActive(GPIO_PORT_1, GPIO_PIN_0);      // MOTOR_IN1----P1_0∏≥∏ﬂµÁ∆Ω													  //MOTOR_IN2µÕ£¨ MOTOR_IN1∏ﬂ  µÁª˙’˝œÚ◊™∂Ø		
				GPIO_SetActive(GPIO_PORT_2, GPIO_PIN_7);	   //µÁª˙5VµÁ‘¥π©µÁ πƒ‹£¨  øÿ÷∆AAT1218 EN∂À
				GPIO_SetActive(GPIO_PORT_0, GPIO_PIN_0);	   //µÁª˙π©µÁø™πÿ Q6ø™πÿ πƒ‹£¨ øÿ÷∆8050µƒBº´øÿ÷∆ PMOS
				
				if(setTime > 100)  //»Áπ˚ ±º‰µΩ  //100*10ms=1s   —πÀı‘À∂Ø ±º‰1s÷”
				{
					  setTime =0;
						needStop = true;
					
					  needPress_flag =false ;
					
					
						MotorOn = 0; 					
						GPIO_SetInactive(GPIO_PORT_2, GPIO_PIN_7);     //πÿ±’ µÁª˙5VµÁ‘¥π©µÁ πƒ‹£¨  øÿ÷∆AAT1218 EN∂À
						GPIO_SetInactive(GPIO_PORT_0, GPIO_PIN_0);    //πÿ±’ µÁª˙π©µÁø™πÿ Q6ø™πÿ πƒ‹£¨ øÿ÷∆8050µƒBº´øÿ÷∆ PMOS
						MotorFlag = 0;
					
				}
				if(VIsense_Avg>=0x0220)  //»Áπ˚¡¶œ»µΩ
				{
						
						iseccnt4_flag=1;  //µΩ¥Ô—πÀı¡¶º´œﬁ£¨ø™ ºº∆ ±£¨±£≥÷¡¶500ms
					 
				}
				
				if(iseccnt4>30)  //º´œﬁ—πÀı¡¶±£≥÷ ±º‰  60*10ms
				{      
             iseccnt4	=0;	
             iseccnt4_flag =0;

					   if(VIsense_Avg>=0x0220)  //»Áπ˚¡¶œ»µΩ
				     {
							needStop = true;
							needPress_flag =false ;
					
							MotorOn = 0; 					
							GPIO_SetInactive(GPIO_PORT_2, GPIO_PIN_7);     //πÿ±’ µÁª˙5VµÁ‘¥π©µÁ πƒ‹£¨  øÿ÷∆AAT1218 EN∂À
							GPIO_SetInactive(GPIO_PORT_0, GPIO_PIN_0);    //πÿ±’ µÁª˙π©µÁø™πÿ Q6ø™πÿ πƒ‹£¨ øÿ÷∆8050µƒBº´øÿ÷∆ PMOS
							MotorFlag = 0;
					   }
				}
			}

}




/*
µÁª˙øÿ÷∆≥Ã–Ú
*/
int motor_handle(ke_msg_id_t const msgid, 
                       void const *param, 
			  ke_task_id_t const dest_id, 
			   ke_task_id_t const src_id)
{ 
				adc_enable_channel(ADC_CHANNEL_P02);        //µÁª˙∂¬◊™µÁ¡˜≤…—˘£¨   ≤…—˘µÁ◊Ë 5.1≈∑ƒ∑
				if(kai==0){iseccnt3=0;kai=1;}	
        		VIsense[VIsenseCount] = adc_get_sample(); 
			    VIsenseCount++;
				VIsense_Avg= (VIsense[0]+VIsense[1]+VIsense[2])/3;    //3¥Œ≤…—˘µƒ∆Ωæ˘÷µ
				if(VIsenseCount == 3)					     					   						
				{VIsenseCount = 0;}		         
				

//¿œµÁª˙∞Ê±æ		£¨—πÀı
//				GPIO_SetInactive(GPIO_PORT_1, GPIO_PIN_0);		// MOTOR_IN1----P1_0∏≥µÕµÁ∆Ω
//				GPIO_SetActive(GPIO_PORT_1, GPIO_PIN_1);      // MOTOR_IN2----P1_1∏≥∏ﬂµÁ∆Ω
																  //MOTOR_IN1µÕ£¨ MOTOR_IN2∏ﬂ  µÁª˙’˝œÚ◊™∂Ø
					
			//–¬µÁª˙∞Ê±æ		£¨—πÀı
			GPIO_SetInactive(GPIO_PORT_1, GPIO_PIN_1);		// MOTOR_IN2----P1_1∏≥µÕµÁ∆Ω
			GPIO_SetActive(GPIO_PORT_1, GPIO_PIN_0);      // MOTOR_IN1----P1_0∏≥∏ﬂµÁ∆Ω
														  //MOTOR_IN2µÕ£¨ MOTOR_IN1∏ﬂ  µÁª˙’˝œÚ◊™∂Ø
			
			GPIO_SetActive(GPIO_PORT_2, GPIO_PIN_7);	   //µÁª˙5VµÁ‘¥π©µÁ πƒ‹£¨  øÿ÷∆AAT1218 EN∂À
			GPIO_SetActive(GPIO_PORT_0, GPIO_PIN_0);	   //µÁª˙π©µÁø™πÿ Q6ø™πÿ πƒ‹£¨ øÿ÷∆8050µƒBº´øÿ÷∆ PMOS

		
//			if(VIsense_Avg>=0x0220||iseccnt3>1)                  //≈–∂œ≤…—˘µÁ—π £¨ 0x015E= 350£¨ ªªÀ„Œ™µÁ—πŒ350*1.2/1024=0.4101 V  Uad= VIsense_Avg*1.2/1024  (1.2V,ƒ⁄≤ø≤ŒøºµÁ—π£¨ 1024 Œª10ŒªAD≤…—˘£©
//			{  
//				if(VIsense_Avg>=0x0220)  //¡¶œ»µΩ                  //≈–∂œ≤…—˘µÁ—π £¨ 0x0220= 544£¨ 544*1.2/1024=0.6375    ????= 125mA
//				{
//					iseccnt4_flag=1;  //µΩ¥Ô—πÀı¡¶º´œﬁ£¨ø™ ºº∆ ±£¨±£≥÷¡¶500ms
//					
//					if(iseccnt4>2)  //º´œﬁ—πÀı¡¶±£≥÷ ±º‰
//					{
//						MotorOn = 0;
//						kai=0;
//						iseccnt4_flag=1;  //µΩ¥Ô—πÀı¡¶º´œﬁ±Í÷æŒª
//						iseccnt3=0; 
//						iseccnt4=0;							
//						GPIO_SetInactive(GPIO_PORT_2, GPIO_PIN_7);   
//						GPIO_SetInactive(GPIO_PORT_0, GPIO_PIN_0);   		
//						VIsense[0]=0;   
//						VIsense[1]=0;
//						VIsense[2]=0;
//						VIsense[3]=0;
//						VIsense[4]=0;
//						VIsense_Avg=0;
//						VIsenseCount=0;
//						MotorFlag = 0;     
////						app_timer_set(ADC_TIMER, TASK_APP, 20);     
//						ke_timer_clear(MOTOR_TIMER,TASK_APP);	       
//					}
//					else
//					{
//						app_timer_set(MOTOR_TIMER, TASK_APP, 50);
//						MotorFlag = 1;
//					}	
//				 }	
				 if(iseccnt3>1)  // ±º‰œ»µΩ
				 {
					kai=0;
					iseccnt3=0;
					iseccnt4=0;
					MotorOn = 0; 					
					GPIO_SetInactive(GPIO_PORT_2, GPIO_PIN_7);     //πÿ±’ µÁª˙5VµÁ‘¥π©µÁ πƒ‹£¨  øÿ÷∆AAT1218 EN∂À
					GPIO_SetInactive(GPIO_PORT_0, GPIO_PIN_0);    //πÿ±’ µÁª˙π©µÁø™πÿ Q6ø™πÿ πƒ‹£¨ øÿ÷∆8050µƒBº´øÿ÷∆ PMOS
					VIsense[0]=0;   
					VIsense[1]=0;
					VIsense[2]=0;
					VIsense[3]=0;
					VIsense[4]=0;
					VIsense_Avg=0;
					VIsenseCount=0;
					MotorFlag = 0;     
//					app_timer_set(ADC_TIMER, TASK_APP, 20);     
					ke_timer_clear(MOTOR_TIMER,TASK_APP);	 
				}	
//        }
        else  // ±º‰∫Õ¡¶∂º√ªµΩ£¨ºÃ–¯◊™∂Ø
        {
					app_timer_set(MOTOR_TIMER, TASK_APP, 60);   //   ±º‰Œ¥µΩ£¨ºÃ–¯’˝◊™£¨ µ»¥˝500ms∫Û£¨‘ŸΩ¯»Î∫Ø ˝≈–∂œ
					MotorFlag = 1;               //MOTO FLAGŒ™1“‘∫Û ‘Ÿ“≤≤ªª·¥•∑¢ADC∆Ù∂Ø¡À
        }				 
				
	return 0;
}
void motor_reset_handleEx(void)
{
    resetTime=0;    // Õ∑≈º∆ ˝«Â¡„£¨ø™ ºº∆ ±
    VIsense[0]=0;  
    VIsense[1]=0;
    VIsense[2]=0;
    VIsense[3]=0;
    VIsense[4]=0;
    VIsense_Avg=0;
    VIsenseCount=0;
	  iseccnt5=0;
    iseccnt5_flag=0;  //µΩ¥Ô—πÀı¡¶º´œﬁ±Í÷æŒª
    MotorFlag = 0;
    kai = 0;
    resetTime = 0;
  
	  needRelease_flag =true ;//–Ë“™÷¥––—πÀı±Í ∂
}

void Release_EXE(void)
{
	    if(needRelease_flag == true)
			{
				adc_enable_channel(ADC_CHANNEL_P02);        //µÁª˙∂¬◊™µÁ¡˜≤…—˘£¨   ≤…—˘µÁ◊Ë 5.1≈∑ƒ∑
				VIsense[VIsenseCount] = adc_get_sample(); 
				VIsenseCount++;
				VIsense_Avg= (VIsense[0]+VIsense[1]+VIsense[2])/3;    //3¥Œ≤…—˘µƒ∆Ωæ˘÷µ
				if(VIsenseCount == 3)					     					   						
				{VIsenseCount = 0;}	
	
				GPIO_SetInactive(GPIO_PORT_1, GPIO_PIN_0);	        // MOTOR_IN1----P1_0∏≥µÕµÁ∆Ω
        GPIO_SetActive(GPIO_PORT_1, GPIO_PIN_1);            // MOTOR_IN2----P1_1∏≥∏ﬂµÁ∆Ω
        GPIO_SetActive(GPIO_PORT_0, GPIO_PIN_0);	          //µÁª˙π©µÁø™πÿ Q6ø™πÿ πƒ‹£¨P0_0øÿ÷∆8050µƒBº´øÿ÷∆ PMOS 
	      GPIO_SetActive(GPIO_PORT_2, GPIO_PIN_7);	   				//µÁª˙5VµÁ‘¥π©µÁ πƒ‹£¨  P2_7øÿ÷∆AAT1218 EN∂À	
				
				
				if(resetTime > 200)  //»Áπ˚ ±º‰µΩ  //200*10ms=2s    Õ∑≈‘À∂Ø ±º‰2s÷”
				{
					  resetTime = 0;
						needRelease_flag =false ;
					
					
						MotorOn = 1; 					
						GPIO_SetInactive(GPIO_PORT_2, GPIO_PIN_7);     //πÿ±’ µÁª˙5VµÁ‘¥π©µÁ πƒ‹£¨  øÿ÷∆AAT1218 EN∂À
						GPIO_SetInactive(GPIO_PORT_0, GPIO_PIN_0);    //πÿ±’ µÁª˙π©µÁø™πÿ Q6ø™πÿ πƒ‹£¨ øÿ÷∆8050µƒBº´øÿ÷∆ PMOS
						MotorFlag = 0;
					
				}
				if(VIsense_Avg>=0x0096)  //»Áπ˚¡¶œ»µΩ
				{
						
						iseccnt5_flag=1;  //µΩ¥Ô—πÀı¡¶º´œﬁ£¨ø™ ºº∆ ±£¨±£≥÷¡¶500ms
					 
				}
				if(iseccnt5>30)  //º´œﬁ—πÀı¡¶±£≥÷ ±º‰  10*10ms
				{      
             iseccnt4	=0;	
             iseccnt5_flag =0;

					   if(VIsense_Avg>=0x0096)  //»Áπ˚¡¶œ»µΩ
				     {
							needStop = true;
							needPress_flag =false ;
					
							MotorOn = 1; 					
							GPIO_SetInactive(GPIO_PORT_2, GPIO_PIN_7);     //πÿ±’ µÁª˙5VµÁ‘¥π©µÁ πƒ‹£¨  øÿ÷∆AAT1218 EN∂À
							GPIO_SetInactive(GPIO_PORT_0, GPIO_PIN_0);    //πÿ±’ µÁª˙π©µÁø™πÿ Q6ø™πÿ πƒ‹£¨ øÿ÷∆8050µƒBº´øÿ÷∆ PMOS
							MotorFlag = 0;
					   }
				}
				
			}
				
}


/*
µÁª˙ ∏¥Œªøÿ÷∆≥Ã–Ú
*/

uint8_t ResetCnt=0;

int motor_reset_handle(ke_msg_id_t const msgid,
	                         void const *param,
			        ke_task_id_t const dest_id,
					 ke_task_id_t const src_id)
{
	
		adc_enable_channel(ADC_CHANNEL_P02);      
    VIsense[VIsenseCount] = adc_get_sample(); 
		VIsenseCount++;
		VIsense_Avg= (VIsense[0]+VIsense[1]+VIsense[2])/3; 
	  if(VIsenseCount == 3)					     					   						
		{VIsenseCount = 0;}		
	  if(kai==0){resetTime=0;kai=1;}	  
//¿œµÁª˙∞Ê±æ		£¨ Õ∑≈
//	    GPIO_SetInactive(GPIO_PORT_1, GPIO_PIN_1);	        // MOTOR_IN2----P1_1∏≥µÕµÁ∆Ω
//      GPIO_SetActive(GPIO_PORT_1, GPIO_PIN_0);            // MOTOR_IN1----P1_0∏≥∏ﬂµÁ∆Ω
                                                                //MOTOR_IN1∏ﬂ£¨ MOTOR_IN2µÕ  µÁª˙∑¥œÚ◊™∂Ø 
//–¬µÁª˙∞Ê±æ		£¨ Õ∑≈
        GPIO_SetInactive(GPIO_PORT_1, GPIO_PIN_0);	        // MOTOR_IN1----P1_0∏≥µÕµÁ∆Ω
        GPIO_SetActive(GPIO_PORT_1, GPIO_PIN_1);            // MOTOR_IN2----P1_1∏≥∏ﬂµÁ∆Ω
                                                                //MOTOR_IN2∏ﬂ£¨ MOTOR_IN1µÕ  µÁª˙∑¥œÚ◊™∂Ø								
								
		GPIO_SetActive(GPIO_PORT_0, GPIO_PIN_0);	          //µÁª˙π©µÁø™πÿ Q6ø™πÿ πƒ‹£¨P0_0øÿ÷∆8050µƒBº´øÿ÷∆ PMOS 
	    GPIO_SetActive(GPIO_PORT_2, GPIO_PIN_7);	   				//µÁª˙5VµÁ‘¥π©µÁ πƒ‹£¨  P2_7øÿ÷∆AAT1218 EN∂À		
				
//		if(VIsense_Avg>=0x00C8)                            //≈–∂œ≤…—˘µÁ—π £¨ 0x00C8=200 £¨   
				
								
//      if(VIsense_Avg>=0x0064)                            //≈–∂œ≤…—˘µÁ—π £¨ 0x0064=100  ªªÀ„Œ™µÁ—π100*1.2/1024=0.117V  Uad= VIsense_Avg*1.2/1024  (1.2V,ƒ⁄≤ø≤ŒøºµÁ—π£¨ 1024 Œª10ŒªAD≤…—˘£©¨£¨ 
		
		//”√¡¶≈–∂œ
		if(VIsense_Avg>=0x0096)           // 0x0096=150   150*1.2/1024=0.1758V   0.1758/5.1= 34.47mA
		{  
//          MotorOn = 0; 	
      MotorOn = 1;		   //jc 2018.3.1∏¸∏ƒŒ™1£¨‘≠¿¥Œ™0£¨  ≈–∂œ“¿æ›Œ™  µÁª˙∏¥Œª±Í ∂ MotorOn=1£¨±Í ∂∏¥Œª			
			GPIO_SetInactive(GPIO_PORT_2, GPIO_PIN_7);    //µÁª˙5VµÁ‘¥π©µÁ ,Ω˚÷π πƒ‹£¨  P2_7øÿ÷∆AAT1218 EN∂À	
			GPIO_SetInactive(GPIO_PORT_0, GPIO_PIN_0);    //µÁª˙π©µÁø™πÿ Q6ø™πÿΩ˚÷π πƒ‹£¨P0_0øÿ÷∆8050µƒBº´øÿ÷∆ PMOS 	 		
			VIsense[0]=0;  
			VIsense[1]=0;
			VIsense[2]=0;
			VIsense[3]=0;
			VIsense[4]=0;
			VIsense_Avg=0;
			VIsenseCount=0;
			MotorFlag = 0;
			app_timer_set(ADC_TIMER, TASK_APP, 20);     
			ke_timer_clear(MOTOR_RESET,TASK_APP);	      			
     }
		
			 //”√ ±º‰≈–∂œ
		if(resetTime > 7)
		{
      MotorOn = 1;		   //jc 2018.3.1∏¸∏ƒŒ™1£¨‘≠¿¥Œ™0£¨  ≈–∂œ“¿æ›Œ™  µÁª˙∏¥Œª±Í ∂ MotorOn=1£¨±Í ∂∏¥Œª			
			GPIO_SetInactive(GPIO_PORT_2, GPIO_PIN_7);    //µÁª˙5VµÁ‘¥π©µÁ ,Ω˚÷π πƒ‹£¨  P2_7øÿ÷∆AAT1218 EN∂À	
			GPIO_SetInactive(GPIO_PORT_0, GPIO_PIN_0);    //µÁª˙π©µÁø™πÿ Q6ø™πÿΩ˚÷π πƒ‹£¨P0_0øÿ÷∆8050µƒBº´øÿ÷∆ PMOS 	 		
			VIsense[0]=0;  
			VIsense[1]=0;
			VIsense[2]=0;
			VIsense[3]=0;
			VIsense[4]=0;
			VIsense_Avg=0;
			VIsenseCount=0;
			MotorFlag = 0;
			kai = 0;
			resetTime = 0;
//			app_timer_set(ADC_TIMER, TASK_APP, 20);     
			ke_timer_clear(MOTOR_RESET,TASK_APP);	
        }
		else
        { 
            app_timer_set(MOTOR_RESET, TASK_APP, 50);
			MotorFlag = 1;
        }		
   return 0;

}

int key_scan_handle(ke_msg_id_t const msgid, 
                           void const *param, 
													 ke_task_id_t const dest_id, 
													 ke_task_id_t const src_id)
{
						
	return 0;
}
							
int key_dispose_handle(ke_msg_id_t const msgid, 
                           void const *param, 
													 ke_task_id_t const dest_id, 
													 ke_task_id_t const src_id)
{
	uint8_t j=0;
	
	return 0;
}

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
*/

 /**
 ****************************************************************************************
 * @brief app_api function. Project's actions in app_init
 *
 * @return void.
 ****************************************************************************************
*/

void app_init_func(void)
{	

/**************************************************
Call initializiation function for supported profiles
i.e.    
#if (BLE_DIS_SERVER)    
	app_dis_init();
#endif
****************************************************/   
#if (BLE_DIS_SERVER)        
    app_dis_init();
#endif	

	
#if (BLE_SPOTA_RECEIVER)    
	app_spotar_init();
#endif
}

/**
 ****************************************************************************************
 * @brief app_api function. Project's actions in app_sec_init during system initialization
 *
 * @return void.
 ****************************************************************************************
*/

void app_sec_init_func(void)
{

/**************************************************
Initialize security environment. 
i.e.    
//Setup required auth mode.    
#if (BLE_APP_SEC)
	app_sec_env.auth = (GAP_AUTH_REQ_NO_MITM_BOND);
#endif
****************************************************/    
    
}


/**
 ****************************************************************************************
 * @brief app_api function. Project's actions in app_connect (device connection)
 *
 * @param[in] taskid     App task's id.
 * @param[in] param      Received gapc_connection_req_ind.
 *
 * @return bool     true for connection request acception / false rejection 
 ****************************************************************************************
*/

void app_connection_func(struct gapc_connection_req_ind const *param)
{  
	#if (M10DEBUG)
	printf_string("app_connection_func\n\r");
		#endif
    if (app_env.conidx != GAP_INVALID_CONIDX)
    {     
        ke_timer_clear(APP_CONN_TIMER, TASK_APP);
			
//        ke_timer_clear(MY_TIMER, TASK_APP);
//			  adv_timer_flag = 0;
        ke_state_set(TASK_APP, APP_CONNECTED);
        		
	      #if (BLE_APP_PRESENT)               //M10 
        app_dis_enable_prf(app_env.conhdl);
        #endif		
			
        #if (BLE_SPOTA_RECEIVER)               //M10 OTA
        app_spotar_enable();
        #endif //BLE_SPOTA_RECEIVER			
			
#if BLE_INTEGRATED_HOST_GTL
        struct app_ext_connect_ind *cmd = KE_MSG_ALLOC(APP_EXT_CONNECT_CMP_IND,
                                               TASK_GTL, TASK_APP, app_ext_connect_ind);
	
        cmd->state = true;
        cmd->conn_handle = param->conhdl;

        cmd->addr_type = param->peer_addr_type;
        memcpy(cmd->addr, param->peer_addr.addr, BD_ADDR_LEN);
    
        // Send the message
        ke_msg_send(cmd);
#endif        

#if (STREAMDATA_QUEUE)        
        stream_setup_int_tomax(param->con_interval);
#endif
        
#if BLE_STREAMDATA_DEVICE        
        app_streamdatad_enable();
#endif
        
        // Retrieve the connection info from the parameters
        //app_env.conhdl = param->conhdl;

        //app_env.peer_addr_type = param->peer_addr_type;
        //memcpy(app_env.peer_addr.addr, param->peer_addr.addr, BD_ADDR_LEN);

# if (BLE_APP_SEC)
            
        // send connection confirmation
        app_connect_confirm(app_sec_env.auth);
        
# else // (BLE_APP_SEC)
        // send connection confirmation
        app_connect_confirm(GAP_AUTH_REQ_NO_MITM_NO_BOND);            
# endif // (BLE_APP_SEC)
    }
    else
    {
        // No connection has been establish, restart advertising
        app_adv_start();
    }    
    return;
}

/**
 ****************************************************************************************
 * @brief app_api function. Project advertising function. Setup advertise string.
 *
 *
 *
 * @return void.
 ****************************************************************************************
*/
void app_adv_func(struct gapm_start_advertise_cmd *cmd)
{
    printf_string("\n app_adv_func()----app_timer_set(MOTOR_RESET ");	
//		app_timer_set(MOTOR_RESET, TASK_APP, 50);        //  20180322--JC∆¡±Œ £¨ ∑≈÷√∂œø™¿∂—¿∂Ø◊˜
		
		#if (M10DEBUG)
     printf_string("app_adv_func\n\r");
		#endif
//  Start advertising. Fill GAPM_START_ADVERTISE_CMD message
//    app_timer_set(MY_TIMER, TASK_APP, 100);
	  app_timer_set(ADC_TIMER, TASK_APP, 20);


	  adv_timer_flag = 1;
     //  Device Name Length
    uint8_t device_name_length;
    int8_t device_name_avail_space;
    uint8_t device_name_temp_buf[64];

    cmd->op.code     = GAPM_ADV_UNDIRECT;
    cmd->op.addr_src = GAPM_PUBLIC_ADDR;
    cmd->intv_min    = 150;//APP_ADV_INT_MIN;
    cmd->intv_max    = 150;//APP_ADV_INT_MAX;
    cmd->channel_map = APP_ADV_CHMAP;

    cmd->info.host.mode = GAP_GEN_DISCOVERABLE;

    /*-----------------------------------------------------------------------------------
     * Set the Advertising Data and the Scan Response Data
     *---------------------------------------------------------------------------------*/
    cmd->info.host.adv_data_len       = APP_ADV_DATA_MAX_SIZE;
    cmd->info.host.scan_rsp_data_len  = APP_SCAN_RESP_DATA_MAX_SIZE;
    
    // Advertising Data
    #if (NVDS_SUPPORT)
    if(nvds_get(NVDS_TAG_APP_BLE_ADV_DATA, &cmd->info.host.adv_data_len,
                &cmd->info.host.adv_data[0]) != NVDS_OK)
    #endif //(NVDS_SUPPORT)
    {
        cmd->info.host.adv_data_len = APP_ADV_DATA_LEN;
        memcpy(&cmd->info.host.adv_data[0], APP_ADV_DATA, cmd->info.host.adv_data_len);
    }

    // Scan Response Data
    #if (NVDS_SUPPORT)
    if(nvds_get(NVDS_TAG_APP_BLE_SCAN_RESP_DATA, &cmd->info.host.scan_rsp_data_len,
                &cmd->info.host.scan_rsp_data[0]) != NVDS_OK)
    #endif //(NVDS_SUPPORT)
    {
        cmd->info.host.scan_rsp_data_len = APP_SCNRSP_DATA_LENGTH;
        memcpy(&cmd->info.host.scan_rsp_data[0], APP_SCNRSP_DATA, cmd->info.host.scan_rsp_data_len);
    }

    // Get remaining space in the Advertising Data - 2 bytes are used for name length/flag
    device_name_avail_space = APP_ADV_DATA_MAX_SIZE - cmd->info.host.adv_data_len - 2;

    // Check if data can be added to the Advertising data
    if (device_name_avail_space > 0)
    {
        // Get the Device Name to add in the Advertising Data (Default one or NVDS one)
        #if (NVDS_SUPPORT)
        device_name_length = NVDS_LEN_DEVICE_NAME;
        if (nvds_get(NVDS_TAG_DEVICE_NAME, &device_name_length, &device_name_temp_buf[0]) != NVDS_OK)
        #endif //(NVDS_SUPPORT)
        {
            // Get default Device Name (No name if not enough space)
            device_name_length = strlen(APP_DEVICE_NAME);
            memcpy(&device_name_temp_buf[0], APP_DEVICE_NAME, device_name_length);
        }

        if(device_name_length > 0)
        {
            // Check available space
            device_name_length = co_min(device_name_length, device_name_avail_space);

            // Fill Length
            cmd->info.host.adv_data[cmd->info.host.adv_data_len]     = device_name_length + 1;
            // Fill Device Name Flag
            cmd->info.host.adv_data[cmd->info.host.adv_data_len + 1] = '\x09';
            // Copy device name
            memcpy(&cmd->info.host.adv_data[cmd->info.host.adv_data_len + 2], device_name_temp_buf, device_name_length);

            // Update Advertising Data Length
            cmd->info.host.adv_data_len += (device_name_length + 2);
        }
    }
    
    return;
}

/**
 ****************************************************************************************
 * @brief app_api function. Project's actions in app_disconnect
 *
 * @param[in] taskid     App task's id.
 * @param[in] param      Received gapc_disconnect_ind msg.
 *
 * @return void.            
 ****************************************************************************************
*/

void app_disconnect_func(ke_task_id_t task_id, struct gapc_disconnect_ind const *param)
{
    printf_string("\r app_disconnect_func\n\r");
    uint8_t state = ke_state_get(task_id);
    
#if BLE_BATT_SERVER
 app_batt_poll_stop();
#endif // BLE_BATT_SERVER
    
    if ((state == APP_SECURITY) || (state == APP_CONNECTED)  || (state == APP_PARAM_UPD))
    {
        
/**************************************************
            Handle disconnection event
***************************************************/    

        ke_state_set(TASK_APP, APP_CONNECTABLE);
#if (STREAMDATA_QUEUE)        
        test_state = APP_STREAM_STOP;
#endif
        
#if BLE_INTEGRATED_HOST_GTL        
        
        struct app_ext_connect_ind *cmd = KE_MSG_ALLOC(APP_EXT_DISCONNECT_CMP_IND,
                                               TASK_GTL, TASK_APP, app_ext_connect_ind);
	
        cmd->conn_handle = param->conhdl;
    
        // Send the message
        ke_msg_send(cmd);
        
#endif

        app_adv_start();
                        
    }
    else
    {
        // We are not in a Connected State
        ASSERT_ERR(0);
    }
      
}    
						


/**
 ****************************************************************************************
 * @brief app_api function. Project's actions for profiles' database initialization
 *
 * @return void.
 ****************************************************************************************
*/

volatile uint8_t dbg; 
bool app_db_init_func(void)
{
		#if (M10DEBUG)
     printf_string("app_db_init_func\n\r");
		#endif
/**************************************************
Initialize next supported profile's Database.    
Check if all supported profiles' Databases are initialized and return status.
***************************************************/    

    
    // Indicate if more services need to be added in the database
    bool end_db_create = false;
    
    dbg = APP_PRF_LIST_STOP;
    
    // Check if another should be added in the database
    if (app_env.next_prf_init < APP_PRF_LIST_STOP)
    {
        switch (app_env.next_prf_init)
        {
            #if (BLE_STREAMDATA_DEVICE)
            case (APP_STREAM_TASK):
            {
                // Add proxr Service in the DB
                app_stream_create_db();
            } break;
          #endif
						
            #if (BLE_DIS_SERVER)    //  
            case (APP_DIS_TASK):
            {
                app_dis_create_db_send();
            } break;
            #endif //BLE_DIS_SERVER							
						
            #if (BLE_SPOTA_RECEIVER)      //M10 OTA
            case (APP_SPOTAR_TASK):
            {
                // Add spotar Service in the DB
                app_spotar_create_db();
            } break;
            #endif //BLE_SPOTA_RECEIVER
						
            default:
            {
                ASSERT_ERR(0);
            } break;

        }

        // Select following service to add
        app_env.next_prf_init++;
    }
    else
    {
        end_db_create = true;
    }

    return end_db_create;
}

/**
 ****************************************************************************************
 * @brief app_api function. Sends request to update connection's paramaters.
 *
 * @return void.
 ****************************************************************************************
*/

void app_param_update_func(void)
{

/**************************************************
Create and send connection param update message.
***************************************************/    

    return;
}

/**
 ****************************************************************************************
 * @brief app_api function. Project configures GAPM. Called upon reset completion
 *
 * @param[in] task_id    id of the kernel task calling this function
 * @param[in] cmd        parameters to pass to the stack
 *
 * @return void.
 ****************************************************************************************
*/

void app_configuration_func(ke_task_id_t const task_id, struct gapm_set_dev_config_cmd *cmd)
{
    

/**************************************************
Create GAPM_SET_DEV_CONFIG message to configure BLE stack
***************************************************/    
    	#if (M10DEBUG)
     printf_string("app_configuration_func\n\r");  	
	#endif
    // set device configuration
    cmd->operation = GAPM_SET_DEV_CONFIG;
    // Device Role
    cmd->role = GAP_PERIPHERAL_SLV;
	
    // Device IRK
    // cmd->irk = ;

    // Device Appearance
    cmd->appearance = 0x0000;

    // Device Appearance write permission requirements for peer device
    cmd->appearance_write_perm = GAPM_WRITE_DISABLE;
    // Device Name write permission requirements for peer device
    cmd->name_write_perm = GAPM_WRITE_DISABLE;
    
    // Maximum trasnimt unit size
    cmd->max_mtu = 23;
		
    /*
    // Peripheral only: *****************************************************************
    // Slave preferred Minimum of connection interval
    cmd->con_intv_min = 8;         // 10ms (8*1.25ms)
    // Slave preferred Maximum of connection interval
    cmd->con_intv_max = 16;        // 20ms (16*1.25ms)
    // Slave preferred Connection latency
    cmd->con_latency  = 0;
    // Slave preferred Link supervision timeout
    cmd->superv_to    = 100;

    // Privacy settings bit field
    cmd->flags = 0;
    */
		
    return;   
}

/**
 ****************************************************************************************
 * @brief app_api function. Called upon device's configuration completion
 *
 * @return void.
 ****************************************************************************************
*/

void app_set_dev_config_complete_func(void)
{  
		#if (M10DEBUG)
	printf_string("app_set_dev_config_complete_func\n\r");
    	#endif
	  ke_state_set(TASK_APP, APP_DB_INIT);
    
    if (app_db_init())
    {
        // No service to add in the DB -> Start Advertising
        app_adv_start();
    }
    
#if BLE_INTEGRATED_HOST_GTL    
	struct app_ready_ind *ind = KE_MSG_ALLOC(APP_READY_IND, TASK_GTL, TASK_APP, app_ready_ind);
   
	ke_msg_send(ind);    
#endif //BLE_INTEGRATED_HOST_GTL
	
    return;
}

/**
 ****************************************************************************************
 * @brief app_api function. Called upon connection param's update rejection
 *
 * @param[in] status        Error code
 *
 * @return void.
 ****************************************************************************************
*/

void app_update_params_rejected_func(uint8_t status)
{
    return;
}

/**
 ****************************************************************************************
 * @brief app_api function. Called upon connection param's update completion
 *
 * @return void.
 ****************************************************************************************
*/

void app_update_params_complete_func(void)
{

/**************************************************
Handle connection parameters update event 
***************************************************/    
    return;
}

/**
 ****************************************************************************************
 * @brief app_api function. Handles undirect advertising completion.
 *
 * @param[in] status        Command complete message status
 *
 * @return void.
 ****************************************************************************************
*/

void app_adv_undirect_complete(uint8_t status)
{
    
/**************************************************
Handle undirected advirtising complete event
***************************************************/    
    
    return;
}

/**
 ****************************************************************************************
 * @brief app_api function. Handles direct advertising completion.
 *
 * @param[in] status        Command complete message status
 *
 * @return void.
 ****************************************************************************************
*/

void app_adv_direct_complete(uint8_t status)
{
    
/**************************************************
Handle directed advirtising complete event
***************************************************/    
    
    return;
}

/**
 ****************************************************************************************
 * @brief app_api function. Handles Database creation. Start application.
 *
 * @return void.
 ****************************************************************************************
*/

void app_db_init_complete_func(void)
{
		#if (M10DEBUG)
    printf_string("app_db_init_complete_func\n\r");
		#endif	
    app_adv_start();
    
    return;
}

/**
 ****************************************************************************************
 * @brief app_api function. Handles scan procedure completion.
 *
 * @return void.
 ****************************************************************************************
*/

void app_scanning_completed_func(void)
{
	#if (M10DEBUG)
  printf_string("app_scanning_completed_func\n\r");
		#endif
}

/**
 ****************************************************************************************
 * @brief app_api function. Fill gapm_start_connection_cmd msg params.
 *
 * @param[in] msg        gapm_start_connection_cmd message
 *
 * @return void.
 ****************************************************************************************
*/

void app_connect_func(struct gapm_start_connection_cmd *msg)                           
{
	#if (M10DEBUG)
  printf_string("app_connect_func\n\r");
		#endif
    
}

/**
 ****************************************************************************************
 * @brief app_api function. Handles reception of gapm_adv_report_ind msg.
 *
 * @param[in] param     gapm_adv_report_ind message
 *
 * @return void.
 ****************************************************************************************
*/

void app_adv_report_ind_func(struct gapm_adv_report_ind *param)
{
	#if (M10DEBUG)
  printf_string("app_adv_report_ind_func\n\r");
		#endif
}

/**
 ****************************************************************************************
 * @brief app_api function. Handles connection request failure.
 *
 * @return void.
 ****************************************************************************************
*/

void app_connect_failed_func(void)
{  
 	#if (M10DEBUG)
  printf_string("app_connect_failed_func\n\r");
		#endif   
}

#if (BLE_APP_SEC)
void app_send_pairing_rsp_func(struct gapc_bond_req_ind *param)
{

/**************************************************
Handle pairring request message. send a peiring response 
i.e.    
    
    struct gapc_bond_cfm* cfm = KE_MSG_ALLOC(GAPC_BOND_CFM, TASK_GAPC, TASK_APP, gapc_bond_cfm);

    cfm->request = GAPC_PAIRING_RSP;
    cfm->accept = true;

    // OOB information
    cfm->data.pairing_feat.oob            = GAP_OOB_AUTH_DATA_NOT_PRESENT;
    // Encryption key size
    cfm->data.pairing_feat.key_size       = KEY_LEN;
    // IO capabilities
    cfm->data.pairing_feat.iocap          = GAP_IO_CAP_NO_INPUT_NO_OUTPUT;
    // Authentication requirements
    cfm->data.pairing_feat.auth           = GAP_AUTH_REQ_NO_MITM_BOND;
    //Security requirements
    cfm->data.pairing_feat.sec_req        = GAP_NO_SEC;
    //Initiator key distribution
    //GZ cfm->data.pairing_feat.ikey_dist      = GAP_KDIST_NONE;
    cfm->data.pairing_feat.ikey_dist      = GAP_KDIST_SIGNKEY;
    //Responder key distribution
    cfm->data.pairing_feat.rkey_dist      = GAP_KDIST_ENCKEY;
    
    ke_msg_send(cfm);
    
***************************************************/
}

void app_send_tk_exch_func(struct gapc_bond_req_ind *param)
{
    
/**************************************************
Send Temporary key 
i.e.        
    struct gapc_bond_cfm* cfm = KE_MSG_ALLOC(GAPC_BOND_CFM, TASK_GAPC, TASK_APP, gapc_bond_cfm);
    uint32_t pin_code = app_sec_gen_tk();
    cfm->request = GAPC_TK_EXCH;
    cfm->accept = true;
    
    memset(cfm->data.tk.key, 0, KEY_LEN);
    
    cfm->data.tk.key[3] = (uint8_t)((pin_code & 0xFF000000) >> 24);
    cfm->data.tk.key[2] = (uint8_t)((pin_code & 0x00FF0000) >> 16);
    cfm->data.tk.key[1] = (uint8_t)((pin_code & 0x0000FF00) >>  8);
    cfm->data.tk.key[0] = (uint8_t)((pin_code & 0x000000FF) >>  0);
    
    ke_msg_send(cfm);
***************************************************/    
}

void app_send_irk_exch_func(struct gapc_bond_req_ind *param)
{
    
/**************************************************
Send IRK
***************************************************/        
    return;
}

void app_send_csrk_exch_func(struct gapc_bond_req_ind *param)
{
/**************************************************
Send CSRK
i.e.
    
    struct gapc_bond_cfm* cfm = KE_MSG_ALLOC(GAPC_BOND_CFM, TASK_GAPC, TASK_APP, gapc_bond_cfm);

    cfm->request = GAPC_CSRK_EXCH;

    cfm->accept = true;

    memset((void *) cfm->data.csrk.key, 0, KEY_LEN);
    memcpy((void *) cfm->data.csrk.key, (void *)"\xAB\xAB\x45\x55\x23\x01", 6);

    ke_msg_send(cfm);
***************************************************/        
}

void app_send_ltk_exch_func(struct gapc_bond_req_ind *param)
{
/**************************************************
Send Long term key 
i.e.
    
    struct gapc_bond_cfm* cfm = KE_MSG_ALLOC(GAPC_BOND_CFM, TASK_GAPC, TASK_APP, gapc_bond_cfm);

    // generate ltk
    app_sec_gen_ltk(param->data.key_size);

    cfm->request = GAPC_LTK_EXCH;

    cfm->accept = true;

    cfm->data.ltk.key_size = app_sec_env.key_size;
    cfm->data.ltk.ediv = app_sec_env.ediv;

    memcpy(&(cfm->data.ltk.randnb), &(app_sec_env.rand_nb) , RAND_NB_LEN);
    memcpy(&(cfm->data.ltk.ltk), &(app_sec_env.ltk) , KEY_LEN);

    ke_msg_send(cfm);
***************************************************/        
}

void app_paired_func(void)
{
/**************************************************
Handle Pairing/Bonding copletion event
**************************************************/    
    return;
}

bool app_validate_encrypt_req_func(struct gapc_encrypt_req_ind *param)
{
    
/**************************************************
Handle Pairing/Bonding copletion event
**************************************************/        
    
    return true;
}

void app_sec_encrypt_ind_func(void)
{
    
    return; 
}

void app_sec_encrypt_complete_func(void)
{
/**************************************************
Handle encryption completed event. 
****************************************************/    
    
    return; 
}

void app_mitm_passcode_entry_func(ke_task_id_t const src_id, ke_task_id_t const dest_id)
{
    
/**************************************************
Handle encryption completed event. 
****************************************************/        
    
    return;
}

#endif //BLE_APP_SEC

#if BLE_INTEGRATED_HOST_GTL

/**
 ****************************************************************************************
 * @brief Handles reception of uart test message from external processor application. Responds with APP_EXT_TEST_RSP
 * to confirm that channel is alive. 
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */

int app_uart_test_req_handler(ke_msg_id_t const msgid,
                                           const struct app_uart_test_req *param,
                                           ke_task_id_t const dest_id,
                                           ke_task_id_t const src_id)
{ 
    struct app_uart_test_rsp *cmd = KE_MSG_ALLOC(APP_EXT_TEST_RSP,
                                           src_id , TASK_APP,
                                           app_uart_test_rsp);

    cmd->value = param->value + 1;

    // Send the message
    ke_msg_send(cmd);

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of disconnect command from external processor application. Disconnects from peripherals
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */

int app_ext_disconnect_cmd_handler(ke_msg_id_t const msgid,
                             const struct app_ext_disconnect_cmd *param,
                             ke_task_id_t const dest_id,
                             ke_task_id_t const src_id)
{
 	#if (M10DEBUG)
  printf_string("app_ext_disconnect_cmd_handler\n\r");
		#endif 
    uint8_t conidx;
    
    conidx = gapc_get_conidx(param->conn_handle);
    
    struct gapc_disconnect_cmd *cmd = KE_MSG_ALLOC(GAPC_DISCONNECT_CMD,
                                              KE_BUILD_ID(TASK_GAPC, conidx), TASK_APP,
                                              gapc_disconnect_cmd);

    cmd->operation = GAPC_DISCONNECT;
    cmd->reason = CO_ERROR_REMOTE_USER_TERM_CON;

    // Send the message
    ke_msg_send(cmd);
	return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of start streaming command from external processor application. Starts/stops transmition proccess
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */

int app_ext_transmit_cmd_handler(ke_msg_id_t const msgid,
                             const struct app_ext_transmit_cmd *param,
                             ke_task_id_t const dest_id,
                             ke_task_id_t const src_id)
{
 	#if (M10DEBUG)
  printf_string("app_ext_transmit_cmd_handler\n\r");
		#endif 
    /// Start transmission
#if (STREAMDATA_QUEUE)        
    if (param->action == APP_STREAM_START)
    {
        if (test_state != APP_STREAM_START)
        {
//            set_pxact_gpio();
            test_pkt_init();
        }
        test_state = APP_STREAM_START;
    }
    else
        test_state = APP_STREAM_STOP;
#endif 
    
    return (KE_MSG_CONSUMED);
}

#endif //BLE_INTEGRATED_HOST_GTL

/**
 ****************************************************************************************
 * @brief Sends streaming counters to external processor application. 
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */

void app_ext_stats_report_send(void)
{
 	#if (M10DEBUG)
  printf_string("app_ext_stats_report_send\n\r");
		#endif
    if (ke_state_get(TASK_APP) == APP_CONNECTED)
    {
#ifdef METRICS
        //set_pxact_gpio();
#if  BLE_INTEGRATED_HOST_GTL
        struct app_ext_stats_report *cmd = KE_MSG_ALLOC(APP_EXT_STATS_REPORT,
                                                   TASK_GTL, TASK_APP, app_ext_stats_report);

        //memset(cmd, 0, sizeof(struct app_ext_stats_report));

        cmd->type =  0;
        cmd->conn_handle =  app_env.conhdl;
        memcpy(&cmd->timeperiod, &global_metrics_tosend, sizeof(uint16_t) * 6);
                
        // Send the message
        ke_msg_send(cmd);
#endif
#endif        //METRICS
        
    }
}

#if (STREAMDATA_QUEUE)

/**
 ****************************************************************************************
 * @brief Throughput test Function initialise application level buffers.
 *
 * @return void.
 ****************************************************************************************
*/

void test_pkt_init (void)
{
 	#if (M10DEBUG)
  printf_string("test_pkt_init\n\r");
		#endif
  int i;
  for (i=0;i<MAX_TX_BUFS;i++)
  {
    memset (&test_pkt[i] [0],0,2);
    memset (&test_pkt[i] [2],i,STREAMDATAD_PACKET_SIZE-2);
  }
  
}

/**
 ****************************************************************************************
 * @brief Callback function. Registered when packet is provided to stream queue. Called when packet is sent.
 *
 * @return void.
 ****************************************************************************************
*/

void test_callback (void* addr, int handle)
{
 	#if (M10DEBUG)
  printf_string("test_callback\n\r");
		#endif
  volatile uint16 *val = (volatile uint16 *) addr;
  (*val)++;
}

/**
 ****************************************************************************************
 * @brief Generate messages and provide to stream queue.
 *
 * @return void.
 ****************************************************************************************
*/

void test_pkt_gen (void)
{ 

  static int pt=0;
  static int hnd=0;
    
  if (test_state == APP_STREAM_START)
  {    
        
      while (stream_fifo_add (&test_pkt [pt], STREAMDATAD_PACKET_SIZE, STREAMDATAD_DIR_VAL_HANDLE(hnd), L2C_CODE_ATT_HDL_VAL_NTF, test_callback)>0)  //send as many as possible
      {
        pt++; if (pt>=MAX_TX_BUFS) pt=0;
        hnd++; if (hnd>=STREAMDATAD_MAX) hnd=0;
        
      };
  }
}
#endif

static unsigned char LEDCnt=0;
int my_timer_handler(ke_msg_id_t const msgid,
                               const void *param,
                               ke_task_id_t const dest_id,
                               ke_task_id_t const src_id)
{ 
	//printf_string("my_timer_handler\n\r");
	static unsigned char timer_flag = 0;	
	
	   if(LEDCnt<5)
			 {
					if(timer_flag)
					{			
						 timer_flag = 0;
						if((M10_Master_conready==0||LEDCnt%2==0)&&BLE_MAC_Edit!=1)  
						{
						 GPIO_SetInactive(LED_PORT, LED_PIN);  //LED
						 GPIO_SetInactive(GPIO_PORT_2, GPIO_PIN_0);  //LED2		
						}
					 
					}
					else
					{
						timer_flag = 1;
						GPIO_SetActive(LED_PORT, LED_PIN);  //LED¬Ã
						GPIO_SetActive(GPIO_PORT_2, GPIO_PIN_0);  //LED2	∫Ï
						LEDCnt++;
					}
					app_timer_set(MY_TIMER, TASK_APP, 20); 
			 }
			else
			 {
						 GPIO_SetInactive(LED_PORT, LED_PIN);  //LED
						 GPIO_SetInactive(GPIO_PORT_2, GPIO_PIN_0);  //LED2	
						 LEDCnt	= 0;	 
				     BLE_MAC_Edit = 0;
				     ke_timer_clear(MY_TIMER, TASK_APP);	
			 }
	 
	
  return 0;
}


#endif  //BLE_APP_PRESENT
/// @} APP
