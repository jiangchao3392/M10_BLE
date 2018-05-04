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

³õÊ¼»¯:
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

gapc_connection_req_ind_handler //Ö÷Éè±¸Á¬Èë´¦Àí
app_connection_func
app_connect_confirm
gapm_device_ready_ind_handler


prf_cleanup_func  //Ö÷Éè±¸¶Ï¿ª´¦Àí
gapc_cmp_evt_handler            
app_disconnect_func
app_adv_start
app_adv_func


gattc_write_cmd_ind_handler//

//ÏêÏ¸ 
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

uint8_t test_state __attribute__((section("retention_mem_area0"),zero_init)); //@RETENTION MEMORY

#define MAX_TX_BUFS (32)
#define STREAMDATAD_PACKET_SIZE (20)
uint8 test_pkt [MAX_TX_BUFS][STREAMDATAD_PACKET_SIZE];

#ifdef METRICS
extern struct metrics global_metrics_tosend;
#endif

unsigned char adv_timer_flag = 0;
uint8_t uBatRemainPercent = 0,BatBuf[10],uBatRemainPercentMA=0,adbattery_status=0;
		float AD_value1 = 0.000;
	static	uint8_t MotorTimCnt=0,MotorFlag=0;
extern volatile uint8_t	Power,MotorOn,Key1,Keylong,Keylonglong,KeylongCount,first_count_status,first55_flag,keypress,SetFlag,SwordClose,ResetFlg;
extern uint16_t LEDCount,Adc_vbat,lkeyDownCount,iSecCount,iSecCnt2,SystemCount,count_present,BeSecTmp[2];
extern unsigned char cIptWarning,cFunctionType;
uint8_t LacalSetting[5]={0xAB,0xBC,0xCD,0xD1,0x05};		//"ABBCCDD105" ²éÑ¯±¾µØµØÖ·
extern float Adc_div ;

int adc_timer_handle(ke_msg_id_t const msgid, 
                           void const *param, 
													 ke_task_id_t const dest_id, 
													 ke_task_id_t const src_id)
{
	    uint8_t BatCnt = 0;
	    uint16_t adc_data = adc_get_sample();
	
	
//	   	adc_data	&= 0x3ff;						
//			AD_value1=(adc_data*1.000/1023)*3.6;										
////			uBatRemainPercent = (char)(((AD_value1 - 2.381)/(2.857 - 2.381))*100);    //Õý³£·ÖÑ¹
//      uBatRemainPercent = (char)(((AD_value1 - 2.120)/(2.525 - 2.120))*100);  //47k-100k
//	    uBatRemainPercent = (char)(((adc_data-670.000)/(782.000 - 670.000))*100);  //4.2--3.5  2.830--2.355  4.7k-10k
    	uBatRemainPercent = (char)(((adc_data-602.000)/(739.000 - 602.000))*100);   //4.05--3.26/4.20-3.4  300k--82k  4.2--768/4.05--741/3.26--596/3.31--606   /*   4.05*82/382 * 1023/1.2  */
	                                                                                // rsx501l Ñ¹½µ0.11-0.232  3.5£¨3.39£©-- 620  3.4(3.29)-- 602  4.15(4.04)
//	AD_value1 = adc_data*Adc_div/1000;
//	uBatRemainPercent = (char)(((AD_value1 - 2.350)/(2.825 - 2.350))*100);    			
	
	    BatBuf[BatCnt] =  uBatRemainPercent;
			BatCnt++;		
				
			if(BatCnt>=10)
			{	 
				adbattery_status=1;
				BatCnt = 0;					
								
				uBatRemainPercentMA = fliter(BatBuf,10);			   	

			}
	    if( adbattery_status==0)	   
			   {
					 uBatRemainPercentMA=uBatRemainPercent; //µç³ØµçÁ¿¾ùÖµÂË²¨Î´Íê³ÉÊ±·¢ËÍÊµÊ±Êý¾Ý
			   }
			if(uBatRemainPercentMA>100)
					uBatRemainPercentMA=100;

//		if((adc_data-602.000)< 15.000 && adc_data!=0)	 // 10%  3.48(3.37) 617
			if(uBatRemainPercentMA<=0x0A)
			{
			if((adc_data-602.000)<=0)  //3.4v
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
									 
				if((adc_data-602.000)< 0.000)		//3.4
						{   		
							app_timer_set(MOTOR_TIMER, TASK_APP, 50);
							if(MotorFlag==0)		
								Power = 0;														
						}
			 }
				  
				 
//	    printf_string("BatPercent\n\r");
//	    print_hword(uBatRemainPercentMA);
		  if(MotorFlag == 1)
			ke_timer_clear(ADC_TIMER,TASK_APP);
		else
			app_timer_set(ADC_TIMER, TASK_APP, 50);
//			printf_string("\n\radc_data:");
			print_hword(adc_data);
	
			return 0;
}

/////////´®¿Úhandle  º¯ÊýÊµÌå////////////////////////
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
int motor_handle(ke_msg_id_t const msgid, 
                           void const *param, 
													 ke_task_id_t const dest_id, 
													 ke_task_id_t const src_id)
{
//	printf_string("MOTOR_DEMO\n\r");

	  if(MotorTimCnt==2)
			{
				  GPIO_SetInactive(GPIO_PORT_1, GPIO_PIN_1);
											for(j=0;j<150;j++)
												{
													GPIO_SetActive(GPIO_PORT_1, GPIO_PIN_1);			
													delay_us(10);
													GPIO_SetInactive(GPIO_PORT_1, GPIO_PIN_1);
													delay_us(20);
												}			
						GPIO_SetInactive(GPIO_PORT_0, GPIO_PIN_0);
															
						MotorTimCnt = 0;				
						MotorOn = 0;  		
						ke_timer_clear(MOTOR_TIMER,TASK_APP);		
					GPIO_SetInactive(GPIO_PORT_2, GPIO_PIN_7);   //¹Ø5VµçÔ´ 
						MotorFlag = 0;	
       app_timer_set(ADC_TIMER, TASK_APP, 200);												
			}
		else
		 {
			 MotorFlag = 1; //adc_flag
			 GPIO_SetActive(GPIO_PORT_2, GPIO_PIN_7);	   //¿ª5VµçÔ
		 								GPIO_SetActive(GPIO_PORT_0, GPIO_PIN_0);	
                    GPIO_SetInactive(GPIO_PORT_1, GPIO_PIN_0);			 
										GPIO_SetActive(GPIO_PORT_1, GPIO_PIN_1);			 	

									app_timer_set(MOTOR_TIMER, TASK_APP, 50);	
			 MotorTimCnt++;
		 }				
	return 0;
}

uint8_t ResetCnt=0;
int motor_reset_handle(ke_msg_id_t const msgid, 
                           void const *param, 
													 ke_task_id_t const dest_id, 
													 ke_task_id_t const src_id)
{
	 if(ResetCnt==2)
	 {
			GPIO_SetInactive(GPIO_PORT_1 , GPIO_PIN_0);	
		  GPIO_SetInactive(GPIO_PORT_0, GPIO_PIN_0); //powerctrl
			
			
//			SetBeep();
//			delay_us(12);
//			SetBeep();
//			delay_us(12); 
//			SetBeep();
//			delay_us(12);
//			SetBeep(); 
//			delay_us(12);
//			SetBeep();
//			delay_us(12); 
//			SetBeep();
//			delay_us(12);
//			SetBeep(); 
//			GPIO_SetInactive(LED_PORT, LED_PIN);
			ke_timer_clear(MOTOR_RESET,TASK_APP);	
		 GPIO_SetInactive(GPIO_PORT_2, GPIO_PIN_7);   //¹Ø5VµçÔ´ 
		 MotorFlag = 0; //adc_flag
		 app_timer_set(ADC_TIMER, TASK_APP, 200);
		}
	 else
	  {
			MotorFlag = 1; //adc_flag
			GPIO_SetActive(GPIO_PORT_2, GPIO_PIN_7);	   //¿ª5VµçÔ´
				
			  GPIO_SetActive(GPIO_PORT_0, GPIO_PIN_0);	//powerctrl ON
			  GPIO_SetInactive(GPIO_PORT_1, GPIO_PIN_1);
		  	GPIO_SetActive(GPIO_PORT_1, GPIO_PIN_0);	

			 app_timer_set(MOTOR_RESET, TASK_APP, 50);
			ResetCnt++;
	  }	
   return 0;
//		GPIO_SetInactive(GPIO_PORT_2, GPIO_PIN_7);   //¹Ø5VµçÔ´
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

		 app_timer_set(MOTOR_RESET, TASK_APP, 50);
		
		#if (M10DEBUG)
     printf_string("app_adv_func\n\r");
		#endif
//  Start advertising. Fill GAPM_START_ADVERTISE_CMD message
//    app_timer_set(MY_TIMER, TASK_APP, 100);
	  app_timer_set(ADC_TIMER, TASK_APP, 200);


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
    printf_string("app_disconnect_func\n\r");
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
						if(M10_Master_conready==0||LEDCnt%2==0)
						{
						 GPIO_SetInactive(LED_PORT, LED_PIN);  //LED
						 GPIO_SetInactive(GPIO_PORT_2, GPIO_PIN_0);  //LED2		
						}							
					}
					else
					{
						timer_flag = 1;
				//		GPIO_SetInactive(ADV_LED_PORT, ADV_LED_PIN);
						GPIO_SetActive(LED_PORT, LED_PIN);  //LED
						GPIO_SetActive(GPIO_PORT_2, GPIO_PIN_0);  //LED2	
						LEDCnt++;
					}
					app_timer_set(MY_TIMER, TASK_APP, 20); 
			 }
			else
			 {
						 GPIO_SetInactive(LED_PORT, LED_PIN);  //LED
						 GPIO_SetInactive(GPIO_PORT_2, GPIO_PIN_0);  //LED2	
						 LEDCnt	= 0;	 
				 ke_timer_clear(MY_TIMER, TASK_APP);	
			 }
	 
	
  return 0;
}


#endif  //BLE_APP_PRESENT
/// @} APP
