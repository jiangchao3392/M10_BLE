/**
 ****************************************************************************************
 *
 * @file streamdatad_task.c
 *
 * @brief StreamData Device profile task
 *
 * Copyright (C) RivieraWaves 2009-2012
 *
 * $Rev: $
 *
 ****************************************************************************************
 */
 
 /**
 ****************************************************************************************
 *
 * @file streamdatad_task.c
 *
 * @brief StreamData Device profile task.
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

/**
 ****************************************************************************************
 * @addtogroup STREAMDATADTASK
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwble_config.h"

#if (BLE_STREAMDATA_DEVICE)

#include "gap.h"
#include "gattc_task.h"
#include "attm_util.h"
#include "atts_util.h"
#include "attm_cfg.h"
#include "prf_utils.h"

#include "streamdatad_task.h"
#include "streamdatad.h"
#include "llc_task.h" // llc_nb_of_pkt_evt_complete
#include "uart.h"
#include "periph_setup.h"
#include "gpio.h"
#include "spi_flash.h"
#include "app_api.h" 

/// Pong Service
static const att_svc_desc_t streamdatad_svc = STREAMDATAD_SERVICE_UUID;
/// Enable Characteristic

static const struct att_char_desc streamdatad_enable_char = ATT_CHAR(ATT_CHAR_PROP_RD | ATT_CHAR_PROP_WR_NO_RESP, 0, STREAMDATAD_ENABLE_UUID);

/// Data Characteristic
static const struct att_char_desc streamdatad_d0_char = ATT_CHAR(ATT_CHAR_PROP_RD | ATT_CHAR_PROP_WR_NO_RESP | ATT_CHAR_PROP_NTF, 0, STREAMDATAD_D0_UUID);

/// Enable description
static const uint8_t streamdatad_enable_desc[] = STREAMDATAD_ENABLE_DESC;

/// Data description
static const uint8_t streamdatad_d_desc[] = STREAMDATAD_D_DESC;
                                       
                                       
/// Full STREAMDATAD Database Description - Used to add attributes into the database
static const struct attm_desc streamdatad_att_db[STREAMDATAD_IDX_NB] =
{
    [STREAMDATAD_IDX_PRIM_SVC]   =   /* StreamData Device service */
                               {ATT_DECL_PRIMARY_SERVICE, PERM(RD, ENABLE),
                                       sizeof(streamdatad_svc), sizeof(streamdatad_svc),
                                       (uint8_t*) &streamdatad_svc},
									   
    [STREAMDATAD_IDX_ENABLE_CHAR]  =        {ATT_DECL_CHARACTERISTIC, PERM(RD, ENABLE), sizeof(streamdatad_enable_char), sizeof(streamdatad_enable_char), (uint8_t*) &streamdatad_enable_char},
    [STREAMDATAD_IDX_ENABLE_VAL] =          {STREAMDATAD_ENABLE_UUID,  (PERM(RD, ENABLE) | PERM(WR, ENABLE) ), 20, 0, (uint8_t*) NULL},
    [STREAMDATAD_IDX_ENABLE_EN] =           {ATT_DESC_CLIENT_CHAR_CFG,(PERM(RD, ENABLE) | PERM(WR, ENABLE) | PERM(NTF, ENABLE)), sizeof(uint16_t), 0, (uint8_t*) NULL},
    [STREAMDATAD_IDX_ENABLE_DESC] =         {ATT_DESC_CHAR_USER_DESCRIPTION,  PERM(RD, ENABLE), STREAMDATAD_ENABLE_DESC_LEN, STREAMDATAD_ENABLE_DESC_LEN, (uint8_t*) streamdatad_enable_desc},
									   
    [STREAMDATAD_IDX_STREAMDATAD_D0_CHAR] = {ATT_DECL_CHARACTERISTIC, PERM(RD, ENABLE), sizeof(streamdatad_d0_char), sizeof(streamdatad_d0_char), (uint8_t*) &streamdatad_d0_char},
    [STREAMDATAD_IDX_STREAMDATAD_D0_VAL] =  {STREAMDATAD_D0_UUID, PERM(RD, ENABLE) | PERM(WR, ENABLE) | PERM(NTF, ENABLE), 20, 0, (uint8_t*) NULL},
    [STREAMDATAD_IDX_STREAMDATAD_D0_EN] =   {ATT_DESC_CLIENT_CHAR_CFG,(PERM(RD, ENABLE) | PERM(WR, ENABLE) | PERM(NTF, ENABLE)), sizeof(uint16_t), 0, (uint8_t*) NULL},
    [STREAMDATAD_IDX_STREAMDATAD_D0_DESC] = {ATT_DESC_CHAR_USER_DESCRIPTION, PERM(RD, ENABLE), STREAMDATAD_D_DESC_LEN, STREAMDATAD_D_DESC_LEN, (uint8_t*) streamdatad_d_desc},
};


/**
 ****************************************************************************************
 * @brief Handles reception of the @ref STREAMDATAD_CREATE_DB_REQ message.
 * The handler adds STREAMDATAD Service into the database using the database
 * configuration value given in param.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int streamdatad_create_db_req_handler(ke_msg_id_t const msgid,
                                      struct streamdatad_create_db_req const *param,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id)
{  
		#if (M10DEBUG)
	printf_string("streamdatad_create_db_req_handler\n\r");
		#endif
    //Service Configuration Flag
    uint32_t cfg_flag = 0xFFFFFFFF;
    //Database Creation Status
    uint8_t status;

    //Save Application ID
    streamdatad_env.appid = src_id;


    // set start handle or automatically set it when creating database (start_hdl = 0)
    streamdatad_env.shdl=param->start_hdl;

    //Add Service Into Database
    status = attm_svc_create_db(&streamdatad_env.shdl, (uint8_t *)&cfg_flag,  STREAMDATAD_IDX_NB, NULL,
                               dest_id, &streamdatad_att_db[0]);

    //Disable GLS
    attmdb_svc_set_permission(streamdatad_env.shdl, PERM(SVC, DISABLE));

    //Go to Idle State
    if (status == ATT_ERR_NO_ERROR)
    {
        //If we are here, database has been fulfilled with success, go to idle test
        ke_state_set(TASK_STREAMDATAD, STREAMDATAD_IDLE);
    }

    //Send response to application
    struct streamdatad_create_db_cfm * cfm = KE_MSG_ALLOC(STREAMDATAD_CREATE_DB_CFM, streamdatad_env.appid,
                                                    TASK_STREAMDATAD, streamdatad_create_db_cfm);
    cfm->status = status;
    ke_msg_send(cfm);

    return (KE_MSG_CONSUMED);
}


/*
 * MESSAGE HANDLERS
 ****************************************************************************************
 */
/**
 ****************************************************************************************
 * @brief Handles reception of the @ref STREAMDATAD_ENABLE_REQ message.
 * The handler enables the StreamData Device profile.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int streamdatad_enable_req_handler(ke_msg_id_t const msgid,
                                    struct streamdatad_enable_req const *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id)
{  
	#if (M10DEBUG)
	printf_string("int streamdatad_enable_req_handler\n\r");
			#endif
    uint16_t disable_val = 0x00;

    //Save Application ID
    //streamdatad_env.con_info.appid = src_id;
    streamdatad_env.con_info.prf_id = dest_id;
    streamdatad_env.con_info.appid = src_id;
    
    streamdatad_env.appid = src_id;
    //streamdatad_env.con_info.prf_id = dest_id;
    
    // Save the connection handle associated to the profile
    streamdatad_env.con_info.conidx = gapc_get_conidx(param->conhdl);
    // Save the connection handle associated to the profile
    streamdatad_env.conhdl = param->conhdl;

	streamdatad_env.next_attribute_idx = 0;
	streamdatad_env.nr_enabled_attributes = 0;
	streamdatad_env.stream_enabled = 0;

    
    attmdb_att_set_value(STREAMDATAD_HANDLE(STREAMDATAD_IDX_ENABLE_VAL), sizeof(uint16_t), (uint8_t*) &(disable_val));

    attmdb_att_set_value(STREAMDATAD_HANDLE(STREAMDATAD_IDX_STREAMDATAD_D0_EN), sizeof(uint16_t),(uint8_t*) &(disable_val));
	
	//Enable Service
	attmdb_svc_set_permission(streamdatad_env.shdl, PERM(SVC, ENABLE));
    // Go to active state
    ke_state_set(TASK_STREAMDATAD, STREAMDATAD_ACTIVE);

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref STREAMDATAD_DISABLE_REQ message.
 * The handler disables the streamdataderometer profile.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int streamdatad_disable_req_handler(ke_msg_id_t const msgid,
                                     void const *param,
                                     ke_task_id_t const dest_id,
                                     ke_task_id_t const src_id)
{ 
 	#if (M10DEBUG)	
	printf_string("streamdatad_disable_req_handler\n\r");
	#endif		
    // Go to idle state
    ke_state_set(TASK_STREAMDATAD, STREAMDATAD_IDLE);

    return (KE_MSG_CONSUMED);
}


/**
 ****************************************************************************************
 * @brief Handles reception of the @ref GATT_WRITE_CMD_IND message.
 * The handler checks if the stream needs to be turned on.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */

uint8_t periph_addr_storage[6],i,BLE_MAC_Edit=0;//periph_addr_type_storage
static int gattc_write_cmd_ind_handler(ke_msg_id_t const msgid,
                                      struct gattc_write_cmd_ind const *param,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id)
{   
		#if (M10DEBUG)
	printf_string("gattc_write_cmd_ind_handler\n\r");
			#endif
    // Update the attribute value
	  unsigned char value[20] = {0};//,rx_cnt=0rxdata[6],btrd=0
    attmdb_att_update_value(param->handle, param->length, param->offset,
            (uint8_t*)&(param->value[0]));

    switch (STREAMDATAD_IDX(param->handle))
    {
        case STREAMDATAD_IDX_ENABLE_VAL:
			atts_write_rsp_send(streamdatad_env.conhdl, param->handle, PRF_ERR_OK);
				memcpy(value, &(param->value[0]), param->length);
				//if(value[0] == '1')¡
//				    printf_byte(value[0]);
//            printf_byte(value[1]);
//				    printf_byte(value[2]);

//				if(value[rx_cnt-1]==0x0A&&value[rx_cnt-2]==0x0D)
				
//					{				
					//////////////  ///////////////////////
				    if( value[0]==0xAA)   
							{
//								printf_string("Head Correct\n\r"); 				
//							 if(value[1]==0xAA)   
//							 {printf_string("0k_scan\n\r");  
//							 app_start_scanning(); 
//							 }
							 
							 if(value[1]==0xBB)   
							 {
//								 GPIO_SetActive(LED_PORT, LED_PIN);
//								periph_addr_type_storage=value[2];
								//////////////////////////////////////////
								periph_addr_storage[5]= value[2];
								periph_addr_storage[4]= value[3];
								periph_addr_storage[3]= value[4];
								periph_addr_storage[2]= value[5];
								periph_addr_storage[1]= value[6];
								periph_addr_storage[0]= value[7];
								//CE-BB-00-04:00:00:CA:EA:80-0D0A/
								 //CEBB0080EACA0000040D0A
							GPIO_ConfigurePin( SPI_PORT, SPI_PIN_CLK, OUTPUT, PID_SPI_CLK, false );
	          RESERVE_GPIO( SPI_CLK, SPI_PORT, SPI_PIN_CLK, PID_SPI_CLK);								 
								 
								 ///////////////////////////////////////
								 spi_flash_block_erase(0xf000,SECTOR_ERASE);
								 
								 	spi_flash_page_program(periph_addr_storage, 0xf000, 6);
								 BLE_MAC_Edit = 1;
                 app_timer_set(MY_TIMER, TASK_APP, 20); 
								 printf_string("storage_ok\n\r");
//								 btrd = spi_flash_read_data(rxdata,0x7000,6);								 			         							  
//								 for(i=0;i<6;i++)
//								  { 
//										uart_send_byte(rxdata[i]);
//										uart_send_byte(0xee);
//									}
									  GPIO_ConfigurePin(GPIO_PORT_0, GPIO_PIN_0, OUTPUT, PID_GPIO, false); 	
           	RESERVE_GPIO(GPIO, GPIO_PORT_0, GPIO_PIN_0, PID_GPIO);
//								 app_connect(periph_addr_type_storage, periph_addr_storage, 80); 
//								GPIO_SetInactive(LED_PORT, LED_PIN);

							 }
							}
//									for(i=0;i<param->length;i++)
//											{
//												uart_send_byte(value[i]);
//											}
//					 }
				// app_timer_set(GAPC_DISCONNECT_IND,TASK_APP,1); //¹Ø±ÕÁ´½Óµ÷ÊÔ
				

				
			break;

         case STREAMDATAD_IDX_STREAMDATAD_D0_EN:

			atts_write_rsp_send(streamdatad_env.conhdl, param->handle, PRF_ERR_OK);
            break;
        
        case STREAMDATAD_IDX_STREAMDATAD_D0_VAL:
        break;
        
    }

    return (KE_MSG_CONSUMED);
}



/*
 * TASK DESCRIPTOR DEFINITIONS
 ****************************************************************************************
 */
/// Disabled State handler definition.
const struct ke_msg_handler streamdatad_disabled[] =
{
    {STREAMDATAD_CREATE_DB_REQ,   (ke_msg_func_t) streamdatad_create_db_req_handler}
};


/// IDLE State handlers definition
const struct ke_msg_handler streamdatad_idle[] =
{
    {STREAMDATAD_ENABLE_REQ,     (ke_msg_func_t)streamdatad_enable_req_handler}
};

/// ACTIVE State handlers definition
const struct ke_msg_handler streamdatad_active[] =
{
    {STREAMDATAD_DISABLE_REQ, (ke_msg_func_t)streamdatad_disable_req_handler},
    {GATTC_WRITE_CMD_IND, (ke_msg_func_t)gattc_write_cmd_ind_handler},
//    {GATTC_CMP_EVT,       (ke_msg_func_t)gattc_cmp_evt_handler},
}; 

/// Specifies the message handler structure for every input state
const struct ke_state_handler streamdatad_state_handler[STREAMDATAD_STATE_MAX] =
{
    /// DISABLE State message handlers.
    [STREAMDATAD_DISABLED]  = KE_STATE_HANDLER(streamdatad_disabled),
    /// IDLE State message handlers.
    [STREAMDATAD_IDLE]      = KE_STATE_HANDLER(streamdatad_idle),
    /// ACTIVE State message handlers.
    [STREAMDATAD_ACTIVE]    = KE_STATE_HANDLER(streamdatad_active),

};

/// Specifies the message handlers that are common to all states.
const struct ke_state_handler streamdatad_default_handler = KE_STATE_HANDLER_NONE;

/// Defines the placeholder for the states of all the task instances.
ke_state_t streamdatad_state[STREAMDATAD_IDX_MAX]; // __attribute__((section("retention_mem_area0"),zero_init)); //@RETENTION MEMORY

#endif /* BLE_STREAMDATA_DEVICE */
/// @} STREAMDATADTASK
