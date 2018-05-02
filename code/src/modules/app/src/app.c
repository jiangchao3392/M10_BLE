/**
 ****************************************************************************************
 *
 * @file app.c
 *
 * @brief Application entry point
 *
 * Copyright (C) RivieraWaves 2009-2013
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup APP
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"             // SW configuration

#if (BLE_APP_PRESENT)
#include "app_task.h"                // Application task Definition
#include "app.h"                     // Application Definition
#include "gapm_task.h"               // GAP Manager Task API
#include "gapc_task.h"               // GAP Controller Task API
#include "co_math.h"                 // Common Maths Definition
#include "app_api.h"                // Application task Definition
#include "uart.h"
#if (BLE_APP_SEC)
#include "app_sec.h"                 // Application security Definition
#endif // (BLE_APP_SEC)

#if (NVDS_SUPPORT)
#include "nvds.h"                    // NVDS Definitions
#endif //(NVDS_SUPPORT)
///////////////
uint8_t M10_Master_conready=0;
/*
 * ENUMERATIONS
 ****************************************************************************************
 */


/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/// Application Environment Structure
struct app_env_tag app_env __attribute__((section("retention_mem_area0"),zero_init)); //@RETENTION MEMORY

/*
 * LOCAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/// Application Task Descriptor
static const struct ke_task_desc TASK_DESC_APP = {NULL, &app_default_handler,
                                                  app_state, APP_STATE_MAX, APP_IDX_MAX};

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Application task initialization.
 *
 * @return void
 ****************************************************************************************
 */

void app_init(void)
{ 
    #if (NVDS_SUPPORT)
    uint8_t length = NVDS_LEN_SECURITY_ENABLE;
    #endif // NVDS_SUPPORT

    // Reset the environment
    memset(&app_env, 0, sizeof(app_env));

    // Initialize next_prf_init value for first service to add in the database
    app_env.next_prf_init = APP_PRF_LIST_START + 1;

    #if (NVDS_SUPPORT)
    // Get the security enable from the storage
    if (nvds_get(NVDS_TAG_SECURITY_ENABLE, &length, (uint8_t *)&app_env.sec_en) != NVDS_OK)
    #endif // NVDS_SUPPORT
    {
        // Set true by default (several profiles requires security)
        app_env.sec_en = true;
    }

	app_init_func();    
		
    // Create APP task
    ke_task_create(TASK_APP, &TASK_DESC_APP);

    // Initialize Task state
    ke_state_set(TASK_APP, APP_DISABLED);

    #if (BLE_APP_SEC)
    app_sec_init();
    #endif // (BLE_APP_SEC)
}

/**
 ****************************************************************************************
 * @brief Profiles's Database initialization sequence.
 *
 * @return void
 ****************************************************************************************
 */

bool app_db_init(void)
{
    
    // Indicate if more services need to be added in the database
    bool end_db_create = false;
    
    end_db_create = app_db_init_func();
        
    return end_db_create;
}

/**
 ****************************************************************************************
 * @brief Send BLE disconnect command
 *
 * @return void
 ****************************************************************************************
 */

void app_disconnect(void)
{ 
		#if (M10DEBUG)
	printf_string("app_disconnect\n\r");
			#endif
    struct gapc_disconnect_cmd *cmd = KE_MSG_ALLOC(GAPC_DISCONNECT_CMD,
                                              KE_BUILD_ID(TASK_GAPC, app_env.conidx), TASK_APP,
                                              gapc_disconnect_cmd);

    cmd->operation = GAPC_DISCONNECT;
    cmd->reason = CO_ERROR_REMOTE_USER_TERM_CON;

    // Send the message
    ke_msg_send(cmd);
}

/**
 ****************************************************************************************
 * @brief Sends a connection confirmation message
 *
 * @return void
 ****************************************************************************************
 */

void app_connect_confirm(uint8_t auth)
{  
	#if (M10DEBUG)
  printf_string("app_connect_confirm\n\r");
		#endif
		
    // confirm connection
    struct gapc_connection_cfm *cfm = KE_MSG_ALLOC(GAPC_CONNECTION_CFM,
            KE_BUILD_ID(TASK_GAPC, app_env.conidx), TASK_APP,
            gapc_connection_cfm);

    cfm->auth = auth;
    cfm->authorize = GAP_AUTHZ_NOT_SET;
    // Send the message
    ke_msg_send(cfm);
		printf_string("\n\r BLE start connect lian jie\n");
		M10_Master_conready=1;		
		uart_send_byte(0x4F);
    uart_send_byte(0x4B); //OK ASCII
	  uart_send_byte(0xFC); //���ؿ���ָ��

		app_timer_set(MY_TIMER, TASK_APP, 20);  //���豸����ָʾ��
}


/**
 ****************************************************************************************
 * Advertising Functions
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Start Advertising. Setup Advertsise and Scan Response Message
 *
 * @return void
 ****************************************************************************************
 */

void app_adv_start(void)
{
		#if (M10DEBUG)
   printf_string("app_adv_start\n\r");
		#endif
    // Allocate a message for GAP
    struct gapm_start_advertise_cmd *cmd = KE_MSG_ALLOC(GAPM_START_ADVERTISE_CMD,
                                                TASK_GAPM, TASK_APP,
                                                gapm_start_advertise_cmd);
  

    app_adv_func(cmd);
    
    // Send the message
    ke_msg_send(cmd);

    // We are now connectable
    ke_state_set(TASK_APP, APP_CONNECTABLE);
}

/**
 ****************************************************************************************
 * @brief Stop Advertising
 *
 * @return void
 ****************************************************************************************
 */

void app_adv_stop(void)
{  
	#if (M10DEBUG)
  printf_string("app_adv_stop\n\r");
		#endif
    // Disable Advertising
    struct gapm_cancel_cmd *cmd = KE_MSG_ALLOC(GAPM_CANCEL_CMD,
                                           TASK_GAPM, TASK_APP,
                                           gapm_cancel_cmd);

    cmd->operation = GAPM_CANCEL;

    // Send the message
    ke_msg_send(cmd);
}

/**
 ****************************************************************************************
 * @brief Send a connection param update request message
 *
 * @return void
 ****************************************************************************************
 */

void app_param_update_start(void)
{
    app_param_update_func();
}


/**
 ****************************************************************************************
 * @brief Start a kernel timer
 *
 * @return void
 ****************************************************************************************
 */

void app_timer_set(ke_msg_id_t const timer_id, ke_task_id_t const task_id, uint16_t delay)
{
    // Delay shall not be more than maximum allowed
    if(delay > KE_TIMER_DELAY_MAX)
    {
        delay = KE_TIMER_DELAY_MAX;

    }
    // Delay should not be zero
    else if(delay == 0)
    {
        delay = 1;
    }
    
    ke_timer_set(timer_id, task_id, delay);
}

/*
 ****************************************************************************************
 * @brief Send a connection request command. Used in central role only.
 *
 * @param[in] peer_addr_type        Peer to connect address type
 * @param[in] peer_addr             Peer to connect address 
 *
 * @return void
 ****************************************************************************************
*/

void app_connect(uint8_t peer_addr_type, uint8_t *peer_addr, uint16_t intv)
{  
	#if (M10DEBUG)
  printf_string("app_connect\n\r");
		#endif
    struct gapm_start_connection_cmd *msg;
   
    msg = (struct gapm_start_connection_cmd *) KE_MSG_ALLOC_DYN(GAPM_START_CONNECTION_CMD , TASK_GAPM, TASK_APP, 
                                                        gapm_start_connection_cmd, sizeof(struct gap_bdaddr));
    
    msg->nb_peers = 1;
    memcpy((void *) &msg->peers[0].addr, (void *)peer_addr, BD_ADDR_LEN);
    msg->peers[0].addr_type = peer_addr_type;
    msg->con_intv_max = intv;
    msg->con_intv_min = intv; 
    
    app_connect_func(msg);
            
    ke_msg_send((void *) msg);
       
}

#endif //(BLE_APP_PRESENT)

/// @} APP
