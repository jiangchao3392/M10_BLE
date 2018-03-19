/**
 ****************************************************************************************
 *
 * @file streamdatah_task.c
 *
 * @brief StreamData Host Task implementation.
 *
 * Copyright (C) RivieraWaves 2009-2012
 *
 * $Rev: 7682 $
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup STREAMDATAHTASK
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwble_config.h"

#if (BLE_STREAMDATA_HOST)

#include "gap.h"
#include "gapc.h"
#include "streamdatah_task.h"
#include "gattc_task.h"
#include "prf_types.h"
#include "prf_utils.h"
#include "streamdatad.h" // STREAMDATAD_SERVICE_UUID

#include "uart_test.h"

/// State machine used to retrieve Device Information Service characteristics information
const struct prf_char_def streamdatah_streamdata_char[STREAMDATAH_CHAR_MAX] =
{
	[STREAMDATAD_ENABLE_CHAR]   = {STREAMDATAD_ENABLE_UUID, ATT_MANDATORY,ATT_CHAR_PROP_RD | ATT_CHAR_PROP_WR_NO_RESP},
    [STREAMDATAH_D0_CHAR]       = {STREAMDATAD_D0_UUID, ATT_MANDATORY,ATT_CHAR_PROP_RD | ATT_CHAR_PROP_WR_NO_RESP | ATT_CHAR_PROP_NTF},
};

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Enable the StreamData Host role, used after connection.
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance
 * @param[in] src_id    ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int streamdatah_enable_req_handler(ke_msg_id_t const msgid,
                                    struct streamdatah_enable_req const *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id)
{
    // Status
    uint8_t status;
    // Device Information Service Client Role Task Environment
    struct streamdatah_env_tag *streamdatah_env;
    // Connection Information
    struct prf_con_info con_info;

    // Fill the Connection Information structure
    con_info.conidx = gapc_get_conidx(param->conhdl);
    con_info.prf_id = dest_id;
    con_info.appid  = src_id;

    // Add an environment for the provided device
    status =  PRF_CLIENT_ENABLE(con_info, param, streamdatah);

    if (status == PRF_ERR_FEATURE_NOT_SUPPORTED)
    {
        // The message has been forwarded to another task id.
        return (KE_MSG_NO_FREE);
    }
    else if (status == PRF_ERR_OK)
    {
        streamdatah_env = PRF_CLIENT_GET_ENV(dest_id, streamdatah);
		
		streamdatah_env->streamon = 0;
		streamdatah_env->next_attribute_idx = 0;
		streamdatah_env->receive_sequence_number = 0;

        // Discovery connection
        if (param->con_type == PRF_CON_DISCOVERY)
        {
            // Start discovering LLS on peer
            streamdatah_env->last_uuid_req     = STREAMDATAD_SERVICE_UUID;
            streamdatah_env->last_svc_req      = STREAMDATAD_SERVICE_UUID;
            prf_disc_svc_send(&streamdatah_env->con_info, streamdatah_env->last_svc_req);

            // Set state to discovering
            ke_state_set(dest_id, STREAMDATAH_DISCOVERING);
        }
        else
        {
            //copy over data that has been stored
            streamdatah_env->streamdatah     = param->streamdatah;

            //send confirmation of enable request to application
            streamdatah_enable_cfm_send(streamdatah_env, &con_info, PRF_ERR_OK);
        }
    }
    else
    {
        streamdatah_enable_cfm_send(NULL, &con_info, status);
    }

    // message is consumed
    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref GATT_DISC_SVC_BY_UUID_CMP_EVT message.
 * The handler disables the Health Thermometer Profile Collector Role.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gatt_disc_svc_by_uuid_evt_handler(ke_msg_id_t const msgid,
                                             struct gattc_disc_svc_ind const *param,
                                             ke_task_id_t const dest_id,
                                             ke_task_id_t const src_id)
{
    // Get the address of the environment
    struct streamdatah_env_tag *streamdatah_env = PRF_CLIENT_GET_ENV(dest_id, streamdatah);

    // Even if we get multiple responses we only store 1 range
    if (streamdatah_env->last_uuid_req == STREAMDATAD_SERVICE_UUID)
    {
        streamdatah_env->streamdatah.svc.shdl = param->start_hdl;
        streamdatah_env->streamdatah.svc.ehdl = param->end_hdl;
        streamdatah_env->nb_streamdatrad_svc++;
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref GATT_DISC_CHAR_ALL_CMP_EVT message.
 * Characteristics for the currently desired service handle range are obtained and kept.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gatt_disc_char_all_evt_handler(ke_msg_id_t const msgid,
                                          struct gattc_disc_char_ind const *param,
                                          ke_task_id_t const dest_id,
                                          ke_task_id_t const src_id)
{
    // Get the address of the environment
    struct streamdatah_env_tag *streamdatah_env = PRF_CLIENT_GET_ENV(dest_id, streamdatah);

    if(streamdatah_env->last_svc_req == STREAMDATAD_SERVICE_UUID)
    {
        //
        prf_search_chars(streamdatah_env->streamdatah.svc.ehdl, STREAMDATAH_CHAR_MAX,
                         &streamdatah_env->streamdatah.chars[0], &streamdatah_streamdata_char[0],
                         param, &streamdatah_env->last_char_code);
        
        //set_pxact_gpio();
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref GATT_CMP_EVT message.
 * This generic event is received for different requests, so need to keep track.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gatt_cmp_evt_handler(ke_msg_id_t const msgid,
                                struct gattc_cmp_evt const *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id)
{
    
    uint8_t state = ke_state_get(dest_id);
    uint8_t status = 0;
    // Get the address of the environment
    struct streamdatah_env_tag *streamdatah_env = PRF_CLIENT_GET_ENV(dest_id, streamdatah);

    if (state == STREAMDATAH_DISCOVERING)
    {
    
        if ((param->status == ATT_ERR_ATTRIBUTE_NOT_FOUND)||
            (param->status == ATT_ERR_NO_ERROR))
        {
            //Currently discovering Link Loss Service
            if (streamdatah_env->last_svc_req == STREAMDATAD_SERVICE_UUID)
            {
                if (streamdatah_env->last_uuid_req == STREAMDATAD_SERVICE_UUID)
                {
                    if (streamdatah_env->streamdatah.svc.shdl == ATT_INVALID_HANDLE)
                    {
                        streamdatah_enable_cfm_send(streamdatah_env, &streamdatah_env->con_info, PRF_ERR_STOP_DISC_CHAR_MISSING);
                    }
                    // Too many services found only one such service should exist
                    else if(streamdatah_env->nb_streamdatrad_svc > 1)
                    {
                        // stop discovery procedure.
                        streamdatah_enable_cfm_send(streamdatah_env, &streamdatah_env->con_info, PRF_ERR_MULTIPLE_SVC);
                    }
                    else
                    {
                        // Discover StreamData Device characteristics
                        prf_disc_char_all_send(&(streamdatah_env->con_info), &(streamdatah_env->streamdatah.svc));

                        // Keep last UUID requested and for which service in env
                        streamdatah_env->last_uuid_req = ATT_DECL_CHARACTERISTIC;
                    }
                }
                else if (streamdatah_env->last_uuid_req == ATT_DECL_CHARACTERISTIC)
                {
                    status = prf_check_svc_char_validity(STREAMDATAH_CHAR_MAX, streamdatah_env->streamdatah.chars,
                                                                               streamdatah_streamdata_char);

                    if (status == PRF_ERR_OK)
                    {
                        streamdatah_env->nb_streamdatrad_svc = 0;
                    }

                    streamdatah_enable_cfm_send(streamdatah_env, &streamdatah_env->con_info, status);
                }
            }
        }
    }
    else if (state == STREAMDATAH_CONNECTED)
    {
        switch(param->req_type)
        {
            case GATTC_WRITE:
            {
                //proxm_write_char_rsp_send(proxm_env, param->status);
                streamdatah_write_char_rsp_send(streamdatah_env, param->status);
            }
            break;

            case GATTC_READ:
            {
                if(param->status != GATT_ERR_NO_ERROR)
                {
                    // an error occurs while reading peer device attribute
                     prf_client_att_info_rsp((prf_env_struct*) streamdatah_env, STREAMDATAH_RD_CHAR_RSP,
                                    param->status, NULL);
                }
            }
            break;
            default: break;
        }
    }
    
    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Disconnection indication to StreamData Host Profile
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance
 * @param[in] src_id    ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gap_discon_cmp_evt_handler(ke_msg_id_t const msgid,
                                        struct gapc_disconnect_ind const *param,
                                        ke_task_id_t const dest_id,
                                        ke_task_id_t const src_id)
{
    PRF_CLIENT_DISABLE_IND_SEND(streamdatah_envs, dest_id, STREAMDATAH, param->conhdl);
        
    // message is consumed
    return (KE_MSG_CONSUMED);
}



/**
 ****************************************************************************************
 * @brief Handles reception of the @ref STREAMDATAH_STREAMON_REQ message.
 * Turns on the stream by writing a nonzero value to the STREAMDATAD_ENABLE attribute.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int streamdatah_rd_streamon_enable_req_handler(ke_msg_id_t const msgid,
                                    struct streamdatah_rd_streamon_enable_req const *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id)
{
    // Get the address of the environment
    struct streamdatah_env_tag *streamdatah_env = PRF_CLIENT_GET_ENV(dest_id, streamdatah);
	
	prf_read_char_send(&streamdatah_env->con_info,
					   streamdatah_env->streamdatah.svc.shdl, streamdatah_env->streamdatah.svc.ehdl,
					   streamdatah_env->streamdatah.chars[STREAMDATAD_ENABLE_CHAR].val_hdl);
    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref GATT_HANDLE_VALUE_NTF message.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gattc_event_ind_handler(ke_msg_id_t const msgid,
                                         struct gattc_event_ind const *param,
                                         ke_task_id_t const dest_id,
                                         ke_task_id_t const src_id)
{      
    unsigned char value[20],i;   
    struct streamdatah_env_tag *streamdatah_env = PRF_CLIENT_GET_ENV(dest_id, streamdatah);

    if (KE_IDX_GET(src_id) == streamdatah_env->con_info.conidx)
    {
				memcpy(value, &(param->value[0]), param->length);
				for(i=0;i<20;i++)
				{
				  uart_send_byte(value[i]);
				}
    }
    return (KE_MSG_CONSUMED);
}



/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

// CONNECTED State handlers definition.
const struct ke_msg_handler streamdatah_connected[] =
{
	{STREAMDATAH_RD_STREAM_ENABLE_REQ, (ke_msg_func_t)streamdatah_rd_streamon_enable_req_handler},
    {GATTC_EVENT_IND,               (ke_msg_func_t)gattc_event_ind_handler}
};

/// Specifies the Discovering state message handlers
const struct ke_msg_handler streamdatah_discovering[] =
{
    {GATTC_DISC_SVC_IND,            (ke_msg_func_t)gatt_disc_svc_by_uuid_evt_handler},
    {GATTC_DISC_CHAR_IND,           (ke_msg_func_t)gatt_disc_char_all_evt_handler},
    {GATTC_CMP_EVT,                  (ke_msg_func_t)gatt_cmp_evt_handler},
};

/// Default State handlers definition
const struct ke_msg_handler streamdatah_default_state[] =
{
    {STREAMDATAH_ENABLE_REQ,            (ke_msg_func_t)streamdatah_enable_req_handler},
    {GAPC_DISCONNECT_IND,               (ke_msg_func_t)gap_discon_cmp_evt_handler},
};

// Specifies the message handler structure for every input state.
const struct ke_state_handler streamdatah_state_handler[STREAMDATAH_STATE_MAX] =
{
    [STREAMDATAH_IDLE]        = KE_STATE_HANDLER_NONE,
    [STREAMDATAH_CONNECTED]   = KE_STATE_HANDLER(streamdatah_connected),
    [STREAMDATAH_DISCOVERING] = KE_STATE_HANDLER(streamdatah_discovering),
};

// Specifies the message handlers that are common to all states.
const struct ke_state_handler streamdatah_default_handler = KE_STATE_HANDLER(streamdatah_default_state);

// Defines the place holder for the states of all the task instances.
ke_state_t streamdatah_state[STREAMDATAH_IDX_MAX] __attribute__((section("retention_mem_area0"),zero_init)); //@RETENTION MEMORY

#endif //BLE_STREAMDATA_HOST

/// @} STREAMDATAHTASK
