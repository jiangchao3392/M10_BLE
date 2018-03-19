 /**
 ****************************************************************************************
 *
 * @file streamdatad.c
 *
 * @brief Battery application.
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
 * @addtogroup STREAMDATAD
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
#include "l2cm.h"

#include "streamdatad.h"
#include "streamdatad_task.h"

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
struct streamdatad_env_tag streamdatad_env; // __attribute__((section("retention_mem_area0"),zero_init)); //@RETENTION MEMORY

/// PROXR task descriptor
static const struct ke_task_desc TASK_DESC_STREAMD = {streamdatad_state_handler, &streamdatad_default_handler, streamdatad_state, STREAMDATAD_STATE_MAX, STREAMDATAD_IDX_MAX};
/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */
void streamdatad_init(void)
{
    // Reset the streamdataderometer environment
    memset(&streamdatad_env, 0, sizeof(streamdatad_env));
    
    // Create PROXR task
    ke_task_create(TASK_STREAMDATAD, &TASK_DESC_STREAMD);

    // Go to IDLE state
    ke_state_set(TASK_STREAMDATAD, STREAMDATAD_DISABLED);
}

#endif /* BLE_STREAMDATA_DEVICE */

/// @} STREAMDATAD
