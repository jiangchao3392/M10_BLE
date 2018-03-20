/**
 ****************************************************************************************
 *
 * @file arch_main.c
 *
 * @brief Main loop of the application.
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
 * INCLUDES
 ****************************************************************************************
 */
#include "da14580_scatter_config.h"
#include "arch.h"
#include "arch_sleep.h"
#include <stdlib.h>
#include <stddef.h>     // standard definitions
#include <stdint.h>     // standard integer definition
#include <stdbool.h>    // boolean definition
#include "boot.h"       // boot definition
#include "rwip.h"       // BLE initialization
#include "syscntl.h"    // System control initialization
#include "emi.h"        // EMI initialization
#include "intc.h"       // Interrupt initialization
#include "timer.h"      // TIMER initialization
#include "em_map_ble.h"
#include "ke_mem.h"
#include "ke_event.h"
#include "smpc.h"
#include "llc.h"
#include "pll_vcocal_lut.h"
#include "periph_setup.h"
#include "spi_flash.h"
#include "spi.h"
#include "attm_db.h"
#include "prf_utils.h"
#include "streamdatad.h"
#include "app.h"
#include "app_throughput_peripheral_proj.h"

#if PLF_UART
#include "uart.h"       // UART initialization
#endif //PLF_UART

#include "nvds.h"       // NVDS initialization

#if (BLE_EMB_PRESENT)
#include "rf.h"         // RF initialization
#endif // BLE_EMB_PRESENT

#if (BLE_HOST_PRESENT)
#include "rwble_hl.h"   // BLE HL definitions
#include "gapc.h"
#include "smpc.h"
#include "gattc.h"
#include "attc.h"
#include "atts.h"
#include "l2cc.h"
#endif //BLE_HOST_PRESENT

#if (BLE_APP_PRESENT)
#include "app.h"       // application functions
#include "app_sleep.h"
#endif // BLE_APP_PRESENT

#include "gtl_env.h"
#include "gtl_task.h"

#if PLF_DEBUG
#include "dbg.h"       // For dbg_warning function
#endif //PLF_DEBUG

#include "global_io.h"

#include "datasheet.h"

#include "em_map_ble_user.h"
#include "em_map_ble.h"

#include "lld_sleep.h"
#include "rwble.h"
#include "rf_580.h"
#include "gpio.h"
#include "gtl_task.h"
#include "nvds.h"        // nvds definitions



#ifdef CFG_PRINTF
#include "app_console.h"
#endif

#if (STREAMDATA_QUEUE)
#include "app_stream_queue.h"
#endif

/**
 * @addtogroup DRIVERS
 * @{
 */

/*
 * DEFINES
 ****************************************************************************************
 */
/// NVDS location in FLASH : 0x000E0000 (896KB (1Mo - 128KB))
#ifndef __DA14581__
#define NVDS_FLASH_ADDRESS          (0x00000340)
#else
#define NVDS_FLASH_ADDRESS          (0x00000350)
#endif

/// NVDS size in RAM : 0x00010000 (128KB)
#define NVDS_FLASH_SIZE             (0x00000100)

#if DEVELOPMENT_DEBUG
    #warning "==============================================================> DEVELOPMENT_DEBUG is set!"
#endif		


extern int l2cc_pdu_recv_ind_handler(ke_msg_id_t const msgid, struct l2cc_pdu_recv_ind *param,
                                        ke_task_id_t const dest_id, ke_task_id_t const src_id);

extern int smpc_pairing_cfm_handler(ke_msg_id_t const msgid,
                                    struct smpc_pairing_cfm *param,
                                    ke_task_id_t const dest_id, ke_task_id_t const src_id);
 
extern void my_llc_con_update_req_ind(uint16_t conhdl, struct llcp_con_up_req const *param);
                                    
extern void my_llc_ch_map_req_ind (uint16_t conhdl, struct llcp_channel_map_req const *param);

struct gapm_start_advertise_cmd;
extern uint8_t patched_gapm_adv_op_sanity(struct gapm_start_advertise_cmd *adv);
																																																				

/*
 * STRUCTURE DEFINITIONS
 ****************************************************************************************
 */

/// Description of unloaded RAM area content
struct unloaded_area_tag
{
    uint32_t error;
};

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
#ifdef __DA14581__
uint32_t error;              /// Variable storing the reason of platform reset
#endif

#if 0   //PLF_DEBUG
volatile int dbg_assert_block = 1;  /// Variable to enable infinite loop on assert
#endif //PLF_DEBUG

/// Pointer to access unloaded RAM area
extern struct unloaded_area_tag* unloaded_area;

extern uint32_t error;              /// Variable storing the reason of platform reset

extern uint32_t last_temp_time;     /// time of last temperature count measurement
extern uint16_t last_temp_count;    /// temperature counter
extern uint32_t lp_clk_sel;

/// Reserve space for Exchange Memory, this section is linked first in the section "exchange_mem_case"
extern volatile uint8 dummy[];
extern uint8_t func_check_mem_flag;
extern struct lld_sleep_env_tag lld_sleep_env;
extern struct arch_sleep_env_tag sleep_env;
extern struct gtl_env_tag gtl_env;

uint32_t lld_sleep_lpcycles_2_us_func(uint32_t);

#ifdef __DA14581__
uint8_t atts_read_resp_patch(uint8_t conidx, struct l2cc_att_rd_req* req);

struct lld_evt_tag *lld_adv_start_patch(struct advertising_pdu_params *adv_par,
                                  struct co_buf_tx_node *adv_pdu,
                                  struct co_buf_tx_node *scan_rsp_pdu,
                                  uint8_t adv_pwr);

#endif



volatile uint8 descript[EM_SYSMEM_SIZE] __attribute__((section("BLE_exchange_memory"), zero_init)); //CASE_15_OFFSET
#if ((EM_SYSMEM_START != EXCHANGE_MEMORY_BASE) || (EM_SYSMEM_SIZE > EXCHANGE_MEMORY_SIZE))
#error("Error in Exhange Memory Definition in the scatter file. Please correct da14580_scatter_config.h settings.");
#endif
bool sys_startup_flag __attribute__((section("retention_mem_area0"), zero_init));
#ifndef __DA14581__
#if (BLE_CONNECTION_MAX_USER > 4)
volatile uint8_t cs_table[EM_BLE_CS_COUNT_USER * REG_BLE_EM_CS_SIZE] __attribute__((section("cs_area"), zero_init));
#endif
#else
#if (BLE_CONNECTION_MAX_USER > 1)
volatile uint8_t cs_table[(BLE_CONNECTION_MAX + 2) * REG_BLE_EM_WPB_SIZE * 2] __attribute__((section("cs_area"), zero_init));
#endif
#endif

#ifdef __DA14581__
uint32_t arch_adv_int __attribute__((section("retention_mem_area0"), zero_init));
#endif


/*
 * LOCAL FUNCTION DECLARATIONS
 ****************************************************************************************
 */


#include "gtl.h"
#include "gapm_task.h"

#if ((BLE_APP_PRESENT == 0 || BLE_INTEGRATED_HOST_GTL == 1)  && BLE_HOST_PRESENT )
int gtl_sleep_to_handler(ke_msg_id_t const msgid,
                        void const *param,
                        ke_task_id_t const dest_id,
                        ke_task_id_t const src_id);
int gtl_polling_to_handler(ke_msg_id_t const msgid,
                        void const *param,
                        ke_task_id_t const dest_id,
                        ke_task_id_t const src_id);

/// KE TASK element structure
struct ke_task_elem
{
    uint8_t   type;
    struct ke_task_desc * p_desc;
};

/// KE TASK environment structure
struct ke_task_env_tag
{
    uint8_t task_cnt;
    struct ke_task_elem task_list[];
};
extern volatile struct ke_task_env_tag ke_task_env;

#define MAX_GTL_PENDING_PACKETS_ADV (50)
#define MAX_GTL_PENDING_PACKETS     (MAX_GTL_PENDING_PACKETS_ADV + 10)

/**
****************************************************************************************
* @brief Function called to send a message through UART.
*
* @param[in]  msgid   U16 message id from ke_msg.
* @param[in] *param   Pointer to parameters of the message in ke_msg.
* @param[in]  dest_id Destination task id.
* @param[in]  src_id  Source task ID.
*
* @return             Kernel message state, must be KE_MSG_NO_FREE.
*****************************************************************************************
*/
static int my_gtl_msg_send_handler (ke_msg_id_t const msgid,
                          void *param,
                          ke_task_id_t const dest_id,
                          ke_task_id_t const src_id)
{
     //extract the ke_msg pointer from the param passed and push it in GTL queue
    struct ke_msg *msg = ke_param2msg(param);

    // Check if there is no transmission ongoing
    if (ke_state_get(TASK_GTL) != GTL_TX_IDLE)
    {
        if(gtl_env.tx_queue.tx_data_packet > MAX_GTL_PENDING_PACKETS_ADV)
        {
            if(msgid == GAPM_ADV_REPORT_IND || gtl_env.tx_queue.tx_data_packet > MAX_GTL_PENDING_PACKETS)
                return KE_MSG_CONSUMED;
        }
        co_list_push_back(&gtl_env.tx_queue, &(msg->hdr));
    }
    else
    {
        // send the message
        gtl_send_msg(msg);

        // Set GTL task to TX ONGOING state
        ke_state_set(TASK_GTL, GTL_TX_ONGOING);
    }

    //return NO_FREE always since gtl_eif_write handles the freeing
    return KE_MSG_NO_FREE;
}

const struct ke_msg_handler my_gtl_default_state[] =
{
    /** Default handler for GTL TX message, this entry has to be put first as table is
        parsed from end to start by Kernel */
    {KE_MSG_DEFAULT_HANDLER,  (ke_msg_func_t)my_gtl_msg_send_handler},

    #if (DEEP_SLEEP)
    {GTL_SLEEP_TO, (ke_msg_func_t)gtl_sleep_to_handler},
    {GTL_POLLING_TO, (ke_msg_func_t)gtl_polling_to_handler},
    #endif // DEEP_SLEEP
};

struct ke_state_handler my_gtl_default_handler = KE_STATE_HANDLER(my_gtl_default_state);
struct ke_task_desc TASK_DESC_GTL = {NULL, &my_gtl_default_handler, gtl_state, GTL_STATE_MAX, GTL_IDX_MAX};

void patch_gtl_task()
{
    uint8_t hdl;
    //struct ke_task_desc * p_task_desc = NULL;
    volatile struct ke_task_elem * curr_list = ke_task_env.task_list;
    uint8_t curr_nb = ke_task_env.task_cnt;

    // Search task handle
    for(hdl=0 ; hdl < curr_nb; hdl++)
    {
        if(curr_list[hdl].type == TASK_GTL)
        {
            ke_task_env.task_list[hdl].p_desc = &TASK_DESC_GTL;
            return;
        }
    }
}

#endif // #if (BLE_APP_PRESENT == 0 || BLE_INTEGRATED_HOST_GTL == 1)

#ifndef __DA14581__
static bool cmp_abs_time(struct co_list_hdr const * timerA, struct co_list_hdr const * timerB)
{
    uint32_t timeA = ((struct ke_timer*)timerA)->time;
    uint32_t timeB = ((struct ke_timer*)timerB)->time;

    return (((uint32_t)( (timeA - timeB) & 0xFFFF) ) > KE_TIMER_DELAY_MAX);
}

#endif

#ifdef __DA14581__
#if (!BLE_HOST_PRESENT)
/**
 ****************************************************************************************
 * @brief Handles the command HCI read local legacy supported features.
 * The handler processes the command by checking the local legacy supported features set
 * in the controller and sends back the dedicated command complete event with the status
 * and the features.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int llm_rd_local_supp_feats_cmd_handler(ke_msg_id_t const msgid,
        void const *param,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{
    // structure type for the complete command event
    struct llm_rd_local_supp_feats_cmd_complete *event;
    // allocate the complete event message
    event = KE_MSG_ALLOC(LLM_RD_LOCAL_SUPP_FEATS_CMP_EVT, src_id, TASK_LLM,
            llm_rd_local_supp_feats_cmd_complete);

    // get the local features
    event->feats.feats[0]= BT_FEATURES_BYTE0;
    event->feats.feats[1]= BT_FEATURES_BYTE1;
    event->feats.feats[2]= BT_FEATURES_BYTE2;
    event->feats.feats[3]= BT_FEATURES_BYTE3;
    event->feats.feats[4]= 0x60;//BT_FEATURES_BYTE4;
    event->feats.feats[5]= BT_FEATURES_BYTE5;
    event->feats.feats[6]= BT_FEATURES_BYTE6;
    event->feats.feats[7]= BT_FEATURES_BYTE7;

    // update the status
    event->status = CO_ERROR_NO_ERROR;
    // send the message
    ke_msg_send(event);

    return (KE_MSG_CONSUMED);
}
#endif
#endif
// h/w 
const uint32_t * const patch_table[]={
    
#ifdef __DA14581__
	#if (BLE_HOST_PRESENT)
    [0] = (const uint32_t *) atts_read_resp_patch,
	#else
	[0] = (const uint32_t *) llm_rd_local_supp_feats_cmd_handler,	
	#endif	
    [1] = (const uint32_t *) lld_adv_start_patch,  	
#endif         
    
#ifndef __DA14581__
	(const uint32_t *) cmp_abs_time,
	(const uint32_t *) l2cc_pdu_recv_ind_handler,
	(const uint32_t *) smpc_send_pairing_req_ind,
	(const uint32_t *) smpc_check_pairing_feat,
	(const uint32_t *) smpc_pairing_cfm_handler,
    (const uint32_t *) my_llc_con_update_req_ind,
    (const uint32_t *) my_llc_ch_map_req_ind,	
    (const uint32_t *) patched_gapm_adv_op_sanity,
#else
    (const uint32_t *) NULL
#endif
};

// Coarse Calibration configuration
const struct LUT_CFG_struct LUT_CFG =
    { 
        /*.HW_LUT_MODE               =*/ 1,                     // 1: HW LUT, 0: SW LUT    
        /*.RX_HSI_ENABLED 			 =*/ 1, 
		/*.PA_PULLING_OFFSET 		 =*/ 0, 
		/*.NR_CCUPD_1ST 		     =*/ 10, 
		/*.NR_CCUPD_REST 		     =*/ 4, 
		/*.NR_CCUPD_OL               =*/ 40, 
        /*.BLE_BAND_MARGIN           =*/ 10,                    // in MHz
		/*.EST_HALF_OVERLAP 		 =*/ 4,                     // in MHz
        /*.REQUIRED_CHAN_OVERLAP     =*/ 2,                     // At least 2 channels should be synthesizable by two adjacent calcaps (both of them)
        /*.PLL_LOCK_VTUNE_NUMAVGPOW  =*/ 3,                     // So 2^3=8 samples averaging
        /*.PLL_LOCK_VTUNE_LIMIT_LO   =*/ (1024*200/1200),       // Min acceptable Vtune = 0200mV
        /*.PLL_LOCK_VTUNE_LIMIT_HI   =*/ (1024*1000/1200),      // Max acceptable Vtune = 1000mV
        /*.PLL_LOCK_VTUNE_P2PVAR     =*/ (1024*50/1200),        // Vtune has to be stable within 50mV
        /*.PLL_LOCK_TIMING			 =*/ 416,    				// 416*62.5nsec=26usec
		/*.VCO_CALCNT_STARTVAL	 	 =*/ 0xFFFF,    			// Just in case it is modified by the Metal Fixes
		/*.VCO_CALCNT_TIMEOUT	 	 =*/ 300,    				// Just in case the while loops lock in meas_precharge_freq()
};


/**
 ****************************************************************************************
 * @brief SVC_Handler. Handles h/w patching mechanism IRQ
 *
 * @return void 
 ****************************************************************************************
 */

void SVC_Handler_c(unsigned int * svc_args)

{
// Stack frame contains:
// r0, r1, r2, r3, r12, r14, the return address and xPSR
// - Stacked R0 = svc_args[0]
// - Stacked R1 = svc_args[1]
// - Stacked R2 = svc_args[2]
// - Stacked R3 = svc_args[3]
// - Stacked R12 = svc_args[4]
// - Stacked LR = svc_args[5]
// - Stacked PC = svc_args[6]
// - Stacked xPSR= svc_args[7]

	unsigned int svc_number;
    
	svc_number = ((char *)svc_args[6])[-2];
    
	if (svc_number < (sizeof patch_table)/4)
		svc_args[6] = (uint32_t)patch_table[svc_number];
	else
		while(1);

	return;

}

/**
 ****************************************************************************************
 * @brief Set and enable h/w Patch functions
 *
 * @return void
 ****************************************************************************************
 */

void patch_func (void)
{

#ifdef __DA14581__ 

    //0x00024b1d lld_adv_start
    SetWord32(PATCH_ADDR0_REG, 0x00024b1C);
    SetWord32(PATCH_DATA0_REG, 0xb5ffdf01); //lld_adv_start svc 1 (+ enabling of interrupts)
    
  #if (BLE_HOST_PRESENT)	
    //0x0002a8c5 atts_read_resp
    SetWord32(PATCH_ADDR1_REG, 0x0002a8c4);
    SetWord32(PATCH_DATA1_REG, 0xb5f8df00); //atts_read_resp svc 0 (+ enabling of interrupts)
   #else
		//0x000278f9 llm_rd_local_supp_feats_cmd_handler
    SetWord32(PATCH_ADDR1_REG, 0x000278f8);
    SetWord32(PATCH_DATA1_REG, 0x4619df00); //llm_rd_local_supp_feats_cmd_handler svc 0 

    SetWord32(PATCH_ADDR2_REG, 0x000278c0); // llm_rd_local_ver_info_cmd_handler
    SetWord32(PATCH_DATA2_REG, 0x80c121d2); // patch manufacturer ID
	
	  SetWord32(PATCH_ADDR3_REG, 0x0002299c); // llc_version_ind_pdu_send
    SetWord32(PATCH_DATA3_REG, 0x80d020d2); // patch manufacturer ID

	#endif
    
#endif
	
#ifndef __DA14581__

    //0x00032795 cmp_abs_time
    SetWord32(PATCH_ADDR0_REG, 0x00032794);
    SetWord32(PATCH_DATA0_REG, 0xdf00b662); //cmp_abs_time svc 0 (+ enabling of interrupts)

    //0x0002a32b l2cc_pdu_recv_ind_handler (atts)
    SetWord32(PATCH_ADDR1_REG, 0x0002a328);
    SetWord32(PATCH_DATA1_REG, 0xdf014770); //l2cc_pdu_recv_ind_handler svc 1
	
    //0x0002ca1f  smpc_send_pairing_req_ind
    SetWord32(PATCH_ADDR2_REG, 0x0002ca1c);
    SetWord32(PATCH_DATA2_REG, 0xdf02bdf8); //smpc_send_pairing_req_ind svc 2

    //0x0002cb43  smpc_check_pairing_feat
    SetWord32(PATCH_ADDR3_REG, 0x0002cb40);
    SetWord32(PATCH_DATA3_REG, 0xdf03e7f5); //smpc_check_pairing_feat svc 3
	
    //0x0002d485  smpc_pairing_cfm_handler
    SetWord32(PATCH_ADDR4_REG, 0x0002d484);
    SetWord32(PATCH_DATA4_REG, 0xb089df04); //smpc_pairing_cfm_handler svc 4

    //0x000233bf  llc_con_update_req_ind
    SetWord32(PATCH_ADDR5_REG, 0x000233bc);
    SetWord32(PATCH_DATA5_REG, 0xdf05bdf8); //llc_con_update_req_ind svc 5

    //0x0002341b  llc_ch_map_req_ind
    SetWord32(PATCH_ADDR6_REG, 0x00023418);
    SetWord32(PATCH_DATA6_REG, 0xdf06bdf8); //llc_ch_map_req_ind svc 6

    //0x00030cef gapm_adv_op_sanity
    SetWord32(PATCH_ADDR7_REG, 0x00030cec);
    SetWord32(PATCH_DATA7_REG, 0xdf07bd70); //gapm_adv_op_sanity svc 7

    NVIC_DisableIRQ(SVCall_IRQn);
    NVIC_SetPriority(SVCall_IRQn, 0);
    NVIC_EnableIRQ(SVCall_IRQn);
#endif
}


extern void set_sleep_delay(void);

void lld_sleep_init_func(void)
{
    // Clear the environment
    memset(&lld_sleep_env, 0, sizeof(lld_sleep_env));

    // Set wakeup_delay
    set_sleep_delay();
    
    // Enable external wake-up by default
    ble_extwkupdsb_setf(0);
}

/**
 ****************************************************************************************
 * @brief otp_prepare()
 *
 * About: Prepare OTP Controller in order to be able to reload SysRAM at the next power-up
 ****************************************************************************************
 */
static __inline void  otp_prepare(uint32 code_size)
{
    // Enable OPTC clock in order to have access
    SetBits16 (CLK_AMBA_REG, OTP_ENABLE, 1);

    // Wait a little bit to start the OTP clock...
    for(uint8 i=0;i<10;i++); //change this later to a defined time  

    SetBits16(SYS_CTRL_REG, OTP_COPY, 1);

    // Copy the size of software from the first word of the retaintion mem.
    SetWord32 (OTPC_NWORDS_REG, code_size - 1);

    // And close the OPTC clock to save power
    SetBits16 (CLK_AMBA_REG, OTP_ENABLE, 0);
}


/*
 * EXPORTED FUNCTION DEFINITIONS
 ****************************************************************************************
 */


/*
 * MAIN FUNCTION
 ****************************************************************************************
 */
extern void set_system_clocks(void);
extern void init_pwr_and_clk_ble(void);
extern void rf_workaround_init(void);
 
extern uint16_t value0;													//Á½ÒºµÎ¼ä¸ôÊ±¼ä¼ÆÊ±
extern uint16_t DropINT; 												//¼ì²âµ½ÒºµÎ±êÊ¶
extern uint16_t count_present; 											//µ±Ç°ÒºµÎ¼ÆÊý
volatile uint8_t Activeflag = 0; 										//Èí¼þÊÇ·ñ¿ªÆô 0-Î´¿ªÆô 1-¿ªÆô
volatile uint8_t alarm_signal=0; 										//±¨¾¯ÐÅºÅ 0-Õý³£ 1-Òì³£
volatile uint8_t Power=1; 			 									//µçÔ´ÉÏµç±êÖ¾ 1-ÉÏµç 0-¶Ïµç
volatile uint8_t MotorOn=1;      										//µç»úËøËÀ±êÖ¾ 0-ËøËÀ 1-Î´ËøËÀ
volatile uint8_t Motor = 1;                                             //                                          
volatile uint8_t clamp=0;                                               //¿ª¸Ç±êÖ¾ 0-Î´¿ª¸Ç 1-ÒÑ¿ª¸Ç                                                                                         
volatile uint8_t keypress=0;                                            //¸´Î»¼üÊÇ·ñ°´ÏÂ 0-Î´°´ÏÂ 1-°´ÏÂ
volatile uint8_t senddata = 0;                                          //ÊÇ·ñ·¢ËÍÊý¾Ý 1-·¢ËÍ 0-²»·¢ËÍ
volatile uint8_t speed_five_status=0;                                   //ÊÇ·ñÒÑ¼ì²â5µÎ 1-ÊÇ 0-·ñ
volatile uint8_t first_count_status=1;                                  //ÊÇ·ñÎªÉÏµçºóÊ×µÎ 1-ÊÇ 0-·ñ
volatile uint8_t first55_flag=2;                                        //
volatile uint8_t halt=0;                                                //
volatile uint8_t ResetFlg = 0;                                          //°´¼ü¸´Î»±êÖ¾ 0-Î´¸´Î» 1-ÒÑ¸´Î»
uint8_t iTempMIN=0;                                                     //ÉÏ´Î·¢ËÍµÄµÎÒºËÙ¶È
uint8_t txBuffer[32];                                                   //´æ´¢ÊäÒº×´Ì¬Êý×é
uint8_t Key1 = 0;                                                       //¶Ì°´¸´Î»¼ü±êÖ¾ 1-¶Ì°´ 0-Î´¶Ì°´
uint8_t Keylong=0;                                                      //³¤°´¸´Î»¼ü±êÖ¾ 1-³¤°´ 0-Î´³¤°´
uint8_t SwordClose=0;                                                   //
uint8_t rf_send_sta=0x55;                                               //·¢ËÍÊý¾ÝÄÚÈÝ
uint8_t RFOFF=0;                                                        //·¢ËÍ¼ä¸ô¼ÆÊý
uint8_t Keylonglong=0;                                                  //ÊÇ·ñ¶à´Î¸´Î» 1-ÊÇ 0-·ñ
uint8_t KeylongCount=0;                                                 //¸´Î»´ÎÊý
uint8_t SetFlag = 0;                                                    //
uint8_t ZigbFlg=0;	
unsigned char cIptWarning = 00; 	                                    //±¨¾¯ÀàÐÍ 00-Õý³£Ý”Òº  55-Ý”ÒºÍê®…  	DD-ÒâÍâÖÐÖ¹
unsigned char cFunctionType = FUNC_DROP;                                //ÐÅºÅÀàÐÍ FUNC_DROP-Õý³£ÊäÒº  FUNC_KEY-¸´Î»°´¼ü 0xCC-¹Ø»úÔ¤¾¯
unsigned char avg_value=1;                                              //Æ½¾ùµÎËÙ
extern uint16_t iSecCount;                                              //ÊäÒº¹ÜÍÑÀëÊ±¼ä¼ÆÊý
extern uint16_t iSecCnt2;                                               //¿ª¸ÇÊ±¼ä
extern uint16_t LEDCount;                                               //LEDµÆÉÁË¸´ÎÊý
extern uint16_t SystemCount;                                            //Î´¼ì²âµ½ÒºµÎÏµÍ³¼ÆÊ±¼ÆÊý
extern uint16_t opencount;                                              //¿ª¸ÇÊäÒº¼ÆÊ±¼ÆÊý
extern uint8_t alarm_temp;
extern uint8_t alarm_sta;                                               //ÏìÉù±¨¾¯×´Ì¬
extern uint8_t uBatRemainPercentMA;                                     //µç³ØµçÁ¿
uint16_t txCount=0;                                                     //À¶ÑÀ·¢ËÍÊý¾Ý´ÎÊý
uint16_t BeSecTmp[2] = {0,0};                                           //·äÃù´ÎÊýÅÐ¶ÏÊý×é                                             
volatile	uint16_t lkeyDownCount=0;		                            //¸´Î»¼ü°´ÏÂ´ÎÊý

static uint8_t i;
static uint16_t value1 = 0;
static uint16_t value = 0;
static uint16_t value2 = 0;
static uint16_t warntime=15;                                            //warntime-±¨¾¯Ê±¼ä
static	uint8_t maBuffer[20],iMaCount = 0;	
static	uint64_t unique_id;
static	uint8_t FLASHid[8];

#pragma pack (1)
typedef struct
{	
	unsigned short	Zigbee_ADD;	    //zigbee±¾µØµØÖ·
 	unsigned char   deviceType;	    //Êý¾ÝÀàÐÍ
	unsigned short	RFCount;        //µ±Ç°Êý¾Ý°ü	
	unsigned short  cnt;            //Í³¼ÆÖµ£¬µ±Ç°ÊÇµÚ¼¸µÎ
	unsigned char   Data_length;	//Êý¾Ý³¤¶È
	unsigned char   rchPower;       //µç³ØÊ£ÓàµçÁ¿  rchPower%
	unsigned char   cSpeedDrop;     //µÎËÙ  µ¥Î» µÎ/·Ö
	unsigned char   cAlarm;         //±¨¾¯ÐÅºÅ
	unsigned char   ID0;     		//Î¨Ò»ID
	unsigned char   ID1;   			//  
	unsigned char   ID2; 			//    
	unsigned char   ID3;   			//  
	unsigned char   ID4;  			//   
	unsigned char   ID5;     		//
	unsigned char   ID6;			//
	unsigned char   ID7;			//
	unsigned char   verify;         //±£Áô×Ö		
}dropStutas_t;

 dropStutas_t* M10frm = ((dropStutas_t*)&txBuffer[0]);

uint8_t BCC_verify(void);
void KeyScan(void)
{	
	if(!GPIO_GetPinStatus(BUTTON_PORT, BUTTON1_PIN))
	{
		lkeyDownCount++;
        keypress =1;
	}
	else
	{
		keypress =0;
	}
	if(lkeyDownCount < 3000 && lkeyDownCount!=0 && keypress ==0)  //¶Ì°´
	{ 
		Key1 = 1;   //¶Ì°´±êÖ¾Î»
		lkeyDownCount = 0;
	}
	if(lkeyDownCount > 3500) 
	{
		Keylong = 1;    //³¤°´±êÖ¾Î»
		lkeyDownCount = 0;
	}
			
     //¸´Î»´ÎÊý´óÓÚ5			 
	if(KeylongCount > 5) 		 //  && keypress ==0    
	{
		Keylonglong = 1;
		KeylongCount = 0;
	}		 
			 
}
void KeyDispose(void)
{
	KeyScan();
									   
	if(Key1 == 1)     //¼ì²âµ½¶Ì°´¸´Î»										 
	{	 						
		Key1 = 0;									
		if(MotorOn == 0) 							
		{	
			first55_flag=2;//¶Ì°´½âËøºóÖØÖÃflag±êÖ¾Î» 	
			GPIO_SetActive(LED_PORT, LED_PIN);//					LEDON; 						
			app_timer_set(MOTOR_RESET, TASK_APP, 50);                //µç»úÊÍ·Å
			GPIO_SetActive(GPIO_PORT_2, GPIO_PIN_7);	   //¿ª5VµçÔ
			delay_us(20);										
			SetBeep();									
		    GPIO_SetInactive(GPIO_PORT_2, GPIO_PIN_7);   //¹Ø5VµçÔ´ 	
			iSecCount=0;							
			iSecCnt2 = 1;							
			LEDCount=0;	  							
			timer[3] = 3;						    
																											 			
			MotorOn = 1;   							
			SwordClose =0;									

			BeSecTmp[0] = 0;
			BeSecTmp[1] = 0;
						
			GPIO_SetInactive(LED_PORT, LED_PIN);//LEDOFF;
		}
	}
	
	
	if(Keylong==1)		 // ¼ì²âµ½³¤°´¸´Î»		
	{
		Keylong = 0;								
		if(MotorOn == 1) 							
		{	
			GPIO_SetActive(LED_PORT, LED_PIN);//LEDON;					   				
			GPIO_SetActive(GPIO_PORT_2, GPIO_PIN_7);	   //¿ª5VµçÔ
			delay_us(20);										
			SetBeep();								
			delay_us(1200);							   	
			SetBeep();
		    GPIO_SetInactive(GPIO_PORT_2, GPIO_PIN_7);   //¹Ø5VµçÔ´ 					
			KeylongCount++;//¼ÇÂ¼¸´Î»´ÎÊý
			SystemCount = 0;						
			count_present = 0; 						
			cIptWarning =0x00; 											
			cFunctionType = FUNC_KEY;		  //°´¼ü¸´Î»0xBB
			ResetFlg = 1;														
					
			BeSecTmp[0] = 0;
			BeSecTmp[1] = 0;								
			GPIO_SetInactive(LED_PORT, LED_PIN);//LEDOFF;
		}
		else   										
		{   
			cIptWarning=0x00;
			SystemCount=0;
			GPIO_SetActive(LED_PORT, LED_PIN);//LEDON; 				  							
			app_timer_set(MOTOR_RESET, TASK_APP, 50);		// µç»ú¸´Î»
			GPIO_SetActive(GPIO_PORT_2, GPIO_PIN_7);	   //¿ª5VµçÔ
			delay_us(20);					
			SetBeep();								
			delay_us(800);
			delay_us(400);
			SetBeep();			
		    GPIO_SetInactive(GPIO_PORT_2, GPIO_PIN_7);   //¹Ø5VµçÔ´ 					
			first55_flag=2;//³¤°´½âËøºóÖØÖÃflag±êÖ¾Î»
			iSecCount=0;							
			LEDCount=0;								
			count_present = 0;	
										
			cFunctionType = FUNC_KEY;	 		 //°´¼ü¸´Î»0xBB												 										
			MotorOn = 1;   							
			SwordClose =0;		
			ResetFlg = 1;
														
			BeSecTmp[0] = 0;
			BeSecTmp[1] = 0;
			GPIO_SetInactive(LED_PORT, LED_PIN);//LEDOFF;
		}			
	}
	if(Keylonglong==1)  //¸´Î»´ÎÊý´óÓÚ5´Î
	{
		Keylonglong=0;
		GPIO_SetActive(GPIO_PORT_2, GPIO_PIN_7);	   //¿ª5VµçÔ
		delay_us(20);										
		SetBeep();//±êÖ¾333								
		delay_us(560);							   	
		SetBeep(); //±êÖ¾444
		delay_us(560);							   	
		SetBeep(); //±êÖ¾555
		GPIO_SetInactive(GPIO_PORT_2, GPIO_PIN_7);   //¹Ø5VµçÔ´ 	
		ZigBee_Wake();
        SetFlag = 1;   //zigbeeÉèÖÃ
					
		delay_us(560);
        GPIO_SetActive(LED_PORT, LED_PIN);//LEDON; 
					
	}
}
/*
************************************************************************************
* Platform initialization
************************************************************************************
*/
void InitPlatform(void)
{
    #if (USE_WDOG)
    SetWord16(WATCHDOG_REG, 0xC8);          // 200 * 10.24ms = ~2sec active time!
    SetWord16(WATCHDOG_CTRL_REG, 0);        // Generate an NMI when counter reaches 0 and a WDOG (SYS) Reset when it reaches -16!
                                            // WDOG can be frozen by SW!
    SetWord16(RESET_FREEZE_REG, FRZ_WDOG);  // Start WDOG
#else
    SetWord16(SET_FREEZE_REG, FRZ_WDOG);
#endif
    
#if defined(CFG_USE_DEFAULT_XTAL16M_TRIM_VALUE_IF_NOT_CALIBRATED)
#define DEFAULT_XTAL16M_TRIM_VALUE (1302)
    // Apply the default XTAL16 trim value if a trim value has not been programmed in OTP
    if ( 0 == GetWord16(CLK_FREQ_TRIM_REG) )
    {
        SetBits16(CLK_16M_REG, RC16M_ENABLE, 1);                      // enable RC 16MHz
        for (volatile int i = 0; i < 20; i++);

        SetBits16(CLK_CTRL_REG, SYS_CLK_SEL, 1);                      // switch to  RC16
        while( (GetWord16(CLK_CTRL_REG) & RUNNING_AT_RC16M) == 0 );   // wait for actual switch

        SetBits16(CLK_CTRL_REG, XTAL16M_DISABLE, 1);                  // disable XTAL16
        SetWord16(CLK_FREQ_TRIM_REG, DEFAULT_XTAL16M_TRIM_VALUE);     // set default trim value
        SetBits16(CLK_CTRL_REG, XTAL16M_DISABLE, 0);                  // enable XTAL16
        while( (GetWord16(SYS_STAT_REG) & XTAL16_SETTLED) == 0 );     // wait for XTAL16 settle

        SetBits16(CLK_CTRL_REG , SYS_CLK_SEL ,0);                     // switch to  XTAL16
        while( (GetWord16(CLK_CTRL_REG) & RUNNING_AT_XTAL16M) == 0 ); // wait for actual switch
    }
#endif
    
    set_system_clocks();    //ÏµÍ³Ê±ÖÓ³õÊ¼»¯
    GPIO_init();             //GPIO³õÊ¼»¯
    periph_init();           //´®¿Ú  SPI  FLASH ³õÊ¼»¯
}

/*
************************************************************************************
* BLE initialization
************************************************************************************
*/
void InitBLE(void)
{
    init_pwr_and_clk_ble(); 

#if UNCALIBRATED_AT_FAB
    SetBits16(BANDGAP_REG, BGR_TRIM, 0x0);  // trim RET Bandgap
    SetBits16(BANDGAP_REG, LDO_RET_TRIM, 0xA);  // trim RET LDO
    SetWord16(RF_LNA_CTRL1_REG, 0x24E);
    SetWord16(RF_LNA_CTRL2_REG, 0x26);
    SetWord16(RF_LNA_CTRL3_REG, 0x7);
    SetWord16(RF_VCO_CTRL_REG, 0x1);
    SetBits16(CLK_16M_REG, RC16M_TRIM, 0xA);
#endif

    // Initialize BLE stack 
    NVIC_ClearPendingIRQ(BLE_SLP_IRQn);     
    NVIC_ClearPendingIRQ(BLE_EVENT_IRQn); 
    NVIC_ClearPendingIRQ(BLE_RF_DIAG_IRQn);
    NVIC_ClearPendingIRQ(BLE_RX_IRQn);
    NVIC_ClearPendingIRQ(BLE_CRYPT_IRQn);
    NVIC_ClearPendingIRQ(BLE_FINETGTIM_IRQn);	
    NVIC_ClearPendingIRQ(BLE_GROSSTGTIM_IRQn);	
    NVIC_ClearPendingIRQ(BLE_WAKEUP_LP_IRQn);     	
    rwip_init(error);
    
#if ((BLE_APP_PRESENT == 0 || BLE_INTEGRATED_HOST_GTL == 1) && BLE_HOST_PRESENT )
    patch_gtl_task();
#endif // #if (BLE_APP_PRESENT == 0 || BLE_INTEGRATED_HOST_GTL == 1)
    
    /* Set spi to HW (Ble)
     * Necessary: So from this point the BLE HW can generate spi burst iso SW
     * SPI BURSTS are necessary for the radio TX and RX burst, done by hardware
     * beause of the accurate desired timing 
     */
    //FPGA
#ifdef FPGA_USED    
    SetBits32(BLE_CNTL2_REG,SW_RPL_SPI ,1);
#endif

    //Enable BLE core    
    SetBits32(BLE_RWBTLECNTL_REG,RWBLE_EN ,1); 

    
#if RW_BLE_SUPPORT && HCIC_ITF

    // If FW initializes due to FW reset, send the message to Host
    if(error != RESET_NO_ERROR)
    {
        rwble_send_message(error);
    }
#endif
}

/*
************************************************************************************
* Flash initialization
************************************************************************************
*/
void InitFlash(void)
{
    GPIO_SetActive(GPIO_PORT_2, GPIO_PIN_0);  //LED             
	Activeflag = 1;	
	if(Activeflag==1)
	{ 		  
		 GPIO_SetActive(GPIO_PORT_1 , GPIO_PIN_3);  //power_ON  Èí¼þ¿ªÏµÍ³µçÔ´
	}		 
	GPIO_SetActive(GPIO_PORT_2, GPIO_PIN_7);	   //¿ª5VµçÔ´		 
	unique_id = spi_read_flash_unique_id();					
	FLASHid[0] = ((unique_id>>32)>>24)&0xFF;    
	FLASHid[1] = ((unique_id>>32)>>16)&0xFF;  
	FLASHid[2] = ((unique_id>>32)>>8)&0xFF;  
	FLASHid[3] = (unique_id>>32)&0xFF;         
	FLASHid[4] = (unique_id>>24)&0xFF;         
	FLASHid[5] = (unique_id>>16)&0xFF;         
	FLASHid[6] = (unique_id>>8)&0xFF;          
	FLASHid[7] = (unique_id)&0xFF;      
    spi_flash_read_data(nvds_data_storage.NVDS_TAG_BD_ADDRESS,0xf000,6);

	GPIO_ConfigurePin(GPIO_PORT_0, GPIO_PIN_0, OUTPUT, PID_GPIO, false); 	
	RESERVE_GPIO(GPIO, GPIO_PORT_0, GPIO_PIN_0, PID_GPIO);
	delay_us(10);
	SetBeep();
	delay_us(12);
	SetBeep();
	delay_us(12); 
	SetBeep();
	delay_us(12);
	SetBeep(); 
	delay_us(12);
	SetBeep();
	delay_us(12); 
	SetBeep();
	delay_us(12);
	SetBeep(); 
	ZigBee_Sleep();	//ZigBeeÄ£¿é½øÈëÐÝÃß					
	GPIO_SetInactive(GPIO_PORT_2, GPIO_PIN_0);   //LED
	GPIO_SetInactive(GPIO_PORT_2, GPIO_PIN_7);   //¹Ø5VµçÔ´ 
	first55_flag=2;
			 
/*****************                 ***********************/ 		
          
    /* Don't remove next line otherwhise dummy[0] could be optimized away
     * The dummy array is intended to reserve the needed Exch.Memory space in retention memory
     */
    dummy[0] = dummy[0];
    descript[0] = descript[0];
#ifndef __DA14581__    
    
#if (BLE_CONNECTION_MAX_USER > 4)
    cs_table[0] = cs_table[0];
#endif
#else

#if (BLE_CONNECTION_MAX_USER > 1)
    
    cs_table[0] = cs_table[0];
#endif
#endif
                                         
    /* Don't remove next line otherwhise data__1 is optimized away.
     * The address 0x9010 is used by the ROM code (rand.o) and cannot be used by the 
     * application code!
     */
    //GZ data__1 = 0;
                                            
    // Initialize unloaded RAM area
    //unloaded_area_init();

    // Initialize random process
    srand(1);

    // Initialize the exchange memory interface, emi in RAM for the time being, so no init necessary
#if 0
    emi_init();
#endif

    // Initialize NVDS module
    nvds_init((uint8_t *)NVDS_FLASH_ADDRESS, NVDS_FLASH_SIZE);

    //check and read BDADDR from OTP
    nvds_read_bdaddr_from_otp();

#ifdef RADIO_580
    iq_trim_from_otp();
#endif

}

/*
************************************************************************************
* Sleep mode initializations (especially for full embedded)
************************************************************************************
*/
void InitSleepMode(void)
{
    #if (EXT_SLEEP_ENABLED)
     app_set_extended_sleep();
#elif (DEEP_SLEEP_ENABLED)
     app_set_deep_sleep();
#else
     app_disable_sleep();
#endif    
   
    if (lp_clk_sel == LP_CLK_RCX20)
    {    
        calibrate_rcx20(20);
        read_rcx_freq(20);  
    }
}

/*
************************************************************************************
* Application initializations
************************************************************************************
*/
void InitAppliaction(void)
{
    #if (BLE_APP_PRESENT)    
    {
        app_init();         // Initialize APP
    }
#endif /* #if (BLE_APP_PRESENT) */

    lld_sleep_init_func();
    
    SetWord16(TRIM_CTRL_REG, 0xA2);
    SetBits16(CLK_16M_REG, XTAL16_CUR_SET, 0x5);
    
//    // Gives 1dB higher sensitivity - UNTESTED
    
// Now enable the TX_EN/RX_EN interrupts, depending on the RF mode of operation (PLL-LUT and MGC_KMODALPHA combinations)

   enable_rf_diag_irq(RF_DIAG_IRQ_MODE_RXTX); 

#if BLE_APP_SPOTAR
    //app_spotar_exec_patch();
#endif

    if ( (app_get_sleep_mode() == 2) || (app_get_sleep_mode() == 1) )
    {
         SetWord16(SET_FREEZE_REG, FRZ_WDOG);            // Stop WDOG until debugger is removed
         while ((GetWord16(SYS_STAT_REG) & DBG_IS_UP) == DBG_IS_UP) {}; 
         SetBits16(SYS_CTRL_REG, DEBUGGER_ENABLE, 0);    // close debugger
    }	
}

/*
************************************************************************************
* Watchdog initializations
************************************************************************************
*/
void InitWatchdog(void)
{
    #if (USE_WDOG)
    SetWord16(WATCHDOG_REG, 0xC8);          // 200 * 10.24ms active time for initialization!
    SetWord16(RESET_FREEZE_REG, FRZ_WDOG);  // Start WDOG
#endif

    
#if (STREAMDATA_QUEUE)
    stream_fifo_init ();
#endif   
}

/*
************************************************************************************
* SetAndWaitInterrupts 
************************************************************************************
*/
uint8_t SetAndWaitInterrupts(sleep_mode_t* sleep_mode)
{
  	// schedule all pending events
		if(GetBits16(CLK_RADIO_REG, BLE_ENABLE) == 1) 
		{ // BLE clock is enabled
			  if(GetBits32(BLE_DEEPSLCNTL_REG, DEEP_SLEEP_STAT) == 0 && !(rwip_prevent_sleep_get() & RW_WAKE_UP_ONGOING)) 
				{ // BLE is running
#ifndef FPGA_USED            
                uint8_t ble_evt_end_set = ke_event_get(KE_EVENT_BLE_EVT_END); // BLE event end is set. conditional RF calibration can run.
#endif                
                rwip_schedule();
#ifndef FPGA_USED            
               if (ble_evt_end_set)
                {
                    uint32_t sleep_duration = 0;
                    if (lp_clk_sel == LP_CLK_RCX20)
                        read_rcx_freq(20);
                    if (lld_sleep_check(&sleep_duration, 4)) //6 slots -> 3.750 ms
                        conditionally_run_radio_cals(); // check time and temperature to run radio calibrations. 
                }
#endif
                
#if (BLE_APP_PRESENT)
				if ( app_asynch_trm() )
					return 1; // so that rwip_schedule() is called again
#endif
                
#ifdef CFG_PRINTF
                {
                    arch_printf_process();
                }
#endif                
			}	
		}
		
#if (BLE_APP_PRESENT)
		// asynchronous events processing
		if (app_asynch_proc())
			return 1; // so that rwip_schedule() is called again
#endif

#if (STREAMDATA_QUEUE)        
        if (stream_queue_more_data( ))
            return 1;
#endif   
#if (!BLE_APP_PRESENT)
        if (check_gtl_state())
#endif			
			{
            GLOBAL_INT_STOP();

#if (BLE_APP_PRESENT)
            app_asynch_sleep_proc();
#endif        

//            // set wake-up delay only for RCX (to cover small frequency shifts due to temerature variation)
            // if app has turned sleep off, rwip_sleep() will act accordingly
            // time from rwip_sleep() to WFI() must be kept as short as possible!
            *sleep_mode = rwip_sleep();
        
            // BLE is sleeping ==> app defines the mode
            if (*sleep_mode == mode_sleeping) 
			{
                if (sleep_env.slp_state == ARCH_EXT_SLEEP_ON) 
				{
                    *sleep_mode = mode_ext_sleep;
                } 
				else 
				{
                    *sleep_mode = mode_deep_sleep;
                }
            }
            
            if (*sleep_mode == mode_ext_sleep || *sleep_mode == mode_deep_sleep) 
            {
                SetBits16(PMU_CTRL_REG, RADIO_SLEEP, 1); // turn off radio
                
                if (jump_table_struct[nb_links_user] > 1)
                {
                    if( (*sleep_mode == mode_deep_sleep) && func_check_mem() && test_rxdone() && ke_mem_is_empty(KE_MEM_NON_RETENTION) )
                    {
                        func_check_mem_flag = 2;//true;
                    }
                    else
                        *sleep_mode = mode_ext_sleep;
                }
                else
                {
                    if( (*sleep_mode == mode_deep_sleep) && ke_mem_is_empty(KE_MEM_NON_RETENTION) )
                    {
                        func_check_mem_flag = 1;//true;
                    }
                    else
                        *sleep_mode = mode_ext_sleep;
                }
                
#if (BLE_APP_PRESENT)
                // hook for app specific tasks when preparing sleeping
                app_sleep_prepare_proc(sleep_mode);
#endif
                
                
                if (*sleep_mode == mode_ext_sleep || *sleep_mode == mode_deep_sleep)
                {
                    SCB->SCR |= 1<<2; // enable sleepdeep mode bit in System Control Register (SCR[2]=SLEEPDEEP)
                    
                    SetBits16(SYS_CTRL_REG, PAD_LATCH_EN, 0);           // activate PAD latches
                    SetBits16(PMU_CTRL_REG, PERIPH_SLEEP, 1);           // turn off peripheral power domain
                    if (*sleep_mode == mode_ext_sleep)	{
                        SetBits16(SYS_CTRL_REG, RET_SYSRAM, 1);         // retain System RAM
                        SetBits16(SYS_CTRL_REG, OTP_COPY, 0);           // disable OTP copy	  
                    } else { // mode_deep_sleep
#if DEVELOPMENT_DEBUG
                        SetBits16(SYS_CTRL_REG, RET_SYSRAM, 1);         // retain System RAM		
#else
                        SetBits16(SYS_CTRL_REG, RET_SYSRAM, 0);         // turn System RAM off => all data will be lost!
#endif
                        otp_prepare(0x1FC0);                            // this is 0x1FC0 32 bits words, so 0x7F00 bytes 
                    }
                }

                SetBits16(CLK_16M_REG, XTAL16_BIAS_SH_DISABLE, 0);
                
#if (BLE_APP_PRESENT)
                // hook for app specific tasks just before sleeping
                app_sleep_entry_proc(sleep_mode);
#endif

#if ((EXTERNAL_WAKEUP) && (!BLE_APP_PRESENT)) // external wake up, only in external processor designs
                ext_wakeup_enable(EXTERNAL_WAKEUP_GPIO_PORT, EXTERNAL_WAKEUP_GPIO_PIN, EXTERNAL_WAKEUP_GPIO_POLARITY);
#endif
                
                WFI();

#if (BLE_APP_PRESENT)
                // hook for app specific tasks just after waking up
                app_sleep_exit_proc(*sleep_mode);
#endif

#if ((EXTERNAL_WAKEUP) && (!BLE_APP_PRESENT)) // external wake up, only in external processor designs
                // Disable external wakeup interrupt
                ext_wakeup_disable();
#endif

                // reset SCR[2]=SLEEPDEEP bit else the mode=idle WFI will cause a deep sleep 
                // instead of a processor halt
                SCB->SCR &= ~(1<<2);
            }
            else if (*sleep_mode == mode_idle) 
            {
#if (!BLE_APP_PRESENT)              
                if (check_gtl_state())
#endif
                {
                    WFI();
                }
            }
            
            // restore interrupts
            GLOBAL_INT_START();
     }
        
#if (USE_WDOG)        
        SetWord16(WATCHDOG_REG, 0xC8);          // Reset WDOG! 200 * 10.24ms active time for normal mode!
#endif
     return 0;
}
/*
************************************************************************************
* CheckStatus 
************************************************************************************
*/
void CheckStatus(void)
{
//     // µçÔ´¼ì²â Èí¼þ¹Ø»ú		 
//		if(Power ==0) 			
//		{
//			GPIO_SetInactive(GPIO_PORT_1, GPIO_PIN_3);	 //POWER_ON  ÏµÍ³¶Ïµç		
//			GPIO_SetInactive(LED_PORT, LED_PIN);	 		
//			Activeflag =0;			 					
//		}		
		KeyDispose();   //°´¼ü¼ì²â
		ZigBee_Setting();

/////////////////////////·ðÉ½8-29////////////////////////////		
/////¿ª¸Ç¼ì²â
		if(GPIO_GetPinStatus(GPIO_PORT_2, GPIO_PIN_5)==1)			 // P2_5  CLAMP¿ØÖÆ¿Ú   ¿ª¸ÇÔò½øÈëÏÂÃæº¯Êý
		{
			clamp=1;     //¿ª¸Ç±êÖ¾Î»
			Power = 1;	 //×ÜµçÔ´ÉÏµç±êÖ¾Î»
        					
		}
        else
        {
            clamp = 0;
        }
}

//ÏµÍ³¶Ïµç¹Ø»ú
void PowerOff(void) 
{
    GPIO_SetInactive(GPIO_PORT_1, GPIO_PIN_3);	 //POWER_ON  ÏµÍ³¶Ïµç		
	GPIO_SetInactive(LED_PORT, LED_PIN);	 		
	Activeflag =0;    
}


//¼ì²âÊÇ·ñÐèÒª×Ô¶¯¹Ø»ú
void CheckAutoPowerOffStatus(void)
{
    if(clamp ==1 && (DropINT == 1||count_present<7))		   //¿ª¸Ç ²¢ÇÒ ¼ì²âµ½ÓÐÒºµÎ»òÕßÒºµÎÊýÐ¡ÓÚ7£¬ ÔòÈí¼þ¹Ø»ú  		 
	{					                                           //DropINT ¼ì²âµ½ÒºµÎ±êÖ¾
		clamp = 0;    
		SystemCount=0;  //ÏµÍ³¼ÆÊ±¼ÆÊý  0*0.5s
														
	}	
	if(count_present<7  && cIptWarning==0x00 && clamp==0 )	  //ºÏ¸Ç ²¢ÇÒÒºµÎÊýÐ¡ÓÚ7 ²¢ÇÒ ±¨¾¯ÀàÐÍÎª00-Õý³£ÊäÒº¯Ò 	
	{								
		if(SystemCount>=60&&SystemCount<=80)   //30sÄÚ¼ì²â²»µ½ÒºµÎ£¬¹Ø»úÇ°·¢ËÍ¹Ø»úÔ¤¾¯ÐÅºÅ0XCC
		{  
			cFunctionType = 0xcc;   //¹Ø»úÔ¤¾¯ 0xCC
			if(SystemCount%2==0)
            {
				timer[3]=3;
            }
		}
		else if(SystemCount>80)   //³¬¹ý40s
		{
			SystemCount = 0;															      
			PowerOff();		  //ÏµÍ³¶Ïµç¹Ø»ú  P1.3 =0
		}						
	}				
//Êý¾ÝÀàÐÍÅÐ¶¨  0xAA:Õý³£ÊäÒº   0xBB£º°´¼ü¸´Î»			
	if(cFunctionType ==FUNC_KEY&&count_present<=3)		//¸´Î»ÐÅºÅ³ÖÐø3µÎ	cFunctionType:Êý¾ÝÀàÐÍ		FUNC_KEY=0xBB
    {
        alarm_signal=1;		
    }            
	else 
    {            
		alarm_signal=0;		   //		µÈÓÚÁãÊ±ÅÐ¶¨ cFunctionType = FUNC_DROP;
    }
}

void OpenClamStatus(void)
{
    Motor = 1;							   	
	iSecCount = 0;	                              //ÐÞÕýÊäÒº¹ÜÍÑÀëËÀÇøÊ±¼ä					   	
    if(cIptWarning == 0xEE && count_present>6)			 //¿ª¸ÇÊäÒº±¨¾¯  ²¢ÇÒÒºµÎÊý´óÓÚ6  
	{
		if(opencount%14==0)	//ÏìÉùÌáÊ¾²»ºöÂÔ    Ã¿7S±¨¾¯Ò»´Î
		{
			GPIO_SetActive(GPIO_PORT_2, GPIO_PIN_7);	   //¿ª5VµçÔ
			delay_us(20);								
			SetBeep();
			GPIO_SetInactive(GPIO_PORT_2, GPIO_PIN_7);   //¹Ø5VµçÔ´ 								
			opencount=1;
		}
	}
						
	if(iSecCnt2%20==0)//¿ª¸Ç10s·¢Ò»´Î±¨¾¯						
	{						        																			
		cIptWarning = 0xDD;              	
		cFunctionType = FUNC_DROP;
		value2 =  avg_value;			
  /////ÊäÒºÍÑÀë20min£¬Ò»ºÏ¸Ç¾Í¹Ø»ú
		if(iSecCnt2>2400)
		{
			PowerOff();							  //¹Ø»ú
		}										 								        															
		if(RFOFF == 0)  
        {
            halt = 1;	
        }			
	}	
						
	if(halt == 1 && RFOFF == 0)	   		
	{    
		if(iSecCnt2<2390)	 //¿ª¸ÇÊ±¼ä³¬¹ý10s£¬Ã¿¸ô10s±¨¾¯Ò»´ÎÐ¡ÓÚ20minÊ±  20minÒÔÍâ 5sÈßÓà 
		{
			GPIO_SetActive(GPIO_PORT_2, GPIO_PIN_7);	   //¿ª5VµçÔ
			delay_us(20);																
			SetBeep();		//¿ª¸ÇÊ±¼ä³¬¹ý10s£¬Ã¿¸ô10s±¨¾¯Ò»´Î						
            GPIO_SetInactive(GPIO_PORT_2, GPIO_PIN_7);   //¹Ø5VµçÔ´ 			
		} 
		count_present += (avg_value/6);  		 
		rf_send_sta=0xaa;			
	}
}

//Ô¤±¨¾¯´¦Àí
void PreAlarm(void)
{
    ////////////////¶¯Ì¬ËøËÀÊ±¼äÔÚ8sÒÔÏÂµÄ²»½øÐÐÔ¤±¨¾¯/////////////////////////////
    if(warntime>16 && iSecCount ==13 && count_present>6)	 //16*0.5=8s    13*0.5=6.5s ±¨¾¯Ê±¼ä´óÓÚ8s£¬6.5ÃëÎÞÒºµÎºó½øÐÐÔ¤±¨¾¯
	{	
		GPIO_SetActive(GPIO_PORT_2, GPIO_PIN_7);	   //¿ª5VµçÔ
		delay_us(20);
		SetBeep();								
		delay_us(600);							
		SetBeep();
		delay_us(800);
		delay_us(400);
		SetBeep();
		delay_us(600);
		SetBeep();
		GPIO_SetInactive(GPIO_PORT_2, GPIO_PIN_7);   //¹Ø5VµçÔ´ 
		cIptWarning = 0x33;
		rf_send_sta=0xaa;
		LEDCount=0;	
		iSecCount = 14;
	}	
				
	if(iSecCount >=warntime  && count_present>6  )      //25µÎÊ±¼ä	 1.5ML µ½´ïÔ¤±¨¾¯Ê±¼ä
	{	  						  	 
		if(!SwordClose)  
		{ 
			timer[3]=24;  			  //		³õÊ¼»¯Ê±¼ä12s
			Motor = 0;	                                                                                                                                                                                                                                                                                                                                                      									
		} 				
		if(Motor == 0)									
		{
            app_timer_set(MOTOR_TIMER, TASK_APP, 50);	// µç»úËøËÀ			
			Motor = 1;									
			SwordClose =1;		
			LEDCount = 0;
		}				
		if(!BeSecTmp[0])
		{											
						// ÔÚwarntimeºÜÐ¡µÄÊ±ºò DI DI DI DI Ö»»áÏìÒ»´Î
			BeSecTmp[0] = 20;//DI DI DI DI¹Ì¶¨ÏìÈý´Î				
			BeSecTmp[1] = BeSecTmp[0];
		}			
		if(BeSecTmp[1]==BeSecTmp[0]-5||BeSecTmp[1]==BeSecTmp[0]-11||BeSecTmp[1]==BeSecTmp[0]-17)   
		{ 					
			GPIO_SetActive(GPIO_PORT_2, GPIO_PIN_7);	   //¿ª5VµçÔ
			delay_us(20);
			beep();		
			GPIO_SetInactive(GPIO_PORT_2, GPIO_PIN_7);   //¹Ø5VµçÔ´ 											
		}
		if(LEDCount%2==0&&LEDCount<21)	  
		{
			timer[3]=3;	
		}			
		if(( (iSecCount-BeSecTmp[0])%20==0 ||first55_flag!=0 ) && Key1!=1 )//10sÒ»´Î||(BeSecTmp[1]%2==0&&BeSecTmp[1]>1) 
		{							  
			timer[3]=3;	 					
			cIptWarning = 0x55;	  			
			cFunctionType = FUNC_DROP;
			value2 =  0;	
			halt = 1;						 	
			alarm_temp=0;
			first55_flag=first55_flag-1;				
		} 										
        if((MotorOn==0)&&(iSecCnt2-warntime>2405))	
		{
			PowerOff(); //¹Ø»ú
		}									
								
	}
}

//µÎÒº´¦Àí
void Drip(void)
{
    DropINT = 0;
	SetFlag = 0;
	if(count_present<=60) 	//´¦Àí³õÊ¼60µÎ				
	{  
		GPIO_SetActive(LED_PORT, LED_PIN);  
		delay_us(600);   //×î¿ªÊ¼60µÎ£¬ Ã¿µÎLEDµÆÁÁÒ»´Î
	}
    if(alarm_signal==0)
    {		
        cFunctionType = FUNC_DROP;   //Êý¾ÝÀàÐÍ  0xAA£ºÕý³£ÊäÒº
    }
	iSecCount = 0;	   			
	iSecCnt2 = 1; 				
	SystemCount = 0;			
	alarm_sta=0x11;			   	
	alarm_temp=0;				
	//ºÏ¸Ç¼ì²â			
	if(GPIO_GetPinStatus(GPIO_PORT_2, GPIO_PIN_5)==0)  //  ºÏÉÏ¸Ç
	{
		cIptWarning = 0x00;   //  Õý³£ÊäÒº×´Ì¬ 0x00						
	} 
	else  if(count_present>6)      //¿ª¸Ç²¢ÇÒÒºµÎÊý´óÓÚ6µÎ  
	{ 
		cIptWarning = 0xEE;    //  ¿ª¸ÇÊäÒº×´Ì¬ 0xEE
    }		
	//ÒºµÎËÙ¶È¼ÆËã				
	value1=value0&0xffff;		   // value1 Á½ÒºµÎÖ®¼äµÄ¼ä¸ôÊ±¼ä
	if(value1<470)		//470*0.5ms=	235ms   	
	{
		value=255;  //ÒºÌåËÙ¶È  255µÎ/·ÖÖÓ     1*60*1000/235=255  µÎ/·ÖÖÓ
	}
	else
	{
		value=120000/value1;	
	}									
	maBuffer[++iMaCount] = value;	   
	maBuffer[6]= maBuffer[iMaCount];	
    if(iMaCount == 5)					     					   						
	{  
		iMaCount = 0;
        speed_five_status=1;  //ÒÑ¼ì5µÎ
	}
	if(speed_five_status==1)		
	{  
		value2 = (maBuffer[1]+maBuffer[2]+maBuffer[3]+maBuffer[4]+maBuffer[5])/5;
	}
	else
	{ 					//Ê×´Î ÔÚ1ÖÁ4µÎÊ± ÇóÆ½¾ù  
		value2=(maBuffer[1]+maBuffer[2]+maBuffer[3]+maBuffer[4]+maBuffer[5])/iMaCount;
	} 				
	avg_value = (maBuffer[1]+maBuffer[2]+maBuffer[3]+maBuffer[4]+maBuffer[5]-maBuffer[6])/4;					
	if( value2!=0 && value2<=255 &&(value2-iTempMIN>=5||iTempMIN-value2>=5))			//abs(value2 - iTempMIN)>=10 &&Sleep!=1
	{ 
		senddata = 1;	// µÎËÙ´óÓÚ0Ð¡ÓÚ255Ê±£¬  ²¢ÇÒÓëÉÏ´Î·¢ËÍÊäÒºËÙ¶È¶Ô±È±ä»¯³¬¹ý5£¬  Ôò·¢ËÍÊý¾Ý				
		timer[4] = 24;						    
	}				
	else if(value2!=0&&value2<=255&&(value2-iTempMIN<5||iTempMIN-value2<5))		//Ð¡ÓÚ200¸Ä³ÉÐ¡ÓÚµÈÓÚ255
	{
		if(timer[4] < 5)  // µÎËÙ´óÓÚ0Ð¡ÓÚ255Ê±£¬ÓëÉÏ´Î·¢ËÍËÙ¶È±ä»¯Ð¡ÓÚ5£¬  Ôò¼ä¸ô(24-5+1)*0.5s=10s·¢ËÍÊý¾Ý					
		{
			senddata = 1;				
			timer[4] = 24;				
		}
	}		 
	//¶¯Ì¬±¨¾¯Ê±¼ä¼ÆËã	 
	if(avg_value>10&&avg_value<=200&&count_present>6) 
    {
        warntime = (uint8_t)(((60*1.0/avg_value)*1.0)*30);	  //¶¯Ì¬±¨¾¯Ê±¼ä¼ÆËã         //Ã¿ºÁÉý15µÎ
    } 
	else if(avg_value<=10&&avg_value!=0&&count_present>6)
	{	
        warntime = 180;	   //1800/10=180   180*0.5=90s
    }
	else if(avg_value>200)
    {
		warntime = 7;     //7*0.5=3.5s	
    }        
	if(count_present<5)  //ÒºµÎ¼ÆÊý
	{
        senddata=1;
    }				
	if(count_present<2&&first_count_status==1)  //ÉÏµçÊ×µÎ ¶ªÆúÊý¾Ý Ç¿ÖÆÎÞÏß²»·¢ËÍ
	{
		count_present=0;
		first_count_status=0;  //³õÊ¼ÖµÎª1 
		senddata=0;
		iMaCount=0;//Æ½¾ùÊý¾Ý¼ÆÊýÇåÁã
	}
	if(senddata==1)		  	
	{
		senddata = 0;					
		iTempMIN = value2;	//Ã¿·¢ËÍÒ»´ÎÊý¾Ý£¬°Ñ·¢ËÍËÙ¶È¼ÇÂ¼ÎªÉÏ´Î·¢ËÍËÙ¶È			
		if(RFOFF == 0)
        {            
            rf_send_sta=0xaa;	
        }            
		else 
		{
            rf_send_sta=0x55; 
        }            
	}
						
	GPIO_SetInactive(LED_PORT, LED_PIN); 
}

//ÎÞÏßÊý¾Ý·¢ËÍ
void BleSendData(void)
{
    if(cIptWarning == 0x33||alarm_signal==1)
	{		
		if(LEDCount==21) 				 //21*0.5=10.5s		
		{ 	
			rf_send_sta=0xaa;		     
			LEDCount=0;
		}
	}						
	if(timer[3]==2 && RFOFF==0)		 		//2*0.5=1s
	{
		rf_send_sta = 0xaa;					   	
	}			
	if((rf_send_sta==0xaa||ResetFlg==1)&&SetFlag != 1)		// 	Õý³£ÊäÒº»òÕß¸´Î» ²¢ÇÒzigbeeÎ´ÉèÖÃ			
	{
		ZigBee_Wake();
		delay_us(800);	
        if(iSecCount >= warntime)	 				
		{			  	   					 
			for(i = 0;i<5;i++)		
			{
				maBuffer[i]=0;					
			}					       	
		}
		if(count_present>60) 	
        {            
			GPIO_SetActive(LED_PORT, LED_PIN);
        }            

		M10frm->Zigbee_ADD = 0x031B;				//cFunctionType;
		M10frm->deviceType = cFunctionType;  	//Êý¾ÝÀàÐÍ  0xAA or 0xBB
		M10frm->RFCount = txCount;	  //ÎÞÏß·¢ËÍµÄ´ÎÊý
		M10frm->cnt = count_present;  //ÒºµÎ×ÜµÎÊý
		M10frm->Data_length = 0x14;	  //Êý¾Ý³¤¶È
		M10frm->rchPower = uBatRemainPercentMA;		//  µç³ØµçÁ¿
		M10frm->cSpeedDrop = value2;				//ÒºµÎËÙ¶È							
		M10frm->cAlarm = cIptWarning;   //±¨¾¯ÀàÐÍ

		M10frm->ID0 = FLASHid[0];	 //Î¨Ò»ID
		M10frm->ID1 = FLASHid[1];
		M10frm->ID2 = FLASHid[2];
		M10frm->ID3 = FLASHid[3];
		M10frm->ID4 = FLASHid[4];
		M10frm->ID5 = FLASHid[5];
		M10frm->ID6 = FLASHid[6];
		M10frm->ID7 = FLASHid[7];
							
		M10frm->verify = BCC_verify();   //BCCÐ£Ñé

		delay_us(60);					
		for(i=0;i<20;i++)
		{
			uart_send_byte(txBuffer[i]);
		}				
		ZigbFlg =1;
		timer[5] = 6;					
		if(M10_Master_conready==1)        //À¶ÑÀÒÑÁ¬½Ó
		{ 		
			attmdb_att_set_value(STREAMDATAD_DIR_VAL_HANDLE(0),20, (uint8_t*)&(txBuffer[0])); //
			prf_server_send_event((prf_env_struct *)&(streamdatad_env.con_info), false, STREAMDATAD_DIR_VAL_HANDLE(0));
		}

		txCount=txCount+1;			
		if(txCount>0xfffe)
		{
            txCount=1;
        }
		halt =0;
			
		RFOFF = 2;	//·¢ËÍ¼ä¸ô¼ÆÊ±  2*0.5s=1s
		rf_send_sta=0x55;
		ResetFlg = 0;				
		GPIO_SetInactive(LED_PORT, LED_PIN); 
	}	
    if(ZigbFlg == 1&&SetFlag != 1)
	{
		if(timer[5]<5)
        {		
			ZigBee_Sleep();	
             ZigbFlg=0;
		}		
	}	
}
/**
 ****************************************************************************************
 * @brief BLE main function.
 *
 * This function is called right after the booting process has completed.
 ****************************************************************************************
 */
int main_func(void) __attribute__((noreturn));

int main_func(void)
{		
    sleep_mode_t sleep_mode; // keep at system RAM. On each while loop it will get a new value.    
    sys_startup_flag = true; //ÏµÍ³Æô¶¯±êÖ¾	
    InitPlatform();     //Æ½Ì¨³õÊ¼»¯        		
	InitFlash();	    //Flash³õÊ¼»¯			   
    InitBLE();          //À¶ÑÀ³õÊ¼»¯
    InitSleepMode();    //ÐÝÃßÄ£Ê½³õÊ¼»¯  
    InitAppliaction();  //Ó¦ÓÃ³õÊ¼»¯
	InitWatchdog();     //¿´ÃÅ¹·³õÊ¼»¯  	
    /*
     ************************************************************************************
     * Main loop
     ************************************************************************************
     */
    while(1)
    {   	     
        if(SetAndWaitInterrupts(&sleep_mode) == 1)    //ÉèÖÃ²¢µÈ´ýÖÐ¶Ï
        {
            continue;
        }
        CheckStatus();                                //¼ì²â¼°¸üÐÂ×´Ì¬
        if(DropINT==1)                                //¼ì²âµ½ÒºµÎ
		{			
			Drip();	
		}	
        CheckAutoPowerOffStatus();                    //¼ì²âÊÇ·ñÐèÒª×Ô¶¯¹Ø»ú				
		if(clamp ==1  && MotorOn==1)                  //¿ª¸Ç²¢ÇÒµç»úÎ´ËøËÀ
		{							   							 
			OpenClamStatus();	                      //¿ª¸Ç×´Ì¬±¨¾¯¼°×Ô¶¯¹Ø»ú						 					
		}
		else                                          
		{							
			PreAlarm();		                          //Ô¤±¨¾¯  						
		}				
		BleSendData();                                //ÎÞÏßÊý¾Ý·¢ËÍ	
    }
}

/// @} DRIVERS
uint8_t BCC_verify(void)
{
	uint8_t i=3,Data=0,CheckData[20];  
    for(i=3;i<=19;i++)
    {
		CheckData[i] = txBuffer[i]; 
        Data^=CheckData[i];		  	
    }   
	return Data;    
}	
