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
extern uint16_t ADCcount;

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

extern uint16_t value0,DropINT,count_present;
volatile uint8_t Activeflag = 0,alarm_signal=0,Power=1,MotorOn=1,Motor = 1,Retry = 0,clamp=0,Sleep=0,keypress=0;  //valuecount=0,,motorflag=0,
volatile uint8_t senddata = 0,speed_five_status=0,speed_right_status=0,first_count_status=1,first55_flag=2,halt=0,ResetFlg = 0;
uint8_t iTempMIN=0,iTempMAX=255, txBuffer[32],Key1 = 0,Keylong=0,SwordClose=0,rf_send_sta=0x55,RFOFF=0,Keylonglong=0,KeylongCount=0,SetFlag = 0,ZigbFlg=0;	
unsigned char cIptWarning = 00; 	//0-正常輸液  55-輸液完畢  	DD-意外中止
unsigned char cFunctionType = FUNC_DROP,avg_value=1;
extern uint16_t iSecCount,iSecCnt2,iWaringSend,LEDCount,SystemCount,opencount;
extern uint8_t alarm_temp,alarm_sta,uBatRemainPercentMA;
extern uint8_t ResetCnt;
uint16_t txCount=0,BeSecTmp[2] = {0,0};
//extern uint8_t timer[10];
volatile	uint16_t lkeyDownCount=0;


#pragma pack (1)
typedef struct
{	
	unsigned short	Zigbee_ADD;	   //zigbee本地地址
 	unsigned char   deviceType;	   //数据类型
	
	unsigned short	RFCount;       //当前数据包	
//	unsigned short  PackLoad;	 //总包数
		unsigned short  cnt;           //统计值，当前是第几滴
	unsigned char   Data_length;	 //数据长度
	unsigned char   rchPower;      //电池剩余电量  rchPower%
	unsigned char   cSpeedDrop;    //滴速  单位 滴/分
	unsigned char   cAlarm;        //报警信号
//	unsigned char   RFCount;      //RF计数标志字
	unsigned char   ID0;     
	unsigned char   ID1;     
	unsigned char   ID2;     
	unsigned char   ID3;     
	unsigned char   ID4;     
	unsigned char   ID5;     
	unsigned char   ID6;
	unsigned char   ID7;

	unsigned char   verify;        //保留字		
}dropStutas_t;

 dropStutas_t* M10frm = ((dropStutas_t*)&txBuffer[0]);

uint8_t BCC_verify(void);
void KeyScan(void)
{	

		if(!GPIO_GetPinStatus(BUTTON_PORT, BUTTON1_PIN))  // 检测到复位按下
			{
				lkeyDownCount++;
				keypress =1;
			}
		else
			{
				 keypress =0;
			}
		if(lkeyDownCount < 3000 && lkeyDownCount!=0 && keypress ==0)  //短按
			{ 
				Key1 = 1;   //短按标志位
				lkeyDownCount = 0;
				printf_string("\n\rjian ce dao duan an\n\r");
			}
	  if(lkeyDownCount > 3500) 
			 {
				Keylong = 1;    //长按标志位
			 lkeyDownCount = 0;
			 }
			
     //复位次数大于5			 
	   if(KeylongCount > 5) 		 //  && keypress ==0    
			 {
				Keylonglong = 1;
				KeylongCount = 0;
			 }		 
			 
}
void KeyDispose(void)
{
	uint8_t j=0;
	
	KeyScan();
									   
	if(Key1 == 1)     //检测到短按复位										 
		{	 						
			Key1 = 0;									
			if(MotorOn == 0) 			//电机压缩状态标识	，电机压缩后MotorOn =0	
				{	
					first55_flag=2;//短按解锁后重置flag标志位 	
					GPIO_SetActive(LED_PORT, LED_PIN);//					LEDON; 						
//					GPIO_SetInactive(GPIO_PORT_0, GPIO_PIN_0);		
//					delay_us(2); 			
//					GPIO_SetActive(GPIO_PORT_1, GPIO_PIN_0); 		 
//					delay_ms(3300);
//					GPIO_SetInactive(GPIO_PORT_1, GPIO_PIN_0); 	 		
//					for(j=0;j<100;j++)
//					{
//						GPIO_SetActive(GPIO_PORT_1, GPIO_PIN_0);	
//						delay_us(20);
//						GPIO_SetInactive(GPIO_PORT_1, GPIO_PIN_0);
//						delay_us(20);
//					}
//					GPIO_SetActive(GPIO_PORT_0, GPIO_PIN_0);	 
					ResetCnt=0;
					
					printf_string("\n duan an  MOTOR_RESET   ");
					//app_timer_set(MOTOR_RESET, TASK_APP, 50);                //电机释放
                    motor_reset_handleEx();
					GPIO_SetActive(GPIO_PORT_2, GPIO_PIN_7);	   //开5V电�
					delay_us(20);										
					SetBeep();		  //蜂鸣器响 1声				
		      GPIO_SetInactive(GPIO_PORT_2, GPIO_PIN_7);   //关5V电源 	
					iSecCount=0;							
					iSecCnt2 = 1;		//关机计时复位					
					LEDCount=0;	  							
					timer[3] = 3;						    
					Motor = 0;																						 			
					MotorOn = 1;   							
					SwordClose =0;									

					BeSecTmp[0] = 0;
					BeSecTmp[1] = 0;
						
					GPIO_SetInactive(LED_PORT, LED_PIN);//LEDOFF;
				}
	}
	
	
	if(Keylong==1)		 // 检测到长按复位		
		{
			Keylong = 0;								
			if(MotorOn == 1) 				//电机释放状态标识	，电机释放后MotorOn =1			
				{	
					printf_string("\n chang an,MotorOn == 1,dian ji shi fang zhuang tai  ");
					GPIO_SetActive(LED_PORT, LED_PIN);//LEDON;					   
//					Sleep = 0;				
					GPIO_SetActive(GPIO_PORT_2, GPIO_PIN_7);	   //开5V电�
					delay_us(20);										
					SetBeep();			//蜂鸣器响 第一声					
					delay_us(1200);							   	
					SetBeep();      //蜂鸣器响 第二声	
		      GPIO_SetInactive(GPIO_PORT_2, GPIO_PIN_7);   //关5V电源 					
					KeylongCount++;//记录复位次数
					SystemCount = 0;						
					count_present = 0; 						// 长按清零后，液滴总滴数为0	
					cIptWarning =0x00; 						// 长按清零后，报警类型为0x00		
					cFunctionType = FUNC_KEY;		  //按键复位0xBB，   长按清零后，数据类型为0xBB
					ResetFlg = 1;														
					
					BeSecTmp[0] = 0;
					BeSecTmp[1] = 0;								
					GPIO_SetInactive(LED_PORT, LED_PIN);//LEDOFF;
				}
			else   									//电机压缩状态下长按复位
				{   
					cIptWarning=0x00;
						SystemCount=0;
					GPIO_SetActive(LED_PORT, LED_PIN);//LEDON; 				  							
//					GPIO_SetInactive(GPIO_PORT_0, GPIO_PIN_0);		
//					delay_us(2); 			
//					GPIO_SetActive(GPIO_PORT_1, GPIO_PIN_0); 		 
//					delay_ms(3300);
//					GPIO_SetInactive(GPIO_PORT_1, GPIO_PIN_0); 	 		
//					
//					for(j=0;j<100;j++)
//					{
//						GPIO_SetActive(GPIO_PORT_1, GPIO_PIN_0);	
//						delay_us(20);
//						GPIO_SetInactive(GPIO_PORT_1, GPIO_PIN_0);
//						delay_us(20);
//					}
//					GPIO_SetActive(GPIO_PORT_0, GPIO_PIN_0);
					ResetCnt=0;
					//app_timer_set(MOTOR_RESET, TASK_APP, 50);		// 电机复位
                    motor_reset_handleEx();
					GPIO_SetActive(GPIO_PORT_2, GPIO_PIN_7);	   //开5V电�
					delay_us(20);					
					SetBeep();				//蜂鸣器响 第1声					
					delay_us(800);
					delay_us(400);
					SetBeep();			  //蜂鸣器响 第2声	
		      GPIO_SetInactive(GPIO_PORT_2, GPIO_PIN_7);   //关5V电源 					
						first55_flag=2;//长按解锁后重置flag标志位
					iSecCount=0;							
					LEDCount=0;								
					count_present = 0;	
										
					cFunctionType = FUNC_KEY;	 		 //按键复位0xBB												 										
					MotorOn = 1;   							
					SwordClose =0;		
//					rf_send_sta=0xaa;	
//           senddata = 1;		
					ResetFlg = 1;
					Motor = 0;									
					BeSecTmp[0] = 0;
					BeSecTmp[1] = 0;
					GPIO_SetInactive(LED_PORT, LED_PIN);//LEDOFF;
				}			
		}
			if(Keylonglong==1)  //复位次数大于5次
				{
					 Keylonglong=0;
					GPIO_SetActive(GPIO_PORT_2, GPIO_PIN_7);	   //开5V电�
						delay_us(20);										
						SetBeep();////蜂鸣器响 第1声							
					delay_us(560);							   	
					SetBeep(); ////蜂鸣器响 第2声
					delay_us(560);							   	
					SetBeep(); ////蜂鸣器响 第3声
				  GPIO_SetInactive(GPIO_PORT_2, GPIO_PIN_7);   //关5V电源 	
//					ResetFlg = 0;
					ZigBee_Wake();
          SetFlag = 1;   //zigbee设置
					
					delay_us(560);
          GPIO_SetActive(LED_PORT, LED_PIN);//LEDON; 
//	for(j=0;j<5;j++)
//			{
//			uart_send_byte(LacalSetting[j]);
//			}
					
//				ZigBee_Setting();
					
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
	uint8_t i;//,j
	uint16_t value1 = 0,value = 0,value2 = 0,warntime=15; 
	uint8_t maBuffer[20],iMaCount = 0;	
	uint64_t unique_id;
	uint8_t FLASHid[8];	//,BleADD[6]
//	uint32_t jedec_id;

	
    sleep_mode_t sleep_mode; // keep at system RAM. On each while loop it will get a new value. 
    
    sys_startup_flag = true;
	

    /*
     ************************************************************************************
     * Platform initialization
     ************************************************************************************
     */
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
    
    set_system_clocks();    //系统时钟初始化
    GPIO_init();             //GPIO初始化
    periph_init();           //串口  SPI  FLASH 初始化
/*****************                 ***********************/ 
						
//	   GPIO_SetActive(LED_PORT, LED_PIN);  //LED
     GPIO_SetActive(GPIO_PORT_2, GPIO_PIN_0);  //LED
              
		 Activeflag = 1;	
		 if(Activeflag==1)
		 { 		  
			 GPIO_SetActive(GPIO_PORT_1 , GPIO_PIN_3);  //power_ON  软件开系统电源
		 }
		 
			 GPIO_SetActive(GPIO_PORT_2, GPIO_PIN_7);	   //开5V电源		
					 		 
//				jedec_id = spi_read_flash_jedec_id();
//				printf_string("\n\rSPI flash JEDEC ID is ");
//				printf_byte((jedec_id>>16)&0xFF);
//				printf_byte((jedec_id>>8)&0xFF);
//				printf_byte((jedec_id)&0xFF);
 
				unique_id = spi_read_flash_unique_id();					
				printf_string("\nSPI flash Unique ID Number is ");
        printf_byte(((unique_id>>32)>>24)&0xFF);
        printf_byte(((unique_id>>32)>>16)&0xFF);
        printf_byte(((unique_id>>32)>>8)&0xFF);
        printf_byte((unique_id>>32)&0xFF);
        printf_byte((unique_id>>24)&0xFF);
        printf_byte((unique_id>>16)&0xFF);
        printf_byte((unique_id>>8)&0xFF);
        printf_byte((unique_id)&0xFF);
			FLASHid[0] = ((unique_id>>32)>>24)&0xFF;    
			FLASHid[1] = ((unique_id>>32)>>16)&0xFF;  
			FLASHid[2] = ((unique_id>>32)>>8)&0xFF;  
			FLASHid[3] = (unique_id>>32)&0xFF;         
			FLASHid[4] = (unique_id>>24)&0xFF;         
			FLASHid[5] = (unique_id>>16)&0xFF;         
			FLASHid[6] = (unique_id>>8)&0xFF;          
			FLASHid[7] = (unique_id)&0xFF;             

			spi_flash_read_data(nvds_data_storage.NVDS_TAG_BD_ADDRESS,0xf000,6);   //此处将M10 MAC地址从flash读出

		  GPIO_ConfigurePin(GPIO_PORT_0, GPIO_PIN_0, OUTPUT, PID_GPIO, false); 	
		  RESERVE_GPIO(GPIO, GPIO_PORT_0, GPIO_PIN_0, PID_GPIO);
	    delay_us(10);

      

			
//			GPIO_SetInactive(GPIO_PORT_0, GPIO_PIN_0);
//		  GPIO_SetActive(GPIO_PORT_1, GPIO_PIN_0);	
//			 delay_ms(3300);    		
//			GPIO_SetInactive(GPIO_PORT_1 , GPIO_PIN_0);	
//		  GPIO_SetActive(GPIO_PORT_0, GPIO_PIN_0);	
//			
          
//			  开机上电响声 ’Bi‘长响声
					SetBeep();      //蜂鸣器响 第1声
					delay_us(12);
					SetBeep();      //蜂鸣器响 第2声
					delay_us(12); 
					SetBeep();      //蜂鸣器响 第3声
					delay_us(12);
					SetBeep();      //蜂鸣器响 第4声
					delay_us(12);
					SetBeep();      //蜂鸣器响 第5声
					delay_us(12); 
					SetBeep();      //蜂鸣器响 第6声
					delay_us(12);
					SetBeep();      //蜂鸣器响 第7声
				  ZigBee_Sleep();	//ZigBee模块进入休眠					
		//		GPIO_SetInactive(LED_PORT, LED_PIN); 
				  GPIO_SetInactive(GPIO_PORT_2, GPIO_PIN_0);   //LED
				  GPIO_SetInactive(GPIO_PORT_2, GPIO_PIN_7);   //关5V电源 
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

    /*
     ************************************************************************************
     * BLE initialization
     ************************************************************************************
     */
     
    init_pwr_and_clk_ble(); 
    //diagnostic();

  //  rf_init(&rwip_rf);
  //  SetBits32(BLE_RADIOCNTL1_REG, XRFSEL, 3);

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

    /*
     ************************************************************************************
     * Sleep mode initializations (especially for full embedded)
     ************************************************************************************
     */
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
    
    /*
     ************************************************************************************
     * Application initializations
     ************************************************************************************
     */
     
#if (BLE_APP_PRESENT)    
    {
        app_init();         // Initialize APP
    }
#endif /* #if (BLE_APP_PRESENT) */
  
    
    /*
    ************************************************************************************
    * Main loop
    ************************************************************************************
    */
    lld_sleep_init_func();
    
    SetWord16(TRIM_CTRL_REG, 0xA2);
    SetBits16(CLK_16M_REG, XTAL16_CUR_SET, 0x5);
    
//    // Gives 1dB higher sensitivity - UNTESTED
//    if (GetBits16(ANA_STATUS_REG, BOOST_SELECTED) == 0x1) 
//    { 
//        // Boost-mode
//        SetBits16(DCDC_CTRL2_REG, DCDC_CUR_LIM, 0x8); // 80mA
//    }
//    else 
//    { 
//        // Buck-mode
//        SetBits16(DCDC_CTRL2_REG, DCDC_CUR_LIM, 0x4); // 40mA
//    }
    
// Now enable the TX_EN/RX_EN interrupts, depending on the RF mode of operation (PLL-LUT and MGC_KMODALPHA combinations)

   enable_rf_diag_irq(RF_DIAG_IRQ_MODE_RXTX); 

#if BLE_APP_SPOTAR
    //app_spotar_exec_patch();
#endif
    

    if( (app_get_sleep_mode() == 2) || (app_get_sleep_mode() == 1) )
    {
         SetWord16(SET_FREEZE_REG, FRZ_WDOG);            // Stop WDOG until debugger is removed
         while ((GetWord16(SYS_STAT_REG) & DBG_IS_UP) == DBG_IS_UP) {}; 
         SetBits16(SYS_CTRL_REG, DEBUGGER_ENABLE, 0);    // close debugger
    }	
	
		
    /*
     ************************************************************************************
     * Watchdog
     ************************************************************************************
     */
#if (USE_WDOG)
    SetWord16(WATCHDOG_REG, 0xC8);          // 200 * 10.24ms active time for initialization!
    SetWord16(RESET_FREEZE_REG, FRZ_WDOG);  // Start WDOG
#endif

    
#if (STREAMDATA_QUEUE)
      stream_fifo_init ();
#endif    
    
		
		//app_timer_set(MOTOR_RESET, TASK_APP, 1);  //上电初始化，松开
    motor_reset_handleEx();
	  delay_us(1000);
    app_timer_set(ADC_TIMER, TASK_APP, 20);  //检测电量
		
    /*
     ************************************************************************************
     * Main loop
     ************************************************************************************
     */
		 
		
    while(1)
    {   
        if(ADCcount == 600) //5min获取一次电量
        {
           app_timer_set(ADC_TIMER, TASK_APP, 20);  //检测电量
           ADCcount = 0;
        }
			
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
					continue; // so that rwip_schedule() is called again
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
			continue; // so that rwip_schedule() is called again
#endif

#if (STREAMDATA_QUEUE)        
        if (stream_queue_more_data( ))
            continue;
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
//            if (lp_clk_sel == LP_CLK_RCX20)
//                set_sleep_delay();
        
            // if app has turned sleep off, rwip_sleep() will act accordingly
            // time from rwip_sleep() to WFI() must be kept as short as possible!
            sleep_mode = rwip_sleep();
        
            // BLE is sleeping ==> app defines the mode
            if (sleep_mode == mode_sleeping) 
						{
                if (sleep_env.slp_state == ARCH_EXT_SLEEP_ON) 
								{
                    sleep_mode = mode_ext_sleep;
                } 
								else 
								{
                    sleep_mode = mode_deep_sleep;
                }
            }
            
            if (sleep_mode == mode_ext_sleep || sleep_mode == mode_deep_sleep) 
            {
                SetBits16(PMU_CTRL_REG, RADIO_SLEEP, 1); // turn off radio
                
                if (jump_table_struct[nb_links_user] > 1)
                {
                    if( (sleep_mode == mode_deep_sleep) && func_check_mem() && test_rxdone() && ke_mem_is_empty(KE_MEM_NON_RETENTION) )
                    {
                        func_check_mem_flag = 2;//true;
                    }
                    else
                        sleep_mode = mode_ext_sleep;
                }
                else
                {
                    if( (sleep_mode == mode_deep_sleep) && ke_mem_is_empty(KE_MEM_NON_RETENTION) )
                    {
                        func_check_mem_flag = 1;//true;
                    }
                    else
                        sleep_mode = mode_ext_sleep;
                }
                
#if (BLE_APP_PRESENT)
                // hook for app specific tasks when preparing sleeping
                app_sleep_prepare_proc(&sleep_mode);
#endif
                
                
                if (sleep_mode == mode_ext_sleep || sleep_mode == mode_deep_sleep)
                {
                    SCB->SCR |= 1<<2; // enable sleepdeep mode bit in System Control Register (SCR[2]=SLEEPDEEP)
                    
                    SetBits16(SYS_CTRL_REG, PAD_LATCH_EN, 0);           // activate PAD latches
                    SetBits16(PMU_CTRL_REG, PERIPH_SLEEP, 1);           // turn off peripheral power domain
                    if (sleep_mode == mode_ext_sleep)	{
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
                app_sleep_entry_proc(&sleep_mode);
#endif

#if ((EXTERNAL_WAKEUP) && (!BLE_APP_PRESENT)) // external wake up, only in external processor designs
                ext_wakeup_enable(EXTERNAL_WAKEUP_GPIO_PORT, EXTERNAL_WAKEUP_GPIO_PIN, EXTERNAL_WAKEUP_GPIO_POLARITY);
#endif
                
                WFI();

#if (BLE_APP_PRESENT)
                // hook for app specific tasks just after waking up
                app_sleep_exit_proc(sleep_mode);
#endif

#if ((EXTERNAL_WAKEUP) && (!BLE_APP_PRESENT)) // external wake up, only in external processor designs
                // Disable external wakeup interrupt
                ext_wakeup_disable();
#endif

                // reset SCR[2]=SLEEPDEEP bit else the mode=idle WFI will cause a deep sleep 
                // instead of a processor halt
                SCB->SCR &= ~(1<<2);
            }
            else if (sleep_mode == mode_idle) 
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

		 
//// 软件关机		 
				if(Power ==0) 			
				{
					GPIO_SetInactive(GPIO_PORT_1, GPIO_PIN_3);	 //POWER_ON  系统断电		
					GPIO_SetInactive(LED_PORT, LED_PIN);	 		
					Activeflag =0;			 					
				}
					
				
				KeyDispose();
				ZigBee_Setting();

/////////////////////////佛山8-29////////////////////////////		
/////开盖检测
			if(GPIO_GetPinStatus(GPIO_PORT_2, GPIO_PIN_5)==1)			 // P2_5  CLAMP控制口   开盖则进入下面函数
				{
					clamp=1;     //开盖标志位
					Power = 1;	 //总电源上电标志位
        					
				}
//               else
//               {
//                   clamp = 0;
//               }
			
///////				
//1， 软件关机情况 （合盖，小于+7滴，30秒后关机）
			if(clamp ==1 && (DropINT == 1||count_present<7))		   //开盖 并且 检测到液体液滴数小于7， 则软件关机  		 
				{					                                           //DropINT 检测到液滴标志
					  clamp = 0;    
						SystemCount=0;  //系统计时计数  0*0.5s
				}	
				
			if(count_present<7  && cIptWarning==0x00 && clamp!=1 )	   	
				{								
					if(SystemCount>=60&&SystemCount<=80)   //30s内检测不到液滴，关机前发送关机预警信号0XCC
					  {  
						cFunctionType = 0xcc;   //关机预警 0xCC
					  if(SystemCount%2==0)
							timer[3]=3;
//						rf_send_sta=0xaa;
					  }
					else	if(SystemCount>80)   //超过40s
					{
						SystemCount = 0;															      
						Power = 0;		  //系统断电关机  P1.3 =0
					}						
				}	
				
///////				
//数据类型判定  0xAA:正常输液   0xBB：按键复位			
			if(cFunctionType ==FUNC_KEY&&count_present<=3)		//复位信号持续3滴	cFunctionType:数据类型		FUNC_KEY=0xBB
						 alarm_signal=1;				
			else   
						 alarm_signal=0;		   //		等于零时判定 cFunctionType = FUNC_DROP;
			
			
///////				
//开盖输液		
			if(clamp ==1  && MotorOn==1)	//开盖， 电机释放状态 开盖后立即if判断
//			if(clamp ==1)
			{							   							 
					Motor = 1;							   	
//					Sleep = 0;
					iSecCount = 0;	                              //修正输液管脱离死区时间	

	////开盖输液（开盖，有液滴检测得到）		
          if(cIptWarning == 0xEE && count_present>6)			 //开盖输液报警    
					  {
							if(opencount%14==0)	//响声提示不忽略    每7S声音报警一次
							{
								GPIO_SetActive(GPIO_PORT_2, GPIO_PIN_7);	   //开5V电�
								delay_us(20);								
								SetBeep();      //蜂鸣器响 第1声
								GPIO_SetInactive(GPIO_PORT_2, GPIO_PIN_7);   //关5V电源 								
								opencount=1;
							}
					  }
	/////输液管脱离输液情况（开盖，无液滴检测）				
					  if(iSecCnt2%20==0)//关机计时每10s进入一次	，每检测到一滴液体iSecCnt2=1
           	{						        																			
							cIptWarning = 0xDD;          //开盖，输液管脱离，报警类型 0xDD       	
							cFunctionType = FUNC_DROP;   //数据类型 0xAA 正常输液
							value2 =  avg_value;         //输液速度
							//if(iSecCnt2%2400==0)					  //输液脱离20min，一合盖就关机
  /////输液脱离20min，一合盖就关机
							if(iSecCnt2>2400)            //时间超过20min
							{
								Power = 0;							  //关机
							}										 								        															
						 if(RFOFF == 0)  halt = 1;	  //RFOFF发送完后置2			
						}	
						
						if(halt == 1 && RFOFF == 0)	   		
						{    
							if(iSecCnt2<2390)	 //开盖时间超过10s，每隔10s报警一次小于20min时  20min以外 5s冗余 
							{
								GPIO_SetActive(GPIO_PORT_2, GPIO_PIN_7);	   //开5V电�
								delay_us(20);																
							  SetBeep();		//开盖时间超过10s，每隔10s报警一次						
								GPIO_SetInactive(GPIO_PORT_2, GPIO_PIN_7);   //关5V电源 
//								halt = 0;				
							} 
						  count_present += (avg_value/6);  		 
							rf_send_sta=0xaa;			//允许蓝牙发送标识
						}		
                     if(iSecCount >=warntime  && count_present>6  )      //25滴时间	 1.5ML   //warntime值为 7~180之间的值
						 {	  						  	 
								if(!SwordClose)     //SwordClose=0时进入，SwordClose初始值为0， 仅在电机制动后置1
									{ 
										timer[3]=24;  			  //		初始化时间12s
										Motor = 0;	          //电机未动作标识  ，仅在电机制动锁死后置1                                                                                                                                                                                                                                                                                                                                         									
									} 
									
								if(Motor == 0)				//电机未动作标识 					
								 {
/////////
// 电机锁死					
                    printf_string("\n SUO SI app_timer_set(MOTOR_TIMER) ");										
                    //app_timer_set(MOTOR_TIMER, TASK_APP, 20);					// 电机锁死	
                                        motor_handleEx();
										Motor = 1;				//电机制动锁死标识					
										SwordClose =1;		
										LEDCount = 0;
								  } 
                              }                                 
			 }
			else     //合盖或者电机压缩状态
			{							
						////////////////动态锁死时间在8s以下的不进行预报警/////////////////////////////
						if(warntime>16 && iSecCount ==13 && count_present>6)	 //16*0.5=8s    13*0.5=6.5s  检测到液体iSecCount=0，然后每0.5s加1
							{	
								//warntime值为 7~180之间的值 iSecCount开盖电机复位状态清零，  检测到液滴清零
								
								GPIO_SetActive(GPIO_PORT_2, GPIO_PIN_7);	   //开5V电�
								delay_us(20);
								SetBeep();		  //蜂鸣器响 第1声						
								delay_us(600);							
								SetBeep();      //蜂鸣器响 第2声
								delay_us(800);
								delay_us(400);
								SetBeep();      //蜂鸣器响 第3声
								delay_us(600);
								SetBeep();      //蜂鸣器响 第4声    


								printf_string("\n Dong Tai Bao Jing 4 bibibibi ");	
								
								GPIO_SetInactive(GPIO_PORT_2, GPIO_PIN_7);   //关5V电源 
								cIptWarning = 0x33;
								rf_send_sta=0xaa;  //允许蓝牙发送标识
								LEDCount=0;	
								iSecCount = 14;  //开盖电机复位状态一直清零，  检测到液滴清零，另每隔0.5s加1
							}	
				
						if(iSecCount >=warntime  && count_present>6  )      //25滴时间	 1.5ML   //warntime值为 7~180之间的值
						 {	  						  	 
								if(!SwordClose)     //SwordClose=0时进入，SwordClose初始值为0， 仅在电机制动后置1
									{ 
										timer[3]=24;  			  //		初始化时间12s
										Motor = 0;	          //电机未动作标识  ，仅在电机制动锁死后置1                                                                                                                                                                                                                                                                                                                                         									
									} 
									
								if(Motor == 0)				//电机未动作标识 					
								  {
/////////
// 电机锁死					
                    printf_string("\n SUO SI app_timer_set(MOTOR_TIMER) ");										
                   // app_timer_set(MOTOR_TIMER, TASK_APP, 20);					// 电机锁死
                                    motor_handleEx();                                      
                                        delay_ms(100);
										Motor = 1;				//电机制动锁死标识					
										SwordClose =1;		
										LEDCount = 0;
								  }
									
								if(!BeSecTmp[0])       //BeSecTmp[0]=0时进入， 仅在有效复位或清零后置0，初始值为0
									{
										///BeSecTmp[0] = iSecCount+5;   // 修改warntime后,如使用动态BeSecTmp									
										// 在warntime很小的时候 DI DI DI DI 只会响一次
										BeSecTmp[0] = 20;//DI DI DI DI固定响三次		  BeSecTmp[0]=0仅在此处改变	，后一直保持为20	
										BeSecTmp[1] = BeSecTmp[0];     //BeSecTmp[1]=20仅在此处改变一次	，后每隔0.5s减1，直到减到2
									}
	
								
							 if(BeSecTmp[1]==BeSecTmp[0]-5||BeSecTmp[1]==BeSecTmp[0]-11||BeSecTmp[1]==BeSecTmp[0]-17)   
								{ 					
									GPIO_SetActive(GPIO_PORT_2, GPIO_PIN_7);	   //开5V电
									delay_us(20);
									beep();		
									GPIO_SetInactive(GPIO_PORT_2, GPIO_PIN_7);   //关5V电源 											
								}
								
						   if(LEDCount%2==0&&LEDCount<21)	  
								{
									timer[3]=3;	
								}		
								
								//first55_flag	初始值为2， 复位后为2
								if(( (iSecCount-BeSecTmp[0])%20==0 ||first55_flag!=0 ) && Key1!=1 )//10s一次||(BeSecTmp[1]%2==0&&BeSecTmp[1]>1) 
									{							  
										timer[3]=3;	 					
										cIptWarning = 0x55;	  			//报警类型复位为0x55
										cFunctionType = FUNC_DROP;  //数据类型复位为0xAA
										value2 =  0;	              //速度值复位为0
										halt = 1;						 	
										alarm_temp=0;
										first55_flag=first55_flag-1;				
									} 										
											
								 if((MotorOn==0)&&(iSecCnt2-warntime>2405))	  //电机锁紧状态， 超过20分钟后，软件关机
								 {
									Power = 0;
								 }									
								
						  }		
			}
///////
//检测到液滴
			  if(DropINT==1)              //检测到液滴标识， 每检测到一次液滴进入一次。
				{			
					printf_string("\n Detects a drop DropINT==1 ");
					DropINT = 0;
					SetFlag = 0;              //允许蓝牙发送标识	

					if(count_present<=60) 	//处理初始60滴				
					{  
					 GPIO_SetActive(LED_PORT, LED_PIN);  
					 delay_us(600);   //最开始60滴， 每滴LED灯亮一次
																						
					}
				 
					if(alarm_signal==0)		
					cFunctionType = FUNC_DROP;   //数据类型  0xAA：正常输液

					iSecCount = 0;	   			
					iSecCnt2 = 1; 				
					SystemCount = 0;			
					alarm_sta=0x11;			   	
					alarm_temp=0;				

					///////
					//合盖检测			
					if(GPIO_GetPinStatus(GPIO_PORT_2, GPIO_PIN_5)==0)  //  合上盖
						{
							cIptWarning = 0x00;   //  正常输液状态 0x00
							
						} 
					else   
						if(count_present>6)      //开盖并且液滴数大于6滴  
						{
							cIptWarning = 0xEE;    //  开盖输液状态 0xEE，判断条件 ，开盖并且液滴总数大于6滴
            }		
				
					 ///////
					 //液滴速度计算				
					value1=value0&0xffff;		   // value1 两液滴之间的间隔时间
					if(value1<470)		//470*0.5ms=	235ms   	
					{
						value=255;  //液体速度  255滴/分钟     1*60*1000/235=255  滴/分钟
					}
					else    // 液滴间隔时间大于235ms
					{
						value=120000/value1;	
					}									
					maBuffer[++iMaCount] = value;	   
					maBuffer[6]= maBuffer[iMaCount];	
					
					if(speed_five_status==1)//5+1滴
						 {speed_right_status=1;}	//	
						 
					if(iMaCount == 5)					     					   						
						{  
							iMaCount = 0;speed_five_status=1;  //已检5滴
						}
					if(speed_five_status==1)	//够5滴	
						{  
							value2 = (maBuffer[1]+maBuffer[2]+maBuffer[3]+maBuffer[4]+maBuffer[5])/5;
						}
					else  //不足5滴时
						{ 					//首次 在1至4滴时 求平均  
						 value2=(maBuffer[1]+maBuffer[2]+maBuffer[3]+maBuffer[4]+maBuffer[5])/iMaCount;
						} 
						
					avg_value = (maBuffer[1]+maBuffer[2]+maBuffer[3]+maBuffer[4]+maBuffer[5]-maBuffer[6])/4;
						
					if( value2!=0 && value2<=255 &&(value2-iTempMIN>=5||iTempMIN-value2>=5))			//abs(value2 - iTempMIN)>=10 &&Sleep!=1
						 { 
								senddata = 1;	// 滴速大于0小于255时，  并且与上次发送输液速度对比变化超过5，  则发送数据				
								timer[4] = 24;						    
						 }				
					else if(value2!=0&&value2<=255&&(value2-iTempMIN<5||iTempMIN-value2<5))		//小于200改成小于等于255
						 {
							if(timer[4] < 5)  // 滴速大于0小于255时，与上次发送速度变化小于5，  则间隔(24-5+1)*0.5s=10s发送数据					
								{
									senddata = 1;				
									timer[4] = 24;				
								}
						 }
					 
					///////
					//动态报警时间计算	 
					if(avg_value>10&&avg_value<=200&&count_present>6) 
						warntime = (uint8_t)(((60*1.0/avg_value)*1.0)*30);	  //动态报警时间计算         //每毫升15滴
					else if(avg_value<=10&&avg_value!=0&&count_present>6)
						warntime = 180;	   //1800/10=180   180*0.5=90s
					else if(avg_value>200)
						warntime = 7;     //7*0.5=3.5s
					
						clamp = 0;
					
							if(count_present<5)  //液滴计数
								{senddata=1;}
						
							//////////////////////////////////////////////////////
							if(count_present<2&&first_count_status==1)  //上电首滴 丢弃数据 强制无线不发送  first_count_status默认初始值为1
							{
								count_present=0;
								first_count_status=0;  //初始值为1 ，  first_count_status此变量实际未起效， 此if函数进入一次后first_count_status==1将不再满足
								senddata=0;
								iMaCount=0;//平均数据计数清零
							}
								

							if(senddata==1)		  	//液滴数小于5满足此条件
								{
									senddata = 0;					
									iTempMIN = value2;	//每发送一次数据，把发送速度记录为上次发送速度			
									if(RFOFF == 0)	 				
										rf_send_sta=0xaa;	 		//允许蓝牙发送标识， 
									else 
										rf_send_sta=0x55;    													
								}
								
							GPIO_SetInactive(LED_PORT, LED_PIN); 

				}	
				
				
				
				
			if(cIptWarning == 0x33||alarm_signal==1)  //预报警，或者液体小于3滴
			{		
				if(LEDCount==21) 				 //21*0.5=10.5s		
				{ 	
					rf_send_sta=0xaa;		    //允许蓝牙发送标识 
					LEDCount=0;
				}
			}						

		 if(timer[3]==2 && RFOFF==0)		 		//2*0.5=1s
		 {

		  rf_send_sta = 0xaa;		//允许蓝牙发送标识			   	
		 }
		 
		 
					
		 if((rf_send_sta==0xaa||ResetFlg==1)&&SetFlag != 1)		// 	ResetFlg在按键复位后为1，SetFlag初始值为0			
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
					GPIO_SetActive(LED_PORT, LED_PIN);  
                    

					M10frm->Zigbee_ADD = 0x031B;				//cFunctionType;
					M10frm->deviceType = cFunctionType;  	//数据类型  0xAA or 0xBB
					M10frm->RFCount = txCount;	  //无线发送的次数
					M10frm->cnt = count_present;  //液滴总滴数
					M10frm->Data_length = 0x14;	  //数据长度
					M10frm->rchPower = uBatRemainPercentMA;		//  电池电量
					M10frm->cSpeedDrop = value2;				//液滴速度							
					M10frm->cAlarm = cIptWarning;   //报警类型

					M10frm->ID0 = FLASHid[0];	 //唯一ID
					M10frm->ID1 = FLASHid[1];
					M10frm->ID2 = FLASHid[2];
					M10frm->ID3 = FLASHid[3];
					M10frm->ID4 = FLASHid[4];
					M10frm->ID5 = FLASHid[5];
					M10frm->ID6 = FLASHid[6];
					M10frm->ID7 = FLASHid[7];
					
					M10frm->verify = BCC_verify();   //BCC校验

				  delay_us(60);	
			
				  for(i=0;i<20;i++)
					{
//						printf_string("\n BLE send date, (txBuffer[i])=");
						delay_us(10);
						uart_send_byte(txBuffer[i]);
						delay_us(2);
					}
					
					ZigbFlg =1;
					timer[5] = 6;	
					
				 if(M10_Master_conready==1)
				 {

          printf_string("\n BLE send date 20, (uint8_t*)&(txBuffer[0]) ");					 
					attmdb_att_set_value(STREAMDATAD_DIR_VAL_HANDLE(0),20, (uint8_t*)&(txBuffer[0])); //
					prf_server_send_event((prf_env_struct *)&(streamdatad_env.con_info), false, STREAMDATAD_DIR_VAL_HANDLE(0));
					
					
				 }

				 txCount=txCount+1;			
				 if(txCount>0xfffe)
				 {txCount=1;}
				 halt =0;
	
				 RFOFF = 2;	//发送间隔计时  2*0.5s=1s
			 	 rf_send_sta=0x55;      
				 ResetFlg = 0;
//			 delay_ms(900);

					
					GPIO_SetInactive(LED_PORT, LED_PIN); 

		//			  ZigBee_Sleep();	

			  }	

//				
////    if(BLESnd == 1)		
////		  {
//			if(ZigbFlg == 1&&SetFlag != 1)
//			  {
//					if(timer[5]<5)
//           	{		
//				      ZigBee_Sleep();	ZigbFlg=0;
//				    }		
//				}
////			}
		
				Press_EXE();
				Release_EXE();
				
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
