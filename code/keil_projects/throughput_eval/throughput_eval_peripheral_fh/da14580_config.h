/**
 ****************************************************************************************
 *
 * @file da14580_config.h
 *
 * @brief Compile configuration file.
 *
 * Copyright (C) 2014. Dialog Semiconductor Ltd, unpublished work. This computer 
 * program includes Confidential, Proprietary Information and is a Trade Secret of 
 * Dialog Semiconductor Ltd.  All use, disclosure, and/or reproduction is prohibited 
 * unless authorized in writing. All Rights Reserved.
 *
 * <bluetooth.support@diasemi.com> and contributors.
 *
 ****************************************************************************************
 */

#ifndef DA14580_CONFIG_H_
#define DA14580_CONFIG_H_

#include "da14580_stack_config.h"

/////////////////////////////////////////////////////////////
/*FullEmbedded - FullHosted*/
#define CFG_APP  
/////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////
/*Profiles*/


#define CFG_PRF_STREAMDATAD

#undef CFG_PRF_BASS            
#define CFG_PRF_DISS            1
#undef CFG_PRF_PXPR            
#undef CFG_PRF_FMPL        
#define CFG_PRF_SPOTAR
#ifdef CFG_PRF_SPOTAR
    #define SPOTAR_PATCH_AREA   1   // Placed in the RetRAM when SPOTAR_PATCH_AREA is 0 and in SYSRAM when 1
    #define CFG_APP_SPOTAR      1
#endif
/////////////////////////////////////////////////////////////

/*Misc*/

/* Application Definition. Project's configuration */
#define CFG_APP_THROUGHPUT_PERIPHERAL

#undef CFG_INTEGRATED_HOST_GTL 
/*Wake up the external processor*/
#undef CFG_WKUP_EXT_PROCESSOR

/* NVDS structure */
#define CFG_NVDS

/* BLE Security  */
#define CFG_APP_SEC

/* Coarse calibration */
#define CFG_LUT_PATCH

/*Watchdog*/
#undef CFG_WDOG 

/*Sleep modes*/
#undef CFG_EXT_SLEEP  
#undef CFG_DEEP_SLEEP  

/*Maximum user connections*/
#define BLE_CONNECTION_MAX_USER 1

/*Build for OTP or JTAG*/
#define DEVELOPMENT_DEBUG     1       //0: code at OTP, 1: code via JTAG

/*Low power clock selection*/
#define CFG_LP_CLK              0x00    //0x00: XTAL32, 0xAA: RCX20, 0xFF: Select from OTP Header

/*Fab Calibration - Must be defined for calibrated devices*/
#define CFG_CALIBRATED_AT_FAB  

/* Use a default trim value for XTAL16M if a trim value has not been programmed in OTP */
#undef CFG_USE_DEFAULT_XTAL16M_TRIM_VALUE_IF_NOT_CALIBRATED


/*Stream data queue */
#undef CFG_STREAMDATA_QUEUE
#undef METRICS

/*
 * Scatterfile: Memory maps
 */
#if defined(CFG_EXT_SLEEP) || !defined(CFG_DEEP_SLEEP)
#define REINIT_DESCRIPT_BUF     0       //0: keep in RetRAM, 1: re-init is required (set to 0 when Extended Sleep is used)
#define USE_MEMORY_MAP          EXT_SLEEP_SETUP

#else
/*The following are valid for BLE_CONNECTION_MAX_USER == 1. Please use the results of the excel tool! */
#define REINIT_DESCRIPT_BUF     0       //0: keep in RetRAM, 1: re-init is required (set to 0 when Extended Sleep is used)
#define USE_MEMORY_MAP          DEEP_SLEEP_SETUP
#define DB_HEAP_SZ              1024
#define ENV_HEAP_SZ             328
#define MSG_HEAP_SZ             1312
#define NON_RET_HEAP_SZ         1024
#endif

/*
 * HW configuration
 */
//#define HW_CONFIG_BASIC_DK  // Basic DK (DA14580DEVKT-B)
//#define HW_CONFIG_PRO_DK    // Pro DK (DA14580DEVKT-P)

#endif // DA14580_CONFIG_H_
