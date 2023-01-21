/*
 *  @name          bsp_ppg.h
 *  @date          17/10/22
 *  @author        Benjamin Chilton
 *  @university_id bdc1g19
 *
 *
 */

#ifndef ULTRA_PORTABLE_VITAL_MONITORING_BSP_PPG_H
#define ULTRA_PORTABLE_VITAL_MONITORING_BSP_PPG_H

/****************************** - Library Includes - *******************************/
/******************************** - User Includes - ********************************/

#include "zephyr_typedefs.h"

/*********************************** - Defines - ***********************************/
#define BSP_HEART_RATE_READINGS_TO_BUFFER 1200U
/************************************ - Enums - ************************************/
/********************************** - Typedefs - ***********************************/
/********************************* - Structures - **********************************/
/**************************** - Function Prototypes - ******************************/

void bsp_heart_rate_signal_process( zephyr_k_work_t* calling_work );

/****************************** - Global Variables - *******************************/

#endif //ULTRA_PORTABLE_VITAL_MONITORING_BSP_PPG_H
