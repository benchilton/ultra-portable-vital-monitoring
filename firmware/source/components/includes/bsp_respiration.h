/*
 *  @name          bsp_respiration.h
 *  @date          17/10/2022
 *  @author        Benjamin Chilton , Thomas Bain
 *  @university_id bdc1g19 , trb1g19
 *
 *
 */

#ifndef ULTRA_PORTABLE_VITAL_MONITORING_BSP_RESPIRATION_H
#define ULTRA_PORTABLE_VITAL_MONITORING_BSP_RESPIRATION_H

/****************************** - Library Includes - *******************************/
/******************************** - User Includes - ********************************/

#include "zephyr_typedefs.h"

/*********************************** - Defines - ***********************************/
#define BSP_RESPIRATION_READINGS_TO_BUFFER 256U
/************************************ - Enums - ************************************/
/********************************** - Typedefs - ***********************************/
/********************************* - Structures - **********************************/
/**************************** - Function Prototypes - ******************************/

void bsp_respiration_signal_process( zephyr_k_work_t* calling_work );

/****************************** - Global Variables - *******************************/

#endif //ULTRA_PORTABLE_VITAL_MONITORING_BSP_RESPIRATION_H
