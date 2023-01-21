/*
 *  @name          bsp_thermometer.c
 *  @date          17/10/22
 *  @author        Benjamin Chilton, Harry Snell
 *  @university_id bdc1g19, hss1g19
 *
 *
 */

#ifndef ULTRA_PORTABLE_VITAL_MONITORING_BSP_THERMOMETER_H
#define ULTRA_PORTABLE_VITAL_MONITORING_BSP_THERMOMETER_H

/****************************** - Library Includes - *******************************/
/******************************** - User Includes - ********************************/

#include "zephyr_typedefs.h"

/*********************************** - Defines - ***********************************/
#define BSP_THERMOMETER_READINGS_TO_BUFFER 16U
#define BSP_THERMOMETER_GAIN_CORRECTION 0
#define BSP_THERMOMETER_OFFSETT_CORRECTION 0
/************************************ - Enums - ************************************/
/********************************** - Typedefs - ***********************************/
/********************************* - Structures - **********************************/
/**************************** - Function Prototypes - ******************************/

void bsp_thermometer_signal_process(zephyr_k_work_t* calling_work );

/****************************** - Global Variables - *******************************/

#endif //ULTRA_PORTABLE_VITAL_MONITORING_BSP_THERMOMETER_H
