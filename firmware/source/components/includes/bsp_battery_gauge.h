/*
 *  @name          bsp_battery_gauge.h
 *  @date          12/11/2022
 *  @author        Benjamin Chilton
 *  @university_id bdc1g19
 *
 *
 */

#ifndef ULTRA_PORTABLE_VITAL_MONITORING_BSP_BATTERY_GAUGE_H
#define ULTRA_PORTABLE_VITAL_MONITORING_BSP_BATTERY_GAUGE_H

/****************************** - Library Includes - *******************************/
/******************************** - User Includes - ********************************/
#include "zephyr_typedefs.h"
/*********************************** - Defines - ***********************************/
/************************************ - Enums - ************************************/
/********************************** - Typedefs - ***********************************/
/********************************* - Structures - **********************************/
/**************************** - Function Prototypes - ******************************/

zephyr_err_t bsp_battery_gauge_init();

void bsp_battery_gauge_get_parameters( float* voltage , float* temperature , uint8_t* rsoc , uint8_t* health );
void bsp_battery_gauge_refetch_parameters();

/****************************** - Global Variables - *******************************/

#endif //ULTRA_PORTABLE_VITAL_MONITORING_BSP_BATTERY_GAUGE_H
