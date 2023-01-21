/*
 *  @name          bsp.h
 *  @date          17/11/22
 *  @author        Benjamin Chilton
 *  @university_id bdc1g19
 *
 *
 */

#ifndef ULTRA_PORTABLE_VITAL_MONITORING_BSP_H
#define ULTRA_PORTABLE_VITAL_MONITORING_BSP_H

/****************************** - Library Includes - *******************************/
/******************************** - User Includes - ********************************/
#include "bsp_ad469x.h"
#include "zephyr_typedefs.h"
#include "vital_defines.h"
/*********************************** - Defines - ***********************************/

#define BSP_APP_QUEUE_SIZE 2U

/************************************ - Enums - ************************************/
/********************************** - Typedefs - ***********************************/
/********************************* - Structures - **********************************/
typedef struct {
    float voltage;
    float temperature;
    uint8_t rsoc;
    uint8_t health;
} bsp_battery_status_t;

/**************************** - Function Prototypes - ******************************/

zephyr_err_t bsp_init();

zephyr_err_t bsp_get_data( vitals_vital_sign_data_t* vitals , bsp_battery_status_t* battery );
zephyr_err_t bsp_print_data( vitals_vital_sign_data_t vitals , bsp_battery_status_t battery );

void bsp_signal_work();
void bsp_submit_work();

void bsp_pause_sampling();
void bsp_resume_sampling();

/****************************** - Global Variables - *******************************/

#endif //ULTRA_PORTABLE_VITAL_MONITORING_BSP_H

