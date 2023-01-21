/*
 *  @name          vital_gatt.h
 *  @date          21/11/2022
 *  @author        Benjamin Chilton
 *  @university_id bdc1g19
 *
 *
 */

#ifndef ULTRA_PORTABLE_VITAL_MONITORING_VITAL_GATT_H
#define ULTRA_PORTABLE_VITAL_MONITORING_VITAL_GATT_H

/****************************** - Library Includes - *******************************/
/******************************** - User Includes - ********************************/
#include "zephyr_typedefs.h"
#include "vital_defines.h"
/*********************************** - Defines - ***********************************/
/************************************ - Enums - ************************************/

typedef enum {
    VITAL_GATT_CONFIG_ID_SET_UPDATE_PERIOD = 0x01,
    VITAL_GATT_CONFIG_ID_SET_VITAL_VAL  = 0x02,
    VITAL_GATT_CONFIG_ID_ACKNOWLEGE_RX  = 0xff,
    VITAL_GATT_CONFIG_ID_MAX
} vital_gatt_config_ids_e ;

/********************************** - Typedefs - ***********************************/
/********************************* - Structures - **********************************/
/**************************** - Function Prototypes - ******************************/
zephyr_err_t vital_gatt_init();
zephyr_err_t vital_gatt_update_reading( vitals_vital_type_id_e vital_to_update );
zephyr_err_t vital_gatt_update_readings();
zephyr_err_t vital_gatt_add_to_queue(vitals_vital_sign_data_t vital_data);
/****************************** - Global Variables - *******************************/

#endif //ULTRA_PORTABLE_VITAL_MONITORING_VITAL_GATT_H
