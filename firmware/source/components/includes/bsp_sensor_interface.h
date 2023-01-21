/*
 *  @name          bsp_sensor_interface.h
 *  @date          07/12/22
 *  @author        Benjamin Chilton
 *  @university_id bdc1g19
 *
 *
 */

#ifndef ULTRA_PORTABLE_VITAL_MONITORING_BSP_SENSOR_INTERFACE_H
#define ULTRA_PORTABLE_VITAL_MONITORING_BSP_SENSOR_INTERFACE_H

/****************************** - Library Includes - *******************************/
/******************************** - User Includes - ********************************/

#include "zephyr_typedefs.h"

#include "vital_defines.h"

/*********************************** - Defines - ***********************************/

#define BSP_SENSOR_INTERFACE_MAX_USED_SLOTS 3

/************************************ - Enums - ************************************/
/********************************** - Typedefs - ***********************************/

typedef struct {
    /*What type of vital is it?*/
    vitals_vital_type_id_e vital;
    /*Buffer to store the reading, size could be reduced to largest datatype used*/
    uint8_t data_buffer[16];
    uint8_t data_size;
} bsp_sensor_interface_data;

typedef struct{
    zephyr_k_work_t  work;
    k_work_handler_t work_handler;
    zephyr_k_mutex_t lock;

    const uint16_t    readings_to_buffer;
    /*in_data is structured to be a 2d array with <slots_used> rows and <readings_to_buffer> columns*/
    uint32_t**       in_data;
    uint16_t*        amount_filled;
    uint8_t          slots_used;
    uint8_t          slots_to_watch[BSP_SENSOR_INTERFACE_MAX_USED_SLOTS];

    const uint8_t    readings_to_skip;
    uint8_t          skipped_readings;

} bsp_signal_processing_work_t;

/********************************* - Structures - **********************************/
/**************************** - Function Prototypes - ******************************/

zephyr_err_t     bsp_sensor_interface_init();
zephyr_err_t     bsp_sensor_interface_publish_vital( vitals_vital_type_id_e vital , void* data , uint8_t sizeof_data );
zephyr_err_t     bsp_sensor_interface_receive_vital( void* data , k_timeout_t timeout );

void             bsp_sensor_interface_resume_sampling();
void             bsp_sensor_interface_pause_sampling();

zephyr_err_t     bsp_sensor_interface_power_on_afe();
zephyr_err_t     bsp_sensor_interface_power_off_afe();

zephyr_k_msgq_t* bsp_sensor_interface_get_data_queues_handle( void );
uint32_t         bsp_sensor_interface_get_sample_rate();

zephyr_err_t     bsp_sensor_interface_ppg_set( uint8_t led ,  uint8_t on_or_off );

/****************************** - Global Variables - *******************************/

#endif //ULTRA_PORTABLE_VITAL_MONITORING_BSP_SENSOR_INTERFACE_H
