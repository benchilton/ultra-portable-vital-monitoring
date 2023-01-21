/*
 *  @name          vital_bluetooth.c
 *  @date          18/11/2022
 *  @author        Benjamin Chilton
 *  @university_id bdc1g19
 *
 *
 */

#ifndef ULTRA_PORTABLE_VITAL_MONITORING_BLUETOOTH_H
#define ULTRA_PORTABLE_VITAL_MONITORING_BLUETOOTH_H

/****************************** - Library Includes - *******************************/
#include <sys/ring_buffer.h>
/******************************** - User Includes - ********************************/
#include "zephyr_typedefs.h"
/*********************************** - Defines - ***********************************/
#define VITAL_GATT_DATA_BUFFER_SIZE 16
/************************************ - Enums - ************************************/
/********************************** - Typedefs - ***********************************/
/********************************* - Structures - **********************************/
typedef struct
{
    zephyr_ring_buf_t ring_buffer;
    zephyr_k_mutex_t  lock;
    uint8_t           data[VITAL_GATT_DATA_BUFFER_SIZE];
    uint8_t*          data_ptr;
    uint16_t          len;
} vital_gatt_service_data_t;
/**************************** - Function Prototypes - ******************************/
zephyr_err_t vital_bluetooth_init(void);

/****************************** - Global Variables - *******************************/

#endif //ULTRA_PORTABLE_VITAL_MONITORING_BLUETOOTH_H
