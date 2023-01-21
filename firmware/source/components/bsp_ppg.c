/*
 *  @name          bsp_ppg.c
 *  @date          17/10/2022
 *  @author        Benjamin Chilton
 *  @university_id bdc1g19
 *
 *
 */

/****************************** - Library Includes - *******************************/

#include <sys/util.h>
#include <kernel.h>

/******************************** - User Includes - ********************************/

#include "bsp_sensor_interface.h"

#include "bsp_ppg.h"

/*********************************** - Defines - ***********************************/
/************************************ - Enums - ************************************/
/********************************* - Structures - **********************************/
/**************************** - Function Prototypes - ******************************/
/********************************* - Constants - ***********************************/
/********************************* - Variables - ***********************************/

static uint16_t  ppg_data_storage[BSP_HEART_RATE_READINGS_TO_BUFFER];

/***************************** - Public Functions - ********************************/



void bsp_heart_rate_signal_process( zephyr_k_work_t* calling_work )
{
    bsp_signal_processing_work_t* work_to_do = CONTAINER_OF( calling_work , bsp_signal_processing_work_t , work );
    /*Here is your data for the work*/
    uint32_t* raw_readings = work_to_do->in_data[0];
    uint8_t   raw_reading_num = work_to_do->readings_to_buffer;

    /*Ensure no other signal processing work thread is running when this is*/
    zephyr_err_t err = k_mutex_lock( &work_to_do->lock , K_FOREVER );
    if( ZEPHYR_ERR_SUCCESS == err )
    {
        /*Implement your signal processing here*/

        for (uint16_t i = 0; i < BSP_HEART_RATE_READINGS_TO_BUFFER ; i++)
        {
            ppg_data_storage[i] = raw_readings[i];
#ifdef BSP_PPG_GIVE_RAW_READINGS
            printk("%u,",raw_readings[i]);
#endif
        }

        uint16_t** data_storage_ptr = (uint16_t **) &ppg_data_storage;

        bsp_sensor_interface_publish_vital( VITALS_VITAL_TYPE_ID_HEART_RATE ,
                                            &data_storage_ptr , sizeof(uint16_t**) );
        k_mutex_unlock(&work_to_do->lock);
    }

}

/***************************** - Private Functions - *******************************/