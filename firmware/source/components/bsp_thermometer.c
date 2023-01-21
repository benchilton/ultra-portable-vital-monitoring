/*
 *  @name          bsp_thermometer.c
 *  @date          17/10/22
 *  @author        Benjamin Chilton, Harry Snell
 *  @university_id bdc1g19, hss1g19
 *
 *
 */

/****************************** - Library Includes - *******************************/

#include <sys/util.h>
#include <string.h>
#include <kernel.h>

/******************************** - User Includes - ********************************/

#include "vital_defines.h"

#include "bsp_sensor_interface.h"

#include "bsp_thermometer.h"

#include "bsp_ad469x_calibrations.h"

/*********************************** - Defines - ***********************************/
/************************************ - Enums - ************************************/
/********************************* - Structures - **********************************/
/**************************** - Function Prototypes - ******************************/
/********************************* - Constants - ***********************************/
/********************************* - Variables - ***********************************/
/***************************** - Public Functions - ********************************/

void bsp_thermometer_signal_process(zephyr_k_work_t* calling_work)
{
    /*Get the data for the work
     * Elements are:
     * in_data, in_data_len
     */
    bsp_signal_processing_work_t* work_to_do = CONTAINER_OF( calling_work , bsp_signal_processing_work_t , work );
    /*Here is your data for the work*/
    uint32_t* raw_readings = work_to_do->in_data[0];
    /*from 0 to raw_reading_num - 1 readings*/
    uint16_t   raw_reading_num = work_to_do->readings_to_buffer;

    /*Ensure no other signal processing work thread is running when this is*/
    zephyr_err_t err = k_mutex_lock( &work_to_do->lock , K_FOREVER );
    if( ZEPHYR_ERR_SUCCESS == err )
    {
        /*Implement your signal processing here*/

        // If using in-amp
        //(*raw_readings-19968) >> 6;

        // Without in-amp
        // do sign extension from 19-bit input
        uint32_t neweset_reading = raw_readings[raw_reading_num - 1];/*-1 as the buffer is 0 to raw_reading_num - 1*/
        uint32_t raw_temperature = (neweset_reading & BIT(18)) ?
                (neweset_reading  | ((uint32_t) GENMASK(31,19))) : (neweset_reading & ((uint32_t) GENMASK(18,0)));
        int32_t temperature_old;
        memcpy(&temperature_old , &raw_temperature , sizeof(int32_t));

        temperature_old += BSP_AD469X_CALIBRATIONS_TEMPERATURE_OFFSET;
        temperature_old = (int32_t) (1.4862f * ((float) temperature_old));
        // Map codes to temperature
        int32_t temperature = 200+((-65536 - temperature_old) >> 7);

#ifdef BSP_THERMOMETER_GIVE_RAW_READINGS
        printk("0x%x,%d,%d,\n",neweset_reading,temperature_old,temperature);
#endif


        bsp_sensor_interface_publish_vital( VITALS_VITAL_TYPE_ID_TEMPERATURE ,
                                            &temperature , sizeof(int32_t) );

        k_mutex_unlock(&work_to_do->lock);
    }

}

/***************************** - Private Functions - *******************************/