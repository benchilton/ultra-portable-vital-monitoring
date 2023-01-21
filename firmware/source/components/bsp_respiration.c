/*
 *  @name          bsp_respiration.c
 *  @date          17/10/2022
 *  @author        Benjamin Chilton , Thomas Bain
 *  @university_id bdc1g19 , trb1g19
 *
 *
 */

/****************************** - Library Includes - *******************************/

#include <sys/util.h>
#include <kernel.h>

/******************************** - User Includes - ********************************/

#include "vital_defines.h"

#include "bsp_sensor_interface.h"

#include "bsp_respiration.h"

#include "bsp_fft.h"

/*********************************** - Defines - ***********************************/

#define AS_FLOAT(X) ((float)X)

/************************************ - Enums - ************************************/
/********************************* - Structures - **********************************/
/**************************** - Function Prototypes - ******************************/
/********************************* - Constants - ***********************************/
/********************************* - Variables - ***********************************/
/***************************** - Public Functions - ********************************/

void bsp_respiration_signal_process( zephyr_k_work_t* calling_work )
{
    bsp_signal_processing_work_t* work_to_do = CONTAINER_OF( calling_work , bsp_signal_processing_work_t , work );
    /*Here is your data for the work*/
    int32_t*  raw_readings = work_to_do->in_data[0];
    uint16_t   raw_reading_num = work_to_do->readings_to_buffer;
    int16_t   raw_readings_16[work_to_do->readings_to_buffer];

    /*Ensure no other signal processing work thread is running when this is*/
    zephyr_err_t err = k_mutex_lock( &work_to_do->lock , K_FOREVER );
    if( ZEPHYR_ERR_SUCCESS == err )
    {
        /*Implement your signal processing here*/
        // Subtract average to remove 0Hz peak
        int32_t total = 0;
        for(uint16_t i = 0 ; i < raw_reading_num ; i++)
        {
            /*Divide by 8 to make sure there is no overflow/underflow when subbing the average value*/
            total += (raw_readings[i]/8);
#ifdef BSP_RESPIRATION_GIVE_RAW_READINGS
            printk("%u,\n",raw_readings[i]/1000);
#endif
        }
#ifdef BSP_RESPIRATION_GIVE_RAW_READINGS
        printk("-Averaged readings-\n");
#endif
        int32_t average = total / (raw_reading_num);
        for(uint16_t i = 0 ; i < raw_reading_num ; i++)
        {
            raw_readings_16[i] = (int16_t) (raw_readings[i]/8 - average);

#ifdef BSP_RESPIRATION_GIVE_RAW_READINGS
            printk("%d,\n",raw_readings_16[i]);
#endif
        }

        // Create Output array
        complex_t* Y = ( complex_t* ) k_calloc( raw_reading_num , sizeof(complex_t) );

        // Perform fft
        bsp_fft(raw_readings_16, raw_reading_num, Y);

        // Find frequency peak
        uint16_t peak_idx = 0U;
        float peak = 0.0f;
        bsp_fft_find_peak( Y , raw_reading_num , &peak , &peak_idx , 0 );

        uint8_t respiration_rate = (uint8_t) (60.0f * AS_FLOAT(peak_idx) * (1.0f/(AS_FLOAT(bsp_sensor_interface_get_sample_rate() * (1 + work_to_do->readings_to_skip))/1000.0f))/AS_FLOAT(256));

        // Publish respiration rate frequency
        bsp_sensor_interface_publish_vital( VITALS_VITAL_TYPE_ID_RESPIRATION ,
                                            &respiration_rate , sizeof(uint8_t) );

        k_free( Y );

        k_mutex_unlock(&work_to_do->lock);
    }
}

/***************************** - Private Functions - *******************************/
