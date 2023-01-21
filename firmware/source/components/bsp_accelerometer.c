/*
 *  @name          bsp_accelerometer.c
 *  @date          17/10/2022
 *  @author        Benjamin Chilton , Thomas Bain
 *  @university_id bdc1g19 , trb1g19
 *
 *
 */

/****************************** - Library Includes - *******************************/

#include <sys/util.h>
#include <kernel.h>
#include <fastmath.h>

/******************************** - User Includes - ********************************/

#include "vital_defines.h"
#include "bsp_sensor_interface.h"
#include "bsp_accelerometer.h"

/*********************************** - Defines - ***********************************/

#define AS_FLOAT(x)     ((float)x)

#define ACCELEROMETER_FALL_THRESHOLD 100

#define ACCELEROMETER_MOBILE_THRESHOLD 50

#define ACCELEROMETER_IMMOBILE_WARNING_THRESHOLD 10

#define ACCELEROMETER_FALL_WARNING_MASK 0x01

#define ACCELEROMETER_IMMOBILE_WARNING_MASK 0x02

/************************************ - Enums - ************************************/
/********************************* - Structures - **********************************/
/**************************** - Function Prototypes - ******************************/
/********************************* - Constants - ***********************************/
/********************************* - Variables - ***********************************/
/***************************** - Public Functions - ********************************/

void bsp_acceleration_signal_process(zephyr_k_work_t* calling_work)
{
    /*Get the data for the work
     * Elements are:
     * in_data, in_data_len
     */
    bsp_signal_processing_work_t* work_to_do = CONTAINER_OF( calling_work , bsp_signal_processing_work_t , work );
    /*Here is your data for the work*/
    int32_t* raw_readings_x = work_to_do->in_data[0];
    int32_t* raw_readings_y = work_to_do->in_data[1];
    int32_t* raw_readings_z = work_to_do->in_data[2];

    uint8_t   raw_reading_num = work_to_do->readings_to_buffer;

    // Value to be posted to the BLE characteristic has format
    // |<-immobile_periods(16)->|<-mobile_readings(8)->|<-status_information(8)->|
    uint32_t acceleration_vital_report = 0;

    /*Ensure no other signal processing work thread is running when this is*/
    zephyr_err_t err = k_mutex_lock( &work_to_do->lock , K_FOREVER );
    if( ZEPHYR_ERR_SUCCESS == err )
    {
        /*Implement your signal processing here*/
        static uint16_t immobile_periods = 0;

        uint32_t reading_magnitude = 0;
        uint8_t mobile_readings = 0;
        uint8_t status_information = 0;
        for (uint8_t i = 0 ; i < raw_reading_num ; i++)
        {
            // Calculate the square of the acceleration magnitude
            // Doesent sqrt to save processing
            reading_magnitude = (uint32_t) sqrtf( AS_FLOAT( raw_readings_x[i]*raw_readings_x[i] +
                                raw_readings_y[i]*raw_readings_y[i] +
                                raw_readings_z[i]*raw_readings_z[i]));
#ifdef BSP_ACCELEROMETER_GIVE_RAW_READINGS
            printk("%u,%d,%d,%d,\n", reading_magnitude , raw_readings_x[i] , raw_readings_y[i] , raw_readings_z[i] );
#endif
            // Gives a fall alert if acceleration over threshold
            if(reading_magnitude > ACCELEROMETER_FALL_THRESHOLD)
            {
                /*Give fall alert*/
                status_information |= ACCELEROMETER_FALL_WARNING_MASK;
            }

            // Decides a person is mobile if acceleration above threshold
            if(reading_magnitude > ACCELEROMETER_MOBILE_THRESHOLD)
                mobile_readings++;
        }

        // Keeps track of readings made since person was mobile
        if(mobile_readings == 0)
            immobile_periods++;
        else
            immobile_periods = 0;

        // Produces waarning if person immobile for too long
        if(immobile_periods > ACCELEROMETER_IMMOBILE_WARNING_THRESHOLD)
        {
            /*Give immobility warning*/
            status_information |= ACCELEROMETER_IMMOBILE_WARNING_MASK;
        }

        acceleration_vital_report |= immobile_periods << 16;
        acceleration_vital_report |= mobile_readings << 8;
        acceleration_vital_report |= status_information;
        bsp_sensor_interface_publish_vital( VITALS_VITAL_TYPE_ID_ACCELERATION ,
                                            &acceleration_vital_report , sizeof(uint32_t) );

        k_mutex_unlock(&work_to_do->lock);
    }

}

/***************************** - Private Functions - *******************************/
