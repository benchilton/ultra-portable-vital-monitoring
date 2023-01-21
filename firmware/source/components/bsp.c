/*
 *  @name          bsp.c
 *  @date          17/11/22
 *  @author        Benjamin Chilton
 *  @university_id bdc1g19
 *
 *
 */

/****************************** - Library Includes - *******************************/

#include <string.h>

#include <zephyr/kernel.h>

/******************************** - User Includes - ********************************/

#include "vital_defines.h"

#include "bsp.h"

#include "bsp_sensor_interface.h"
#include "bsp_ppg.h"
#include "bsp_accelerometer.h"
#include "bsp_respiration.h"
#include "bsp_thermometer.h"
#include "bsp_battery_gauge.h"

#include "bsp_display.h"

#include "vital_app.h"


/*********************************** - Defines - ***********************************/

#define BSP_ALL_VITALS (0b1111)

/************************************ - Enums - ************************************/
/********************************* - Structures - **********************************/

typedef struct {
    zephyr_k_work_t          work;
    bool                     vitals_set[VITALS_VITAL_TYPE_ID_MAX];
    vitals_vital_sign_data_t vitals;
} bsp_work_t;

/**************************** - Function Prototypes - ******************************/
static zephyr_err_t bsp_init_internal();
static zephyr_err_t bsp_match_received_vital( vitals_vital_sign_data_t* vitals , bsp_sensor_interface_data vitals_data );

static void         bsp_vital_prepping_work( zephyr_k_work_t* calling_work );

static void         bsp_vital_submit_vital_reading( vitals_vital_sign_data_t reading );

/********************************* - Constants - ***********************************/
/********************************* - Variables - ***********************************/

static bsp_work_t                bsp_work;
static zephyr_k_msgq_t           bsp_data_queue;
static uint8_t                   bsp_queue_buffer[sizeof(vitals_vital_sign_data_t) * BSP_APP_QUEUE_SIZE];

/***************************** - Public Functions - ********************************/

zephyr_err_t bsp_init()
{
    zephyr_err_t err = -EIO;
    bsp_work.vitals.heart_rate = ~(0);

    do
    {
        /*Add init functions to this part*/
        err = bsp_battery_gauge_init();
        if( ZEPHYR_ERR_SUCCESS != err )
        {
            printk("Battery gauge failed to init, err = %d\n" , err);
        }
        err = bsp_display_init();
        if( ZEPHYR_ERR_SUCCESS != err )
        {
            printk("Display failed to init, err = %d\n" , err);
        }
        err = bsp_sensor_interface_init();
        if( ZEPHYR_ERR_SUCCESS != err )
        {
            printk("Sensor Interface failed to init, err = %d\n" , err);
        }
        err = bsp_init_internal();
        if( ZEPHYR_ERR_SUCCESS != err )
        {
            printk("Sensor Interface failed to init, err = %d\n" , err);
        }

    } while( ZEPHYR_ERR_SUCCESS != err );

    bsp_sensor_interface_pause_sampling();

    return err;
}

/*The BSP module will interface directly with the app so no need for this to have its own fancy thread*/

zephyr_err_t bsp_get_data( vitals_vital_sign_data_t* vitals , bsp_battery_status_t* battery )
{
    zephyr_err_t err = ZEPHYR_ERR_SUCCESS;
    bsp_battery_gauge_get_parameters( &battery->voltage , &battery->temperature , &battery->rsoc , &battery->health );

    err = k_msgq_get( &bsp_data_queue , vitals , K_NO_WAIT );

    return err;
}

void bsp_pause_sampling()
{
    printk("sampling paused\n");
    bsp_sensor_interface_pause_sampling();
    bsp_sensor_interface_power_off_afe();
}

void bsp_resume_sampling()
{
    printk("sampling resumed\n");
    bsp_sensor_interface_power_on_afe();
    bsp_sensor_interface_resume_sampling();
}

zephyr_err_t bsp_print_data( vitals_vital_sign_data_t vitals , bsp_battery_status_t battery )
{
    zephyr_err_t err = bsp_display_set_text_pos( 0 , 0 );

    if( ZEPHYR_ERR_SUCCESS == err )
    {
        err =
            bsp_display_print_string("Vitals status:\n"
                                 " - Temperature: %fm°C\n"
                                 " - Heart Rate: %u BPM\n"
                                 " - Acceleration %4.1f m/s/s\n"
                                 " - Respiration rate: %u\n"
                                 "Battery Status:\n"
                                 " - Health: %u\n"
                                 " - Charge: %u\n",
                                 vitals.temperature , vitals.heart_rate , vitals.acceleration , vitals.respiration ,
                                 battery.health , battery.rsoc);
    }
    return err;
}

void bsp_signal_work()
{
    vital_app_flag_event(VITAL_APP_EVENTS_WORK_TO_DO);
}

void bsp_submit_work()
{
    k_work_submit(&bsp_work.work);
}

/***************************** - Private Functions - *******************************/

static zephyr_err_t bsp_init_internal()
{
    zephyr_err_t err = ZEPHYR_ERR_SUCCESS;

    k_msgq_init(&bsp_data_queue , (char *) bsp_queue_buffer,
                sizeof(vitals_vital_sign_data_t) , BSP_APP_QUEUE_SIZE );

    k_work_init(&bsp_work.work , bsp_vital_prepping_work);

    return err;
}

static zephyr_err_t bsp_match_received_vital( vitals_vital_sign_data_t* vitals , bsp_sensor_interface_data vitals_data )
{
    zephyr_err_t err = ZEPHYR_ERR_SUCCESS;;
    void* target_ptr = vitals_data.data_buffer;
    uint8_t size = 0;
    switch ( vitals_data.vital )
    {
        case VITALS_VITAL_TYPE_ID_TEMPERATURE:
            target_ptr = &vitals->temperature;
            size = sizeof(vitals->temperature);
        break;
        case VITALS_VITAL_TYPE_ID_RESPIRATION:
            target_ptr = &vitals->respiration;
            size = sizeof(vitals->respiration);
        break;
        case VITALS_VITAL_TYPE_ID_HEART_RATE:
            target_ptr = &vitals->heart_rate_raw;
            size = sizeof(vitals->heart_rate_raw);
            vitals->heart_rate_raw_len = BSP_HEART_RATE_READINGS_TO_BUFFER;
        break;
        case VITALS_VITAL_TYPE_ID_ACCELERATION:
            target_ptr = &vitals->acceleration;
            size = sizeof(vitals->acceleration);
        break;
        case VITALS_VITAL_TYPE_ID_MAX:
        default:
            err = -EINVAL;
        break;
    }

    memcpy( target_ptr , &vitals_data.data_buffer[0] , size );

    return err;
}

/*
 * This is going to monitor the sensor interface queue
 * and then push to a queue for the app once all vitals have been received
 * */
static void bsp_vital_prepping_work( zephyr_k_work_t* calling_work )
{
    bsp_work_t* work = CONTAINER_OF( calling_work , bsp_work_t , work );
    bsp_sensor_interface_data vital_data;
    bool all_set = true;
    zephyr_err_t err = bsp_sensor_interface_receive_vital( (void*) &vital_data , K_NO_WAIT );
    if( (ZEPHYR_ERR_SUCCESS == err) && ( work->vitals_set[vital_data.vital] == false ) )
    {
        work->vitals_set[vital_data.vital] = true;

        err = bsp_match_received_vital( &work->vitals , vital_data );

        for (uint8_t idx = 0; (idx < VITALS_VITAL_TYPE_ID_MAX) && (all_set == true); idx++)
        {
            all_set = all_set & work->vitals_set[idx];
        }
        if( true == all_set )
        {
            bsp_vital_submit_vital_reading( work->vitals );

            memset( &work->vitals_set[0] , false , sizeof(bool) * VITALS_VITAL_TYPE_ID_MAX );

        }

    }

}


static void bsp_vital_submit_vital_reading( vitals_vital_sign_data_t reading )
{
    zephyr_err_t err = k_msgq_put( &bsp_data_queue , &reading , K_NO_WAIT );
    if( ZEPHYR_ERR_SUCCESS == err )
    {
        vital_app_flag_event( VITAL_APP_EVENTS_WORK_TO_DO );
        bsp_pause_sampling();
    }
}