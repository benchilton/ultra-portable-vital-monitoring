/*
 *  @name          vital_app.c
 *  @date          13/12/2022
 *  @author        Benjamin Chilton
 *  @university_id bdc1g19
 *
 *
 */

/****************************** - Library Includes - *******************************/
#include <zephyr/kernel.h>
#include <string.h>
#include <math.h>

/******************************** - User Includes - ********************************/

#include "zephyr_typedefs.h"

#include "vital_defines.h"

#include "vital_bluetooth.h"
#include "vital_gatt.h"

#include "vital_app.h"

#include "bsp.h"

/*********************************** - Defines - ***********************************/
/*Temperature, heart rate, respiration rate, motion , stretch blood oxygen and another?*/

#define APP_THREAD_STACK             (1024UL)
#define APP_THREAD_PRIORITY          3
#define APP_THREAD_OPTIONS           (K_ESSENTIAL )

/************************************ - Enums - ************************************/
/********************************* - Structures - **********************************/

typedef struct
{
    bsp_battery_status_t     battery;
    vitals_vital_sign_data_t vitals;
} vital_app_data_t;

/**************************** - Function Prototypes - ******************************/
__NO_RETURN static void vital_app_application(void* arg1 , void* arg2 , void* arg3 );

static zephyr_err_t vital_app_transmitting_task(void);
static zephyr_err_t vital_app_running_task(void);

static void app_resume_adc_sampling(zephyr_k_timer_t *timer);

static void app_update_state( vital_app_state_e new_state );

static void app_adc_work_handler(zephyr_k_work_t* calling_work);

/********************************* - Constants - ***********************************/

K_THREAD_STACK_DEFINE(app_thread_stack, APP_THREAD_STACK );

const char* app_state_to_str[] = {
        [VITAL_APP_INIT] = "INIT",
        [VITAL_APP_RUNNING] = "RUNNING",
        [VITAL_APP_TRANSMITTING] = "TRANSMITTING",
        [VITAL_APP_IDLE] = "IDLE",
};

/********************************* - Variables - ***********************************/

static zephyr_k_thread_t         app_thread;
static k_tid_t                   app_thread_tid;

static vital_app_state_e         app_state = VITAL_APP_INIT;

static zephyr_k_event_t          app_events;

static vital_app_data_t          app_data;

static k_timeout_t               time_to_next_sample;

static zephyr_k_timer_t          app_adc_sample_timer;

static zephyr_k_work_t           app_adc_work;

/***************************** - Public Functions - ********************************/

zephyr_err_t vital_app_init()
{

    zephyr_err_t err = -EIO;

    time_to_next_sample = K_SECONDS(90);

    k_event_init(&app_events);
    k_work_init( &app_adc_work , app_adc_work_handler );
    k_timer_init(&app_adc_sample_timer, app_resume_adc_sampling, NULL);

    app_thread_tid = k_thread_create(&app_thread, app_thread_stack, APP_THREAD_STACK,
                                     vital_app_application, NULL, NULL, NULL,
                                     APP_THREAD_PRIORITY , APP_THREAD_OPTIONS , K_NO_WAIT);
    k_thread_name_set(app_thread_tid, "vital app" );

    if( NULL != app_thread_tid )
    {
        err = ZEPHYR_ERR_SUCCESS;
    }
    return err;
}

void vital_app_flag_event( vital_app_events_e event_to_flag )
{
    k_event_post( &app_events , event_to_flag );
}

void vital_app_clear_event( vital_app_events_e event_to_clear )
{
    k_event_set_masked( &app_events , 0 , event_to_clear );
}

bool vital_app_event_is_set( vital_app_events_e event , k_timeout_t wait_time )
{
    return (k_event_wait(&app_events , event , false , wait_time) > 0 ) ? true : false ;
}

void vital_app_set_new_adc_period( k_timeout_t refresh_time )
{
    k_timer_start( &app_adc_sample_timer , refresh_time , refresh_time );
    memcpy( &time_to_next_sample , &refresh_time , sizeof(k_timeout_t) );
}

void vital_app_update_vital( uint8_t vital , void* value )
{
    uint32_t val;
    switch (vital)
    {
        case VITALS_VITAL_TYPE_ID_TEMPERATURE:
            memcpy( &app_data.vitals.temperature , value , sizeof(float) );
        break;
        case VITALS_VITAL_TYPE_ID_RESPIRATION:
            val = *((uint32_t*)value);
            app_data.vitals.respiration = (uint8_t) val;
        break;
        case VITALS_VITAL_TYPE_ID_HEART_RATE:
            val = *((uint32_t*)value);
            app_data.vitals.heart_rate = (uint8_t) val;
        break;
        case VITALS_VITAL_TYPE_ID_ACCELERATION:
            memcpy( &app_data.vitals.acceleration , value , sizeof(float) );
        break;
        default:
            break;
    }
}

/***************************** - Private Functions - *******************************/

__NO_RETURN static void vital_app_application(void* arg1 , void* arg2 , void* arg3 )
{
    zephyr_err_t err = ZEPHYR_ERR_SUCCESS;
    uint32_t event_flags = 0;

    while(1)
    {
        switch (app_state)
        {
            case VITAL_APP_INIT:
                k_timer_start( &app_adc_sample_timer , time_to_next_sample , time_to_next_sample );
                app_update_state(VITAL_APP_IDLE);
                bsp_resume_sampling();
            break;
            case VITAL_APP_RUNNING:

                err = vital_app_running_task();

                if( ZEPHYR_ERR_SUCCESS == err )
                {

                    printk("Vitals status:\n"
                           " - Temperature: %4.1f°C\n"
                           " - Heart Rate: %u BPM\n"
                           " - Acceleration %4.1f m/s/s\n"
                           " - Respiration rate: %u\n\n"
                           "Battery Status:\n"
                           " - Health: %u\n"
                           " - Charge: %u\n",
                           ((float)app_data.vitals.temperature) / 10.0f , app_data.vitals.heart_rate , app_data.vitals.acceleration,
                           app_data.vitals.respiration , app_data.battery.health , app_data.battery.rsoc);


                    if(true == vital_app_event_is_set(VITAL_APP_EVENTS_CLIENT_SUBSCRIBED,K_NO_WAIT) &&
                      (true == vital_app_event_is_set(VITAL_APP_EVENTS_BLE_CONNECTED,K_NO_WAIT)))
                    {
                        app_update_state(VITAL_APP_TRANSMITTING);
                    }
                    else
                    {
                        vital_gatt_add_to_queue( app_data.vitals );
                    }
                }
                else
                {
                    bsp_pause_sampling();
                    app_update_state(VITAL_APP_IDLE);
                }


            break;
            case VITAL_APP_TRANSMITTING:

                if((true == vital_app_event_is_set(VITAL_APP_EVENTS_BLE_CONNECTED,K_NO_WAIT)) &&
                   (true == vital_app_event_is_set(VITAL_APP_EVENTS_DATA_WAITING,K_NO_WAIT)))
                {

                    err = vital_app_transmitting_task();

                    if( (ZEPHYR_ERR_SUCCESS == err) )
                    {
                        /*Data successfully transmitted*/
                        app_update_state(VITAL_APP_RUNNING);
                    }
                    else
                    {
                        if( -ENODATA == err )
                        {
                            app_update_state(VITAL_APP_RUNNING);
                        }
                        else
                        {
                            /*Failed so wait a second then try again*/
                            k_sleep(K_SECONDS(1));
                        }
                    }
                }
                else
                {
                    app_update_state(VITAL_APP_IDLE);
                }

            break;
            case VITAL_APP_IDLE:
                event_flags = k_event_wait(&app_events , VITAL_APP_EVENTS_WORK_TO_DO | VITAL_APP_EVENTS_CLIENT_SUBSCRIBED ,
                                           false , K_FOREVER );
                if( false != (event_flags & ( VITAL_APP_EVENTS_DATA_WAITING | VITAL_APP_EVENTS_CLIENT_SUBSCRIBED ) ) )
                {
                    app_update_state(VITAL_APP_TRANSMITTING);
                }
                if( false != (event_flags & VITAL_APP_EVENTS_WORK_TO_DO) )
                {
                    app_update_state(VITAL_APP_RUNNING);
                }

            break;
            case VITAL_APP_MAX:
            default:
                printk("APP: ERROR: Entered unknown state\n");
                app_state = VITAL_APP_INIT;
            break;

        }
    }
}


static zephyr_err_t vital_app_running_task(void)
{
    vitals_vital_sign_data_t new_vitals = {
            .temperature = ~(0),
            .acceleration = NAN,
            .respiration = ~(0),
            .heart_rate = ~(0),
    };
    zephyr_err_t err = bsp_get_data( &new_vitals, &app_data.battery );
    if(ZEPHYR_ERR_SUCCESS == err)
    {

        app_data.vitals.temperature = (new_vitals.temperature != (uint32_t)~(0U)) ? new_vitals.temperature : app_data.vitals.temperature ;
        app_data.vitals.acceleration = (new_vitals.acceleration != NAN) ? new_vitals.acceleration : app_data.vitals.acceleration ;
        app_data.vitals.respiration = (new_vitals.respiration != ((uint32_t)~(0U))) ? new_vitals.respiration : app_data.vitals.respiration ;
        app_data.vitals.heart_rate = (new_vitals.heart_rate != ((uint8_t)~(0U))) ? new_vitals.heart_rate : app_data.vitals.heart_rate ;
        app_data.vitals.heart_rate_raw  = new_vitals.heart_rate_raw;
        app_data.vitals.heart_rate_raw_len = new_vitals.heart_rate_raw_len;

        err = bsp_print_data( app_data.vitals , app_data.battery);
    }
    else
    {
        if( -ENOMSG == err )
        {
            /*If that's the error code, then no work left to do.*/
            vital_app_clear_event(VITAL_APP_EVENTS_WORK_TO_DO);
        }
    }
    return err;
}

static zephyr_err_t vital_app_transmitting_task(void)
{
    zephyr_err_t err = -ENODATA;
    if( true == vital_app_event_is_set(VITAL_APP_EVENTS_DATA_WAITING , K_NO_WAIT) )
    {
        err = vital_gatt_update_readings();
    }

    return err;
}

static void app_resume_adc_sampling(zephyr_k_timer_t *timer)
{
    k_work_submit(&app_adc_work);
}

static void app_update_state( vital_app_state_e new_state )
{
    printk("App: changing state from %s to %s\n" , app_state_to_str[app_state] , app_state_to_str[new_state] );
    app_state = new_state;
}

static void app_adc_work_handler(zephyr_k_work_t* calling_work)
{
    bsp_resume_sampling();
}