/*
 *  @name          bsp_sensor_interface.c
 *  @date          07/12/22
 *  @author        Benjamin Chilton
 *  @university_id bdc1g19
 *
 *
 */

/****************************** - Library Includes - *******************************/

#include <zephyr/kernel.h>
#include <string.h>

#include <device.h>
#include <drivers/gpio.h>

/******************************** - User Includes - ********************************/

#include "zephyr_typedefs.h"
#include "bsp_sensor_interface.h"

#include "bsp_thermometer.h"
#include "bsp_ppg.h"
#include "bsp_accelerometer.h"
#include "bsp_respiration.h"
#include "bsp_adc.h"

#include "bsp.h"
#include "bsp_std_includes.h"

/*********************************** - Defines - ***********************************/

/*Temperature, heart rate, respiration rate, motion , stretch blood oxygen and another?*/
#define BSP_SENSOR_INTERFACE_NUM_VITALS         ( (uint8_t) (1+1+1+1))

#define SENSOR_THREAD_STACK                     (512UL)
#define SENSOR_THREAD_PRIORITY                  2
#define SENSOR_THREAD_OPTIONS                   ( /* K_FP_REGS | */ K_ESSENTIAL )

#define BSP_SENSOR_INTERFACE_QUEUE_SIZE         16

#define BSP_SENSOR_INTERFACE_AFE_ENABLE_GPIO    GPIO_DT_SPEC_GET( DT_NODELABEL( afe_settings ) , afe_enable_gpios )
#define BPS_SENSOR_INTERFACE_AFE_OUTPUT_GPIO    GPIO_DT_SPEC_GET( DT_NODELABEL( afe_settings ) , output_enable_gpios )

#define BSP_SENSOR_INTERFACE_PPG_LED_0_GPIO     GPIO_DT_SPEC_GET( DT_NODELABEL( led0 ) , gpios )
#define BSP_SENSOR_INTERFACE_PPG_LED_1_GPIO     GPIO_DT_SPEC_GET( DT_NODELABEL( led1 ) , gpios )
#define BSP_SENSOR_INTERFACE_PPG_LED_2_GPIO     GPIO_DT_SPEC_GET( DT_NODELABEL( led2 ) , gpios )
#define BSP_SENSOR_INTERFACE_PPG_LED_3_GPIO     GPIO_DT_SPEC_GET( DT_NODELABEL( led3 ) , gpios )

/************************************ - Enums - ************************************/
typedef enum
{
    BSP_SENSOR_INTERFACE_PPG_LED_0 = 0,
    BSP_SENSOR_INTERFACE_PPG_LED_1 = 1,
    BSP_SENSOR_INTERFACE_PPG_LED_2 = 2,
    BSP_SENSOR_INTERFACE_PPG_LED_3 = 3,
    BSP_SENSOR_INTERFACE_PPG_LED_MAX
} bsp_sensor_interface_ppg_led_e;
/********************************* - Structures - **********************************/
/**************************** - Function Prototypes - ******************************/
__NO_RETURN static void bsp_sensor_interface_thread(void* arg1 , void* arg2 , void* arg3 );
static uint8_t          bsp_sensor_interface_slot_to_work_thread( uint8_t slot , uint8_t* placed_slot );
static zephyr_err_t     bsp_sensor_interface_gpio_init();
static void             bsp_sensor_interface_afe_enable_cb(const zephyr_device_t* port, zephyr_gpio_callback_t* cb, gpio_port_pins_t pins);
static void             bsp_sensor_interface_output_enable_cb(const zephyr_device_t* port, zephyr_gpio_callback_t* cb, gpio_port_pins_t pins);
static zephyr_err_t     bsp_sensor_interface_init_ppg_gpios(void);
static void             bsp_sensor_interface_ppg_cb(const zephyr_device_t* port, zephyr_gpio_callback_t* cb, gpio_port_pins_t pins);
/********************************* - Constants - ***********************************/

K_THREAD_STACK_DEFINE(sensor_thread_stack, SENSOR_THREAD_STACK );

/********************************* - Variables - ***********************************/

/*This struct holds function pointers to the data processing functions for each sensor*/
static bsp_signal_processing_work_t sensor_work[] = {
    [VITALS_VITAL_TYPE_ID_TEMPERATURE]  = {
        .work_handler = bsp_thermometer_signal_process,
        .readings_to_buffer = BSP_THERMOMETER_READINGS_TO_BUFFER,
        .slots_used         = 1,
        .slots_to_watch     = {0},
        .skipped_readings   = 0,
        .readings_to_skip   = 63,
    },
    [VITALS_VITAL_TYPE_ID_RESPIRATION]  = {
            .work_handler = bsp_respiration_signal_process,
            .readings_to_buffer = BSP_RESPIRATION_READINGS_TO_BUFFER,
            .slots_used         = 1,
            .slots_to_watch     = {1},
            .skipped_readings   = 0,
            .readings_to_skip   = 4,
    },
    [VITALS_VITAL_TYPE_ID_HEART_RATE]   = {
        .work_handler = bsp_heart_rate_signal_process,
        .readings_to_buffer = BSP_HEART_RATE_READINGS_TO_BUFFER,
        .slots_used         = 1,
        .slots_to_watch     = {5},
        .skipped_readings        = 0,
        .readings_to_skip   = 0,
        /*
        .slots_used         = 2,
        .slots_to_watch     = {5 , 6 },
         */
    },
    [VITALS_VITAL_TYPE_ID_ACCELERATION] = {
        .work_handler = bsp_acceleration_signal_process,
        .readings_to_buffer = BSP_ACCELERATION_READINGS_TO_BUFFER,
        .slots_used         = 3,
        .slots_to_watch     = {2 , 3 , 4},
        .skipped_readings        = 0,
        .readings_to_skip   = 31,
    },
    /*
    [VITALS_VITAL_TYPE_ID_BLOOD_OXYGEN]   = {
            .work_handler = bsp_heart_rate_signal_process,
            .readings_to_buffer = BSP_HEART_RATE_READINGS_TO_BUFFER,
            .slots_used         = 2,
            .slots_to_watch     = { 7 , 8 },
    },
    */
};

static zephyr_k_thread_t         sensor_thread;
static k_tid_t                   sensor_thread_tid;
static zephyr_k_msgq_t           sensor_interface_queue;
static uint8_t                   queue_buffer[sizeof(bsp_sensor_interface_data) * BSP_SENSOR_INTERFACE_QUEUE_SIZE];

static bsp_gpio_prop_t           gpio_afe_enable = {
        .gpio_spec = BSP_SENSOR_INTERFACE_AFE_ENABLE_GPIO,
};
static bsp_gpio_prop_t           gpio_output_enable = {
        .gpio_spec = BPS_SENSOR_INTERFACE_AFE_OUTPUT_GPIO,
};

static bsp_gpio_prop_t  ppg_led_gpios[] = {
        [BSP_SENSOR_INTERFACE_PPG_LED_0] = {
                .gpio_spec = BSP_SENSOR_INTERFACE_PPG_LED_0_GPIO,
        },
        [BSP_SENSOR_INTERFACE_PPG_LED_1] = {
                .gpio_spec = BSP_SENSOR_INTERFACE_PPG_LED_1_GPIO,
        },
        [BSP_SENSOR_INTERFACE_PPG_LED_2] = {
                .gpio_spec = BSP_SENSOR_INTERFACE_PPG_LED_2_GPIO,
        },
        [BSP_SENSOR_INTERFACE_PPG_LED_3] = {
                .gpio_spec = BSP_SENSOR_INTERFACE_PPG_LED_3_GPIO,
        },
};


/***************************** - Public Functions - ********************************/

zephyr_err_t bsp_sensor_interface_init()
{

    /*Init all our sensors*/
    /*This module will probably need to spawn thread that */
    zephyr_err_t err = bsp_sensor_interface_gpio_init();
    if( ZEPHYR_ERR_SUCCESS == err)
    {
        err = bsp_adc_init();
        if( ZEPHYR_ERR_SUCCESS == err)
        {
            bsp_adc_set_period(K_MSEC(50) );
            for (uint8_t idx = 0; (idx < VITALS_VITAL_TYPE_ID_MAX) && ( ZEPHYR_ERR_SUCCESS == err) ; idx++)
            {
                k_work_init( &sensor_work[idx].work , sensor_work[idx].work_handler );
                err = k_mutex_init( &sensor_work[idx].lock );

                sensor_work[idx].in_data = k_calloc( sensor_work[idx].slots_used , sizeof(uint32_t*) );
                sensor_work[idx].amount_filled = k_calloc( sensor_work[idx].slots_used , sizeof(uint16_t) );

                for (uint8_t slot_idx = 0; slot_idx < sensor_work[idx].slots_used ; slot_idx++ )
                {
                    sensor_work[idx].in_data[slot_idx] = k_calloc( sensor_work[idx].readings_to_buffer , sizeof(uint32_t) );
                }


            }

            k_msgq_init(&sensor_interface_queue , (char *) queue_buffer,
                        sizeof(bsp_sensor_interface_data) , BSP_SENSOR_INTERFACE_QUEUE_SIZE );

            sensor_thread_tid = k_thread_create(&sensor_thread, sensor_thread_stack, SENSOR_THREAD_STACK,
                                                 bsp_sensor_interface_thread, NULL, NULL, NULL,
                                                 SENSOR_THREAD_PRIORITY, SENSOR_THREAD_OPTIONS, K_NO_WAIT);
            k_thread_name_set(sensor_thread_tid, "sensor interface" );
        }
    }

    return err;
}

zephyr_err_t bsp_sensor_interface_receive_vital( void* data , k_timeout_t timeout )
{
    zephyr_err_t err =  k_msgq_get( &sensor_interface_queue , data , K_NO_WAIT );
    return err;
}

zephyr_err_t bsp_sensor_interface_publish_vital( vitals_vital_type_id_e vital , void* data , uint8_t sizeof_data )
{
    zephyr_err_t err;
    bsp_sensor_interface_data putting_data;
    putting_data.vital= vital;
    putting_data.data_size = sizeof_data;
    memcpy( &putting_data.data_buffer[0] , data , sizeof_data );
    err = k_msgq_put( &sensor_interface_queue , (void*) &putting_data , K_NO_WAIT );

    if( ZEPHYR_ERR_SUCCESS == err )
    {
        /*Tell the app there is some data ready*/
        bsp_submit_work();
    }
    return err;
}

zephyr_err_t bsp_sensor_interface_power_on_afe()
{
    zephyr_err_t err = gpio_pin_set_dt(&gpio_afe_enable.gpio_spec,  true);
    if(ZEPHYR_ERR_SUCCESS == err)
    {
        k_sleep(K_MSEC(40));
        bsp_sensor_interface_ppg_set(BSP_SENSOR_INTERFACE_PPG_LED_0 , true );
        bsp_sensor_interface_ppg_set(BSP_SENSOR_INTERFACE_PPG_LED_1 , true );
        k_sleep(K_MSEC(40));
    }


    return err;
}

zephyr_err_t bsp_sensor_interface_power_off_afe()
{
    zephyr_err_t err = gpio_pin_set_dt(&gpio_afe_enable.gpio_spec,  false);

    if(ZEPHYR_ERR_SUCCESS == err)
    {
        bsp_sensor_interface_ppg_set(BSP_SENSOR_INTERFACE_PPG_LED_0 , false );
        bsp_sensor_interface_ppg_set(BSP_SENSOR_INTERFACE_PPG_LED_1 , false );
    }

    return err;
}

/*Pause ADC Sampling, this will also pause the sensor interface workflow*/
void bsp_sensor_interface_pause_sampling()
{
    bsp_adc_stop_sampling();
}

void bsp_sensor_interface_resume_sampling()
{
    bsp_adc_resume_sampling();
}

zephyr_k_msgq_t* bsp_sensor_interface_get_data_queues_handle( void )
{
    return &sensor_interface_queue;
}

uint32_t bsp_sensor_interface_get_sample_rate()
{
    return (uint32_t) k_ticks_to_ms_ceil64(bsp_adc_get_period().ticks);
}

/***************************** - Private Functions - *******************************/

__NO_RETURN static void bsp_sensor_interface_thread(void* arg1 , void* arg2 , void* arg3 )
{
    ARG_UNUSED(arg1);
    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);

    zephyr_err_t err;

    uint32_t sample;
    bsp_adc_slot_status_t status_flags;
    vitals_vital_type_id_e work_slot;
    uint8_t placed_slot;

    uint8_t slot_max = bsp_adc_get_number_of_channels();

    while( true )
    {

        for (uint8_t slot = 0; slot < slot_max ; slot++ )
        {
            /*Get the any reading reading*/
            err = bsp_adc_get_channel_reading( slot , &sample, &status_flags, K_FOREVER);
            if( ZEPHYR_ERR_SUCCESS == err )
            {
                if( status_flags.over_voltage == false )
                {
                    /*slot is the slot in the AS
                     * work_slot is the sensor_work index
                     * placed_slot is the index in the sensor_work*/
                    work_slot = bsp_sensor_interface_slot_to_work_thread( slot , &placed_slot );

                    if( sensor_work[work_slot].skipped_readings == sensor_work[work_slot].readings_to_skip  )
                    {
                        sensor_work[work_slot].skipped_readings = 0;
                        if( sensor_work[work_slot].amount_filled[placed_slot] < sensor_work[work_slot].readings_to_buffer )
                        {
                            sensor_work[work_slot].in_data[placed_slot][ sensor_work[work_slot].amount_filled[placed_slot] ] = sample;
                            sensor_work[work_slot].amount_filled[placed_slot]++;

                        }
                        if( sensor_work[work_slot].readings_to_buffer == sensor_work[work_slot].amount_filled[placed_slot] )
                        {
                            bool all_set = true;
                            for (uint8_t idx = 0; (idx < sensor_work[work_slot].slots_used) && (all_set == true); idx++) {
                                all_set = (sensor_work[work_slot].readings_to_buffer ==
                                           sensor_work[work_slot].amount_filled[idx]) ?
                                          true : false;
                            }
                            if (true == all_set)
                            {
                                memset(&sensor_work[work_slot].amount_filled[0], 0,
                                       sensor_work[work_slot].slots_used * sizeof(uint16_t));
                                k_work_submit(&sensor_work[work_slot].work);
                            }
                        }
                    }
                    else
                    {
                        sensor_work[work_slot].skipped_readings = sensor_work[work_slot].skipped_readings + 1;
                    }
                }
            }
        }
    }
}

static uint8_t bsp_sensor_interface_slot_to_work_thread( uint8_t slot , uint8_t* placed_slot )
{
    uint8_t ret_val = 0xFF;
    for (uint8_t vitals_idx = 0; vitals_idx < (uint8_t) VITALS_VITAL_TYPE_ID_MAX; vitals_idx++)
    {
        for (uint8_t slots_idx = 0; slots_idx < sensor_work[vitals_idx].slots_used; slots_idx++)
        {
            if( sensor_work[vitals_idx].slots_to_watch[slots_idx] == slot )
            {
                ret_val = vitals_idx;
                *placed_slot = slots_idx;
                goto func_return;
            }
        }
    }
    func_return:
    return ret_val;
}


static zephyr_err_t bsp_sensor_interface_gpio_init()
{
    zephyr_err_t err = gpio_pin_configure_dt(&gpio_afe_enable.gpio_spec, GPIO_OUTPUT);
    if( ZEPHYR_ERR_SUCCESS == err )
    {
        /*Init start convert GPIO and callback*/
        gpio_init_callback(&gpio_afe_enable.gpio_callback, bsp_sensor_interface_afe_enable_cb,
                           BIT(gpio_afe_enable.gpio_spec.pin));
        gpio_add_callback(gpio_afe_enable.gpio_spec.port, &gpio_afe_enable.gpio_callback);

        err = gpio_pin_configure_dt(&gpio_output_enable.gpio_spec, GPIO_OUTPUT);
        if( ZEPHYR_ERR_SUCCESS == err )
        {
            /*Init start convert GPIO and callback*/
            gpio_init_callback(&gpio_output_enable.gpio_callback, bsp_sensor_interface_output_enable_cb,
                               BIT(gpio_output_enable.gpio_spec.pin));
            err = gpio_add_callback(gpio_output_enable.gpio_spec.port, &gpio_output_enable.gpio_callback);
            gpio_pin_set_dt(&gpio_output_enable.gpio_spec , true);

            if( ZEPHYR_ERR_SUCCESS == err )
            {
                err = bsp_sensor_interface_init_ppg_gpios();
            }

        }

        gpio_pin_set_dt(&gpio_afe_enable.gpio_spec , true);
    }

    return err;
}

static void bsp_sensor_interface_afe_enable_cb(const zephyr_device_t* port, zephyr_gpio_callback_t* cb, gpio_port_pins_t pins)
{

}

static void bsp_sensor_interface_output_enable_cb(const zephyr_device_t* port, zephyr_gpio_callback_t* cb, gpio_port_pins_t pins)
{

}

static zephyr_err_t bsp_sensor_interface_init_ppg_gpios(void)
{
    zephyr_err_t err = -EIO;

    for ( uint8_t idx = 0; (idx < (uint8_t) BSP_SENSOR_INTERFACE_PPG_LED_MAX) ; idx++ )
    {
        err = gpio_pin_configure_dt(&ppg_led_gpios[idx].gpio_spec, GPIO_OUTPUT);

        if( ZEPHYR_ERR_SUCCESS == err )
        {
            /*Init start convert GPIO and callback*/
            gpio_init_callback(&ppg_led_gpios[idx].gpio_callback, bsp_sensor_interface_ppg_cb,
                               BIT(ppg_led_gpios[idx].gpio_spec.pin));
            gpio_add_callback(ppg_led_gpios[idx].gpio_spec.port, &ppg_led_gpios[idx].gpio_callback);
        }
    }

    return err;
}

static void bsp_sensor_interface_ppg_cb(const zephyr_device_t* port, zephyr_gpio_callback_t* cb, gpio_port_pins_t pins)
{

}

zephyr_err_t bsp_sensor_interface_ppg_set( uint8_t led ,  uint8_t on_or_off )
{
    return gpio_pin_set_dt( &ppg_led_gpios[led].gpio_spec , on_or_off );
}