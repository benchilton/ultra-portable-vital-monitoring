/*
 *  @name          bsp_adc.c
 *  @date          13/10/22
 *  @author        Benjamin Chilton
 *  @university_id bdc1g19
 *
 *
 */

/****************************** - Library Includes - *******************************/

#include <string.h>

#include <zephyr/kernel.h>
#include <device.h>

/******************************** - User Includes - ********************************/

#include "bsp_adc.h"

#include "bsp_ad469x.h"

/*********************************** - Defines - ***********************************/

#define BSP_ADC_PAUSE_SAMPLING_EVENT    BIT(0)

#define ADC_MAX_NUMBER_OF_CHANNELS AD469x_CHANNEL_NO

#define ADC_EMPTY_QUEUE            ZEPHYR_ERR_SUCCESS

#define ADC_THREAD_STACK           (1024UL)
#define ADC_THREAD_PRIORITY        1

#define ADC_THREAD_OPTIONS         ( K_FP_REGS | K_ESSENTIAL )

/************************************ - Enums - ************************************/
/********************************* - Structures - **********************************/
/**************************** - Function Prototypes - ******************************/

void           bsp_adc_take_reading(void);

static void    bsp_adc_reading_thread(void* arg1 , void* arg2 , void* arg3 );
static uint8_t bsp_adc_event_to_ch( uint32_t event_flags );

/********************************* - Constants - ***********************************/
/********************************* - Variables - ***********************************/

K_THREAD_STACK_DEFINE(adc_thread_stack, ADC_THREAD_STACK);

static zephyr_k_mutex_t  adc_queue_lock;
static zephyr_k_thread_t adc_thread;
static k_tid_t           adc_thread_tid;

static zephyr_k_event_t* ad469x_events;
static zephyr_k_event_t  adc_thread_events;

static zephyr_k_msgq_t*  adc_channel_queue;
static k_timeout_t       adc_sample_period;

/***************************** - Public Functions - ********************************/

zephyr_err_t bsp_adc_init(void)
{
    zephyr_err_t err = bsp_ad469x_init();
    if( ZEPHYR_ERR_SUCCESS == err )
    {
        err = k_mutex_init(& adc_queue_lock );
        if( ZEPHYR_ERR_SUCCESS == err )
        {

            k_event_init(&adc_thread_events);

            adc_channel_queue = bsp_ad469x_get_data_queues_handle();
            ad469x_events    = bsp_ad469x_get_data_event_handle();

            adc_sample_period = K_MSEC( ADC_READ_PERIOD );
            adc_thread_tid = k_thread_create(&adc_thread, adc_thread_stack, ADC_THREAD_STACK,
                                             bsp_adc_reading_thread, NULL, NULL, NULL,
                                             ADC_THREAD_PRIORITY, ADC_THREAD_OPTIONS, K_MSEC(100));
            k_thread_name_set(adc_thread_tid, "adc" );
        }
    }

    return err;
}

zephyr_err_t bsp_adc_queue_channels_for_read(bsp_adc_reading_config_t config )
{
    zephyr_err_t err = k_mutex_lock( &adc_queue_lock , K_FOREVER );
    if( ZEPHYR_ERR_SUCCESS == err )
    {
        bsp_ad469x_queue_channels_adv_seq( config.slots_used , config.slots );
        k_mutex_unlock( & adc_queue_lock );
    }
    return err;
}

uint8_t bsp_adc_get_channel_resolution(uint8_t slot_idx )
{
    return bsp_ad469x_get_slot_resolution( slot_idx );
}

zephyr_err_t bsp_adc_get_newest_reading(uint8_t* slot , uint32_t* sampled_data , bsp_adc_slot_status_t* flags , k_timeout_t timeout )
{
    uint32_t ret_flags = 0;
    bsp_ad469x_reading_t adc_result;

    uint8_t ch_to_check = 0xFF;

    zephyr_err_t err = 0;//;k_mutex_lock( &adc_queue_lock , timeout );
    if( ZEPHYR_ERR_SUCCESS == err )
    {
        /*Check If any events are set or not*/
        ret_flags = k_event_wait(ad469x_events , 0xFFFF , false , timeout );
        if( ret_flags != 0  )
        {
            ch_to_check = bsp_adc_event_to_ch( ret_flags );
            *slot = bsp_ad469x_channel_to_slot( ch_to_check , 0 );
            /*No need to wait because if we are at this point the events have returned true and there is data to fetch*/
            err = k_msgq_get(&adc_channel_queue[ch_to_check] , &adc_result , K_NO_WAIT);
            if ( ZEPHYR_ERR_SUCCESS == err )
            {
                *sampled_data = adc_result.raw_reading;
                flags->over_voltage = adc_result.status.ov_flag;
                flags->channel = adc_result.status.channel;
                /*Clear the event bit of the event we just processed*/
                //ad469x_events->events = ad469x_events->events & ~( 1 << ch_to_check);
                k_event_set_masked(ad469x_events , 0 , 1 << ch_to_check );
            }
        }
        //err = k_mutex_unlock( &adc_queue_lock );
    }
    return err;
}

zephyr_err_t bsp_adc_get_channel_reading(uint8_t slot_idx , uint32_t* sampled_data , bsp_adc_slot_status_t* flags , k_timeout_t timeout )
{
    uint32_t ret_flags = 0;
    bsp_ad469x_reading_t adc_result;

    slot_idx = bsp_ad469x_get_slots_at_idx( slot_idx );
    uint8_t ch_to_check = 0xFF;

    zephyr_err_t err = ZEPHYR_ERR_SUCCESS;//k_mutex_lock( &adc_queue_lock , timeout );
    if( ZEPHYR_ERR_SUCCESS == err )
    {
        /*Check If any events are set or not*/
        ret_flags = k_event_wait(ad469x_events , 1 << slot_idx , false , timeout );
        if( ret_flags != 0  )
        {
            ch_to_check = bsp_adc_event_to_ch( ret_flags );
            /*No need to wait because if we are at this point the events have returned true and there is data to fetch*/
            err = k_msgq_get(&adc_channel_queue[ch_to_check] , &adc_result , K_NO_WAIT);
            if ( ZEPHYR_ERR_SUCCESS == err )
            {
                *sampled_data = adc_result.raw_reading;
                flags->over_voltage = adc_result.status.ov_flag;
                flags->channel = adc_result.status.channel;
                k_event_set_masked(ad469x_events , 0 , 1 << ch_to_check );
            }
        }
    //err = k_mutex_unlock( &adc_queue_lock );
    }
    return err;
}

zephyr_err_t bsp_adc_set_period( k_timeout_t period )
{
    zephyr_err_t err = k_mutex_lock( &adc_queue_lock , adc_sample_period );
    if( ZEPHYR_ERR_SUCCESS == err )
    {
        adc_sample_period = period;
        k_mutex_unlock( &adc_queue_lock );
    }
    return err;
}

k_timeout_t bsp_adc_get_period()
{
    return adc_sample_period;
}

uint8_t bsp_adc_get_number_of_channels(void)
{
    return bsp_ad469x_get_num_slots();
}

void bsp_adc_stop_sampling()
{
    k_event_set_masked(&adc_thread_events , 0 , BSP_ADC_PAUSE_SAMPLING_EVENT );
}

void bsp_adc_resume_sampling()
{
    k_event_set(&adc_thread_events , BSP_ADC_PAUSE_SAMPLING_EVENT );
}

/***************************** - Private Functions - *******************************/

__NO_RETURN static void bsp_adc_reading_thread(void* arg1 , void* arg2 , void* arg3 )
{
    ARG_UNUSED(arg1);
    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);
    zephyr_err_t       err;
    while( true )
    {
        //We are only sampling low frequency signals so the timing is not significantly critical.
        err = k_mutex_lock( &adc_queue_lock , adc_sample_period );
        if( ZEPHYR_ERR_SUCCESS == err )
        {
            /*Take a sample of the desired channels*/
            bsp_ad469x_take_reading();
            /*Unlock the mutex*/
            k_mutex_unlock( &adc_queue_lock );
        }
        /*Used to pause the thread at a known point*/
        k_event_wait(&adc_thread_events , BSP_ADC_PAUSE_SAMPLING_EVENT , false , K_FOREVER);
        k_sleep( adc_sample_period );
    }
}

static uint8_t bsp_adc_event_to_ch( uint32_t event_flags )
{
    uint8_t pos = 0;
    while ( pos < ( 8*sizeof(uint32_t) ) )
    {
        /*If the LSB is set in the event flag*/
        if( 0x1 == (event_flags & 0x01) )
        {
            break;
        }
        event_flags = event_flags >> 1;
        pos++;
    }
    return pos;
}