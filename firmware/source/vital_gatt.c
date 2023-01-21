/*
 *  @name          vital_gatt.c
 *  @date          21/11/2022
 *  @author        Benjamin Chilton
 *  @university_id bdc1g19
 *
 *
 */

/****************************** - Library Includes - *******************************/
#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/bluetooth.h>

/******************************** - User Includes - ********************************/

#include "zephyr_typedefs.h"

#include "vital_defines.h"

#include "vital_app.h"

#include "vital_gatt.h"
#include "vital_bluetooth.h"
/*********************************** - Defines - ***********************************/
#define VITAL_GATT_TX_AMOUNT           (20U)
#define VITAL_GATT_RX_DATA_BUFFER_SIZE (8U)
/************************************ - Enums - ************************************/

typedef enum {
    VITAL_GATT_ACK_TEMPERATURE  = BIT(0),
    VITAL_GATT_ACK_RESPIRATION  = BIT(1),
    VITAL_GATT_ACK_HEART_RATE   = BIT(2),
    VITAL_GATT_ACK_ACCELERATION = BIT(3),
    VITAL_GATT_ACK_MAX
} bt_gatt_ack_event_e;

/********************************* - Structures - **********************************/

typedef struct {
    vital_gatt_service_data_t data;
    const zephyr_bt_uuid_16_t uuid_16;
    uint16_t                  ring_buffer_size;
}vital_gatt_read_service_t;

typedef struct {
    vital_gatt_config_ids_e config_id;
    uint8_t                 data[VITAL_GATT_RX_DATA_BUFFER_SIZE];
} vital_gatt_rx_config_t;

/**************************** - Function Prototypes - ******************************/

static void vital_gatt_ccc_changed_func(const struct bt_gatt_attr *attr, uint16_t value);

static ssize_t vital_gatt_write_callback(struct bt_conn *conn , const struct bt_gatt_attr *attr , const void *buf , uint16_t len , uint16_t offset , uint8_t flags);
static ssize_t vital_gatt_read_callback(struct bt_conn* conn , const struct bt_gatt_attr * attr , void * buf , unsigned short len , unsigned short offset );

static vital_gatt_rx_config_t vital_gatt_parse_rx( uint8_t* buf , uint16_t len);

static zephyr_err_t vital_gatt_notify( vitals_vital_type_id_e vital_to_update , uint16_t bytes_sent , uint16_t bytes_to_send );

/********************************* - Constants - ***********************************/
/********************************* - Variables - ***********************************/

/* UUID for the primary service  */
static zephyr_bt_uuid_16_t vital_gatt_uuid_primary_service = BT_UUID_INIT_16(0x3000);
/* GATT services for configuration  */
static zephyr_bt_uuid_16_t vital_gatt_uuid_configuration = BT_UUID_INIT_16(0x1000);

/*Struct for the vital transmit service*/
static vital_gatt_read_service_t vital_read_service[] = {
    [VITALS_VITAL_TYPE_ID_TEMPERATURE] = {
        .uuid_16          = BT_UUID_INIT_16(0x2000),
        .ring_buffer_size = sizeof(float) * VITAL_GATT_DATA_BUFFER_SIZE ,
        .data.data_ptr = vital_read_service[VITALS_VITAL_TYPE_ID_TEMPERATURE].data.data,
    },
    [VITALS_VITAL_TYPE_ID_RESPIRATION] = {
        .uuid_16 = BT_UUID_INIT_16(0x2001),
        .ring_buffer_size = sizeof(uint8_t) * VITAL_GATT_DATA_BUFFER_SIZE ,
        .data.data_ptr = vital_read_service[VITALS_VITAL_TYPE_ID_RESPIRATION].data.data,
    },
    [VITALS_VITAL_TYPE_ID_HEART_RATE] = {
        .uuid_16 = BT_UUID_INIT_16(0x2002),
        .ring_buffer_size = sizeof(uint16_t**),
        .data.data_ptr = vital_read_service[VITALS_VITAL_TYPE_ID_HEART_RATE].data.data,
    },
    [VITALS_VITAL_TYPE_ID_ACCELERATION] = {
        .uuid_16 = BT_UUID_INIT_16(0x2003),
        .ring_buffer_size = sizeof(uint32_t) * VITAL_GATT_DATA_BUFFER_SIZE ,
        .data.data_ptr = vital_read_service[VITALS_VITAL_TYPE_ID_ACCELERATION].data.data,
    },
};


BT_GATT_SERVICE_DEFINE(vital_monitor_bt_service ,
       BT_GATT_PRIMARY_SERVICE(&vital_gatt_uuid_primary_service),

       BT_GATT_CHARACTERISTIC(&vital_gatt_uuid_configuration.uuid , BT_GATT_CHRC_WRITE_WITHOUT_RESP , BT_GATT_PERM_WRITE ,
                              NULL , vital_gatt_write_callback , (void *)0),

       BT_GATT_CHARACTERISTIC(&vital_read_service[VITALS_VITAL_TYPE_ID_TEMPERATURE].uuid_16.uuid,
                              BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY , BT_GATT_PERM_READ ,
                              vital_gatt_read_callback , NULL ,
                              &vital_read_service[VITALS_VITAL_TYPE_ID_TEMPERATURE].data.data),
       BT_GATT_CCC(vital_gatt_ccc_changed_func, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

       BT_GATT_CHARACTERISTIC(&vital_read_service[VITALS_VITAL_TYPE_ID_RESPIRATION].uuid_16.uuid,
                              BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY , BT_GATT_PERM_READ ,
                              vital_gatt_read_callback , NULL ,
                              &vital_read_service[VITALS_VITAL_TYPE_ID_RESPIRATION].data.data),
       BT_GATT_CCC(vital_gatt_ccc_changed_func, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

       BT_GATT_CHARACTERISTIC(&vital_read_service[VITALS_VITAL_TYPE_ID_HEART_RATE].uuid_16.uuid,
                              BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY , BT_GATT_PERM_READ ,
                              vital_gatt_read_callback , NULL ,
                              &vital_read_service[VITALS_VITAL_TYPE_ID_HEART_RATE].data.data),
       BT_GATT_CCC(vital_gatt_ccc_changed_func, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

       BT_GATT_CHARACTERISTIC(&vital_read_service[VITALS_VITAL_TYPE_ID_ACCELERATION].uuid_16.uuid,
                              BT_GATT_CHRC_NOTIFY , BT_GATT_PERM_READ ,
                              vital_gatt_read_callback , NULL ,
                              &vital_read_service[VITALS_VITAL_TYPE_ID_ACCELERATION].data.data),
       BT_GATT_CCC(vital_gatt_ccc_changed_func, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

);

static volatile bool    notify_enabled = false;

static zephyr_k_event_t gatt_events;

/***************************** - Public Functions - ********************************/

zephyr_err_t vital_gatt_init()
{
    zephyr_err_t err = ZEPHYR_ERR_SUCCESS;
    k_event_init( &gatt_events );
    for (uint8_t idx = 0; (ZEPHYR_ERR_SUCCESS == err) && (idx < (uint8_t) VITALS_VITAL_TYPE_ID_MAX); idx++)
    {
        ring_buf_init(&vital_read_service[idx].data.ring_buffer , vital_read_service[idx].ring_buffer_size , vital_read_service[idx].data.data );
        err = k_mutex_init(&vital_read_service[idx].data.lock);
        vital_read_service[idx].data.len = 0;
    }
    return err;
}

zephyr_err_t vital_gatt_add_to_queue(vitals_vital_sign_data_t vital_data)
{
    zephyr_err_t err = ZEPHYR_ERR_SUCCESS;
    uint32_t put = 0;
    /*Needs to be in the same order as vitals_vital_type_id_e
     * temp , resp, heart , acceleration , blood ox*/
    void* vitals_ptrs[] = { &vital_data.temperature , &vital_data.respiration ,
                            vital_data.heart_rate_raw , &vital_data.acceleration };
    uint16_t vitals_size[] = { sizeof(int32_t) , sizeof(uint8_t) ,
                               sizeof(uint16_t*) , sizeof(float)};
    for (uint8_t idx = 0; (idx < (uint8_t) VITALS_VITAL_TYPE_ID_MAX) ; idx++ )
    {
        if (idx != VITALS_VITAL_TYPE_ID_HEART_RATE)
        {
            put = ring_buf_put( &vital_read_service[idx].data.ring_buffer, vitals_ptrs[idx] , vitals_size[idx] );
            if( put > 0)
            {
                vital_read_service[idx].data.len =
                        (vital_read_service[idx].data.len > vital_read_service[idx].ring_buffer_size) ?
                        vital_read_service[idx].ring_buffer_size : vital_read_service[idx].data.len + vitals_size[idx];
            }
        }
        else
        {
            vital_read_service[idx].data.data_ptr = vitals_ptrs[idx];
            vital_read_service[idx].data.len = vital_data.heart_rate_raw_len;
        }
    }

    if(ZEPHYR_ERR_SUCCESS == err)
    {
        vital_app_flag_event(VITAL_APP_EVENTS_DATA_WAITING);
    }

    return err;
}

zephyr_err_t vital_gatt_update_readings()
{
    zephyr_err_t err = ZEPHYR_ERR_SUCCESS;
    for (uint8_t idx = 0; (idx < (uint8_t) VITALS_VITAL_TYPE_ID_MAX) ; idx++ )
    {
        err += vital_gatt_update_reading( idx );
    }

    if(ZEPHYR_ERR_SUCCESS == err)
    {
        vital_app_clear_event(VITAL_APP_EVENTS_DATA_WAITING);
    }
    else
    {
        err = -EADV;
    }

    return err;
}

zephyr_err_t vital_gatt_update_reading( vitals_vital_type_id_e vital_to_update )
{
    zephyr_err_t err = k_mutex_lock( &vital_read_service[vital_to_update].data.lock , K_NO_WAIT );
    if( ZEPHYR_ERR_SUCCESS == err )
    {
        uint8_t num_whole = vital_read_service[vital_to_update].data.len / VITAL_GATT_TX_AMOUNT;
        uint8_t num_part = vital_read_service[vital_to_update].data.len % VITAL_GATT_TX_AMOUNT;
        for (uint16_t i = 0; (i <  num_whole) && ( -ENOTCONN != err ); i++)
        {
            err = vital_gatt_notify( vital_to_update , VITAL_GATT_TX_AMOUNT * i , VITAL_GATT_TX_AMOUNT );
        }
        if((num_part > 0) && (err == ZEPHYR_ERR_SUCCESS) )
        {
            err = vital_gatt_notify( vital_to_update , num_whole * VITAL_GATT_TX_AMOUNT , num_part );
        }

        if( ZEPHYR_ERR_SUCCESS == err )
        {
            /*Data has been received, so clear the buffered data as it is no longer needed here.*/
            ring_buf_reset( &vital_read_service[vital_to_update].data.ring_buffer );
            vital_read_service[vital_to_update].data.len = 0;
        }
        k_mutex_unlock( &vital_read_service[vital_to_update].data.lock );
    }

    return err;
}

/***************************** - Private Functions - *******************************/

static zephyr_err_t vital_gatt_notify( vitals_vital_type_id_e vital_to_update , uint16_t bytes_sent , uint16_t bytes_to_send )
{
    uint32_t ret_val = 0;
    zephyr_err_t err = bt_gatt_notify_uuid(NULL ,&vital_read_service[vital_to_update].uuid_16.uuid ,
                              vital_monitor_bt_service.attrs,
                              &vital_read_service[vital_to_update].data.data_ptr[bytes_sent],
                               bytes_to_send
    );
    if( ZEPHYR_ERR_SUCCESS == err )
    {
        printk("Submitting, waiting for acknowledgement on %d\n" , vital_to_update);
        ret_val = k_event_wait( &gatt_events , BIT(vital_to_update) , false , K_SECONDS(10) );
        err = (ret_val == 0) ? -EBADE : ZEPHYR_ERR_SUCCESS ;
        if( ZEPHYR_ERR_SUCCESS == err )
        {
            k_event_set_masked( &gatt_events , 0 , BIT(vital_to_update) );
            printk("Acknowledged\n");
            /*Data has been received, so clear the buffered data as it is no longer needed here.*/
            ring_buf_reset( &vital_read_service[vital_to_update].data.ring_buffer );
            vital_read_service[vital_to_update].data.len = 0;
        }
    }
    return err;
}



static void vital_gatt_ccc_changed_func(const struct bt_gatt_attr *attr, uint16_t value)
{
    ARG_UNUSED(attr);
    notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    printk("Notification %s\n", notify_enabled ? "enabled" : "disabled");

    if( true == notify_enabled )
    {
        vital_app_flag_event(VITAL_APP_EVENTS_CLIENT_SUBSCRIBED);
    }
    else
    {
        vital_app_clear_event(VITAL_APP_EVENTS_CLIENT_SUBSCRIBED);
    }

}

static ssize_t vital_gatt_write_callback(struct bt_conn *conn , const struct bt_gatt_attr *attr,
                    const void *buf , uint16_t len, uint16_t offset, uint8_t flags)
{

    vital_gatt_rx_config_t config = vital_gatt_parse_rx( (uint8_t*) buf , len);
    uint16_t rx_as_two_bytes;

    printk("Received config cmd: %x\n" , config.config_id );

    switch (config.config_id)
    {
        case VITAL_GATT_CONFIG_ID_SET_UPDATE_PERIOD:
            rx_as_two_bytes = ((uint16_t*)config.data)[0];
            vital_app_set_new_adc_period( K_SECONDS(rx_as_two_bytes) );
        break;
        case VITAL_GATT_CONFIG_ID_SET_VITAL_VAL:
            vital_app_update_vital( config.data[0] , config.data + 1 );
        break;
        case VITAL_GATT_CONFIG_ID_ACKNOWLEGE_RX:
            rx_as_two_bytes = ((uint16_t*)config.data)[0];
            k_event_post( &gatt_events , BIT(rx_as_two_bytes) );
        break;
        default:
        break;
    }

    return 0;
}

static ssize_t vital_gatt_read_callback(struct bt_conn* conn , const struct bt_gatt_attr * attr, void * buf , unsigned short len , unsigned short offset )
{

    return 0;

}


static vital_gatt_rx_config_t vital_gatt_parse_rx( uint8_t* buf , uint16_t len)
{
    vital_gatt_rx_config_t ret_config;
    memset( ret_config.data , 0 , VITAL_GATT_RX_DATA_BUFFER_SIZE);

    if( len == 3 )
    {
        ret_config.config_id = buf[0];
        memcpy( &ret_config.data , &buf[1] , VITAL_GATT_RX_DATA_BUFFER_SIZE );
    }
    else
    {
        if( len == 6)
        {
            ret_config.config_id = buf[0];
            memcpy( &ret_config.data , &buf[1] , VITAL_GATT_RX_DATA_BUFFER_SIZE );
        }
        else
        {
            ret_config.config_id = 0;
            memset( ret_config.data , 0xFF , VITAL_GATT_RX_DATA_BUFFER_SIZE);
        }
    }


    return ret_config;
}