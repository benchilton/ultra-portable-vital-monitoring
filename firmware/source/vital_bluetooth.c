/*
 *  @name          vital_bluetooth.c
 *  @date          18/11/2022
 *  @author        Benjamin Chilton
 *  @university_id bdc1g19
 *
 *
 */

/****************************** - Library Includes - *******************************/

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/gatt.h>

#include <zephyr/settings/settings.h>

#include <dk_buttons_and_leds.h>

/******************************** - User Includes - ********************************/

#include "vital_bluetooth.h"
#include "vital_gatt.h"
#include "vital_app.h"

/*********************************** - Defines - ***********************************/

#define VITALS_BLUETOOTH_MONITOR_DEVICE_NAME DT_PROP( DT_NODELABEL( bluetooth_settings ) , adv_name )

/************************************ - Enums - ************************************/
/********************************* - Structures - **********************************/
/**************************** - Function Prototypes - ******************************/

static zephyr_err_t vital_bluetooth_adv_start(void);
static void disconnected(zephyr_bt_conn_t *conn, uint8_t reason);
static void connected(zephyr_bt_conn_t *conn, uint8_t err);
static void advertising_work_handle( zephyr_k_work_t *work);
static void connectable_adv_start(void);

/********************************* - Constants - ***********************************/
/********************************* - Variables - ***********************************/
static K_WORK_DEFINE(advertising_work, advertising_work_handle);

static const struct bt_le_adv_param *advertising_parameters =
        BT_LE_ADV_PARAM(BT_LE_ADV_OPT_CONNECTABLE | BT_LE_ADV_OPT_USE_NAME,
                        BT_GAP_ADV_FAST_INT_MIN_2, /* 100 ms */
                        BT_GAP_ADV_FAST_INT_MAX_2, /* 150 ms */
                        NULL);

BT_CONN_CB_DEFINE(conn_callbacks) = {
        .connected = connected,
        .disconnected = disconnected,
};

static const struct bt_data advertising_data[] = {
        BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR),
        BT_DATA_BYTES(BT_DATA_UUID16_ALL, BT_UUID_16_ENCODE(BT_UUID_DIS_VAL))
};

/***************************** - Public Functions - ********************************/

zephyr_err_t vital_bluetooth_init(void)
{
    zephyr_err_t err = bt_enable(NULL);
    if ( ZEPHYR_ERR_SUCCESS == err )
    {

        err = vital_bluetooth_adv_start();

    }
    return err;
}

/***************************** - Private Functions - *******************************/

static void connectable_adv_start(void)
{

    zephyr_err_t err;
    err = bt_le_adv_start(advertising_parameters, advertising_data, ARRAY_SIZE(advertising_data), NULL, 0);

    if ( ZEPHYR_ERR_SUCCESS == err)
    {
        printk("Failed to start connectable advertising (err %d)\n", err);
    }

}

static void advertising_work_handle( zephyr_k_work_t *work)
{
    connectable_adv_start();
}

static void connected(zephyr_bt_conn_t *conn, uint8_t err)
{
    char addr[BT_ADDR_LE_STR_LEN];

    if ( ZEPHYR_ERR_SUCCESS == err )
    {
        bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

        printk("Connected %s\n", addr);

        vital_app_flag_event(VITAL_APP_EVENTS_BLE_CONNECTED);

        printk("Max MTU: %u\n" ,  bt_gatt_get_mtu(conn) );

    }
    else
    {
        printk("Connection Failed, %d\n" , err );
    }
}

static void disconnected(zephyr_bt_conn_t *conn, uint8_t reason)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    printk("Disconnected: %s (reason %u)\n", addr, reason);

    vital_app_clear_event( VITAL_APP_EVENTS_BLE_CONNECTED );

    /* Process the disconnect logic in the workqueue so that
     * the BLE stack is finished with the connection bookkeeping
     * logic and advertising is possible.
     */
    k_work_submit(&advertising_work);
}


static zephyr_err_t vital_bluetooth_adv_start(void)
{
    zephyr_err_t err = bt_set_name(VITALS_BLUETOOTH_MONITOR_DEVICE_NAME);

    if( ZEPHYR_ERR_SUCCESS == err)
    {

        err = bt_le_adv_start(BT_LE_ADV_CONN, advertising_data, ARRAY_SIZE(advertising_data), NULL, 0);

    }

    return err;
}
