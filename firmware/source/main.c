/*
 *  @name          main.c
 *  @date          05/08/2022
 *  @author        Benjamin Chilton
 *  @university_id bdc1g19
 *
 *
 */

/****************************** - Library Includes - *******************************/

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <bluetooth/gatt.h>

/******************************** - User Includes - ********************************/

#include "bsp.h"
#include "vital_bluetooth.h"
#include "vital_gatt.h"
#include "vital_app.h"

/*********************************** - Defines - ***********************************/
/************************************ - Enums - ************************************/
/********************************* - Structures - **********************************/
/**************************** - Function Prototypes - ******************************/
/********************************* - Constants - ***********************************/
/********************************* - Variables - ***********************************/
/***************************** - Public Functions - ********************************/#

#ifdef POWERTEST

#include <drivers/gpio.h>
#include <pm/pm.h>
#include <pm/device.h>

#define SENS_EN GPIO_DT_SPEC_GET( DT_CHILD( DT_NODELABEL( ad469x ) , properties ) , adc_reset_gpios )
#define TFT_EN GPIO_DT_SPEC_GET( DT_CHILD( DT_NODELABEL( ad469x ) , properties ) , adc_reset_gpios )
#define TX GPIO_DT_SPEC_GET( DT_CHILD( DT_NODELABEL( ad469x ) , properties ) , adc_reset_gpios )

struct gpio_dt_spec sens_en = SENS_EN;

struct gpio_dt_spec tft_en = SENS_EN;

struct gpio_dt_spec tx = TX;

#endif

void main(void)
{
#ifdef POWERTEST

    const struct device *const cons = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
    sens_en.pin = 9;
    tft_en.pin  = 19;
    tx.pin = 27;

    printk("Hello new board\n");
    gpio_pin_configure_dt( &sens_en  , GPIO_OUTPUT);
    gpio_pin_set_dt( &sens_en , 0 );
    gpio_pin_configure_dt( &tft_en  , GPIO_OUTPUT);
    gpio_pin_set_dt( &tft_en , 0 );
    gpio_pin_configure_dt( &tx  , GPIO_OUTPUT);
    for (int i = 0; i < 20; ++i)
    {
        gpio_pin_toggle_dt(&tx);
        k_sleep(K_MSEC(100));
    }

    pm_device_action_run(cons, PM_DEVICE_ACTION_SUSPEND);
    pm_state_force(0u, &(struct pm_state_info){PM_STATE_SOFT_OFF, 0, 0});
    k_sleep(K_FOREVER);


#else
    zephyr_err_t err = -EIO;
    do {

        err = bsp_init();
        if( ZEPHYR_ERR_SUCCESS == err )
        {
            //err = vital_bluetooth_init();
            if( ZEPHYR_ERR_SUCCESS == err )
            {
                //err = vital_gatt_init();
                if( ZEPHYR_ERR_SUCCESS == err )
                {
                    err = vital_app_init();
                    if( ZEPHYR_ERR_SUCCESS == err )
                    {
                        printk("Vital's Monitor initialised successfully\n");
                    }
                    else
                    {
                        printk("App failed to Init!\n");
                        k_sleep(K_MSEC(100));
                    }
                }
            }
            else
            {
                printk("Bluetooth failed to Init!\n");
                k_sleep(K_MSEC(100));
            }
        }
        else
        {
            printk("BSP failed to Init!\n");
            k_sleep(K_MSEC(100));
        }

    } while( err != ZEPHYR_ERR_SUCCESS );
#endif
}

/***************************** - Private Functions - *******************************/
