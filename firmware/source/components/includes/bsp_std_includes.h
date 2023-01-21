/*
 *  @name          bsp_std_includes.h
 *  @date          28/11/22
 *  @author        Benjamin Chilton
 *  @university_id bdc1g19
 *
 *
 */

#ifndef ULTRA_PORTABLE_VITAL_MONITORING_BSP_STD_INCLUDES_H
#define ULTRA_PORTABLE_VITAL_MONITORING_BSP_STD_INCLUDES_H

/****************************** - Library Includes - *******************************/

#include <drivers/gpio.h>

/******************************** - User Includes - ********************************/

#include "zephyr_typedefs.h"

/*********************************** - Defines - ***********************************/
/************************************ - Enums - ************************************/
/********************************** - Typedefs - ***********************************/
/********************************* - Structures - **********************************/
typedef struct {
    zephyr_gpio_dt_spec_t  gpio_spec;
    zephyr_gpio_callback_t gpio_callback;
} bsp_gpio_prop_t;
/**************************** - Function Prototypes - ******************************/
/****************************** - Global Variables - *******************************/

#endif //ULTRA_PORTABLE_VITAL_MONITORING_BSP_STD_INCLUDES_H
