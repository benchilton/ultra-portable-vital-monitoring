/*
 *  @name          bsp_adc.h
 *  @date          13/10/22
 *  @author        Benjamin Chilton
 *  @university_id bdc1g19
 *
 *
 */

#ifndef ULTRA_PORTABLE_VITAL_MONITORING_BSP_ADC_H
#define ULTRA_PORTABLE_VITAL_MONITORING_BSP_ADC_H

/****************************** - Library Includes - *******************************/

#include <stdint.h>

/******************************** - User Includes - ********************************/
#include "zephyr_typedefs.h"
/*********************************** - Defines - ***********************************/

#define ADC_SLOT_NUMBER 0x80

#define ADC_READ_PERIOD 50U /*Period in MS*/

#define BSP_ADC_CHANNEL_THERMOMETER_1
#define BSP_ADC_CHANNEL_THERMOMETER_2
#define BSP_ADC_CHANNEL_STRAIN_GUAGE
#define BSP_ADC_CHANNEL_PPG
#define BSP_ADC_CHANNEL_ACCELEROMETER

/************************************ - Enums - ************************************/
/********************************** - Typedefs - ***********************************/
/********************************* - Structures - **********************************/

typedef struct {
    uint8_t slots_used;
    uint8_t slots[ADC_SLOT_NUMBER];
} bsp_adc_reading_config_t;

typedef struct {
    uint8_t channel : 4;
    bool    over_voltage;
} bsp_adc_slot_status_t;

/**************************** - Function Prototypes - ******************************/

zephyr_err_t bsp_adc_init(void);
zephyr_err_t bsp_adc_get_newest_reading(uint8_t* slot , uint32_t* sampled_data , bsp_adc_slot_status_t* flags , k_timeout_t timeout );
zephyr_err_t bsp_adc_get_channel_reading(uint8_t slot_idx , uint32_t* sampled_data , bsp_adc_slot_status_t* flags , k_timeout_t timeout );
zephyr_err_t bsp_adc_queue_channels_for_read(bsp_adc_reading_config_t config );
zephyr_err_t bsp_adc_set_period( k_timeout_t period );
k_timeout_t  bsp_adc_get_period();
uint8_t      bsp_adc_get_number_of_channels(void);
uint8_t      bsp_adc_get_channel_resolution(uint8_t slot_idx );

void         bsp_adc_resume_sampling();
void         bsp_adc_stop_sampling();

/****************************** - Global Variables - *******************************/

#endif //ULTRA_PORTABLE_VITAL_MONITORING_BSP_ADC_H