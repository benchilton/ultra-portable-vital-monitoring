/*
 *  @name          vital_app.h
 *  @date          13/12/2022
 *  @author        Benjamin Chilton
 *  @university_id bdc1g19
 *
 *
 */

#ifndef ULTRA_PORTABLE_VITAL_MONITORING_VITAL_APP_H
#define ULTRA_PORTABLE_VITAL_MONITORING_VITAL_APP_H

/****************************** - Library Includes - *******************************/
/******************************** - User Includes - ********************************/
/*********************************** - Defines - ***********************************/
/************************************ - Enums - ************************************/

typedef enum {
    VITAL_APP_INIT = 0,
    VITAL_APP_RUNNING,
    VITAL_APP_TRANSMITTING,
    VITAL_APP_IDLE,
    VITAL_APP_MAX
} vital_app_state_e;

typedef enum {
    VITAL_APP_EVENTS_WORK_TO_DO        = BIT(0),
    VITAL_APP_EVENTS_BLE_CONNECTED     = BIT(1),
    VITAL_APP_EVENTS_DATA_WAITING      = BIT(2),
    VITAL_APP_EVENTS_CLIENT_SUBSCRIBED = BIT(3),
    VITAL_APP_EVENT_TEST               = BIT(4),
} vital_app_events_e;

/********************************** - Typedefs - ***********************************/
/********************************* - Structures - **********************************/
/**************************** - Function Prototypes - ******************************/
zephyr_err_t      vital_app_init();
void              vital_app_flag_event( vital_app_events_e event_to_flag );
void              vital_app_clear_event( vital_app_events_e event_to_clear );
bool              vital_app_event_is_set( vital_app_events_e event , k_timeout_t wait_time );
void              vital_app_set_new_adc_period( k_timeout_t refresh_time );
void              vital_app_update_vital( uint8_t vital , void* value );
/****************************** - Global Variables - *******************************/

#endif //ULTRA_PORTABLE_VITAL_MONITORING_VITAL_APP_H
