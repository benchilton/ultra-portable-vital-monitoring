/*
 *  @name          zephyr_typedefs.h
 *  @date          17/10/22
 *  @author        Benjamin Chilton
 *  @university_id bdc1g19
 *
 *
 */

#ifndef ULTRA_PORTABLE_VITAL_MONITORING_ZEPHYR_TYPEDEFS_H
#define ULTRA_PORTABLE_VITAL_MONITORING_ZEPHYR_TYPEDEFS_H

/****************************** - Library Includes - *******************************/
/******************************** - User Includes - ********************************/
/*********************************** - Defines - ***********************************/

/*errno uses 0 as success and -<error> for indicating err*/
#define ZEPHYR_ERR_SUCCESS ( (int) 0U)

/************************************ - Enums - ************************************/
/********************************** - Typedefs - ***********************************/

/*Errors used in zephyr*/
typedef int                                 zephyr_err_t;

/*Generic typedefs*/
typedef struct device                       zephyr_device_t;
typedef struct k_thread                     zephyr_k_thread_t;
typedef struct k_event                      zephyr_k_event_t;
typedef struct k_queue                      zephyr_k_queue_t;
typedef struct k_mutex                      zephyr_k_mutex_t;
typedef struct k_pipe                       zephyr_k_pipe_t;
typedef struct k_msgq                       zephyr_k_msgq_t;
typedef struct k_work                       zephyr_k_work_t;
typedef struct k_timer                      zephyr_k_timer_t;

typedef struct ring_buf                     zephyr_ring_buf_t;

/*BLE related*/
typedef struct bt_le_ext_adv                zephyr_bt_le_ext_adv_t;
typedef struct bt_le_ext_adv_connected_info zephyr_bt_le_ext_adv_connected_info_t;
typedef struct bt_le_adv_param              zephyr_bt_le_adv_param_t;
typedef struct bt_data                      zephyr_bt_data_t;
typedef struct bt_conn                      zephyr_bt_conn_t;

/*GPIO specific typedefs*/
typedef struct gpio_dt_spec                 zephyr_gpio_dt_spec_t;
typedef struct gpio_callback                zephyr_gpio_callback_t;
/*PWM typedefs*/
typedef struct pwm_dt_spec                  zephyr_pwm_dt_spec_t;

/*SPI specific typedefs*/
typedef struct spi_cs_control               zephyr_spi_cs_control_t;
typedef struct spi_config                   zephyr_spi_config_t;
typedef struct spi_buf_set                  zephyr_spi_buf_set_t;
typedef struct spi_buf                      zephyr_spi_buf_t;

/*Display Driver*/
typedef struct display_buffer_descriptor    zephyr_display_buffer_descriptor_t;
typedef struct display_capabilities         zephyr_display_capabilities_t;

/*BLE Typedefs*/
typedef struct bt_uuid_16                   zephyr_bt_uuid_16_t;

/********************************* - Structures - **********************************/
/**************************** - Function Prototypes - ******************************/
/****************************** - Global Variables - *******************************/

#endif //ULTRA_PORTABLE_VITAL_MONITORING_ZEPHYR_TYPEDEFS_H