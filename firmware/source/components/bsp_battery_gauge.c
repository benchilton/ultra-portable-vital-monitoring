/*
 *  @name          bsp_battery_gauge.c
 *  @date          12/11/2022
 *  @author        Benjamin Chilton
 *  @university_id bdc1g19
 *
 *
 */

/****************************** - Library Includes - *******************************/
#include <zephyr/kernel.h>
#include <device.h>

#include <drivers/i2c.h>
/******************************** - User Includes - ********************************/
#include "bsp_std_includes.h"
#include "bsp_battery_gauge.h"
/*********************************** - Defines - ***********************************/

#define I2C_DEVICE              DEVICE_DT_GET( DT_PARENT( DT_NODELABEL(lc709204f) ) )
#define I2C_FREQUENCY           DT_PROP( DT_CHILD( DT_NODELABEL( lc709204f ) , properties ) , i2c_frequency )
#define LC709204F_ADDRESS       DT_PROP( DT_CHILD( DT_NODELABEL( lc709204f ) , properties ) , slave_address )

#if DT_NODE_EXISTS(alarm_b_gpios)
    #define LC709204F_ALARM_B_GPIO  GPIO_DT_SPEC_GET( DT_CHILD( DT_NODELABEL( lc709204f ) , properties ) , alarm_b_gpios )
#endif

#define BATTERY_GAUGE_THREAD_STACK           (1024UL)
#define BATTERY_GAUGE_THREAD_PRIORITY        10

#define BATTERY_GAUGE_THREAD_OPTIONS         ( K_FP_REGS | K_ESSENTIAL )

/**LC709204F specific defines**/

/*CRC related defines*/

#define BSP_BATTERY_GAUGE_UPDATE_PARAMS_EVENT BIT(0)

#define CRC_8_ATM_POLY                        0x8380

#define BSP_LC709204F_CMD_POS                 (0U)
#define BSP_LC709204F_DATA_LB_POS             (1U)
#define BSP_LC709204F_DATA_UB_POS             (2U)
#define BSP_LC709204F_CRC_POS                 (3U)

#define BSP_LC709204F_ZERO_DEGREES            (0x0AAC)
#define BSP_LC709204F_TEMP_CONST              (0.1f)

/**LC709204F Commands**/

#define LC709204F_CMD_TIME_TO_EMPTY                (0x03)
#define LC709204F_CMD_TIME_TO_EMPTY_BEFORE_RSOC    (0x04)
#define LC709204F_CMD_TIME_TO_FULL                 (0x05)
#define LC709204F_CMD_TSENSE1_THERMISTOR_CONST_B   (0x06)
#define LC709204F_CMD_INITIAL_RSOC                 (0x07)
#define LC709204F_CMD_CELL_TEMPERATURE             (0x08)
#define BSP_LC709204F_CMD_CELL_VOLTAGE             (0x09)
#define LC709204F_CMD_CURRENT_DIRECTION            (0x0A)
#define LC709204F_CMD_ADJUSTMENT_PACK_APPLICATION  (0x0B)
#define LC709204F_CMD_ADJUSTMENT_PACK_THERMISTOR   (0x0C)
#define LC709204F_CMD_RSOC                         (0x0D)
#define LC709204F_CMD_TSENSE2_THERMISTOR_CONST_B   (0x0E)
#define LC709204F_CMD_INDICATOR_TO_EMPTY           (0x0F)
#define LC709204F_CMD_IC_VERSION                   (0x11)
#define LC709204F_CMD_CHANGE_OF_THE_PARAMETER      (0x12)
#define LC709204F_CMD_ALARM_LOW_RSOC               (0x13)
#define LC709204F_CMD_ALARM_LOW_CELL_VOLTAGE       (0x14)
#define LC709204F_CMD_IC_POWER_MODE                (0x15)
#define LC709204F_CMD_STATUS_BIT                   (0x16)
#define LC709204F_CMD_CYCLE_COUNT                  (0x17)
#define LC709204F_CMD_BATTERY_STATUS               (0x19)
#define LC709204F_CMD_NUMBER_OF_THE_PARAMETER      (0x1A)
#define LC709204F_CMD_TERMINATION_CURRENT_RATE     (0x1C)
#define LC709204F_CMD_EMPTY_CELL_VOLTAGE           (0x1D)
#define LC709204F_CMD_ITE_OFFSET                   (0x1E)
#define LC709204F_CMD_ALARM_HIGH_CELL_VOLTAGE      (0x1F)
#define LC709204F_CMD_ALARM_LOW_TEMPERATURE        (0x20)
#define LC709204F_CMD_ALARM_HIGH_TEMPERATURE       (0x21)
#define LC709204F_CMD_TOTAL_RUN_TIME_LB            (0x24)
#define LC709204F_CMD_TOTAL_RUN_TIME_UB            (0x25)
#define LC709204F_CMD_ACCUMULATED_TEMPERATURE_LB   (0x26)
#define LC709204F_CMD_ACCUMULATED_TEMPERATURE_UB   (0x27)
#define LC709204F_CMD_ACCUMULATED_RSOC_LB          (0x28)
#define LC709204F_CMD_ACCUMULATED_RSOC_UB          (0x29)
#define LC709204F_CMD_MAXIMUM_CELL_VOLTAGE         (0x2A)
#define LC709204F_CMD_MINIMUM_CELL_VOLTAGE         (0x2B)
#define LC709204F_CMD_TSENSE1_MAX_CELL_TEMPERATURE (0x2C)
#define LC709204F_CMD_TSENSE1_MIN_CELL_TEMPERATURE (0x2D)
#define LC709204F_CMD_TSENSE2_AMBIENT_TEMPERATURE  (0x30)
#define LC709204F_CMD_STATE_OF_HEALTH              (0x32)
#define LC709204F_CMD_USER_ID_LB                   (0x36)
#define LC709204F_CMD_USER_ID_UB                   (0x37)


/************************************ - Enums - ************************************/

typedef enum {
    BSP_LC709204F_VOLTAGE_POS     = (0U),
    BSP_LC709204F_TEMPERATURE_POS = (1U),
    BSP_LC709204F_RSOC_POS        = (2U),
    BSP_LC709204F_HEALTH_POS      = (3U),
    BSP_BATTERY_GAUGE_NUM_PARAMS  = (4U),
} bsp_battery_gauge_params;

/********************************* - Structures - **********************************/

typedef struct{
    float voltage;
    float temperature;
    uint8_t rsoc;
    uint8_t health;
}bsp_battery_gauge_parameters_t;

typedef struct
{
    const zephyr_device_t*         i2c_device;
    uint32_t                       i2c_config;
    uint16_t                       slave_address;

    bsp_gpio_prop_t                alarm_gpio;

    bsp_battery_gauge_parameters_t parameters;
    zephyr_k_event_t               events;

}bsp_battery_gauge_device_t;

typedef struct {
    uint8_t  cmd;
    uint16_t data;
    uint8_t  crc;
} bsp_lc709204f_word_t;

/**************************** - Function Prototypes - ******************************/

static zephyr_err_t bsp_lc709204f_init();

static zephyr_err_t bsp_battery_gauge_fetch_params( bsp_battery_gauge_parameters_t* params );
__NO_RETURN static void bsp_battery_gauge_reading_thread(void* arg1 , void* arg2 , void* arg3 );

/**LC709204F IC related methods**/

static zephyr_err_t bsp_lc709204f_write( uint8_t cmd , uint16_t data );
static zephyr_err_t bsp_lc709204f_read( uint8_t cmd , uint16_t* data );

static zephyr_err_t bsp_lc709204f_read_cmd( bsp_lc709204f_word_t* cmd );
static zephyr_err_t bsp_lc709204f_write_cmd( bsp_lc709204f_word_t cmd );

/*CRC Computation*/
static uint8_t bsp_lc709204f_compute_crc( bsp_lc709204f_word_t cmd );
static uint8_t bsp_lc709204f_compute_crc_byte( uint8_t initial_val , uint8_t data );

/********************************* - Constants - ***********************************/
/********************************* - Variables - ***********************************/

K_THREAD_STACK_DEFINE( battery_gauge_stack , BATTERY_GAUGE_THREAD_STACK );
static zephyr_k_thread_t battery_gauge_thread;

static bsp_battery_gauge_device_t lc709204f = {
        .i2c_device = I2C_DEVICE,
        .i2c_config = I2C_SPEED_SET(I2C_SPEED_STANDARD) | I2C_MODE_CONTROLLER,
        .slave_address = LC709204F_ADDRESS,
#if DT_NODE_EXISTS(alarm_b_gpios)
        .alarm_gpio = {
            .gpio_spec = LC709204F_ALARM_B_GPIO,
        },
#endif
};

/***************************** - Public Functions - ********************************/

zephyr_err_t bsp_battery_gauge_init()
{
    zephyr_err_t err = i2c_configure( lc709204f.i2c_device, lc709204f.i2c_config );
    if( ZEPHYR_ERR_SUCCESS == err )
    {
        err = bsp_lc709204f_init();
        if( ZEPHYR_ERR_SUCCESS == err)
        {
            k_event_init(&lc709204f.events);
            k_tid_t id = k_thread_create(&battery_gauge_thread, battery_gauge_stack, BATTERY_GAUGE_THREAD_STACK,
                            bsp_battery_gauge_reading_thread, NULL, NULL, NULL,
                            BATTERY_GAUGE_THREAD_PRIORITY, BATTERY_GAUGE_THREAD_OPTIONS, K_NO_WAIT);
            k_thread_name_set(id, "battery gauge" );
        }
    }

    return err;
}

void bsp_battery_gauge_get_parameters( float* voltage , float* temperature , uint8_t* rsoc , uint8_t* health )
{
    if( NULL != voltage )
    {
        *voltage = lc709204f.parameters.voltage;
    }
    if( NULL != temperature )
    {
        *temperature = lc709204f.parameters.temperature;
    }
    if( NULL != rsoc )
    {
        *rsoc = lc709204f.parameters.rsoc;
    }
    if( NULL != health )
    {
        *health = lc709204f.parameters.health;
    }
}

void bsp_battery_gauge_refetch_parameters()
{
    k_event_set( &lc709204f.events , BSP_BATTERY_GAUGE_UPDATE_PARAMS_EVENT );
}

/***************************** - Private Functions - *******************************/

/**
 *
 *
 * Battery management specifics
 *
 *
 */

__NO_RETURN static void bsp_battery_gauge_reading_thread(void* arg1 , void* arg2 , void* arg3 )
{
    ARG_UNUSED(arg1);
    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);
    while( true )
    {
        bsp_battery_gauge_fetch_params( &lc709204f.parameters );
        k_event_wait( &lc709204f.events , BSP_BATTERY_GAUGE_UPDATE_PARAMS_EVENT , false , K_HOURS( 1 ) );
        k_event_set_masked(&lc709204f.events , 0 , BSP_BATTERY_GAUGE_UPDATE_PARAMS_EVENT );
    }
}

static zephyr_err_t bsp_battery_gauge_fetch_params( bsp_battery_gauge_parameters_t* params )
{
    zephyr_err_t err = ZEPHYR_ERR_SUCCESS;
    static uint8_t  commands[BSP_BATTERY_GAUGE_NUM_PARAMS] = {
            [0] = BSP_LC709204F_CMD_CELL_VOLTAGE,
            [1] = LC709204F_CMD_CELL_TEMPERATURE,
            [2] = LC709204F_CMD_RSOC,
            [3] = LC709204F_CMD_STATE_OF_HEALTH,
    };
    uint16_t rx_data[BSP_BATTERY_GAUGE_NUM_PARAMS];

    for (uint8_t idx = 0; (idx < BSP_BATTERY_GAUGE_NUM_PARAMS) && (ZEPHYR_ERR_SUCCESS == err); idx++)
    {
        err = bsp_lc709204f_read( commands[idx] , &rx_data[idx] );
    }

    if(ZEPHYR_ERR_SUCCESS == err)
    {
        params->voltage     = ( (float) rx_data[BSP_LC709204F_VOLTAGE_POS] ) / 1000.0f;
        params->temperature = BSP_LC709204F_TEMP_CONST * ( (float) rx_data[BSP_LC709204F_TEMPERATURE_POS] - BSP_LC709204F_ZERO_DEGREES );
        params->rsoc        = rx_data[BSP_LC709204F_RSOC_POS];
        params->health      = rx_data[BSP_LC709204F_HEALTH_POS];
    }

    return err;
}


/**
 *
 *
 * LC709204F specifics
 *
 *
 */

static zephyr_err_t bsp_lc709204f_init()
{
    zephyr_err_t err = ZEPHYR_ERR_SUCCESS;
    uint8_t init_cmds[] = {
            [0] = LC709204F_CMD_ADJUSTMENT_PACK_APPLICATION,
            [1] = LC709204F_CMD_CHANGE_OF_THE_PARAMETER,
            [2] = LC709204F_CMD_TSENSE1_THERMISTOR_CONST_B,
            [3] = LC709204F_CMD_IC_POWER_MODE,
            [4] = LC709204F_CMD_STATUS_BIT,
    };
    uint16_t init_data[] = {
            [0] = 0x2D2D,/*From datasheet*/
            [1] = 0x0000,
            [2] = 0x0D34,/*Default value*/
            [3] = 0x0001,
            [4] = 0x0001,
    };
    for (uint8_t idx = 0; (idx < (sizeof(init_cmds) / sizeof(init_cmds[0]))) && ( ZEPHYR_ERR_SUCCESS == err); idx++)
    {
        err = bsp_lc709204f_write( init_cmds[idx] , init_data[idx] );
    }

    return err;
}

static zephyr_err_t bsp_lc709204f_write( uint8_t cmd , uint16_t data )
{
    bsp_lc709204f_word_t to_write = {
            .cmd = cmd,
            .data = data,
            .crc  = 0x00,
    };
    to_write.crc = bsp_lc709204f_compute_crc( to_write );

    return bsp_lc709204f_write_cmd(to_write);

}


static zephyr_err_t bsp_lc709204f_read( uint8_t cmd , uint16_t* data )
{
    bsp_lc709204f_word_t to_read = {
            .cmd = cmd,
            .data = *data,
            .crc  = 0x00,
    };
    zephyr_err_t err = bsp_lc709204f_read_cmd( &to_read);
    if( ZEPHYR_ERR_SUCCESS == err )
    {
        /*If the CRCs match*/
        if( to_read.crc == bsp_lc709204f_compute_crc(to_read) )
        {
            *data = to_read.data;
        }
        else
        {
            err = -EILSEQ;
        }
    }
    return err;
}

/*Low level read/write methods*/

static zephyr_err_t bsp_lc709204f_write_cmd( bsp_lc709204f_word_t cmd )
{
    uint8_t write_amount = sizeof(uint8_t) + sizeof(uint16_t) + sizeof(uint8_t);
    uint8_t write_buffer[sizeof(uint8_t) + sizeof(uint16_t) + sizeof(uint8_t)] = {
            [BSP_LC709204F_CMD_POS] = cmd.cmd,
            [BSP_LC709204F_DATA_LB_POS] = (cmd.data & 0x00FF),
            [BSP_LC709204F_DATA_UB_POS] = (cmd.data & 0xFF00) >> 0x8,
            [BSP_LC709204F_CRC_POS] = cmd.crc,
    };

    return i2c_write( lc709204f.i2c_device , write_buffer , write_amount , lc709204f.slave_address);
}

static zephyr_err_t bsp_lc709204f_read_cmd( bsp_lc709204f_word_t* cmd )
{
    uint8_t* data_ptr = (uint8_t*) &cmd->data;

    return i2c_read( lc709204f.i2c_device , data_ptr , sizeof(uint16_t) , lc709204f.slave_address);
}

static uint8_t bsp_lc709204f_compute_crc( bsp_lc709204f_word_t cmd )
{
    uint8_t ret_val = 0;

    ret_val = bsp_lc709204f_compute_crc_byte( ret_val , ((uint8_t) lc709204f.slave_address) << 1 );
    ret_val = bsp_lc709204f_compute_crc_byte( ret_val , cmd.cmd );
    ret_val = bsp_lc709204f_compute_crc_byte( ret_val , 0xFF & cmd.data );
    ret_val = bsp_lc709204f_compute_crc_byte( ret_val , (0xFF00 & cmd.data) >> 0x8 );

    return ret_val;
}

static uint8_t bsp_lc709204f_compute_crc_byte( uint8_t initial_val , uint8_t data )
{
    uint8_t     ret_val = 0;
    uint16_t    temp_val = (data ^ initial_val) << 0x8;

    for( uint8_t idx = 0 ; idx < 8 ; idx++ )
    {
        /*Check if MSB is set*/
        if( temp_val & 0x8000 )
        {
            temp_val ^= CRC_8_ATM_POLY;
        }
        temp_val = temp_val << 0x1;
    }
    ret_val = (temp_val >> 0x8);

    return ret_val;
}