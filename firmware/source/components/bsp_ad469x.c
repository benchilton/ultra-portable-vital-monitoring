/*
 *  @name          bsp_ad469x.c
 *  @date          08/10/22
 *  @author        Benjamin Chilton
 *  @university_id bdc1g19
 *
 *  Note functions in this file were written with aid or inspiration from the Analog Devices provided example
 *  MBed0S application found at https://os.mbed.com/teams/AnalogDevices/code/EVAL-AD4696//file/edd760d6380f/app/.
 */

/**
# Copyright (C) 2023 Analog Devices, Inc.
#
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without modification,
# are permitted provided that the following conditions are met:
# - Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.
# - Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in
# the documentation and/or other materials provided with the
# distribution.
# - Neither the name of Analog Devices, Inc. nor the names of its
# contributors may be used to endorse or promote products derived
# from this software without specific prior written permission.
# - The use of this software may or may not infringe the patent rights
# of one or more patent holders. This license does not release you
# from the requirement that you obtain separate licenses from these
# patent holders to use this software.
# - Use of the software either in source or binary form, must be run
# on or directly connected to an Analog Devices Inc. component.
#
# THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
# INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT, MERCHANTABILITY AND FITNESS FOR A
# PARTICULAR PURPOSE ARE DISCLAIMED.
#
# IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, INTELLECTUAL PROPERTY
# RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
# BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
# STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
# THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * **/
/****************************** - Library Includes - *******************************/

#include <string.h>

#include <zephyr/kernel.h>
#include <device.h>

#include <drivers/spi.h>
#include <drivers/gpio.h>

/******************************** - User Includes - ********************************/

#include "bsp_std_includes.h"

#include "bsp_ad469x.h"

#include "bsp_ad469x_calibrations.h"

/*********************************** - Defines - ***********************************/

#define BSP_AD469X_CS_CNV_SHARED  (1U)

/**Helper MACROS**/

#define AD469X_REGISTER_POS(x)     ((1U) << x)

/**SPI related defines**/

#define AD469x_SPI_DEVICE         DEVICE_DT_GET( DT_PARENT(DT_NODELABEL(ad469x)) )

#define AD469x_SPI_CS_CONTROL     SPI_CS_GPIOS_DT_SPEC_GET( DT_NODELABEL(ad469x) )

#define AD469x_SPI_CS_DELAY       DT_PROP( DT_CHILD( DT_NODELABEL( ad469x ) , properties ) , cs_delay)
/*In Hz*/
#define AD469X_REG_MODE_SPI_FREQ  DT_PROP( DT_CHILD( DT_NODELABEL( ad469x ) , properties ) , spi_frequency_reg)
#define AD469X_CNV_MODE_SPI_FREQ  DT_PROP( DT_CHILD( DT_NODELABEL( ad469x ) , properties ) , spi_frequency_cnv)

/**GPIO related defines**/

#define AD469X_GPIO_CONVERT_SPEC  GPIO_DT_SPEC_GET( DT_CHILD( DT_NODELABEL( ad469x ) , properties ) , adc_convert_gpios )
#define AD469X_GPIO_BUSY_SPEC     GPIO_DT_SPEC_GET( DT_CHILD( DT_NODELABEL( ad469x ) , properties ) , adc_busy_gpios )
#define AD469X_GPIO_RESET_SPEC    GPIO_DT_SPEC_GET( DT_CHILD( DT_NODELABEL( ad469x ) , properties ) , adc_reset_gpios )

/**AD469X Defines**/

#define AD469X_WRITE_REGISTER       (0U)
#define AD469X_READ_REGISTER        (1U)

#define AD469X_GPIO_CONVERT_EVENT   BIT(0)
#define AD469X_GPIO_BUSY_EVENT      BIT(1)
#define AD469X_GPIO_RESET_EVENT     BIT(2)

#define AD469X_SPI_WRITE_BUF_POS    (0U)
#define AD469X_SPI_READ_BUF_POS_1   (1U)
#define AD469X_SPI_READ_BUF_POS_2   (2U)

#define AD469X_SPI_WRITE            (0U)
#define AD469X_SPI_READ             (1U)

#define AD469X_AC_ENABLE_POS        (0U)

#define SIZE_OF_STATUS_BITS         (5U)

/**Advanced sequencer defaults**/

#define AD469X_CHANNEL_DEFAULTS_LEN         DT_PROP_LEN( DT_NODELABEL( ad469x_settings ) , adc_channels )
#define AD469X_CHANNEL_DEFAULTS             DT_PROP( DT_NODELABEL( ad469x_settings ) , adc_channels )

#define AD469X_AS_DEFAULTS_LEN              DT_PROP_LEN( DT_NODELABEL( ad469x_settings ) , adv_sequencer_slots )
#define AD469X_AS_DEFAULTS                  DT_PROP( DT_NODELABEL( ad469x_settings ) , adv_sequencer_slots )


/* AD469x Sequencer Lower Byte Register */
#define AD469x_REG_SEQ_LB                   AD469x_REG_STD_SEQ_CONFIG

/* AD469x Sequencer Upper Byte Register */
#define AD469x_REG_SEQ_UB                   (AD469x_REG_STD_SEQ_CONFIG + 0x01)

/* AD469x registers */
#define AD469x_REG_SPI_CONFIG_A             0x000
#define AD469x_REG_IF_CONFIG_B              0x001
#define AD469x_REG_DEVICE_TYPE              0x003
#define AD469x_REG_DEVICE_ID_L              0x004
#define AD469x_REG_DEVICE_ID_H              0x005
#define AD469x_REG_SCRATCH_PAD              0x00A
#define AD469x_REG_VENDOR_L                 0x00C
#define AD469x_REG_VENDOR_H                 0x00D
#define AD469x_REG_LOOP_MODE                0x00E
#define AD469x_REG_IF_CONFIG_C              0x010
#define AD469x_REG_IF_STATUS                0x011
#define AD469x_REG_STATUS                   0x014
#define AD469x_REG_ALERT_STATUS1            0x015
#define AD469x_REG_ALERT_STATUS2            0x016
#define AD469x_REG_ALERT_STATUS3            0x017
#define AD469x_REG_ALERT_STATUS4            0x018
#define AD469x_REG_CLAMP_STATUS1            0x01A
#define AD469x_REG_CLAMP_STATUS2            0x01B
#define AD469x_REG_SETUP                    0x020
#define AD469x_REG_REF_CTRL                 0x021
#define AD469x_REG_SEQ_CTRL                 0x022
#define AD469x_REG_AC_CTRL                  0x023
#define AD469x_REG_STD_SEQ_CONFIG           0x024
#define AD469x_REG_GPIO_CTRL                0x026
#define AD469x_REG_GP_MODE                  0x027
#define AD469x_REG_GPIO_STATE               0x028
#define AD469x_REG_TEMP_CTRL                0x029
#define AD469x_REG_CONFIG_IN(x)             ((x & 0x0F) | 0x30)
#define AD469x_REG_AS_SLOT(x)               ((x & 0x7F) | 0x100)

/* 5-bit SDI Conversion Mode Commands */
#define AD469x_CMD_REG_CONFIG_MODE          (0x0A << 3)
#define AD469x_CMD_SEL_TEMP_SNSOR_CH        (0x0F << 3)
#define AD469x_CMD_CONFIG_CH_SEL(x)         ((0x10 | (0x0F & x)) << 3)

/* AD469x_REG_SETUP */
#define AD469x_SETUP_IF_MODE_MASK               (0x01 << 2)
#define AD469x_SETUP_IF_MODE_CONV               (0x01 << 2)
#define AD469x_SETUP_CYC_CTRL_MASK              (0x01 << 1)
#define AD469x_SETUP_CYC_CTRL_SINGLE(x)         ((x & 0x01) << 1)
//Changed
#define AD469x_SETUP_STATUSBIT_MODE_MASK        (0x01 << 5)
#define AD469x_SETUP_STATUSBIT_MODE_CONV        (0x01 << 5)

/* AD469x_REG_GP_MODE */
#define AD469x_GP_MODE_BUSY_GP_EN_MASK          (0x01 << 1)
#define AD469x_GP_MODE_BUSY_GP_EN(x)            ((x & 0x01) << 1)
#define AD469x_GP_MODE_BUSY_GP_SEL_MASK         (0x01 << 4)
#define AD469x_GP_MODE_BUSY_GP_SEL(x)           ((x & 0x01) << 5)

/* AD469x_REG_SEQ_CTRL */
#define AD469x_SEQ_CTRL_STD_SEQ_EN_MASK         (0x01 << 7)
#define AD469x_SEQ_CTRL_STD_SEQ_EN(x)           ((x & 0x01) << 7)
#define AD469x_SEQ_CTRL_NUM_SLOTS_AS_MASK       (0x7f << 0)
#define AD469x_SEQ_CTRL_NUM_SLOTS_AS(x)         ((x & 0x7f) << 0)

/* AD469x_REG_TEMP_CTRL */
#define AD469x_REG_TEMP_CTRL_TEMP_EN_MASK       (0x01 << 0)
#define AD469x_REG_TEMP_CTRL_TEMP_EN(x)         ((x & 0x01) << 0)

/* AD469x_REG_AS_SLOT */
#define AD469x_REG_AS_SLOT_INX(x)               ((x & 0x0f) << 0)

/* AD469x_REG_IF_CONFIG_C */
#define AD469x_REG_IF_CONFIG_C_MB_STRICT_MASK   (0x01 << 5)
#define AD469x_REG_IF_CONFIG_C_MB_STRICT(x)     ((x & 0x01) << 5)

/* AD469x_REG_CONFIG_INn */
#define AD469x_REG_CONFIG_IN_OSR_MASK           (0x03 << 0)
#define AD469x_REG_CONFIG_IN_OSR(x)             ((x & 0x03) << 0)
#define AD469x_REG_CONFIG_IN_HIZ_EN_MASK        (0x01 << 3)
#define AD469x_REG_CONFIG_IN_HIZ_EN(x)          ((x & 0x01) << 3)
#define AD469x_REG_CONFIG_IN_PAIR_MASK          (0x03 << 4)
#define AD469x_REG_CONFIG_IN_PAIR(x)            ((x & 0x03) << 4)
#define AD469x_REG_CONFIG_IN_MODE_MASK          (0x01 << 6)
#define AD469x_REG_CONFIG_IN_MODE(x)            ((x & 0x01) << 6)
#define AD469x_REG_CONFIG_IN_TD_EN_MASK         (0x01 << 7)
#define AD469x_REG_CONFIG_IN_TD_EN(x)           ((x & 0x01) << 7)

#define AD469x_REG_OFFSET_IN_BASE               (0x00A0)
#define AD469x_REG_GAIN_IN_BASE                 (0x00C0)

/************************************ - Enums - ************************************/

typedef enum
{
    BSP_AD469X_REG_SPI_CONFIG_A_POS_SW_RST_LSB    = AD469X_REGISTER_POS(0U),
    BSP_AD469X_REG_SPI_CONFIG_A_POS_ADDR_DIR      = AD469X_REGISTER_POS(5U),
    BSP_AD469X_REG_SPI_CONFIG_A_POS_SW_SW_RST_MSB = AD469X_REGISTER_POS(7U),
} bsp_ad469x_reg_spi_config_a_bits_e;

typedef enum
{
    BSP_AD469X_REG_SETUP_POS_RESERVED_1  = AD469X_REGISTER_POS(0U),
    BSP_AD469X_REG_SETUP_POS_CYC_CTRL    = AD469X_REGISTER_POS(1U),
    BSP_AD469X_REG_SETUP_POS_SPI_MODE    = AD469X_REGISTER_POS(2U),
    BSP_AD469X_REG_SETUP_POS_RESERVED_2  = AD469X_REGISTER_POS(3U),
    BSP_AD469X_REG_SETUP_POS_LDO_EN      = AD469X_REGISTER_POS(4U),
    BSP_AD469X_REG_SETUP_POS_STATUS_EN   = AD469X_REGISTER_POS(5U),
    BSP_AD469X_REG_SETUP_POS_SDO_STATE   = AD469X_REGISTER_POS(6U),
    BSP_AD469X_REG_SETUP_POS_ALERT_MODE  = AD469X_REGISTER_POS(7U),
} bsp_ad469x_reg_setup_bits_e;


typedef enum
{
    BSP_AD469X_REG_AC_CTRL_POS_AC_EN      = AD469X_REGISTER_POS(0U),
    BSP_AD469X_REG_AC_CTRL_POS_AC_CYC_0   = AD469X_REGISTER_POS(1U),
    BSP_AD469X_REG_AC_CTRL_POS_AC_CYC_2   = AD469X_REGISTER_POS(2U),
    BSP_AD469X_REG_AC_CTRL_POS_AC_CYC_3   = AD469X_REGISTER_POS(3U),
    BSP_AD469X_REG_AC_CTRL_POS_RESERVED   = (AD469X_REGISTER_POS(4U) | AD469X_REGISTER_POS(5U) |
                                            AD469X_REGISTER_POS(6U) | AD469X_REGISTER_POS(7U) ),
} bsp_ad469x_reg_ac_ctrl_bits_e;

typedef enum
{
    BSP_AD469X_REG_AC_CTRL_AC_EN           = AD469X_REGISTER_POS(0U),
    BSP_AD469X_REG_AC_CTRL_AC_PERIOD_10US  = (0x00 << 1),
    BSP_AD469X_REG_AC_CTRL_AC_PERIOD_20US  = (0x01 << 1),
    BSP_AD469X_REG_AC_CTRL_AC_PERIOD_40US  = (0x02 << 1),
    BSP_AD469X_REG_AC_CTRL_AC_PERIOD_80US  = (0x03 << 1),
    BSP_AD469X_REG_AC_CTRL_AC_PERIOD_100US = (0x04 << 1),
    BSP_AD469X_REG_AC_CTRL_AC_PERIOD_200US = (0x05 << 1),
    BSP_AD469X_REG_AC_CTRL_AC_PERIOD_400US = (0x06 << 1),
    BSP_AD469X_REG_AC_CTRL_AC_PERIOD_800US = (0x07 << 1),
} bsp_ad469x_reg_ac_ctrl_settings_e;

typedef enum
{
    BSP_AD469X_REG_GP_MODE_POS_ALERT_GP_EN  = AD469X_REGISTER_POS(0U),
    BSP_AD469X_REG_GP_MODE_POS_BUSY_GP_EN   = AD469X_REGISTER_POS(1U),
    BSP_AD469X_REG_GP_MODE_POS_SDO_MODE     = (AD469X_REGISTER_POS(2U) | AD469X_REGISTER_POS(3U)),
    BSP_AD469X_REG_GP_MODE_POS_ALERT_GP_SEL = AD469X_REGISTER_POS(4U),
    BSP_AD469X_REG_GP_MODE_POS_BUSY_GP_SEL  = AD469X_REGISTER_POS(5U),
    BSP_AD469X_REG_GP_MODE_POS_RESERVED     = AD469X_REGISTER_POS(6U),
    BSP_AD469X_REG_GP_MODE_POS_OV_ALT_MODE  = AD469X_REGISTER_POS(7U),
}bsp_ad469x_reg_gp_mode_bits_e;

typedef enum
{
    BSP_AD469X_REG_GP_CTRL_POS_GPO0_EN = AD469X_REGISTER_POS(0U),
    BSP_AD469X_REG_GP_CTRL_POS_GPO1_EN = AD469X_REGISTER_POS(1U),
    BSP_AD469X_REG_GP_CTRL_POS_GPO2_EN = AD469X_REGISTER_POS(2U),
    BSP_AD469X_REG_GP_CTRL_POS_GPO3_EN = AD469X_REGISTER_POS(3U),
    BSP_AD469X_REG_GP_CTRL_POS_GPI0_EN = AD469X_REGISTER_POS(4U),
    BSP_AD469X_REG_GP_CTRL_POS_GPI1_EN = AD469X_REGISTER_POS(5U),
    BSP_AD469X_REG_GP_CTRL_POS_GPI2_EN = AD469X_REGISTER_POS(6U),
    BSP_AD469X_REG_GP_CTRL_POS_GPI3_EN = AD469X_REGISTER_POS(7U),
}bsp_ad469x_reg_gp_ctrl_bits_e;


typedef enum
{
    BSP_AD469X_REG_SEQ_CTRL_POS_NUM_SLOTS_AS  = 0b0111111,
    BSP_AD469X_REG_SEQ_CTRL_POS_STD_SEQ_EN    = AD469X_REGISTER_POS(7U)
}bsp_ad469x_reg_seq_ctrl_bits_e;

/**AS_SLOTn register enumerations**/
typedef enum
{
    BSP_AD469X_REG_AS_SLOTN_POS_SLOT_INX = AD469X_REGISTER_POS(0U) | AD469X_REGISTER_POS(1U) |
                                           AD469X_REGISTER_POS(2U) | AD469X_REGISTER_POS(3U),
    BSP_AD469X_REG_AS_SLOTN_POS_RESERVED = AD469X_REGISTER_POS(4U) | AD469X_REGISTER_POS(5U) |
                                           AD469X_REGISTER_POS(6U) | AD469X_REGISTER_POS(7U),
}bsp_ad469x_reg_as_slotn_bits_e;

/********************************* - Structures - **********************************/

typedef struct {

    /**Device ID**/
    bsp_ad469x_supported_dev_ids_e  device_id;

    /**SPI related properties**/
    const zephyr_device_t*          spi_device;
    zephyr_spi_cs_control_t         spi_cs_control;
    zephyr_spi_config_t             spi_config;

    bsp_gpio_prop_t                 gpio_convert;
    bsp_gpio_prop_t                 gpio_busy;
    bsp_gpio_prop_t                 gpio_reset;

    zephyr_k_event_t                gpio_events;

    /**ADC Related properties**/
    bsp_ad469x_channel_sequencing_e channel_sequence;
    /*OSRs for each channel*/
    bsp_ad469x_osr_ratios_e         channel_osr[AD469x_CHANNEL_NO];
    /*Is the channel paired with COM, REFGND, or another channel*/
    bsp_ad469x_reference_e          channel_reference[AD469x_CHANNEL_NO];
    uint8_t                         channel_resolution[AD469x_CHANNEL_NO];
    bsp_ad469x_calibration_t        channel_calibration[AD469x_CHANNEL_NO];
    /*Channel slots for the advanced sequencer*/
    uint8_t                         channel_slots[AD469x_SLOTS_NO];
    uint8_t                         num_slots;
    /**/
    uint8_t                         capture_data_width;
    bool                            temp_enabled;
    // Calibration for temp sensor

    volatile bool                   continous_data_read;
    volatile bool                   conversion_mode;
    uint32_t                        previous_read_channel;


} bsp_ad469x_device_t;


typedef union {

    struct {
        uint8_t msbaddress : 7;
        uint8_t readwrite  : 1;
        uint8_t lsbaddress : 8;
    };

    uint16_t data;

}bsp_ad469x_address_t;

/**************************** - Function Prototypes - ******************************/

/**Register access methods**/
static zephyr_err_t                ad469x_spi_write(bsp_ad469x_address_t reg , uint8_t data );
static zephyr_err_t                ad469x_spi_read(bsp_ad469x_address_t reg , uint8_t* data );

__attribute__((unused))
static zephyr_err_t                ad469x_spi_read_mask(  uint16_t reg_addr , uint8_t mask, uint8_t *data );
static zephyr_err_t                ad469x_spi_write_set_bits(uint16_t reg_addr , uint8_t mask , uint8_t data );
static zephyr_err_t                ad469x_spi_set_bit( uint16_t reg_addr , uint8_t bit_pos , uint8_t bit_val );
static inline bsp_ad469x_address_t ad469x_uint16_t_to_address(uint16_t input_int);

/**Callbacks**/
static void ad469x_gpio_convert_cb(const zephyr_device_t* port, zephyr_gpio_callback_t* cb, gpio_port_pins_t pins);
static void ad469x_gpio_busy_cb(const zephyr_device_t* port, zephyr_gpio_callback_t* cb, gpio_port_pins_t pins);
static void ad469x_gpio_reset_cb(const zephyr_device_t* port, zephyr_gpio_callback_t* cb, gpio_port_pins_t pins);

/**GPIO private methods**/
static zephyr_err_t                ad469x_gpio_busy_init(void);
static zephyr_err_t                ad469x_gpio_convert_init(void);
static zephyr_err_t                ad469x_gpio_reset_init(void);

/**SPI private methods**/
static void                        ad469x_spi_update_frequency( uint32_t new_frequency );

/**Standard sequencer methods**/
static zephyr_err_t                ad469x_seq_osr_clear();

#if STANDARD_SEQUENCER
static zephyr_err_t                bsp_ad469x_trigger_std_seq( k_timeout_t time_to_wait );
#endif

/**Advanced sequencer private methods**/
static zephyr_err_t                ad469x_adv_seq_init();
static inline uint8_t              ad469x_device_resolution( bsp_ad469x_osr_ratios_e in_osr );
static zephyr_err_t                ad469x_adv_seq_osr_get_util_data(uint16_t current_sample , uint32_t *sample );
static zephyr_err_t                ad469x_set_channel_calibration( uint8_t channel , uint16_t gain , int16_t offset);

/**Other Init methods**/
static void ad469x_queue_init(void);


/**Debug**/
void ad469x_debug_read_reg( uint8_t reg_to_read );
void ad469x_debug_print_bits( uint8_t size_in_bytes , uint32_t value );

/********************************* - Constants - ***********************************/
/********************************* - Variables - ***********************************/

/*Queues used by the module to export data to high levels*/
static zephyr_k_msgq_t      adc_channel_queue[AD469x_CHANNEL_NO + 1];
static bsp_ad469x_reading_t adc_queue_buffers[AD469x_CHANNEL_NO + 1][AD469X_QUEUE_LENGTH];
static zephyr_k_event_t     adc_read_event;

static bsp_ad469x_advanced_sequence_t ad469x_default_as = {
        .slots_used = AD469X_AS_DEFAULTS_LEN,
        .slots      = AD469X_AS_DEFAULTS,
};

static bsp_ad469x_device_t ad469x = {

        .device_id = ID_AD4696,
        /**SPI**/
        .spi_device = AD469x_SPI_DEVICE,
        .spi_cs_control = {
            .gpio  = AD469x_SPI_CS_CONTROL,
            .delay = AD469x_SPI_CS_DELAY,
        },
        .spi_config = {
                .cs = &ad469x.spi_cs_control,
                .frequency = AD469X_REG_MODE_SPI_FREQ,/*Device starts in register configuration mode*/
                .operation = SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_WORD_SET(8U) | SPI_MODE_CPOL | SPI_MODE_CPHA |
                             SPI_LINES_SINGLE | SPI_FULL_DUPLEX | SPI_FRAME_FORMAT_MOTOROLA,
                .slave = 0
        },

        /**GPIO**/
        .gpio_convert = {
                .gpio_spec     = AD469X_GPIO_CONVERT_SPEC,
        },
        .gpio_busy    = {
                .gpio_spec     = AD469X_GPIO_BUSY_SPEC,
        },
        .gpio_reset    = {
                .gpio_spec     = AD469X_GPIO_RESET_SPEC,
        },
        /**ADC**/
        .channel_sequence = AD469X_STANDARD_SEQ,
        .channel_osr = {
                AD469X_OSR_1 , AD469X_OSR_1 , AD469X_OSR_1 , AD469X_OSR_1 ,
                AD469X_OSR_1 , AD469X_OSR_1 , AD469X_OSR_1 , AD469X_OSR_1 ,
                AD469X_OSR_1 , AD469X_OSR_1 , AD469X_OSR_1 , AD469X_OSR_1 ,
                AD469X_OSR_1 , AD469X_OSR_1 , AD469X_OSR_1 , AD469X_OSR_1
                },
        .num_slots    = 0U,
        .temp_enabled = false,
        .channel_calibration = {
                {1 , 0}, {1 , 0}, {1 , 0}, {1 , 0},
                {1 , 0}, {1 , 0}, {1 , 0}, {1 , 0},
                {1 , 0}, {1 , 0}, {1 , 0}, {1 , 0},
                {1 , 0}, {1 , 0}, {1 , 0}, {1 , 0},

        },

        .continous_data_read   = false,
        .previous_read_channel = 0,

};

/***************************** - Public Functions - ********************************/

zephyr_err_t bsp_ad469x_init()
{
    zephyr_err_t err = -ENODEV;

    if( (device_is_ready(ad469x.spi_device) == true) &&
        (device_is_ready(ad469x.gpio_convert.gpio_spec.port) == true) &&
        (device_is_ready(ad469x.gpio_busy.gpio_spec.port) == true) &&
        (device_is_ready(ad469x.gpio_reset.gpio_spec.port) == true) &&
        (device_is_ready(ad469x.spi_cs_control.gpio.port) == true) )
    {

        k_event_init(&ad469x.gpio_events);
        ad469x_queue_init();

        err = ad469x_gpio_reset_init();
        if( ZEPHYR_ERR_SUCCESS == err )
        {
            err = ad469x_gpio_busy_init();
            if( ZEPHYR_ERR_SUCCESS == err )
            {
                #ifdef BSP_AD469X_CS_CNV_SHARED
                    err = ZEPHYR_ERR_SUCCESS;
                #else
                    err = bsp_ad469x_gpio_convert_init();
                #endif
                if( ZEPHYR_ERR_SUCCESS == err )
                {
                    /*Init the ADC registers*/
                    err = bsp_ad469x_trigger_software_reset();
                    if( ZEPHYR_ERR_SUCCESS == err )
                    {
                        err = bsp_ad469x_set_reg_access_mode(AD469x_BYTE_ACCESS);
                        if (ZEPHYR_ERR_SUCCESS == err)
                        {
                            err = bsp_ad469x_init_busy(AD469X_BUSY_GP0);
                            if (ZEPHYR_ERR_SUCCESS == err)
                            {
                                err = ad469x_adv_seq_init();
                                if( ZEPHYR_ERR_SUCCESS == err )
                                {
                                    /*Start the standard sequencer*/
                                    err = bsp_ad469x_setup_advanced_sequencer( ad469x_default_as );

                                    //ad469x_set_channel_calibration( 13 , BSP_AD469X_CALIBRATIONS_TEMPERATURE_GAIN , BSP_AD469X_CALIBRATIONS_TEMPERATURE_OFFSET );

                                    if (ZEPHYR_ERR_SUCCESS == err)
                                    {
                                        err = bsp_ad469x_start_advanced_sequencer(true);
                                    }

                                }
                            }
                        }
                    }
                }
            }
        }
    }
    return err;
}


bsp_ad469x_channel_sequencing_e bsp_ad469x_get_channel_sequencing()
{
    return ad469x.channel_sequence;
};

zephyr_k_event_t* bsp_ad469x_get_data_event_handle(void)
{
    return &adc_read_event;
}

zephyr_k_msgq_t* bsp_ad469x_get_data_queues_handle( void )
{
    return &adc_channel_queue[0];
}

/**
 *
 * @name      bsp_ad469x_reg_write
 *
 * @param[in] register_address The 15-bit register address to be read.
 * @param[in] data             The byte to be written to the register chosen.
 *
 * @returns   ZEPHYR_ERR_SUCCESS if successful, otherwise a -<errno> value.
 *
 * @notes
 *
 */
zephyr_err_t bsp_ad469x_reg_write( uint16_t register_address , uint8_t data )
{

    bsp_ad469x_address_t reg_addr = ad469x_uint16_t_to_address( register_address );
    reg_addr.readwrite = AD469X_WRITE_REGISTER;

    return ad469x_spi_write(reg_addr, data);
}

/**
 *
 * @name       bsp_ad469x_reg_read
 *
 * @param[in]  register_address The 15-bit register address to be read.
 *
 * @param[out] data             The byte of data stored at register_address.
 *
 * @returns    ZEPHYR_ERR_SUCCESS if successful, otherwise a -<errno> value.
 *
 * @notes
 *
 */
zephyr_err_t bsp_ad469x_reg_read( uint16_t register_address , uint8_t* data )
{

    bsp_ad469x_address_t reg_addr = ad469x_uint16_t_to_address( register_address );
    reg_addr.readwrite = AD469X_READ_REGISTER;

    return ad469x_spi_read(reg_addr, data);

}

/**
 *
 * @name       bsp_ad469x_set_reg_access_mode
 *
 * @param[in]  access Access mode to set
 *
 * @returns    ZEPHYR_ERR_SUCCESS if successful, otherwise a -<errno> value.
 *
 * @notes      Configure register access mode
 *
 */
zephyr_err_t bsp_ad469x_set_reg_access_mode( bsp_ad469x_reg_access_e access )
{
    return ad469x_spi_write_set_bits(AD469x_REG_IF_CONFIG_C, AD469x_REG_IF_CONFIG_C_MB_STRICT_MASK,
                                     AD469x_REG_IF_CONFIG_C_MB_STRICT(access));
}

/**
 *
 * @name       bsp_ad469x_adv_seq_osr
 *
 * @param[in]  channel Channel to set
 * @param[in]  ratio   OSR to set
 *
 * @returns    ZEPHYR_ERR_SUCCESS if successful, otherwise a -<errno> value.
 *
 * @notes      Configure over sampling ratio in advanced sequencer mode
 *
 */
zephyr_err_t bsp_ad469x_adv_seq_osr( uint16_t channel , bsp_ad469x_osr_ratios_e ratio)
{
    zephyr_err_t err = -EINVAL;

    if ( (ad469x.channel_sequence != AD469X_SINGLE_CYCLE) && (ad469x.channel_sequence != AD469X_TWO_CYCLE) && (channel < AD469x_CHANNEL_NO ) )
    {
        err = ad469x_spi_write_set_bits(AD469x_REG_CONFIG_IN(channel),
                                        AD469x_REG_CONFIG_IN_OSR_MASK, AD469x_REG_CONFIG_IN_OSR(ratio));

        if( err == ZEPHYR_ERR_SUCCESS)
        {
            ad469x.channel_resolution[channel] = ad469x_device_resolution(ratio);
            /* Set storage to maximum data width */
            ad469x.capture_data_width   = ad469x_device_resolution(AD469X_OSR_64);
        }

    }

    return err;
}

zephyr_err_t bsp_ad469x_adv_seq_pair( uint16_t channel , bsp_ad469x_reference_e reference_pairing)
{
    zephyr_err_t err = -EINVAL;

    if ( (ad469x.channel_sequence != AD469X_SINGLE_CYCLE) && (ad469x.channel_sequence != AD469X_TWO_CYCLE) && (channel < AD469x_CHANNEL_NO ) )
    {
        err = ad469x_spi_write_set_bits(AD469x_REG_CONFIG_IN(channel),
                                        AD469x_REG_CONFIG_IN_PAIR_MASK,
                                        AD469x_REG_CONFIG_IN_PAIR(reference_pairing));

        if( ZEPHYR_ERR_SUCCESS == err )
        {
            err = ad469x_spi_write_set_bits(AD469x_REG_CONFIG_IN(channel),
                                            AD469x_REG_CONFIG_IN_MODE_MASK,
                                            AD469x_REG_CONFIG_IN_MODE( (reference_pairing != AD469X_PAIR_REFGND) ));
        }

    }

    return err;
}


/**
 *
 * @name       bsp_ad469x_std_seq_osr
 *
 * @param[in]  ratio Channel sequence
 *
 * @returns    ZEPHYR_ERR_SUCCESS if successful, otherwise a -<errno> value.
 *
 * @notes      Configure over sampling ratio in standard sequencer mode
 *
 */
zephyr_err_t bsp_ad469x_std_seq_osr( bsp_ad469x_osr_ratios_e ratio )
{
    zephyr_err_t err = -EINVAL;

    if ( (ad469x.channel_sequence != AD469X_SINGLE_CYCLE) && (ad469x.channel_sequence != AD469X_TWO_CYCLE) )
    {
        err = ad469x_spi_write_set_bits(AD469x_REG_CONFIG_IN(0),
                                        AD469x_REG_CONFIG_IN_OSR_MASK, AD469x_REG_CONFIG_IN_OSR(ratio));

        if( ZEPHYR_ERR_SUCCESS == err )
        {

            ad469x.capture_data_width = ad469x_device_resolution(ratio);

        }
    }

    return err;
}

/**
 *
 * @name       bsp_ad469x_set_channel_sequence
 *
 * @param[in]  sequence Channel sequence
 *
 * @returns    ZEPHYR_ERR_SUCCESS if successful, otherwise a -<errno> value.
 *
 * @notes      Set channel sequence.
 *
 */
zephyr_err_t bsp_ad469x_set_channel_sequence( bsp_ad469x_channel_sequencing_e sequence )
{
    zephyr_err_t err = -EINVAL;

    switch (sequence)
    {
        case AD469X_SINGLE_CYCLE:
            err = ad469x_spi_write_set_bits(AD469x_REG_SEQ_CTRL,
                                            AD469x_SEQ_CTRL_STD_SEQ_EN_MASK, AD469x_SEQ_CTRL_STD_SEQ_EN(0));
            if( ZEPHYR_ERR_SUCCESS == err )
            {

                err = ad469x_spi_write_set_bits(AD469x_REG_SEQ_CTRL, AD469x_SEQ_CTRL_NUM_SLOTS_AS_MASK,
                                                AD469x_SEQ_CTRL_NUM_SLOTS_AS(0));
                if( ZEPHYR_ERR_SUCCESS == err )
                {
                    err = ad469x_spi_write_set_bits(AD469x_REG_SETUP, AD469x_SETUP_CYC_CTRL_MASK,
                                                    AD469x_SETUP_CYC_CTRL_SINGLE(0));

                    if( ZEPHYR_ERR_SUCCESS == err )
                    {
                        err = ad469x_seq_osr_clear();
                    }

                }
            }
        break;
        case AD469X_TWO_CYCLE:
            err = ad469x_spi_write_set_bits(AD469x_REG_SEQ_CTRL, AD469x_SEQ_CTRL_STD_SEQ_EN_MASK,
                                            AD469x_SEQ_CTRL_STD_SEQ_EN(0));
            if( ZEPHYR_ERR_SUCCESS == err )
            {

                err = ad469x_spi_write_set_bits(AD469x_REG_SEQ_CTRL, AD469x_SEQ_CTRL_NUM_SLOTS_AS_MASK,
                                                AD469x_SEQ_CTRL_NUM_SLOTS_AS(0));
                if (ZEPHYR_ERR_SUCCESS == err)
                {

                    err = ad469x_spi_write_set_bits(AD469x_REG_SETUP, AD469x_SETUP_CYC_CTRL_MASK,
                                                    AD469x_SETUP_CYC_CTRL_SINGLE(1));
                    if (ZEPHYR_ERR_SUCCESS == err)
                    {
                        err = ad469x_seq_osr_clear();
                    }
                }
            }
        break;
        case AD469X_STANDARD_SEQ:
            err = ad469x_spi_write_set_bits(AD469x_REG_SEQ_CTRL, AD469x_SEQ_CTRL_STD_SEQ_EN_MASK,
                                            AD469x_SEQ_CTRL_STD_SEQ_EN(1));
        break;
        case AD469X_ADVANCED_SEQ:
            err = ad469x_spi_write_set_bits(AD469x_REG_SEQ_CTRL, AD469x_SEQ_CTRL_STD_SEQ_EN_MASK,
                                            AD469x_SEQ_CTRL_STD_SEQ_EN(0b0));
        break;
        default:
            err = -EINVAL;
        break;
    }

    ad469x.channel_sequence = sequence;

    return err;
}

/**
 *
 * @name       bsp_ad469x_adv_sequence_set_num_slots
 *
 * @param[in]  num_slots Number of slots, max value = 0x7f
 *
 * @returns    ZEPHYR_ERR_SUCCESS if successful, otherwise a -<errno> value.
 *
 * @notes      Configure advanced sequencer number of slots, temp channel not included
 *
 */
zephyr_err_t bsp_ad469x_adv_sequence_set_num_slots( uint8_t num_slots )
{
    zephyr_err_t err = -EINVAL;
    uint8_t write_num_slots = 0;

    if ( 0x01 < num_slots )
    {
        write_num_slots = 0x7F & (num_slots - 1);
    }

    err = bsp_ad469x_reg_write( AD469x_REG_SEQ_CTRL , write_num_slots );
    if( ZEPHYR_ERR_SUCCESS == err)
    {
        ad469x.num_slots = num_slots;
    }

    bsp_ad469x_reg_read( AD469x_REG_SEQ_CTRL , &write_num_slots);

    return err;
}

zephyr_err_t bsp_ad469x_toggle_temperature_channel( bool enable )
{
    zephyr_err_t err =  bsp_ad469x_reg_write( AD469x_REG_TEMP_CTRL ,
                                             ( enable == true ) ? AD469x_REG_TEMP_CTRL_TEMP_EN_MASK : 0x00 );
    if( ZEPHYR_ERR_SUCCESS == err )
    {
        ad469x.temp_enabled = enable;
        /*If the temp channel isn't enabled there is no point initialing the temp queue.*/
        k_msgq_init(&adc_channel_queue[AD469x_CHANNEL_TEMP] , (uint8_t*) adc_queue_buffers[AD469x_CHANNEL_TEMP] ,
                    sizeof(bsp_ad469x_reading_t) , AD469X_QUEUE_LENGTH);
    }
    return err;
}

/**
 *
 * @name       bsp_ad469x_adv_sequence_set_slot
 *
 * @param[in]  slot    Slot number, valid range is [0x00, 0x7f]
 * @param[in]  channel Assigned channel, range is [0x00, 0x0f].
 *
 * @returns    ZEPHYR_ERR_SUCCESS if successful, otherwise a -<errno> value.
 *
 * @notes      Advanced sequencer, assign channel to a slot
 *
 */
zephyr_err_t bsp_ad469x_adv_sequence_set_slot( uint8_t slot , uint8_t channel )
{
    zephyr_err_t err = -EINVAL;
    err = bsp_ad469x_reg_write( AD469x_REG_AS_SLOT(slot) , AD469x_REG_AS_SLOT_INX(channel) );

    if( ZEPHYR_ERR_SUCCESS == err)
    {
        ad469x.channel_slots[slot] = channel;
    }

    return err;
}

/**
 * @brief Configure converter busy indicator to the output of the specified port
 * @param [in] dev - ad469x_dev device handler.
 * @param [in] gp_sel - Port.
 * @return \ref SUCCESS in case of success, \ref FAILURE otherwise.
 */
zephyr_err_t bsp_ad469x_init_busy(bsp_ad469x_busy_gp_sel_e gp_sel )
{
    zephyr_err_t err = -EINVAL;

    uint8_t reg_val = BSP_AD469X_REG_GP_CTRL_POS_GPO0_EN;

    err = bsp_ad469x_reg_write( AD469x_REG_GPIO_CTRL , reg_val );

    if( ZEPHYR_ERR_SUCCESS == err )
    {

        /*Enable */
        reg_val = BSP_AD469X_REG_GP_MODE_POS_BUSY_GP_EN | BSP_AD469X_REG_GP_MODE_POS_SDO_MODE;

        err = bsp_ad469x_reg_write( AD469x_REG_GP_MODE , reg_val );

    }

    return err;
}

zephyr_err_t bsp_ad469x_standard_sequencer( bool enable_disable )
{
    zephyr_err_t err = -EINVAL;
    if( true == enable_disable )
    {
        err = bsp_ad469x_set_channel_sequence( AD469X_STANDARD_SEQ );
    }
    else
    {
        err = bsp_ad469x_set_channel_sequence( AD469X_SINGLE_CYCLE );
    }

    return err;
}

/**
 * @brief Enter conversion mode.
 *        To exit conversion mode send a 5 bit conversion mode command
 *        AD469x_CMD_REG_CONFIG_MODE
 * @param [in] dev - ad469x_dev device handler.
 * @return \ref SUCCESS in case of success, \ref FAILURE otherwise.
 */
zephyr_err_t bsp_ad469x_enter_conversion_mode(uint8_t* offset_needed)
{
    zephyr_err_t err = -EIO;

    if( true == ad469x.conversion_mode )
    {
        err = ZEPHYR_ERR_SUCCESS;/*Already in conversion mode*/
        if( NULL != offset_needed)
        {
            *offset_needed = 0;
        }

    }
    else
    {
        err = ad469x_spi_write_set_bits(AD469x_REG_SETUP,
                                        BSP_AD469X_REG_SETUP_POS_STATUS_EN | BSP_AD469X_REG_SETUP_POS_SPI_MODE,
                                        BSP_AD469X_REG_SETUP_POS_STATUS_EN | BSP_AD469X_REG_SETUP_POS_SPI_MODE);
        ad469x.conversion_mode = true;

        /*This is a dirty last minute edit, the PCB has the CS and Convertion tied as this is a value way to use the AD469X
 * However the firmware was written with these two lines being able to be controlled separately.*/
        #ifdef BSP_AD469X_CS_CNV_SHARED
            /*Tell Zephyr to not both toggling*/
            //ad469x.spi_config.operation = ad469x.spi_config.operation | SPI_HOLD_ON_CS | SPI_LOCK_ON;
            ad469x.spi_cs_control.gpio.pin = 22;

            err = gpio_pin_configure_dt( &ad469x.gpio_convert.gpio_spec , GPIO_OUTPUT );
            gpio_pin_set(ad469x.gpio_convert.gpio_spec.port, ad469x.gpio_convert.gpio_spec.pin , 0);
            if( NULL != offset_needed)
            {
                *offset_needed = 2;
            }
            /*When in conversion mode the CS gpio removes itself and the CS/CNV line becomes controlled by CNV*/
        #endif

    }

    /*The device can operate at a higher SPI clock frequency in conversion mode.*/
    ad469x_spi_update_frequency( AD469X_CNV_MODE_SPI_FREQ );


    return err;
}

/**
*
* @name       bsp_ad469x_exit_conversion_mode
*
* @returns    ZEPHYR_ERR_SUCCESS if successful, otherwise a -<errno> value.
*
* @notes      Exit conversion mode. Will need to Enter register mode to read/write registers
*
*/
zephyr_err_t bsp_ad469x_exit_conversion_mode()
{
    zephyr_err_t err = -EIO;
    uint8_t command = 0x0A << 3;

    zephyr_spi_buf_t tx_data = {
        .buf = &command,
        .len = 1,
    };
    zephyr_spi_buf_set_t tx_buf = {
        .buffers = &tx_data,
        .count   = 1,
    };

    err = spi_transceive( ad469x.spi_device , &ad469x.spi_config , &tx_buf , NULL );

    ad469x_spi_update_frequency( AD469X_REG_MODE_SPI_FREQ );

    if( ZEPHYR_ERR_SUCCESS == err )
    {
        ad469x.conversion_mode = false;
    }

    return err;
}

/**
 *
 * @name       bsp_ad469x_read_data
 *
 * @param[in]  channel Selected Channel
 * @param[in]  sample Sample number
 *
 * @param[out] buf output data buffer
 *
 * @returns    ZEPHYR_ERR_SUCCESS if successful, otherwise a -<errno> value.
 *
 * @notes      Read from device. Will need to enter register mode to read/write registers.
 *
 */
zephyr_err_t bsp_ad469x_read_data(uint8_t channel , uint32_t *buf , uint16_t samples )
{
    zephyr_err_t err = -EINVAL;

    uint8_t buffer[10];

    // Dummy Data
    buffer[0] = 0x0;
    buffer[1] = 0x0;

    bsp_ad469x_address_t val;
    val.data = channel;

    err = ad469x_spi_read(val, buffer);
    if ( ZEPHYR_ERR_SUCCESS == err )
    {
        *buf = (uint16_t) (buffer[0] << 8) | buffer[1];
    }

    return err;
}

/*!
 * @brief   Enable input channels according to the mask
 * @param   chn_msk[in] - Mask containing channels to be enabled
 * @return  SUCCESS in case of success, FAILURE otherwise
 */
zephyr_err_t bsp_ad469x_enable_channel_mask( uint16_t chn_msk )
{

    uint8_t lower_byte_bits = (uint8_t) (chn_msk & 0x00FF) ;
    uint8_t upper_byte_bits = (uint8_t) ( (chn_msk & 0xFF00) >> 8);

    zephyr_err_t err = bsp_ad469x_reg_write( AD469x_REG_SEQ_LB , lower_byte_bits );

    if( ZEPHYR_ERR_SUCCESS == err )
    {
        err = bsp_ad469x_reg_write(AD469x_REG_SEQ_UB , upper_byte_bits );
    }

    return err;
}

zephyr_err_t bsp_ad469x_trigger_reading( uint8_t slot_idx ,  k_timeout_t time_to_wait )
{
    /*If the CS and CNV lines are tied, then the rising CS is considered a start conversion pulse,
     * This variable will subtract 2 if a reading is being taken after going into conversion mode*/
    uint8_t offset_account = 0;
    zephyr_err_t err = bsp_ad469x_enter_conversion_mode(&offset_account);
    uint32_t ret_flags = 0;

    if( ZEPHYR_ERR_SUCCESS == err )
    {
                /*Compute the number of samples that need to be taken for the OSR, this code is just a fancy 4^osr*/
                uint8_t iter_amount = (uint8_t) 2 * (0x1 << (ad469x.channel_osr[slot_idx] << 0x1)) - offset_account;
                k_sched_lock();
                gpio_pin_set(ad469x.gpio_convert.gpio_spec.port, ad469x.gpio_convert.gpio_spec.pin, false);
                for (uint8_t idx = 0; idx < iter_amount; idx++) {
                    gpio_pin_toggle(ad469x.gpio_convert.gpio_spec.port, ad469x.gpio_convert.gpio_spec.pin);
                }
                k_sched_unlock();
                /*Wait until the BUSY line falls*/
                ret_flags = k_event_wait_all(&ad469x.gpio_events, AD469X_GPIO_BUSY_EVENT, false, time_to_wait);
                //Clear the GPIO event
                k_event_set_masked(&ad469x.gpio_events, 0, AD469X_GPIO_BUSY_EVENT);

                err = (ret_flags == 0) ? -EIO : ZEPHYR_ERR_SUCCESS;
    }
    return err;
}

zephyr_err_t bsp_ad469x_gather_reading( uint32_t* adc_data )
{
    zephyr_err_t err = -EINVAL;

    zephyr_spi_buf_t spi_buf = {
            .buf = (void*) adc_data,
            .len = sizeof(uint32_t) - 0x1,
    };
    zephyr_spi_buf_set_t samples = {
            .buffers = &spi_buf,
            .count   = 1,
    };

    err = spi_transceive(ad469x.spi_device, &ad469x.spi_config , NULL , &samples );

    return err;
}


bsp_ad469x_reading_t bsp_ad469x_process_raw( uint8_t slot , uint32_t raw_sample)
{

    /*The SPI bits need to be ordered D_MSB , D_(MSB-1) , ... , D_1 , D_0 , OV_FLAG , INX4 , etc..*/
    raw_sample = ( raw_sample   & 0x0000FF00 ) | ( ( raw_sample & 0x00FF0000 ) >> 16 ) | ( ( raw_sample & 0x000000FF ) << 16 );

    bsp_ad469x_reading_t reading = {
            .status.bytes = 0b11111 & raw_sample, /*The first 5 bits will be the status bits*/
            .raw_reading  = raw_sample >> SIZE_OF_STATUS_BITS, /*Remove the 5 status bits*/
    };
    reading.raw_reading = reading.raw_reading >> (0x3 - ad469x.channel_osr[slot] );

    return reading;
}

void bsp_ad469x_take_reading(void)
{
    uint32_t adc_sample = 0;
    uint8_t channel_to_sample;
    bsp_ad469x_reading_t processed_reading;
    for (uint8_t idx = 0; idx < ad469x.num_slots ; idx++)
    {
        channel_to_sample = ad469x.channel_slots[idx];
        bsp_ad469x_trigger_reading( channel_to_sample , K_NO_WAIT/*K_USEC(10)*/ );
        bsp_ad469x_gather_reading( &adc_sample );
        processed_reading = bsp_ad469x_process_raw( channel_to_sample , adc_sample );
        /*Add the processed reading to the queue, the status bits are used to ensure the reading is sent to the correct channel*/
        if( ad469x.channel_slots[idx] == processed_reading.status.channel )
        {
            k_msgq_put(&adc_channel_queue[channel_to_sample] , (void*) &processed_reading , K_NO_WAIT );
            k_event_post( &adc_read_event , (0x1 << channel_to_sample) );
        }

    }

}

zephyr_err_t bsp_ad469x_trigger_hardware_reset(void)
{

    zephyr_err_t err = gpio_pin_set( ad469x.gpio_reset.gpio_spec.port , ad469x.gpio_reset.gpio_spec.pin , true );
    if( ZEPHYR_ERR_SUCCESS == err )
    {
        err = gpio_pin_set( ad469x.gpio_reset.gpio_spec.port , ad469x.gpio_reset.gpio_spec.pin , false );
        if( ZEPHYR_ERR_SUCCESS == err )
        {
            err = gpio_pin_set( ad469x.gpio_reset.gpio_spec.port , ad469x.gpio_reset.gpio_spec.pin , true );
        }
    }

    /*Datasheet states we need to wait 310us after reset*/
    k_sleep( K_USEC(310) );

    return err;
}

zephyr_err_t bsp_ad469x_trigger_software_reset(void)
{
    uint8_t reg_val = 0xFF;
    zephyr_err_t err = bsp_ad469x_reg_read(AD469x_REG_SPI_CONFIG_A , &reg_val );

    if( ZEPHYR_ERR_SUCCESS == err )
    {
        reg_val = reg_val | ( BSP_AD469X_REG_SPI_CONFIG_A_POS_SW_RST_LSB | BSP_AD469X_REG_SPI_CONFIG_A_POS_SW_SW_RST_MSB );
        err = bsp_ad469x_reg_write( AD469x_REG_SPI_CONFIG_A , reg_val );
    }

    /*Datasheet states we need to wait 310us after reset*/
    k_sleep( K_USEC(310) );

    return err;
}

zephyr_err_t bsp_ad469x_set_autocycle( bool enable , uint8_t autocycle_conversion_period )
{
    zephyr_err_t err = -EINVAL;

    if( true == enable )
    {

        uint8_t reg_contents = 0b1 | (autocycle_conversion_period & 0b00001110);
        bsp_ad469x_address_t addr = ad469x_uint16_t_to_address(AD469x_REG_AC_CTRL);

        err = ad469x_spi_write( addr  , reg_contents);

    }
    else
    {
        err = ad469x_spi_set_bit( AD469x_REG_AC_CTRL , AD469X_AC_ENABLE_POS , (uint8_t) false );
    }

    return err;
}

zephyr_err_t bsp_ad469x_start_advanced_sequencer( bool enable_disable )
{
    zephyr_err_t err = -EINVAL;

    if (enable_disable == true )
    {
        err =  bsp_ad469x_set_channel_sequence( AD469X_ADVANCED_SEQ );
    }
    else
    {
        err =  bsp_ad469x_set_channel_sequence( AD469X_SINGLE_CYCLE );
    }

    return err;
}

zephyr_err_t bsp_ad469x_setup_advanced_sequencer( bsp_ad469x_advanced_sequence_t as_settings )
{
    zephyr_err_t err = ZEPHYR_ERR_SUCCESS;

    for (uint8_t idx = 0; (idx < as_settings.slots_used) && (err == ZEPHYR_ERR_SUCCESS); idx++ )
    {
        err = bsp_ad469x_adv_sequence_set_slot( idx , as_settings.slots[idx] );

    }
    if( ZEPHYR_ERR_SUCCESS == err )
    {
        err =  bsp_ad469x_adv_sequence_set_num_slots( as_settings.slots_used );
    }

    return err;
}


zephyr_err_t bsp_ad469x_queue_channels_adv_seq( uint8_t slots_used , uint8_t* slots )
{
    zephyr_err_t err = ZEPHYR_ERR_SUCCESS;

    for (int idx = 0; (idx < slots_used) && (ZEPHYR_ERR_SUCCESS == err); ++idx)
    {
        err = bsp_ad469x_adv_sequence_set_slot(idx, slots[idx]);
    }
    if (ZEPHYR_ERR_SUCCESS == err) {
        err = bsp_ad469x_adv_sequence_set_num_slots(slots_used);
    }

    return err;
}

uint8_t bsp_ad469x_get_num_slots( void )
{
    return DT_PROP_LEN( DT_NODELABEL( ad469x_settings ) , adv_sequencer_slots );
}

uint8_t bsp_ad469x_get_slots_at_idx( uint8_t idx )
{
    return ad469x.channel_slots[ idx ];
}

uint8_t bsp_ad469x_get_slot_resolution( uint8_t slot_idx )
{
    return ad469x_device_resolution( ad469x.channel_osr[ slot_idx ] );
}

uint8_t bsp_ad469x_channel_to_slot( uint8_t channel , uint8_t start_point )
{
    uint8_t idx = start_point;
    for (; idx < AD469x_SLOTS_NO; idx++)
    {
        if( ad469x.channel_slots[idx] == channel )
        {
            break;
        }
    }
    return idx;
}

/**
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 * **************************** - Private Functions - *******************************/

/**
 *
 * @name       ad469x_spi_read
 *
 * @param[in]  reg  The 16-bits to be written
 *
 * @param[out] data The byte of the data to have the data read stored in.
 *
 * @returns    ZEPHYR_ERR_SUCCESS if successful, otherwise a -<errno> value.
 *
 * @notes      Performs a write then read onto the spi bus.
 *
 */

static zephyr_err_t ad469x_spi_read(bsp_ad469x_address_t reg , uint8_t* data )
{
    zephyr_err_t err = -EINVAL;
    reg.readwrite = AD469X_READ_REGISTER;

    zephyr_spi_buf_t spi_buf[] = {
        [AD469X_SPI_WRITE_BUF_POS] = {
            .buf = &reg.data,
            .len = 2,
        },
        [AD469X_SPI_READ_BUF_POS_1] = {
            .buf = NULL,
            .len = 2,//Skip reading 2 bytes as the buffer is NULL
        },
        [AD469X_SPI_READ_BUF_POS_2] = {
            .buf = data,
            .len = 1,//Read 1 byte (should be the response)
        },
    };

    zephyr_spi_buf_set_t spi_bufs[] = {
        [AD469X_SPI_WRITE] = {
                .buffers = &spi_buf[AD469X_SPI_WRITE_BUF_POS],
                .count   = 1,
            },
        [AD469X_SPI_READ] = {
                .buffers = &spi_buf[AD469X_SPI_READ_BUF_POS_1],
                .count   = 2,
        },
    };

    err = spi_transceive(ad469x.spi_device, &ad469x.spi_config,
                         &spi_bufs[AD469X_SPI_WRITE] , &spi_bufs[AD469X_SPI_READ] );

    return err;
}


/**
 *
 * @name       ad469x_spi_write
 *
 * @param[in]  reg  The 16-bits to be written to
 *
 * @param[out] data The byte of the data to be written.
 *
 * @returns    ZEPHYR_ERR_SUCCESS if successful, otherwise a -<errno> value.
 *
 * @notes      Performs a write onto the spi bus.
 *
 */
static zephyr_err_t ad469x_spi_write(bsp_ad469x_address_t reg , uint8_t data )
{

    zephyr_err_t err = -EINVAL;
    reg.readwrite = AD469X_WRITE_REGISTER;

    uint8_t buffer[ sizeof(uint16_t) + sizeof(uint8_t) ];

    memcpy( &buffer[0] , &reg.data , sizeof(uint16_t) );
    buffer[2] = data;

    zephyr_spi_buf_t spi_tx_buf = {
        .buf = &buffer[0],
        .len = sizeof(uint16_t) + sizeof(uint8_t),
    };
    zephyr_spi_buf_set_t tx_buf = {
            .buffers = &spi_tx_buf,
            .count   = 1,
    };

    err = spi_write( ad469x.spi_device , &ad469x.spi_config , &tx_buf );

    return err;
}

/**
 *
 * @name       ad469x_uint16_t_to_address
 *
 * @param[in]  input_int  The uint16_t to be converted
 *
 * @returns    bsp_ad469x_address_t equivalent of input_int
 *
 * @notes
 *
 */
static inline bsp_ad469x_address_t ad469x_uint16_t_to_address( uint16_t input_int )
{
    bsp_ad469x_address_t ret = { .data = input_int };
    ret.data = (ret.data >> 8 ) | ( ret.data << 8 );
    return ret;
}

/**
 *
 * @name       ad469x_device_resolution
 *
 * @param[in]  in_osr The OSR to get the device resolution for
 *
 * @returns    The device resolution when using the input osr
 *
 * @notes
 *
 */
static inline uint8_t ad469x_device_resolution( bsp_ad469x_osr_ratios_e in_osr )
{
    uint8_t ret_val = 0;
    switch (in_osr)
    {
        case AD469X_OSR_1:
            ret_val = 16;
            break;
        case AD469X_OSR_4:
            ret_val = 17;
            break;
        case AD469X_OSR_16:
            ret_val = 18;
            break;
        case AD469X_OSR_64:
            ret_val = 19;
            break;
        default:
            ret_val = ~( (uint8_t) 0U);
            break;
    }
    return ret_val;
}

/**
 *
 * @name       ad469x_spi_read_mask
 *
 * @param[in]  reg_addr - register address to read
 * @param[in]  mask     - Bit mask to be applied to the data
 * @param[in]  data     - The data located at the register address chosen
 *
 * @returns    bsp_ad469x_address_t equivalent of input_int
 *
 * @notes
 *
 */
__attribute__((unused))
static zephyr_err_t ad469x_spi_read_mask( uint16_t reg_addr , uint8_t mask, uint8_t* data )
{
    zephyr_err_t err = -EINVAL;

    uint8_t reg_data;

    err = ad469x_spi_read(ad469x_uint16_t_to_address(reg_addr), &reg_data);
    if ( err == ZEPHYR_ERR_SUCCESS )
    {
        *data = (reg_data & mask);
    }

    return err;
}

/**
 *
 * @name       ad469x_spi_write_set_bits
 *
 * @param[in]  reg_addr - register address to write
 * @param[in]  mask     - Bit mask to be applied to the data
 * @param[in]  data     - The data to be written to the chosen register
 *
 * @returns    ZEPHYR_ERR_SUCCESS if successful, otherwise a -<errno> value.
 *
 * @notes      This function is used to apply bitmasks
 *
 */
static zephyr_err_t ad469x_spi_write_set_bits(uint16_t reg_addr , uint8_t mask , uint8_t data )
{

    zephyr_err_t err = -EINVAL;
    bsp_ad469x_address_t reg = ad469x_uint16_t_to_address(reg_addr);

    uint8_t reg_data = 0xff;

    err = ad469x_spi_read( reg , &reg_data);

    if ( err == ZEPHYR_ERR_SUCCESS )
    {
        reg_data &= ~mask;
        reg_data |= data;

        err = ad469x_spi_write(reg , reg_data);
    }
    return err;
}

static zephyr_err_t ad469x_spi_set_bit( uint16_t reg_addr , uint8_t bit_pos , uint8_t bit_val )
{

    zephyr_err_t err = -EINVAL;
    bsp_ad469x_address_t reg = ad469x_uint16_t_to_address(reg_addr);

    uint8_t reg_val;

    err = ad469x_spi_read( reg , &reg_val);

    reg_val = ( bit_val == true ) ? reg_val | ( bit_val << bit_pos ) : reg_val & ( bit_val << bit_pos )  ;

    err = ad469x_spi_write( reg , reg_val );

    return err;
}

/**
 *
 * @name       ad469x_adv_seq_osr_get_util_data
 *
 * @param[in]  current_sample  Current sample number
 *
 * @param[in] sample Sample data
 *
 * @returns    ZEPHYR_ERR_SUCCESS.
 *
 * @notes      Advanced sequencer, get util data bits in a sample
 *
 */
static zephyr_err_t ad469x_adv_seq_osr_get_util_data(uint16_t current_sample , uint32_t *sample )
{

    uint8_t current_channel;

    uint8_t current_slot = current_sample % ( ad469x.num_slots + ad469x.temp_enabled);
    current_channel = ad469x.channel_slots[current_slot];

    /* Not temperature channel sample */
    if ( ( ad469x.temp_enabled == false ) && ( current_slot != ad469x.num_slots) )
    {
        *sample = (*sample) >> ( ad469x.capture_data_width - ad469x.channel_resolution[current_channel] );
    }

    return ZEPHYR_ERR_SUCCESS;
}

/**
 *
 * @name       ad469x_seq_osr_clear
 *
 * @returns    ZEPHYR_ERR_SUCCESS if successful, otherwise a -<errno> value.
 *
 * @notes      Configure over sampling ratio to 1 in single and two cycle modes.
 *
 */
static zephyr_err_t ad469x_seq_osr_clear()
{
    zephyr_err_t err = -EINVAL;
    uint8_t i = 0;

    for (i = 0; i < AD469x_CHANNEL_NO; i++)
    {
        err = ad469x_spi_write_set_bits(AD469x_REG_CONFIG_IN(i),
                                        AD469x_REG_CONFIG_IN_OSR_MASK, AD469x_REG_CONFIG_IN_OSR(AD469X_OSR_1));
        if( ZEPHYR_ERR_SUCCESS == err )
        {
            ad469x.channel_resolution[i] = ad469x_device_resolution(AD469X_OSR_1);
        }

    }
    /* Set storage to minimum data width */
    ad469x.capture_data_width = ad469x_device_resolution(AD469X_OSR_1);

    return err;
}

static void ad469x_gpio_convert_cb(const zephyr_device_t* port, zephyr_gpio_callback_t* cb, gpio_port_pins_t pins)
{
    k_event_post( &ad469x.gpio_events , AD469X_GPIO_CONVERT_EVENT );
}

static void ad469x_gpio_busy_cb(const zephyr_device_t* port, zephyr_gpio_callback_t* cb, gpio_port_pins_t pins)
{
    k_event_post( &ad469x.gpio_events , AD469X_GPIO_BUSY_EVENT );
}

static void ad469x_gpio_reset_cb(const zephyr_device_t* port, zephyr_gpio_callback_t* cb, gpio_port_pins_t pins)
{
    k_event_post( &ad469x.gpio_events , AD469X_GPIO_RESET_EVENT );
}

static zephyr_err_t ad469x_gpio_busy_init(void)
{
    zephyr_err_t err = gpio_pin_configure_dt(&ad469x.gpio_busy.gpio_spec, GPIO_INPUT );
    if (ZEPHYR_ERR_SUCCESS == err)
    {
        /*The busy indicator is so fast that there isnt enough CPU time to process both edges, so we just check if
         * the falling edge exists as the busy is active high, if it falls it must have risen*/
        err = gpio_pin_interrupt_configure_dt(&ad469x.gpio_busy.gpio_spec, GPIO_INT_EDGE_FALLING );
        if (ZEPHYR_ERR_SUCCESS == err)
        {
            /*Init busy GPIO and callback*/
            gpio_init_callback(&ad469x.gpio_busy.gpio_callback, ad469x_gpio_busy_cb,
                               BIT(ad469x.gpio_busy.gpio_spec.pin));
            err = gpio_add_callback(ad469x.gpio_busy.gpio_spec.port, &ad469x.gpio_busy.gpio_callback);
        }
    }
    return err;
}

static zephyr_err_t ad469x_gpio_convert_init(void)
{
    zephyr_err_t err = gpio_pin_configure_dt(&ad469x.gpio_convert.gpio_spec, GPIO_OUTPUT);
    if( ZEPHYR_ERR_SUCCESS == err )
    {
        /*Init start convert GPIO and callback*/
        gpio_init_callback(&ad469x.gpio_convert.gpio_callback, ad469x_gpio_convert_cb,
                           BIT(ad469x.gpio_convert.gpio_spec.pin));
        err = gpio_add_callback(ad469x.gpio_convert.gpio_spec.port, &ad469x.gpio_convert.gpio_callback);
    }

    return err;
}

static zephyr_err_t ad469x_gpio_reset_init(void)
{
    zephyr_err_t err = gpio_pin_configure_dt(&ad469x.gpio_reset.gpio_spec, GPIO_OUTPUT);
    if( ZEPHYR_ERR_SUCCESS == err )
    {
        /*Init start convert GPIO and callback*/
        gpio_init_callback(&ad469x.gpio_reset.gpio_callback, ad469x_gpio_reset_cb,
                           BIT(ad469x.gpio_reset.gpio_spec.pin));
        err = gpio_add_callback(ad469x.gpio_reset.gpio_spec.port, &ad469x.gpio_reset.gpio_callback);
    }
    bsp_ad469x_trigger_hardware_reset();

    return err;
}

static void ad469x_queue_init(void)
{
    for (uint8_t idx = 0; (idx < AD469x_CHANNEL_NO); idx++)
    {
        k_msgq_init(&adc_channel_queue[idx] , (uint8_t*) adc_queue_buffers[idx] ,
                    sizeof(bsp_ad469x_reading_t) , AD469X_QUEUE_LENGTH);
    }
    k_event_init( &adc_read_event );
}

static zephyr_err_t ad469x_adv_seq_init()
{
    zephyr_err_t err = ZEPHYR_ERR_SUCCESS;

    uint16_t defaults[] = AD469X_CHANNEL_DEFAULTS;
    uint8_t arr_idx = 0;

    for (uint8_t idx = 0; idx < (AD469X_CHANNEL_DEFAULTS_LEN / 3U ); idx++ )
    {
        /*defaults[ idx ] is the channel number , defaults[ idx + 1] is the channels osr,
         * defaults[ idx + 2 ] is the channels reference pairing*/
        arr_idx = 3*idx;
        ad469x.channel_osr[ defaults[arr_idx] ] = defaults[ arr_idx + 1];
        ad469x.channel_resolution[ defaults[arr_idx] ] = ad469x_device_resolution(ad469x.channel_osr[defaults[arr_idx]] );
        ad469x.channel_reference[ defaults[arr_idx] ] = defaults[ arr_idx + 2];
    }
    /*Setup all the channel OSRs*/
    for (uint16_t idx = 0; (idx < AD469x_CHANNEL_NO) && (ZEPHYR_ERR_SUCCESS == err); idx++ )
    {
        err = bsp_ad469x_adv_seq_osr( idx , ad469x.channel_osr[ idx ] );
        err = bsp_ad469x_adv_seq_pair( idx , ad469x.channel_reference[ idx ] );
    }
    /*No slots in use so set to 0*/
    ad469x.num_slots = 0;

    return err;
}

static void ad469x_spi_update_frequency( uint32_t new_frequency )
{
    ad469x.spi_config.frequency = new_frequency;
}


#if STANDARD_SEQUENCER
static zephyr_err_t bsp_ad469x_trigger_std_seq( k_timeout_t time_to_wait )
{
    zephyr_err_t err = bsp_ad469x_enter_conversion_mode();
    uint32_t ret_flags = 0;

    if( ZEPHYR_ERR_SUCCESS == err )
    {
        for (uint8_t idx = 0; idx < ad469x.channel_resolution[ 0 ] ; idx++ )
        {
            gpio_pin_set( ad469x.gpio_convert.gpio_spec.port , ad469x.gpio_convert.gpio_spec.pin , true );
            gpio_pin_set( ad469x.gpio_convert.gpio_spec.port , ad469x.gpio_convert.gpio_spec.pin , false );
        }
        ret_flags = k_event_wait_all( &ad469x.gpio_events , AD469X_GPIO_BUSY_EVENT , false  , time_to_wait );
        k_event_set_masked(&ad469x.gpio_events , 0 , AD469X_GPIO_BUSY_EVENT );
        //ad469x.gpio_events.events = 0;

        err = ( ret_flags == 0) ? -EIO : ZEPHYR_ERR_SUCCESS;
    }
    return err;
}
#endif

#if 1//#ifdef BSP_AD469X_DEBUG_MODE
void ad469x_debug_read_reg( uint8_t reg_to_read )
{
    uint8_t reg_data = 0xFF;
    bsp_ad469x_reg_read( reg_to_read , &reg_data );
    printk("DEBUG: Reg: 0x%X contains the value 0x%x\n" , reg_to_read , reg_data );
}

void ad469x_debug_print_bits( uint8_t size_in_bytes , uint32_t value )
{
    uint16_t size_in_bits = 0x8 * size_in_bytes;
    for (uint16_t idx = 0; idx < size_in_bits ; idx++ )
    {

        if( (( idx % 8 ) == 0) && (idx != 0) )
        {
            printk(" ");
        }

        uint8_t bit = value >> ( size_in_bits - idx - 1 );
        printk( "%x" , 0x1 & bit );

    }
    printk("\n");
}
#endif

static zephyr_err_t ad469x_set_channel_calibration( uint8_t channel , uint16_t gain , int16_t offset)
{
    /*Offset and Gain registers are 16 bits thus are 2 8 bit regs */

    uint16_t base_reg = 2 * channel + AD469x_REG_OFFSET_IN_BASE;
    /*Write the bottom 8 bits first*/
    zephyr_err_t err = bsp_ad469x_reg_write( base_reg , offset & 0xFF );
    if( ZEPHYR_ERR_SUCCESS == err )
    {
        err = bsp_ad469x_reg_write( base_reg + 0x1 , (offset >> 8) & 0xFF );
        if( ZEPHYR_ERR_SUCCESS == err )
        {
            ad469x.channel_calibration[channel].offset = offset;

            base_reg = 2 * channel + AD469x_REG_GAIN_IN_BASE;
            err = bsp_ad469x_reg_write( base_reg , gain & 0xFF );
            if( ZEPHYR_ERR_SUCCESS == err )
            {
                err = bsp_ad469x_reg_write( base_reg + 0x1 , (gain >> 8) & 0xFF );
                if(ZEPHYR_ERR_SUCCESS == err)
                {
                    ad469x.channel_calibration[channel].gain = gain;
                }
            }
        }
    }
    return err;
}