/*
 *  @name          bsp_ad469x.h
 *  @date          08/10/22
 *  @author        Benjamin Chilton
 *  @university_id bdc1g19
 *
 *
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
#ifndef ULTRA_PORTABLE_VITAL_MONITORING_BSP_AD469X_H
#define ULTRA_PORTABLE_VITAL_MONITORING_BSP_AD469X_H

/****************************** - Library Includes - *******************************/
/******************************** - User Includes - ********************************/

#include "zephyr_typedefs.h"

/*********************************** - Defines - ***********************************/

#define AD469x_CHANNEL(x)                       (BIT(x) & 0xFFFF)
#define AD469x_CHANNEL_NO                       16
#define AD469x_SLOTS_NO                         0x80
#define AD469x_CHANNEL_TEMP                     16
#define AD469X_QUEUE_LENGTH                     4

/************************************ - Enums - ************************************/

typedef enum {
    /** Single cycle read */
    AD469X_SINGLE_CYCLE,
    /** Two cycle read */
    AD469X_TWO_CYCLE,
    /** Sequence trough channels, standard mode */
    AD469X_STANDARD_SEQ,
    /** Sequence trough channels, advanced mode */
    AD469X_ADVANCED_SEQ,
    /**Must be the last entry*/
    AD469X_MAX,
} bsp_ad469x_channel_sequencing_e;

/**
  * @enum ad469x_busy_gp_sel
  * @brief Busy state, possible general purpose pin selections
  */
typedef enum {
    /** Busy on gp0 */
    AD469X_BUSY_GP0 = 0,
    /** busy on gp3 */
    AD469X_BUSY_GP3 = 1,
    AD469X_BUSY_MAX,
} bsp_ad469x_busy_gp_sel_e;

/**
  * @enum ad469x_reg_access
  * @brief Register access modes
  */
typedef enum {
    AD469x_BYTE_ACCESS,
    AD469x_WORD_ACCESS,
} bsp_ad469x_reg_access_e;

/**
  * @enum ad469x_supported_dev_ids
  * @brief Supported devices
  */
typedef enum {
    ID_AD4695,
    ID_AD4696,
    ID_AD4697,
    ID_AD469X_MAX,
} bsp_ad469x_supported_dev_ids_e;

/**
  * @enum ad469x_osr_ratios
  * @brief Supported oversampling ratios
  */
typedef enum {
    AD469X_OSR_1  = 0x00,
    AD469X_OSR_4  = 0x01,
    AD469X_OSR_16 = 0x02,
    AD469X_OSR_64 = 0x03,
    AD469x_OSR_MAX,
} bsp_ad469x_osr_ratios_e;

typedef enum {
    AD469X_PAIR_REFGND  = 0x00,
    AD469X_PAIR_COM     = 0x01,
    AD469X_PAIR_DIFF    = 0x02,
    AD469X_PAIR_INVALID = 0x03,
    AD469x_PAIR_MAX,
} bsp_ad469x_reference_e;

/**Advanced Sequencer index enumeration**/
typedef enum
{
    AD469X_AS_INDEX_0  = 0x00,
    AD469X_AS_INDEX_1  = 0x01,
    AD469X_AS_INDEX_2  = 0x02,
    AD469X_AS_INDEX_3  = 0x03,
    AD469X_AS_INDEX_4  = 0x04,
    AD469X_AS_INDEX_5  = 0x05,
    AD469X_AS_INDEX_6  = 0x06,
    AD469X_AS_INDEX_7  = 0x07,
    AD469X_AS_INDEX_8  = 0x08,
    AD469X_AS_INDEX_9  = 0x09,
    AD469X_AS_INDEX_10 = 0x0A,
    AD469X_AS_INDEX_11 = 0x0B,
    AD469X_AS_INDEX_12 = 0x0C,
    AD469X_AS_INDEX_13 = 0x0D,
    AD469X_AS_INDEX_14 = 0x0E,
    AD469X_AS_INDEX_15 = 0x0F,
} bsp_ad469x_reg_as_slot_inx_bits_e;

#define AD469X_ADC_CONVERSION_FINISHED_EVENT    BIT(0)

/********************************** - Typedefs - ***********************************/
/********************************* - Structures - **********************************/

typedef struct {
    uint8_t slots_used;
    bsp_ad469x_reg_as_slot_inx_bits_e slots[AD469x_SLOTS_NO];
} bsp_ad469x_advanced_sequence_t;

typedef struct {
    uint16_t gain;
    int16_t offset;
} bsp_ad469x_calibration_t;

typedef union {
    uint8_t bytes : 5;
    struct
    {
        uint8_t channel : 4;
        bool    ov_flag : 1;
    };
} bsp_ad469x_status_bits_t;

typedef struct {
    uint32_t                  raw_reading;
    bsp_ad469x_status_bits_t  status;
} bsp_ad469x_reading_t;

/**************************** - Function Prototypes - ******************************/

zephyr_err_t bsp_ad469x_set_autocycle( bool enable , uint8_t autocycle_conversion_period );

/*Data capture*/

zephyr_err_t bsp_ad469x_enable_channel_mask(uint16_t chn_msk);


/**Initialisation functions**/
zephyr_err_t bsp_ad469x_init();
zephyr_err_t bsp_ad469x_init_busy(bsp_ad469x_busy_gp_sel_e gp_sel );

zephyr_err_t bsp_ad469x_reg_write( uint16_t register_address , uint8_t data );
zephyr_err_t bsp_ad469x_reg_read( uint16_t register_address , uint8_t* data );

/**Functions to extract data from the ad469x object**/
bsp_ad469x_channel_sequencing_e bsp_ad469x_get_channel_sequencing();
zephyr_k_event_t*               bsp_ad469x_get_data_event_handle(void);
zephyr_k_msgq_t*                bsp_ad469x_get_data_queues_handle( void );
uint8_t                         bsp_ad469x_get_num_slots( void );
uint8_t                         bsp_ad469x_get_slots_at_idx( uint8_t idx );
uint8_t                         bsp_ad469x_get_slot_resolution( uint8_t slot_idx );
uint8_t                         bsp_ad469x_channel_to_slot( uint8_t channel , uint8_t start_point );

/**Register configuration functions**/
zephyr_err_t bsp_ad469x_set_reg_access_mode( bsp_ad469x_reg_access_e access );

/**Data capture**/
zephyr_err_t bsp_ad469x_trigger_reading( uint8_t slot_idx , k_timeout_t time_to_wait );
zephyr_err_t bsp_ad469x_gather_reading( uint32_t* adc_data );
zephyr_err_t bsp_ad469x_exit_conversion_mode();
zephyr_err_t bsp_ad469x_enter_conversion_mode(uint8_t* offset_needed);

/**Device Reset functions**/
zephyr_err_t bsp_ad469x_trigger_software_reset(void);
zephyr_err_t bsp_ad469x_trigger_hardware_reset(void);

/**Standard Sequencer Functions**/
#ifdef STANDARD_SEQUENCER
zephyr_err_t bsp_ad469x_standard_sequencer( bool enable_disable );
zephyr_err_t bsp_ad469x_std_seq_osr( bsp_ad469x_osr_ratios_e ratio );
#endif

/**Advanced Sequencer functions**/
zephyr_err_t bsp_ad469x_start_advanced_sequencer( bool enable_disable );
zephyr_err_t bsp_ad469x_setup_advanced_sequencer( bsp_ad469x_advanced_sequence_t sequence );
zephyr_err_t bsp_ad469x_adv_sequence_set_slot( uint8_t slot , uint8_t channel );
zephyr_err_t bsp_ad469x_adv_sequence_set_num_slots( uint8_t num_slots );
zephyr_err_t bsp_ad469x_set_channel_sequence( bsp_ad469x_channel_sequencing_e sequence );
zephyr_err_t bsp_ad469x_adv_seq_osr( uint16_t channel , bsp_ad469x_osr_ratios_e ratio);
zephyr_err_t bsp_ad469x_queue_channels_adv_seq( uint8_t slots_used , uint8_t* slots );
zephyr_err_t bsp_ad469x_toggle_temperature_channel( bool enable );
void         bsp_ad469x_take_reading(void);

/****************************** - Global Variables - *******************************/

#endif //ULTRA_PORTABLE_VITAL_MONITORING_BSP_AD469X_H