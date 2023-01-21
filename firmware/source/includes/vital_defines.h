/*
 *  @name          vital_defines.h
 *  @date          21/11/2022
 *  @author        Benjamin Chilton
 *  @university_id bdc1g19
 *
 *
 */

#ifndef ULTRA_PORTABLE_VITAL_MONITORING_VITAL_DEFINES_H
#define ULTRA_PORTABLE_VITAL_MONITORING_VITAL_DEFINES_H

/****************************** - Library Includes - *******************************/
/******************************** - User Includes - ********************************/
/*********************************** - Defines - ***********************************/
/************************************ - Enums - ************************************/
typedef enum {
    VITALS_VITAL_TYPE_ID_TEMPERATURE  = 0,
    VITALS_VITAL_TYPE_ID_RESPIRATION,
    VITALS_VITAL_TYPE_ID_HEART_RATE,
    VITALS_VITAL_TYPE_ID_ACCELERATION,
    /*VITALS_VITAL_TYPE_ID_BLOOD_OXYGEN,*/
    /*Must be bottom of the list*/
    VITALS_VITAL_TYPE_ID_MAX
} vitals_vital_type_id_e;
/********************************** - Typedefs - ***********************************/
/********************************* - Structures - **********************************/

typedef struct {
/*Please fill accordingly with what the correct datatype is*/
    int32_t   temperature;

    float     acceleration;

    uint8_t   heart_rate;//In BPM?
    uint16_t* heart_rate_raw;
    uint16_t  heart_rate_raw_len;
    uint8_t   respiration;

} vitals_vital_sign_data_t;

/**************************** - Function Prototypes - ******************************/

/****************************** - Global Variables - *******************************/

#endif //ULTRA_PORTABLE_VITAL_MONITORING_VITAL_DEFINES_H
