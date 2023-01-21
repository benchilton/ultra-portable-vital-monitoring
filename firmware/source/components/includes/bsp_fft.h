/*
 *  @name          bsp_fft.h
 *  @date          22/11/2022
 *  @author        Benjamin Chilton, Thomas Bain
 *  @university_id bdc1g19, trb1g19
 *
 *
 */

#ifndef ULTRA_PORTABLE_VITAL_MONITORING_BSP_FFT_H
#define ULTRA_PORTABLE_VITAL_MONITORING_BSP_FFT_H

/****************************** - Library Includes - *******************************/
#include <stdint.h>
#include <complex.h>
/******************************** - User Includes - ********************************/
/*********************************** - Defines - ***********************************/
/************************************ - Enums - ************************************/

#define bsp_fft_real( z ) (__real__ z)
#define bsp_fft_imag( z ) (__imag__ z)

/********************************** - Typedefs - ***********************************/
typedef float complex complex_t;
/********************************* - Structures - **********************************/
/**************************** - Function Prototypes - ******************************/
void bsp_fft_test(void);
void bsp_fft_slow(int16_t* x , uint16_t N , complex_t* X);
void bsp_fft(int16_t* x , uint16_t N , complex_t* X );
void bsp_fft_find_peak( complex_t* X , uint16_t N , float* peak , uint16_t* peak_idx , uint16_t start_point );
/****************************** - Global Variables - *******************************/

#endif //ULTRA_PORTABLE_VITAL_MONITORING_BSP_FFT_H
