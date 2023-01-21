/*
 *  @name          bsp_fft.c
 *  @date          22/11/2022
 *  @author        Benjamin Chilton, Thomas Bain
 *  @university_id bdc1g19, trb1g19
 *
 *
 */

/* Fast Fourier Transform
 * Cooley-Tukey algorithm with 2-radix DFT
 * Code sourced from https://github.com/brendanashworth/fft-small, ownership of the project goes to Brendan Ashworth
 * and AdeebAbbas-TRI.
 * Changed made by Benjamin Chilton
 */


/****************************** - Library Includes - *******************************/

#include <math.h>
#include <stdlib.h>
#include <complex.h>
#include <zephyr.h>

/******************************** - User Includes - ********************************/
#include "bsp_fft.h"
/*********************************** - Defines - ***********************************/

#define M_PI		3.14159265358979323846
#define M_PI_F		3.14159265358979323846f

#define BSP_FFT_SIN(freq) ((2.0f*M_PI_F*freq))

/************************************ - Enums - ************************************/
/********************************* - Structures - **********************************/
/**************************** - Function Prototypes - ******************************/
static inline void bsp_fft_radix2(int16_t* x, complex_t* X, uint16_t N, uint16_t s);
/********************************* - Constants - ***********************************/
/********************************* - Variables - ***********************************/
/***************************** - Public Functions - ********************************/

void bsp_fft_test(void)
{

    float      sampling_freq = 10.0f;
    float      sample_period = 1.0f / sampling_freq;
    uint8_t    L = 64;
    int16_t*   signal = ( int16_t* ) k_malloc( L * sizeof(int16_t) );
    complex_t* X = ( complex_t* ) k_calloc( L , sizeof(complex_t) );

    for (uint8_t idx = 0; idx < L; idx++)
    {
        signal[idx] = (int16_t) ( 1000.0f * sinf( BSP_FFT_SIN( 3.0f ) * (sample_period * ((float) idx) ) ) );
    }

    bsp_fft(  signal , L , X);

    uint16_t peak_idx = 0U;
    float peak = 0.0f;

    bsp_fft_find_peak( X , L , &peak , &peak_idx , 0 );

    printk("frequency at peak = %f , peak = %f\n" ,
           ( (float) sampling_freq ) * ( (float) peak_idx ) / ( (float) L )
           , peak);

    k_free(signal);
    k_free( X );
}

void bsp_fft_find_peak( complex_t* X , uint16_t N , float* peak , uint16_t* peak_idx , uint16_t start_point )
{
    float old_peak = 0.0f;
    float new_peak = 0.0f;
    for (uint16_t idx = start_point; idx < (uint16_t) ((N - 0x1) / 0x2); idx++)
    {
        new_peak = cabsf( X[idx] ) / ( (float) N);
        if( old_peak < new_peak )
        {
            old_peak = new_peak;
            *peak_idx = idx;
        }

    }
    *peak = old_peak;
}


/**
 *
 * @name       bsp_fft_slow
 *
 * @param[in]   x Input signal vector
 * @param[in]   N Length of the input signal vector
 * @param[out]  X Output fourier transform of x
 *
 * @notes       Performs the fourier transform of x using the FFT algorithm,
 *              More precision than bsp_fft()
 *
 */
void bsp_fft_slow(int16_t* x , uint16_t N , complex_t* X)
{
    // Iterate through, allowing X_K = sum_N of the complex frequencies.
    complex_t z;
    for (uint16_t k = 0; k < N/2; k++)
    {
        for (uint16_t n = 0; n < N; n++)
        {
            z = (-2.0f * M_PI_F * ((float) n) * ((float) k) / ((float) N) ) * I;
            X[k] += x[n] * cexpf( z );
        }
    }
}

/**
 *
 * @name       bsp_fft
 *
 * @param[in]   x Input signal vector
 * @param[in]   N Length of the input signal vector
 * @param[out]  X Output fourier transform of x
 *
 * @notes       Performs the fourier transform of x using the FFT algorithm,
 *              Less precision than bsp_fft_slow()
 *
 */
void bsp_fft( int16_t* x , uint16_t N , complex_t* X )
{
    bsp_fft_radix2( x , X, N, 1);
}

/***************************** - Private Functions - *******************************/

static inline void bsp_fft_radix2( int16_t* x, complex_t* X, uint16_t N, uint16_t step )
{
    // At the lowest level pass through (delta T=0 means no phase).
    if (N == 1)
    {
        X[0] = (complex_t) x[0];
        return;
    }

    uint16_t idx = N / 2U;

    // Cooley-Tukey: recursively split in two, then combine beneath.
    bsp_fft_radix2(x, X, idx, 2 * step);
    bsp_fft_radix2(&x[step], &X[idx], idx, 2 * step);

    complex_t z;
    for (uint16_t k = 0; k < idx; k++)
    {
        z = cexpf( -2.0f * M_PI * I * ( (float) k) /( (float) N) ) * X[k + idx];
        X[k + idx] = X[k] - z;
        X[k]       = X[k] + z;
    }

}
