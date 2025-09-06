#ifndef _ANISO_FILTER_H
#define _ANISO_FILTER_H

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

// Filter types
enum aniso_filter_mode_t
{
    ANISO_FILTER_HIGH_PASS, // High-pass filter
    ANISO_FILTER_LOW_PASS,  // Low-pass filter
    ANISO_FILTER_BAND_PASS, // Band-pass filter
    ANISO_FILTER_BAND_STOP  // Band-stop (notch) filter
};

// Kalman filter structure (one-dimensional linear uniform sampling)
struct aniso_kalman_filter_t
{
    float q;        // Process noise covariance
    float r;        // Observation noise covariance
    float p;        // Estimate covariance
    float k;        // Kalman gain
    float x;        // Input
    float y;        // Output
    uint8_t inited; // Initialization flag
};

// Infinite impulse response filter structure definition (one-dimensional linear uniform sampling)
struct aniso_iir_filter_t
{
    enum aniso_filter_mode_t mode; // Filter type
    float a[3];                    // Denominator coefficients
    float b[3];                    // Numerator coefficients
    float x[3];                    // Input history
    float y[3];                    // Output history
    float q;                       // Quality factor
    float freq;                    // Cutoff frequency
    float sps;                     // Sampling rate
    uint8_t enable;                // Enable flag
    uint8_t inited;                // Initialization flag
};

void aniso_kalman_filter_init(struct aniso_kalman_filter_t *kf, float p, float q, float r);
void aniso_kalman_filter_reset(struct aniso_kalman_filter_t *kf);
void aniso_kalman_filter_preheat(struct aniso_kalman_filter_t *kf, float val);
float aniso_kalman_filter_process(struct aniso_kalman_filter_t *kf, float val);

void aniso_iir_filter_init(struct aniso_iir_filter_t *iirf, enum aniso_filter_mode_t mode, uint8_t enable, float freq, float q, float sps);
void aniso_iir_filter_reset(struct aniso_iir_filter_t *iirf);
void aniso_iir_filter_preheat(struct aniso_iir_filter_t *iirf, float val);
float aniso_iir_filter_process(struct aniso_iir_filter_t *iirf, float val);

#endif // _ANISO_FILTER_H