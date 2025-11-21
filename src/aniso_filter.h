#ifndef _ANISO_FILTER_H
#define _ANISO_FILTER_H

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

// Maximum window size for moving average filter
#define ANISO_MAX_MOVING_AVERAGE_WINDOW 20

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
    uint8_t enable; // Initialization flag
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
};

void aniso_kalman_filter_init(struct aniso_kalman_filter_t *kf, float p, float q, float r);
void aniso_kalman_filter_reset(struct aniso_kalman_filter_t *kf);
void aniso_kalman_filter_preheat(struct aniso_kalman_filter_t *kf, float val);
float aniso_kalman_filter_process(struct aniso_kalman_filter_t *kf, float val);

void aniso_iir_filter_init(struct aniso_iir_filter_t *iirf, enum aniso_filter_mode_t mode, float freq, float q, float sps);
void aniso_iir_filter_reset(struct aniso_iir_filter_t *iirf);
void aniso_iir_filter_preheat(struct aniso_iir_filter_t *iirf, float val);
float aniso_iir_filter_process(struct aniso_iir_filter_t *iirf, float val);

// Moving average filter structure (float version)
struct aniso_moving_average_filter_t
{
    float buffer[ANISO_MAX_MOVING_AVERAGE_WINDOW]; // Buffer for storing samples
    uint16_t size;      // Window size
    uint16_t index;     // Current index in buffer
    float sum;          // Current sum of samples
    uint8_t filled;     // Flag indicating if buffer is filled
    uint8_t enable;     // Enable flag
};

void aniso_moving_average_filter_init(struct aniso_moving_average_filter_t *maf, uint16_t window_size);
void aniso_moving_average_filter_reset(struct aniso_moving_average_filter_t *maf);
float aniso_moving_average_filter_process(struct aniso_moving_average_filter_t *maf, float val);

/**
 * @brief Process moving average filter with outlier removal (remove max and min values)
 *
 * @param maf Pointer to moving average filter structure
 * @param val Input value for this cycle
 * @return float Processed output value (moving average without max and min)
 */
float aniso_moving_average_filter_process_remove_outliers(struct aniso_moving_average_filter_t *maf, float val);

// Moving average filter structure (integer version)
struct aniso_moving_average_filter_int_t
{
    int32_t buffer[ANISO_MAX_MOVING_AVERAGE_WINDOW]; // Buffer for storing samples
    uint16_t size;      // Window size
    uint16_t index;     // Current index in buffer
    int32_t sum;        // Current sum of samples
    uint8_t filled;     // Flag indicating if buffer is filled
    uint8_t enable;     // Enable flag
};

void aniso_moving_average_filter_int_init(struct aniso_moving_average_filter_int_t *maf, uint16_t window_size);
void aniso_moving_average_filter_int_reset(struct aniso_moving_average_filter_int_t *maf);
int32_t aniso_moving_average_filter_int_process(struct aniso_moving_average_filter_int_t *maf, int32_t val);

/**
 * @brief Process moving average filter with outlier removal (remove max and min values) - integer version
 *
 * @param maf Pointer to moving average filter structure
 * @param val Input value for this cycle
 * @return int32_t Processed output value (moving average without max and min)
 */
int32_t aniso_moving_average_filter_int_process_remove_outliers(struct aniso_moving_average_filter_int_t *maf, int32_t val);

#endif // _ANISO_FILTER_H
