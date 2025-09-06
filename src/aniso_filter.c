#include "aniso_filter.h"
#include <math.h> //sinf cosf

// Single precision PI
#ifndef ANISO_PI_F
#define ANISO_PI_F 3.14159265358979323846f
#endif

/**
 * @brief Initialize Kalman filter
 *
 * @param kf Pointer to Kalman filter structure
 * @param p Prediction covariance
 * @param q Process noise covariance
 * @param r Observation noise covariance
 */
void aniso_kalman_filter_init(struct aniso_kalman_filter_t *kf, float p, float q, float r)
{
    if (kf == NULL)
        return;

    kf->p = p;
    kf->q = q;
    kf->r = r;
    kf->k = 0.0f;
    kf->x = 0.0f;
    kf->y = 0.0f;

    kf->inited = 1;
}

/**
 * @brief Reset Kalman filter state
 *
 * @param kf Pointer to Kalman filter structure
 */
void aniso_kalman_filter_reset(struct aniso_kalman_filter_t *kf)
{
    if (kf == NULL || !kf->inited)
        return;

    kf->p = 1.0f;
    kf->k = 0.0f;
    kf->x = 0.0f;
    kf->y = 0.0f;

    kf->inited = 0;
}

/**
 * @brief Preheat Kalman filter
 *
 * @param kf Pointer to Kalman filter structure
 * @param val Preheat value
 */
void aniso_kalman_filter_preheat(struct aniso_kalman_filter_t *kf, float val)
{
    if (kf == NULL || !kf->inited)
        return;

    kf->x = val;
    kf->y = val;
}

/**
 * @brief Process Kalman filter
 *
 * @param kf Pointer to Kalman filter structure
 * @param val Input value for this cycle
 * @return float Processed output value
 */
float aniso_kalman_filter_process(struct aniso_kalman_filter_t *kf, float val)
{
    if (kf == NULL || !kf->inited)
        return val;

    kf->x = val;
    // Update prediction covariance: k-time prediction covariance = k-1-time prediction covariance + process noise covariance
    kf->p += kf->q;

    // Prevent covariance from becoming negative or too large
    if (kf->p < 0.0f)
        kf->p = 0.0f;
    else if (kf->p > 1e6f)
        kf->p = 1e6f;

    // Calculate Kalman gain: Kalman gain = k-time prediction covariance / (k-time prediction covariance + observation noise covariance)
    kf->k = kf->p / (kf->p + kf->r);

    // Limit gain range to increase stability
    if (kf->k < 0.0f)
        kf->k = 0.0f;
    else if (kf->k > 1.0f)
        kf->k = 1.0f;

    // Update corrected value: k-time corrected value = k-1-time corrected value + Kalman gain * (k-time measured value - k-1-time corrected value)
    kf->y += kf->k * (kf->x - kf->y);

    // Update prediction covariance
    kf->p *= (1.0f - kf->k);

    // Prevent covariance from becoming negative
    if (kf->p < 0.0f)
        kf->p = 0.0f;

    return kf->y;
}

/**
 * @brief Update IIR filter coefficients
 *
 * @param iirf Pointer to IIR filter structure
 */
void aniso_iir_filter_coeff(struct aniso_iir_filter_t *iirf)
{
    // Calculate filter parameters
    float omega = 2.0f * ANISO_PI_F * iirf->freq / iirf->sps;
    float sinw = sinf(omega);
    float cosw = cosf(omega);
    float alpha = sinw / (2.0f * iirf->q);

    // Set coefficients according to filter type
    float a0, a1, a2, b0, b1, b2;

    switch (iirf->mode)
    {
    case ANISO_FILTER_HIGH_PASS: // High-pass filter coefficient calculation
        b0 = (1.0f + cosw) * 0.5f;
        b1 = -(1.0f + cosw);
        b2 = (1.0f + cosw) * 0.5f;
        a0 = 1.0f + alpha;
        a1 = -2.0f * cosw;
        a2 = 1.0f - alpha;
        break;

    case ANISO_FILTER_LOW_PASS: // Low-pass filter coefficient calculation
        b0 = (1.0f - cosw) * 0.5f;
        b1 = 1.0f - cosw;
        b2 = (1.0f - cosw) * 0.5f;
        a0 = 1.0f + alpha;
        a1 = -2.0f * cosw;
        a2 = 1.0f - alpha;
        break;

    case ANISO_FILTER_BAND_PASS: // Band-pass filter coefficient calculation
        b0 = sinw * 0.5f;
        b1 = 0.0f;
        b2 = -sinw * 0.5f;
        a0 = 1.0f + alpha;
        a1 = -2.0f * cosw;
        a2 = 1.0f - alpha;
        break;

    case ANISO_FILTER_BAND_STOP: // Notch filter coefficient calculation
    default:
        b0 = 1.0f;
        b1 = -2.0f * cosw;
        b2 = 1.0f;
        a0 = 1.0f + alpha;
        a1 = -2.0f * cosw;
        a2 = 1.0f - alpha;
        break;
    }

    // Normalize coefficients and store
    iirf->b[0] = b0 / a0;
    iirf->b[1] = b1 / a0;
    iirf->b[2] = b2 / a0;
    iirf->a[0] = 1.0f;
    iirf->a[1] = a1 / a0;
    iirf->a[2] = a2 / a0;
}

/**
 * @brief Initialize IIR filter
 *
 * @param iirf Pointer to IIR filter structure
 * @param mode Filter type
 * @param enable Filter enable
 * @param freq Filter frequency
 * @param q Filter quality
 * @param sps Sampling frequency
 */
void aniso_iir_filter_init(struct aniso_iir_filter_t *iirf, enum aniso_filter_mode_t mode, uint8_t enable, float freq, float q, float sps)
{
    if (iirf == NULL)
        return;

    // Parameter check and limitation
    if (sps <= 0.0f)
        sps = 1000.0f; // Default 1kHz sampling rate

    // Limit frequency to reasonable range
    if (freq <= 0.0f)
        freq = 1.0f;
    if (2.0f * freq > sps)
        freq = sps * 0.5f - 1.0f; // Leave some margin

    // Limit quality factor to reasonable range
    if (q <= 0.0f)
        q = 0.707f; // Default Butterworth response
    if (q > 100.0f)
        q = 100.0f;

    iirf->mode = mode;
    iirf->enable = enable;
    iirf->freq = freq;
    iirf->q = q;
    iirf->sps = sps;

    // Calculate filter coefficients
    aniso_iir_filter_coeff(iirf);

    // Clear history data
    iirf->x[0] = iirf->x[1] = iirf->x[2] = 0.0f;
    iirf->y[0] = iirf->y[1] = iirf->y[2] = 0.0f;

    iirf->inited = 1;
}

/**
 * @brief Reset IIR filter state
 *
 * @param iirf Pointer to IIR filter structure
 */
void aniso_iir_filter_reset(struct aniso_iir_filter_t *iirf)
{
    if (iirf == NULL || !iirf->inited)
        return;

    // Clear history data
    iirf->x[0] = iirf->x[1] = iirf->x[2] = 0.0f;
    iirf->y[0] = iirf->y[1] = iirf->y[2] = 0.0f;

    iirf->inited = 0;
}

/**
 * @brief Preheat IIR filter
 *
 * @param iirf Pointer to IIR filter structure
 * @param val Preheat value
 */
void aniso_iir_filter_preheat(struct aniso_iir_filter_t *iirf, float val)
{
    if (iirf == NULL || !iirf->inited)
        return;

    iirf->x[0] = iirf->x[1] = iirf->x[2] = val;
    iirf->y[0] = iirf->y[1] = iirf->y[2] = val;
}

/**
 * @brief Process IIR filter
 *
 * @param iirf Pointer to filter structure
 * @param val Input sample value
 * @return Filtered output value
 */
float aniso_iir_filter_process(struct aniso_iir_filter_t *iirf, float val)
{
    // Parameter check
    if (iirf == NULL || !iirf->inited || !iirf->enable)
        return val;

    // Update history data (circular shift)
    iirf->x[2] = iirf->x[1];
    iirf->x[1] = iirf->x[0];
    iirf->x[0] = val;
    iirf->y[2] = iirf->y[1];
    iirf->y[1] = iirf->y[0];

    // Calculate new output (Direct Form I implementation)
    float res = iirf->b[0] * iirf->x[0] +
                iirf->b[1] * iirf->x[1] +
                iirf->b[2] * iirf->x[2] -
                iirf->a[1] * iirf->y[1] -
                iirf->a[2] * iirf->y[2];

    // Limit output to prevent overflow
    if (isfinite(res))
        iirf->y[0] = res; // Keep previous output when non-finite value occurs

    return iirf->y[0];
}