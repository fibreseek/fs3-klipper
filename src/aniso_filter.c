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

    kf->enable = 1;
}

/**
 * @brief Reset Kalman filter state
 *
 * @param kf Pointer to Kalman filter structure
 */
void aniso_kalman_filter_reset(struct aniso_kalman_filter_t *kf)
{
    if (kf == NULL || !kf->enable)
        return;

    kf->p = 1.0f;
    kf->k = 0.0f;
    kf->x = 0.0f;
    kf->y = 0.0f;

    kf->enable = 0;
}

/**
 * @brief Preheat Kalman filter
 *
 * @param kf Pointer to Kalman filter structure
 * @param val Preheat value
 */
void aniso_kalman_filter_preheat(struct aniso_kalman_filter_t *kf, float val)
{
    if (kf == NULL || !kf->enable)
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
    if (kf == NULL || !kf->enable)
        return val;

    // Check for invalid input values
    if (!isfinite(val)) {
        return kf->y; // Return previous output if input is invalid
    }

    kf->x = val;
    // Update prediction covariance: k-time prediction covariance = k-1-time prediction covariance + process noise covariance
    kf->p += kf->q;

    // Prevent covariance from becoming negative or too large
    kf->p = (kf->p > 1e6f) ? 1e6f : ((kf->p < 0.0f) ? 0.0f : kf->p);

    // Calculate Kalman gain: Kalman gain = k-time prediction covariance / (k-time prediction covariance + observation noise covariance)
    float denominator = kf->p + kf->r;
    if (denominator <= 0.0f || !isfinite(denominator)) {
        kf->k = 0.0f; // Avoid division by zero or invalid values
    } else {
        kf->k = kf->p / denominator;
    }

    // Limit gain range to increase stability
    kf->k = (kf->k > 1.0f) ? 1.0f : ((kf->k < 0.0f) ? 0.0f : kf->k);

    // Update corrected value: k-time corrected value = k-1-time corrected value + Kalman gain * (k-time measured value - k-1-time corrected value)
    float innovation = kf->x - kf->y;
    if (isfinite(innovation)) {
        kf->y += kf->k * innovation;
    }

    // Update prediction covariance
    kf->p *= (1.0f - kf->k);

    // Prevent covariance from becoming negative
    if (kf->p < 0.0f)
        kf->p = 0.0f;

    // Ensure output is finite
    if (!isfinite(kf->y)) {
        kf->y = val; // Reset to input value if output becomes invalid
    }

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
void aniso_iir_filter_init(struct aniso_iir_filter_t *iirf, enum aniso_filter_mode_t mode, float freq, float q, float sps)
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
    q = (q > 100.0f) ? 100.0f : ((q <= 0.0f) ? 0.707f : q);

    iirf->mode = mode;
    iirf->freq = freq;
    iirf->q = q;
    iirf->sps = sps;

    // Calculate filter coefficients
    aniso_iir_filter_coeff(iirf);

    // Clear history data
    iirf->x[0] = iirf->x[1] = iirf->x[2] = 0.0f;
    iirf->y[0] = iirf->y[1] = iirf->y[2] = 0.0f;

    iirf->enable = 1;
}

/**
 * @brief Reset IIR filter state
 *
 * @param iirf Pointer to IIR filter structure
 */
void aniso_iir_filter_reset(struct aniso_iir_filter_t *iirf)
{
    if (iirf == NULL || !iirf->enable)
        return;

    // Clear history data
    iirf->x[0] = iirf->x[1] = iirf->x[2] = 0.0f;
    iirf->y[0] = iirf->y[1] = iirf->y[2] = 0.0f;

    iirf->enable = 0;
}

/**
 * @brief Preheat IIR filter
 *
 * @param iirf Pointer to IIR filter structure
 * @param val Preheat value
 */
void aniso_iir_filter_preheat(struct aniso_iir_filter_t *iirf, float val)
{
    if (iirf == NULL || !iirf->enable)
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
    if (iirf == NULL || !iirf->enable)
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

/**
 * @brief Initialize moving average filter
 *
 * @param maf Pointer to moving average filter structure
 * @param window_size Size of the moving window
 */
void aniso_moving_average_filter_init(struct aniso_moving_average_filter_t *maf, uint16_t window_size)
{
    if (maf == NULL || window_size == 0 || window_size > ANISO_MAX_MOVING_AVERAGE_WINDOW)
        return;

    maf->size = window_size;
    maf->index = 0;
    maf->sum = 0.0f;
    maf->filled = 0;
    maf->enable = 1;

    // Initialize buffer with zeros
    for (uint16_t i = 0; i < window_size; i++) {
        maf->buffer[i] = 0.0f;
    }
}

/**
 * @brief Reset moving average filter state
 *
 * @param maf Pointer to moving average filter structure
 */
void aniso_moving_average_filter_reset(struct aniso_moving_average_filter_t *maf)
{
    if (maf == NULL || !maf->enable)
        return;

    maf->index = 0;
    maf->sum = 0.0f;
    maf->filled = 0;

    // Clear buffer
    for (uint16_t i = 0; i < maf->size; i++) {
        maf->buffer[i] = 0.0f;
    }

    maf->enable = 0;
}

/**
 * @brief Process moving average filter
 *
 * @param maf Pointer to moving average filter structure
 * @param val Input value for this cycle
 * @return float Processed output value (moving average)
 */
float aniso_moving_average_filter_process(struct aniso_moving_average_filter_t *maf, float val)
{
    if (maf == NULL || !maf->enable)
        return val;

    // Check for invalid input values
    if (!isfinite(val)) {
        // Return current average if input is invalid
        if (maf->filled) {
            return maf->sum / maf->size;
        } else if (maf->index > 0) {
            return maf->sum / maf->index;
        } else {
            return 0.0f;
        }
    }

    // If buffer is not yet filled, accumulate samples
    if (!maf->filled) {
        maf->sum += val;
        maf->buffer[maf->index] = val;
        maf->index++;
        
        // Check if buffer is now filled
        if (maf->index >= maf->size) {
            maf->filled = 1;
            maf->index = 0; // Reset index for circular buffer
        }
        
        // Return current average (partial if buffer not filled)
        if (maf->index > 0) {
            return maf->sum / maf->index;
        } else {
            return val;
        }
    }

    // Buffer is filled, use circular buffer approach
    // Subtract the oldest value and add the new value
    float oldest_val = maf->buffer[maf->index];
    if (isfinite(oldest_val)) {
        maf->sum -= oldest_val;
    }
    maf->sum += val;
    maf->buffer[maf->index] = val;
    
    // Update index for next iteration
    maf->index = (maf->index + 1) % maf->size;

    // Return the moving average
    return maf->sum / maf->size;
}

/**
 * @brief Process moving average filter with outlier removal (remove max and min values)
 *
 * @param maf Pointer to moving average filter structure
 * @param val Input value for this cycle
 * @return float Processed output value (moving average without max and min)
 */
float aniso_moving_average_filter_process_remove_outliers(struct aniso_moving_average_filter_t *maf, float val)
{
    if (maf == NULL || !maf->enable)
        return val;

    // Check for invalid input values
    if (!isfinite(val)) {
        // Return current average if input is invalid
        if (maf->filled) {
            return maf->sum / maf->size;
        } else if (maf->index > 0) {
            return maf->sum / maf->index;
        } else {
            return 0.0f;
        }
    }

    // If buffer is not yet filled, use standard moving average
    if (!maf->filled) {
        maf->sum += val;
        maf->buffer[maf->index] = val;
        maf->index++;
        
        // Check if buffer is now filled
        if (maf->index >= maf->size) {
            maf->filled = 1;
            maf->index = 0; // Reset index for circular buffer
        }
        
        // Return current average (partial if buffer not filled)
        if (maf->index > 0) {
            return maf->sum / maf->index;
        } else {
            return val;
        }
    }

    // Buffer is filled, use circular buffer approach
    // Subtract the oldest value and add the new value
    float oldest_val = maf->buffer[maf->index];
    if (isfinite(oldest_val)) {
        maf->sum -= oldest_val;
    }
    maf->sum += val;
    maf->buffer[maf->index] = val;
    
    // Update index for next iteration
    maf->index = (maf->index + 1) % maf->size;

    // Find max and min values in the current buffer
    float max_val = maf->buffer[0];
    float min_val = maf->buffer[0];
    uint16_t valid_count = 0;
    float valid_sum = 0.0f;

    for (uint16_t i = 0; i < maf->size; i++) {
        float current_val = maf->buffer[i];
        if (isfinite(current_val)) {
            valid_count++;
            valid_sum += current_val;
            
            if (current_val > max_val) {
                max_val = current_val;
            }
            if (current_val < min_val) {
                min_val = current_val;
            }
        }
    }

    // Remove outliers only if we have enough valid samples
    if (valid_count >= 3) { // Need at least 3 samples to remove max and min
        // Subtract max and min from sum
        float sum_without_outliers = valid_sum - max_val - min_val;
        uint16_t count_without_outliers = valid_count - 2;
        
        // Return average without outliers
        if (count_without_outliers > 0 && isfinite(sum_without_outliers)) {
            return sum_without_outliers / count_without_outliers;
        }
    }

    // Fallback to standard moving average if not enough samples for outlier removal
    return maf->sum / maf->size;
}

/**
 * @brief Initialize integer moving average filter
 *
 * @param maf Pointer to integer moving average filter structure
 * @param window_size Size of the moving window
 */
void aniso_moving_average_filter_int_init(struct aniso_moving_average_filter_int_t *maf, uint16_t window_size)
{
    if (maf == NULL || window_size == 0 || window_size > ANISO_MAX_MOVING_AVERAGE_WINDOW)
        return;

    maf->size = window_size;
    maf->index = 0;
    maf->sum = 0;
    maf->filled = 0;
    maf->enable = 1;

    // Initialize buffer with zeros
    for (uint16_t i = 0; i < window_size; i++) {
        maf->buffer[i] = 0;
    }
}

/**
 * @brief Reset integer moving average filter state
 *
 * @param maf Pointer to integer moving average filter structure
 */
void aniso_moving_average_filter_int_reset(struct aniso_moving_average_filter_int_t *maf)
{
    if (maf == NULL || !maf->enable)
        return;

    maf->index = 0;
    maf->sum = 0;
    maf->filled = 0;

    // Clear buffer
    for (uint16_t i = 0; i < maf->size; i++) {
        maf->buffer[i] = 0;
    }

    maf->enable = 0;
}

/**
 * @brief Process integer moving average filter
 *
 * @param maf Pointer to integer moving average filter structure
 * @param val Input value for this cycle
 * @return int32_t Processed output value (moving average)
 */
int32_t aniso_moving_average_filter_int_process(struct aniso_moving_average_filter_int_t *maf, int32_t val)
{
    if (maf == NULL || !maf->enable)
        return val;

    // If buffer is not yet filled, accumulate samples
    if (!maf->filled) {
        maf->sum += val;
        maf->buffer[maf->index] = val;
        maf->index++;
        
        // Check if buffer is now filled
        if (maf->index >= maf->size) {
            maf->filled = 1;
            maf->index = 0; // Reset index for circular buffer
        }
        
        // Return current average (partial if buffer not filled)
        if (maf->index > 0) {
            return maf->sum / maf->index;
        } else {
            return val;
        }
    }

    // Buffer is filled, use circular buffer approach
    // Subtract the oldest value and add the new value
    int32_t oldest_val = maf->buffer[maf->index];
    maf->sum -= oldest_val;
    maf->sum += val;
    maf->buffer[maf->index] = val;
    
    // Update index for next iteration
    maf->index = (maf->index + 1) % maf->size;

    // Return the moving average
    return maf->sum / maf->size;
}

/**
 * @brief Process integer moving average filter with outlier removal (remove max and min values)
 *
 * @param maf Pointer to integer moving average filter structure
 * @param val Input value for this cycle
 * @return int32_t Processed output value (moving average without max and min)
 */
int32_t aniso_moving_average_filter_int_process_remove_outliers(struct aniso_moving_average_filter_int_t *maf, int32_t val)
{
    if (maf == NULL || !maf->enable)
        return val;

    // If buffer is not yet filled, use standard moving average
    if (!maf->filled) {
        maf->sum += val;
        maf->buffer[maf->index] = val;
        maf->index++;
        
        // Check if buffer is now filled
        if (maf->index >= maf->size) {
            maf->filled = 1;
            maf->index = 0; // Reset index for circular buffer
        }
        
        // Return current average (partial if buffer not filled)
        if (maf->index > 0) {
            return maf->sum / maf->index;
        } else {
            return val;
        }
    }

    // Buffer is filled, use circular buffer approach
    // Subtract the oldest value and add the new value
    int32_t oldest_val = maf->buffer[maf->index];
    maf->sum -= oldest_val;
    maf->sum += val;
    maf->buffer[maf->index] = val;
    
    // Update index for next iteration
    maf->index = (maf->index + 1) % maf->size;

    // Find max and min values in the current buffer
    int32_t max_val = maf->buffer[0];
    int32_t min_val = maf->buffer[0];
    uint16_t valid_count = maf->size;
    int32_t valid_sum = maf->sum;

    for (uint16_t i = 1; i < maf->size; i++) {
        int32_t current_val = maf->buffer[i];
        
        if (current_val > max_val) {
            max_val = current_val;
        }
        if (current_val < min_val) {
            min_val = current_val;
        }
    }

    // Remove outliers only if we have enough samples
    if (valid_count >= 3) { // Need at least 3 samples to remove max and min
        // Subtract max and min from sum
        int32_t sum_without_outliers = valid_sum - max_val - min_val;
        uint16_t count_without_outliers = valid_count - 2;
        
        // Return average without outliers
        if (count_without_outliers > 0) {
            return sum_without_outliers / count_without_outliers;
        }
    }

    // Fallback to standard moving average if not enough samples for outlier removal
    return maf->sum / maf->size;
}
