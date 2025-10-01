#include "count_steps.h"
#include "stdint.h"
#include "stdio.h"  //using this for printing debug outputs
#include <stdlib.h>

#define DEBUG_PRINT false

//this algorithm is a simple adaptation of the following paper:
//"RecoFit - Using a Wearable Sensor to Find, Recognize, and Count Repetitive Exercises"

#define NUM_AUTOCORR_LAGS       50          //number of lags to calculate for autocorrelation. 50 lags @25Hz corresponds to a step rate of 0.5Hz...its probably not physically possible to walk much slower than this
#define DERIV_FILT_LEN          5           //length of derivative filter
#define LPF_FILT_LEN            9           //length of FIR low pass filter
#define AUTOCORR_DELTA_AMPLITUDE_THRESH 7e8 //this is the min delta between peak and trough of autocorrelation peak
#define AUTOCORR_MIN_HALF_LEN   3           //this is the min number of points the autocorrelation peak should be on either side of the peak

#define SIMPLE_THRESHOLD                15000  // Magnitudes at or above this threshold are considered a step, but can change if USE_WINDOW_AVG is true
#define SIMPLE_THRESHOLD_MULT           1.5 // Multiplier for the moving average threshold adjustment. It was seen in some testing that 50% higher than the average worked well.
#define SIMPLE_SAMP_IGNORE_STEP         3   // After detecting a step, ignore this many samples to avoid double counting
#define USE_WINDOW_AVG                  true   // If true, the step detection threshold will be adjusted based on a moving average of the signal magnitude
#define AVG_WINDOW_SIZE_SHIFT           7  // The size of the moving average window. We are using bitshifting to keep the math efficient since we run once a second
#define AVG_WINDOW_SIZE                 ((1 << AVG_WINDOW_SIZE_SHIFT) - 1)
#define MAX_FIFO_SIZE_SIMPLE            13
#define MAX_SIMPLE_STEPS                (MAX_FIFO_SIZE_SIMPLE / SIMPLE_SAMP_IGNORE_STEP)

static int8_t deriv_coeffs[DERIV_FILT_LEN]        = {-6,31,0,-31,6};            //coefficients of derivative filter from https://www.dsprelated.com/showarticle/814.php
static int8_t lpf_coeffs[LPF_FILT_LEN]            = {-5,6,34,68,84,68,34,6,-5}; //coefficients of FIR low pass filter generated in matlab using FDATOOL
static int32_t lpf[COUNT_STEPS_NUM_TUPLES]                    = {0};                        //hold the low pass filtered signal
static int64_t autocorr_buff[NUM_AUTOCORR_LAGS]   = {0};                        //holds the autocorrelation results
static int64_t deriv[NUM_AUTOCORR_LAGS]           = {0};                        //holds derivative

static void derivative(int64_t *autocorr_buff, int64_t *deriv);
static void autocorr(int32_t *lpf, int64_t *autocorr_buff);
static void remove_mean(int32_t *lpf);
static void lowpassfilt(uint8_t *mag_sqrt, int32_t *lpf);
static uint8_t get_precise_peakind(int64_t *autocorr_buff, uint8_t peak_ind);
static void get_autocorr_peak_stats(int64_t *autocorr_buff, uint8_t *neg_slope_count, int64_t *delta_amplitude_right, uint8_t *pos_slope_count, int64_t *delta_amplitude_left, uint8_t peak_ind);

static uint32_t step_counter_threshold = SIMPLE_THRESHOLD;

/* Approximate l2 norm */
uint32_t count_steps_approx_l2_norm(lis2dw_reading_t reading)
{
    /* Absolute values */
    uint32_t ax = abs(reading.x);
    uint32_t ay = abs(reading.y);
    uint32_t az = abs(reading.z);

    /* *INDENT-OFF* */
    /* Sort values: ax >= ay >= az */
    if (ax < ay) { uint32_t t = ax; ax = ay; ay = t; }
    if (ay < az) { uint32_t t = ay; ay = az; az = t; }
    if (ax < ay) { uint32_t t = ax; ax = ay; ay = t; }
    /* *INDENT-ON* */

    /* Approximate sqrt(x^2 + y^2 + z^2) */
    /* alpha ≈ 0.9375 (15/16), beta ≈ 0.375 (3/8) */
    return ax + ((15 * ay) >> 4) + ((3 * az) >> 3);
}


//take a look at the original autocorrelation signal at index i and see if
//its a real peak or if its just a fake "noisy" peak corresponding to
//non-walking. basically just count the number of points of the
//autocorrelation peak to the right and left of the peak. this function gets
//the number of points to the right and left of the peak, as well as the delta amplitude
static void get_autocorr_peak_stats(int64_t *autocorr_buff, uint8_t *neg_slope_count, int64_t *delta_amplitude_right, uint8_t *pos_slope_count, int64_t *delta_amplitude_left, uint8_t peak_ind) {
    
    //first look to the right of the peak. walk forward until the slope begins decreasing
    uint8_t  neg_slope_ind = peak_ind;
    uint16_t loop_limit    = NUM_AUTOCORR_LAGS-1;
    while ((autocorr_buff[neg_slope_ind+1] - autocorr_buff[neg_slope_ind] < 0) && (neg_slope_ind < loop_limit)) {
        *neg_slope_count = *neg_slope_count + 1;
        neg_slope_ind    = neg_slope_ind + 1;
    }
    
    //get the delta amplitude between peak and right trough
    *delta_amplitude_right = autocorr_buff[peak_ind] - autocorr_buff[neg_slope_ind];
    
    //next look to the left of the peak. walk backward until the slope begins increasing
    uint8_t pos_slope_ind = peak_ind;
    loop_limit    = 0;
    while ((autocorr_buff[pos_slope_ind] - autocorr_buff[pos_slope_ind-1] > 0) && (pos_slope_ind > loop_limit)) {
        *pos_slope_count = *pos_slope_count + 1;
        pos_slope_ind    = pos_slope_ind - 1;
    }
    
    //get the delta amplitude between the peak and the left trough
    *delta_amplitude_left = autocorr_buff[peak_ind] - autocorr_buff[pos_slope_ind];
}


//use the original autocorrelation signal to hone in on the
//exact peak index. this corresponds to the point where the points to the
//left and right are less than the current point
static uint8_t get_precise_peakind(int64_t *autocorr_buff, uint8_t peak_ind) {
    uint8_t loop_limit = 0;
    if ((autocorr_buff[peak_ind] > autocorr_buff[peak_ind-1]) && (autocorr_buff[peak_ind] > autocorr_buff[peak_ind+1])) {
        //peak_ind is perfectly set at the peak. nothing to do
    }
    else if ((autocorr_buff[peak_ind] > autocorr_buff[peak_ind+1]) && (autocorr_buff[peak_ind] < autocorr_buff[peak_ind-1])) {
        //peak is to the left. keep moving in that direction
        loop_limit = 0;
        while ((autocorr_buff[peak_ind] > autocorr_buff[peak_ind+1]) && (autocorr_buff[peak_ind] < autocorr_buff[peak_ind-1]) && (loop_limit < 10)) {
            peak_ind = peak_ind - 1;
            loop_limit++;
        }
    }
    else {
        //peak is to the right. keep moving in that direction
        loop_limit = 0;
        while ((autocorr_buff[peak_ind] > autocorr_buff[peak_ind-1]) && (autocorr_buff[peak_ind] < autocorr_buff[peak_ind+1]) && (loop_limit < 10)) {
            peak_ind = peak_ind + 1;
            loop_limit++;
        }
    }
    return peak_ind;
}


//calculate deriviative via FIR filter
static void derivative(int64_t *autocorr_buff, int64_t *deriv) {
    
    uint8_t n          = 0;
    uint8_t i          = 0;
    int64_t temp_deriv = 0;
    for (n = 0; n < NUM_AUTOCORR_LAGS; n++) {
        temp_deriv = 0;
        for (i = 0; i < DERIV_FILT_LEN; i++) {
            if (n-i >= 0) {     //handle the case when n < filter length
                temp_deriv += deriv_coeffs[i]*autocorr_buff[n-i];
            }
        }
        deriv[n] = temp_deriv;
    }
}


//autocorrelation function
//this takes a lot of computation. there are efficient implementations, but this is more intuitive
static void autocorr(int32_t *lpf, int64_t *autocorr_buff) {
    
    uint8_t lag;
    uint16_t i;
    int64_t temp_ac;
    for (lag = 0; lag < NUM_AUTOCORR_LAGS; lag++) {
        temp_ac = 0;
        for (i = 0; i < COUNT_STEPS_NUM_TUPLES-lag; i++) {
            temp_ac += (int64_t)lpf[i]*(int64_t)lpf[i+lag];
        }
        autocorr_buff[lag] = temp_ac;
    }
}


//calculate and remove the mean
static void remove_mean(int32_t *lpf) {
    
    int32_t sum = 0;
    uint16_t i;
    for (i = 0; i < COUNT_STEPS_NUM_TUPLES; i++) {
        sum += lpf[i];
    }
    sum = sum/(COUNT_STEPS_NUM_TUPLES);
    
    for (i = 0; i < COUNT_STEPS_NUM_TUPLES; i++) {
        lpf[i] = lpf[i] - sum;
    }
}


//FIR low pass filter
static void lowpassfilt(uint8_t *mag_sqrt, int32_t *lpf) {
    
    uint16_t n;
    uint8_t i;
    int32_t temp_lpf;
    for (n = 0; n < COUNT_STEPS_NUM_TUPLES; n++) {
        temp_lpf = 0;
        for (i = 0; i < LPF_FILT_LEN; i++) {
            if (n-i >= 0) {     //handle the case when n < filter length
                temp_lpf += (int32_t)lpf_coeffs[i]*(int32_t)mag_sqrt[n-i];
            }
        }
        lpf[n] = temp_lpf;
    }
}

static bool allzeroes(uint8_t *mag_sqrt) {
    uint16_t n;
    bool all_zero = true;
    for (n = 0; n < COUNT_STEPS_NUM_TUPLES; n++) {
        if (mag_sqrt[n] != 0) {
            all_zero = false;
            break;
        }
    }
    return all_zero;
}

//algorithm interface
uint8_t count_steps(uint8_t *mag_sqrt) {
    //assume data is in the format data = [approx_l2_norm(x1,y1,z1),approx_l2_norm(x2,y2,z2)...etc]
    uint16_t i;

    //check if all of the values are zero (which means the data is empty)
    if (allzeroes(mag_sqrt)) return 0;
    
    //apply low pass filter to mag_sqrt, result is stored in lpf
    lowpassfilt(mag_sqrt, lpf);
    
    //remove mean from lpf, store result in lpf
    remove_mean(lpf);
    
    //do the autocorrelation on the lpf buffer, store the result in autocorr_buff
    autocorr(lpf, autocorr_buff);
    
    //get the derivative of the autocorr_buff, store in deriv
    derivative(autocorr_buff, deriv);
    
    //look for first zero crossing where derivative goes from positive to negative. that
    //corresponds to the first positive peak in the autocorrelation. look at two samples
    //instead of just one to maybe reduce the chances of getting tricked by noise
    uint8_t peak_ind = 0;
    //start index is set to 8 lags, which corresponds to a walking rate of 2.5Hz @20Hz sampling rate. its probably
    //running if its faster than this
    for (i = 8; i < NUM_AUTOCORR_LAGS; i++) {
        if ((deriv[i] > 0) && (deriv[i-1] > 0) && (deriv[i-2] < 0) && (deriv[i-3] < 0)) {
            peak_ind = i-1;
            break;
        }
    }
    
    //hone in on the exact peak index
    peak_ind = get_precise_peakind(autocorr_buff, peak_ind);
    //printf("peak ind: %i\n", peak_ind);
    
    //get autocorrelation peak stats
    uint8_t neg_slope_count = 0;
    int64_t delta_amplitude_right = 0;
    uint8_t pos_slope_count = 0;
    int64_t delta_amplitude_left = 0;
    get_autocorr_peak_stats(autocorr_buff, &neg_slope_count, &delta_amplitude_right, &pos_slope_count, &delta_amplitude_left, peak_ind);
    
    //now check the conditions to see if it was a real peak or not, and if so, calculate the number of steps
    uint8_t num_steps = 0;
    if ((pos_slope_count > AUTOCORR_MIN_HALF_LEN) && (neg_slope_count > AUTOCORR_MIN_HALF_LEN) && (delta_amplitude_right > AUTOCORR_DELTA_AMPLITUDE_THRESH) && (delta_amplitude_left > AUTOCORR_DELTA_AMPLITUDE_THRESH)) {
        //the period is peak_ind/COUNT_STEPS_SAMPLING_RATE seconds. that corresponds to a frequency of 1/period
        //with the frequency known, and the number of seconds is 4 seconds, you can then find out the number of steps
        num_steps = (COUNT_STEPS_NUM_TUPLES)/peak_ind;
    } else {
        //not a valid autocorrelation peak
        num_steps = 0;
    }
    
    //printf("num steps: %i\n", num_steps);
    return num_steps;
}

uint8_t count_steps_simple(lis2dw_fifo_t *fifo_data) {
    uint8_t new_steps = 0;
#if USE_WINDOW_AVG
    uint8_t samples_processed = 0;
    uint32_t samples_sum = 0;
#endif
#if DEBUG_PRINT
    printf("fifo counts: %d\r\n", fifo_data->count);
#endif
    for (uint8_t i = 0; i < fifo_data->count; i++) {
        if (i >= MAX_FIFO_SIZE_SIMPLE) break;
        uint32_t magnitude = count_steps_approx_l2_norm(fifo_data->readings[i]);
#if DEBUG_PRINT
        printf("%lu; ", magnitude);
#endif
        if (magnitude >= step_counter_threshold) {
            new_steps += 1;
            i += SIMPLE_SAMP_IGNORE_STEP;
            continue;
        }
#if USE_WINDOW_AVG
        samples_processed += 1;
        samples_sum += magnitude;
#endif
    }
#if USE_WINDOW_AVG
    if (samples_processed > 0) {
        samples_sum /= samples_processed;
        samples_sum *= SIMPLE_THRESHOLD_MULT;
        step_counter_threshold = ((step_counter_threshold * AVG_WINDOW_SIZE) + samples_sum) >> AVG_WINDOW_SIZE_SHIFT;
    }
#if DEBUG_PRINT
        printf("\r\nThreshold: %lu\r\n", step_counter_threshold);
        printf("New Steps: %d\r\n; ", new_steps);
#endif
#endif
    if (new_steps > MAX_SIMPLE_STEPS) new_steps = MAX_SIMPLE_STEPS;
    return new_steps;
}


/*
 * This file is part of Espruino, a JavaScript interpreter for Microcontrollers
 *
 * Copyright (C) 2021 Gordon Williams <gw@pur3.co.uk>
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * ----------------------------------------------------------------------------
 * Step Counter
 * ----------------------------------------------------------------------------
 */

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

// a1bc34f9a9f5c54b9d68c3c26e973dba195e2105   HughB-walk-1834.csv  1446
// oxford filter                                                   1584
// peak detection                                                  1752
// state machine                                                   1751

// STEPCOUNT_CONFIGURABLE is for use with https://github.com/gfwilliams/step-count
// to test/configure the step counter offline

/*

==========================================================
FIR filter designed with http://t-filter.engineerjs.com/

AccelFilter_get modified to return 8 bits of fractional
data.
==========================================================

*/

#define ACCELFILTER_TAP_NUM 7

typedef struct {
  int8_t history[ACCELFILTER_TAP_NUM];
  unsigned int last_index;
} AccelFilter;

static const int8_t filter_taps[ACCELFILTER_TAP_NUM] = {
    -11, -15, 44, 68, 44, -15, -11
};


static void AccelFilter_init(AccelFilter* f) {
  int i;
  for(i = 0; i < ACCELFILTER_TAP_NUM; ++i)
    f->history[i] = 0;
  f->last_index = 0;
}

static void AccelFilter_put(AccelFilter* f, int8_t input) {
  f->history[f->last_index++] = input;
  if(f->last_index == ACCELFILTER_TAP_NUM)
    f->last_index = 0;
}

static int AccelFilter_get(AccelFilter* f) {
  int acc = 0;
  int index = f->last_index, i;
  for(i = 0; i < ACCELFILTER_TAP_NUM; ++i) {
    index = index != 0 ? index-1 : ACCELFILTER_TAP_NUM-1;
    acc += (int)f->history[index] * (int)filter_taps[i];
  };
  return acc >> 2;
}

AccelFilter accelFilter;

/* =============================================================
*  DC Filter
*/
#define NSAMPLE 12 //Exponential Moving Average DC removal filter alpha = 1/NSAMPLE

static int DCFilter_sample_avg_total = 8192*NSAMPLE;

static int DCFilter(int sample) {
    DCFilter_sample_avg_total += (sample - DCFilter_sample_avg_total/NSAMPLE);
    return sample - DCFilter_sample_avg_total/NSAMPLE;
}


// ===============================================================

// These were calculated based on contributed data
// using https://github.com/gfwilliams/step-count
#define STEPCOUNTERTHRESHOLD_DEFAULT  600

// These are the ranges of values we test over
// when calculating the best data offline
#define STEPCOUNTERTHRESHOLD_MIN 0
#define STEPCOUNTERTHRESHOLD_MAX 2000
#define STEPCOUNTERTHRESHOLD_STEP 100

// This is a bit of a hack to allow us to configure
// variables which would otherwise have been compiler defines
#ifdef STEPCOUNT_CONFIGURABLE
int stepCounterThreshold = STEPCOUNTERTHRESHOLD_DEFAULT;
// These are handy values used for graphing
int accScaled;
#else
#define stepCounterThreshold STEPCOUNTERTHRESHOLD_DEFAULT
#endif

// ===============================================================

/** stepHistory allows us to check for repeated step counts.
Rather than registering each instantaneous step, we now only
measure steps if there were at least 3 steps (including the
current one) in 3 seconds

For each step it contains the number of iterations ago it occurred. 255 is the maximum
*/

int16_t accFiltered; // last accel reading, after running through filter
int16_t accFilteredHist[2]; // last 2 accel readings, 1=newest

// ===============================================================

/**
 * (4) State Machine
 *
 * The state machine ensure all steps are checked that they fall
 * between T_MIN_STEP and T_MAX_STEP. The 2v9.90 firmare uses X steps
 * in Y seconds but this just enforces that the step X steps ago was
 * within 6 seconds (75 samples).  It is possible to have 4 steps
 * within 1 second and then not get the 5th until T5 seconds.  This
 * could mean that the F/W would would be letting through 2 batches
 * of steps that actually would not meet the threshold as the step at
 * T5 could be the last.  The F/W version also does not give back the
 * X steps detected whilst it is waiting for X steps in Y seconds.
 * After 100 cycles of the algorithm this would amount to 500 steps
 * which is a 5% error over 10K steps.  In practice the number of
 * passes through the step machine from STEP_1 state to STEPPING
 * state can be as high as 500 events.  So using the state machine
 * approach avoids this source of error.
 *
 */

typedef enum {
  S_STILL = 0,       // just created state m/c no steps yet
  S_STEP_1 = 1,      // first step recorded
  S_STEP_22N = 2,    // counting 2-X steps
  S_STEPPING = 3,    // we've had X steps in X seconds
} StepState;

// In periods of 12.5Hz
#define T_MIN_STEP 4 // ~333ms
#define T_MAX_STEP 16 // ~1300ms
#define X_STEPS 6 // steps in a row needed
#define RAW_THRESHOLD 15
#define N_ACTIVE_SAMPLES 3

StepState stepState;
unsigned char holdSteps; // how many steps are we holding back?
unsigned char stepLength; // how many poll intervals since the last step?
int active_sample_count = 0;
bool gate_open = false;        // start closed
// ===============================================================

// Init step count
void stepcount_init() {
  AccelFilter_init(&accelFilter);
  DCFilter_sample_avg_total = 8192*NSAMPLE;
  accFiltered = 0;
  accFilteredHist[0] = 0;
  accFilteredHist[1] = 0;
  stepState = S_STILL;
  holdSteps = 0;
  stepLength = 0;
}

static int stepcount_had_step() {
  StepState st = stepState;

  switch (st) {
  case S_STILL:
    stepState = S_STEP_1;
    holdSteps = 1;
    return 0;

  case S_STEP_1:
    holdSteps = 1;
    // we got a step within expected period
    if (stepLength <= T_MAX_STEP && stepLength >= T_MIN_STEP) {
      //stepDebug("  S_STEP_1 -> S_STEP_22N");
      stepState = S_STEP_22N;
      holdSteps = 2;
    } else {
      // we stay in STEP_1 state
      //stepDebug("  S_STEP_1 -> S_STEP_1");
      //reject_count++;
    }
    return 0;

  case S_STEP_22N:
    // we got a step within expected time range
    if (stepLength <= T_MAX_STEP && stepLength >= T_MIN_STEP) {
      holdSteps++;

      if (holdSteps >= X_STEPS) {
        stepState = S_STEPPING;
        //pass_count++;  // we are going to STEPPING STATE
        //stepDebug("  S_STEP_22N -> S_STEPPING");
        return X_STEPS;
      }

      //stepDebug("  S_STEP_22N -> S_STEP_22N");
    } else {
      // we did not get the step in time, back to STEP_1
      stepState = S_STEP_1;
      //stepDebug("  S_STEP_22N -> S_STEP_1");
      //reject_count++;
    }
    return 0;

  case S_STEPPING:
    // we got a step within the expected window
    if (stepLength <= T_MAX_STEP && stepLength >= T_MIN_STEP) {
      stepState = S_STEPPING;
      //stepDebug("  S_STEPPING -> S_STEPPING");
      return 1;
    } else {
      // we did not get the step in time, back to STEP_1
      stepState = S_STEP_1;
      //stepDebug("  S_STEPPING -> S_STEP_1");
      //reject_count++;
    }
    return 0;
  }

  // should never get here
  //stepDebug("  ERROR");
  return 0;
}

// do calculations
int stepcount_new(uint32_t accMag) {
  // scale to fit and clip
  //int v = (accMag-8192)>>5;
  int v = DCFilter(accMag)>>5;
  //printf("v %d\n",v);
  //if (v>127 || v<-128) printf("Out of range %d\n", v);
  if (v>127) v = 127;
  if (v<-128) v = -128;
#ifdef STEPCOUNT_CONFIGURABLE
  accScaled = v;
#endif

  // do filtering
  AccelFilter_put(&accelFilter, v);
  accFilteredHist[0] = accFilteredHist[1];
  accFilteredHist[1] = accFiltered;
  int a = AccelFilter_get(&accelFilter);
  if (a>32767) a=32767;
  if (a<-32768) a=32768;
  accFiltered = a;

  if (v > RAW_THRESHOLD || v < -1*RAW_THRESHOLD) {
    if (active_sample_count < N_ACTIVE_SAMPLES)
      active_sample_count++;
    if (active_sample_count == N_ACTIVE_SAMPLES)
      gate_open = true;
  } else {
    if (active_sample_count > 0)
      active_sample_count--;
    if (active_sample_count == 0)
      gate_open = false;
  }

  // increment step count history counters
  if (stepLength<255)
    stepLength++;

  int stepsCounted = 0;
  // check for a peak in the last sample - in which case call the state machine
  if (gate_open == true && accFilteredHist[1] > accFilteredHist[0] && accFilteredHist[1] > accFiltered) {
    // We now have something resembling a step!
    stepsCounted = stepcount_had_step();
    stepLength = 0;
  }

  return stepsCounted;
}


int stepcount_new_from_fifo(lis2dw_fifo_t *fifo_data) {
    uint8_t new_steps = 0;

    for (uint8_t i = 0; i < fifo_data->count; i++) {
        uint32_t magnitude = count_steps_approx_l2_norm(fifo_data->readings[i]);
        new_steps += stepcount_new(magnitude);
    }

    if (new_steps > MAX_SIMPLE_STEPS) new_steps = MAX_SIMPLE_STEPS;
    return new_steps;
}


