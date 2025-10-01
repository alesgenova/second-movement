#ifndef count_steps_h
#define count_steps_h
#include "stdint.h"
#include "lis2dw.h"

#define COUNT_STEPS_SAMPLING_RATE    25                                                  //25 hz sampling rate
#define COUNT_STEPS_WINDOW_LENGTH    4                                                   //window length in seconds
#define COUNT_STEPS_NUM_TUPLES       COUNT_STEPS_WINDOW_LENGTH*COUNT_STEPS_SAMPLING_RATE //sets of accelerometer readings


typedef struct {
    uint8_t count;
    int magnitude[COUNT_STEPS_NUM_TUPLES];
} count_steps_magnitude_data_t;

uint32_t count_steps_approx_l2_norm(lis2dw_reading_t reading);
uint8_t count_steps(uint8_t *mag_sqrt);
uint8_t count_steps_simple(lis2dw_fifo_t *fifo_data);

/// Initialise step counting
void stepcount_init(void);

/* Registers a new data point for step counting. Data is expected
 * as 12.5Hz, 8192=1g, and accMagSquared = sqrt(x*x + y*y + z*z)
 *
 * Returns the number of steps counted for this accel interval
 */

int stepcount_new(uint32_t accMag);
int stepcount_new_from_fifo(lis2dw_fifo_t *fifo_data);
#endif /* count_steps_h */