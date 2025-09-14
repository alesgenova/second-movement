#ifndef count_steps_h
#define count_steps_h
#include "stdint.h"
#include "lis2dw.h"

#define COUNT_STEPS_SAMPLING_RATE    25                                               //25 hz sampling rate
#define COUNT_STEPS_NUM_TUPLES       100                                              //80 sets of accelerometer readings (so in other words, 100*3 = 300 samples)
#define COUNT_STEPS_WINDOW_LENGTH    COUNT_STEPS_NUM_TUPLES/COUNT_STEPS_SAMPLING_RATE //window length in seconds

uint32_t count_steps_approx_l2_norm(lis2dw_reading_t reading);
uint8_t count_steps(uint8_t *mag_sqrt);

#endif /* count_steps_h */