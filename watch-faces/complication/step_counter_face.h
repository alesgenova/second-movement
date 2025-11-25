/*
 * MIT License
 *
 * Copyright (c) 2025 David Volovskiy
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef STEP_COUNTER_FACE_H_
#define STEP_COUNTER_FACE_H_

#define STEP_COUNTER_NUM_DATA_POINTS (36)

/*
 * STEP_COUNTER face
 *
 */

#include "movement.h"

typedef struct {
    uint8_t day;
    uint32_t step_count;
} step_counter_face_data_point_t;

typedef struct {
    uint32_t step_count_prev;
    uint16_t sec_inactivity : 14;
    uint16_t can_sleep : 1;
    uint16_t sensor_seen : 1;
    uint8_t sec_before_starting;
    uint8_t display_index;  // the index we are displaying on screen
    int32_t data_points;    // the absolute number of data points logged
    step_counter_face_data_point_t data[STEP_COUNTER_NUM_DATA_POINTS];
} step_counter_state_t;

void step_counter_face_setup(uint8_t watch_face_index, void ** context_ptr);
void step_counter_face_activate(void *context);
bool step_counter_face_loop(movement_event_t event, void *context);
void step_counter_face_resign(void *context);
movement_watch_face_advisory_t step_counter_face_advise(void *context);

#define step_counter_face ((const watch_face_t){ \
    step_counter_face_setup, \
    step_counter_face_activate, \
    step_counter_face_loop, \
    step_counter_face_resign, \
    step_counter_face_advise, \
})

#endif // STEP_COUNTER_FACE_H_
