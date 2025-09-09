/*
 * MIT License
 *
 * Copyright (c) 2025 <#author_name#>
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

#include <stdlib.h>
#include <string.h>
#include "step_counter_face.h"

static uint16_t display_step_count_now(void) {
    char buf[10];
    uint16_t step_count = movement_get_step_count();
    sprintf(buf, "%6d", step_count);
    watch_display_text(WATCH_POSITION_BOTTOM, buf);
    return step_count;
}

void step_counter_face_setup(uint8_t watch_face_index, void ** context_ptr) {
    (void) watch_face_index;
    if (*context_ptr == NULL) {
        *context_ptr = malloc(sizeof(step_counter_state_t));
        memset(*context_ptr, 0, sizeof(step_counter_state_t));
    }
}

void step_counter_face_activate(void *context) {
    (void)context;
}

bool step_counter_face_loop(movement_event_t event, void *context) {
    step_counter_state_t *state = (step_counter_state_t *)context;
    uint16_t step_count;
    char buf[10];

    switch (event.event_type) {
        case EVENT_ACTIVATE:
            if (!movement_enable_step_count()) {  // Skip this face if not enabled
                movement_move_to_next_face();
                return false;
            }
            watch_display_text_with_fallback(WATCH_POSITION_TOP, "STEP", "ST");
            state->step_count_prev = display_step_count_now();
            break;
        case EVENT_TICK:
            step_count = movement_get_step_count();
            if (step_count != state->step_count_prev) {
                state->step_count_prev = step_count;
                sprintf(buf, "%6d",  step_count);
                watch_display_text(WATCH_POSITION_BOTTOM, buf);
            }
            break;
        case EVENT_ALARM_BUTTON_UP:
            movement_reset_step_count();
            state->step_count_prev = display_step_count_now();
            break;
        case EVENT_LOW_ENERGY_UPDATE:
            movement_disable_step_count();
            break;
        default:
            return movement_default_loop_handler(event);
    }
    return true;
}

void step_counter_face_resign(void *context) {
    (void) context;
    movement_disable_step_count();
}
