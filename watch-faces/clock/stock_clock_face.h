/* SPDX-License-Identifier: MIT */

/*
 * MIT License
 *
 * Copyright © 2021-2022 Joey Castillo <joeycastillo@utexas.edu> <jose.castillo@gmail.com>
 * Copyright © 2022 Alexsander Akers <me@a2.io>
 * Copyright © 2022 TheOnePerson <a.nebinger@web.de>
 * Copyright © 2023 Alex Utter <ooterness@gmail.com>
 * Copyright © 2024 Matheus Afonso Martins Moreira <matheus.a.m.moreira@gmail.com>
 * Copyright © 2025 Alessandro Genova
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

#ifndef STOCK_CLOCK_FACE_H_
#define STOCK_CLOCK_FACE_H_

/*
 * STOCK CLOCK FACE
 *
 * - it delegates the playing of the hourly chime to another face, merely displaying the bell indicator
 * - it toggles between 12/24H time display when pressing the alarm button
 * - Short pressing the alarm button immediately starts the fast stopwatch. When resetting the stopwatch, will return to clock.
 * - Long pressing the alarm button enters quick timer mode, immediately starting a timer.
 *   Top row shows on the left the length of the timer in minutes, on the right minutes remaining, seconds shows seconds remaining.
 *   Pressing the alarm switches between timer lengths of 1, 3, 5, 10, 15, 20, 25, 30, 45, 60 minutes. When switching,
 *   the timer does not reset, only the length changes. Long pressing the light button resets the timer.
 *   Long pressing alarm switches out of timer mode.
 *
 */

#include "movement.h"

typedef struct
{
    struct
    {
        watch_date_time_t previous;
    } date_time;
    uint8_t last_battery_check;
    uint8_t watch_face_index;
    bool battery_low;
    bool timer_active;
    uint32_t timer_target_timestamp;

    // Stopwatch/timer fields
    bool stopwatch_mode;              // true when displaying stopwatch
    bool timer_mode;                  // true when displaying quick timer (entered from stopwatch long press)
    rtc_counter_t alarm_down_counter; // rtc counter when alarm button was first pressed down
    rtc_counter_t start_counter;      // rtc counter when timing began (set at alarm button down)
    rtc_counter_t stop_counter;       // rtc counter when the stopwatch was stopped
    uint8_t stopwatch_status;         // the status the stopwatch is in (idle, running, stopped)
    bool slow_refresh;                // update the display slowly (same 128Hz timekeeping accuracy)
    struct
    {
        rtc_counter_t seconds;
        rtc_counter_t minutes;
        rtc_counter_t hours;
    } old_display; // the digits currently being displayed on screen
} stock_clock_state_t;

void stock_clock_face_setup(uint8_t watch_face_index, void **context_ptr);
void stock_clock_face_activate(void *context);
bool stock_clock_face_loop(movement_event_t event, void *context);
void stock_clock_face_resign(void *context);

#define stock_clock_face ((const watch_face_t){ \
    stock_clock_face_setup,                     \
    stock_clock_face_activate,                  \
    stock_clock_face_loop,                      \
    stock_clock_face_resign,                    \
    NULL,                                       \
})

#endif // CLOCK_FACE_H_
