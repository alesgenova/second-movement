/*
 * MIT License
 *
 * Copyright (c) 2022 Joey Castillo
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

#ifndef MOVEMENT_CONFIG_H_
#define MOVEMENT_CONFIG_H_

#include "movement_faces.h"

/* The F() below is a trick to be able to get the face as well as its name */

#define FOREACH_PRIMARY_FACE(F) \
    F(stock_clock_face)         \
    F(page_ordering_face)      \
    F(world_clock2_face)        \
    F(sunrise_sunset_face)      \
    F(moon_phase_face)          \
    F(fast_stopwatch_face)      \
    F(countdown_face)

#define FOREACH_SECONDARY_FACE(F) \
    F(interval_face)              \
    F(alarm_face)                 \
    F(tunes_face)                 \
    F(probability_face)

#define FOREACH_TERTIARY_FACE(F)    \
    F(settings_face)                \
    F(set_time_face)                \
    F(temperature_display_face)     \
    F(voltage_face)                 \
    F(finetune_face)                \
    F(nanosec_face)





/* Determines the intensity of the led colors
 * Set a hex value 0-15 with 0x0 being off and 0xF being max intensity
 */
#define MOVEMENT_DEFAULT_RED_COLOR 0xF
#define MOVEMENT_DEFAULT_GREEN_COLOR 0xF
#define MOVEMENT_DEFAULT_BLUE_COLOR 0xF

/* Set to true for 24h mode or false for 12h mode */
#define MOVEMENT_DEFAULT_24H_MODE true

/* Enable or disable the sound on mode button press */
#define MOVEMENT_DEFAULT_BUTTON_SOUND true

#define MOVEMENT_DEFAULT_BUTTON_VOLUME WATCH_BUZZER_VOLUME_SOFT
#define MOVEMENT_DEFAULT_SIGNAL_VOLUME WATCH_BUZZER_VOLUME_LOUD
#define MOVEMENT_DEFAULT_ALARM_VOLUME WATCH_BUZZER_VOLUME_LOUD

/* Set the timeout before switching back to the main watch face
 * Valid values are:
 * 0: 60 seconds
 * 1: 2 minutes
 * 2: 5 minutes
 * 3: 30 minutes
 */
#define MOVEMENT_DEFAULT_TIMEOUT_INTERVAL 0

/* Set the timeout before switching to low energy mode
 * Valid values are:
 * 0: Never
 * 1: 10 minutes
 * 2: 1 hour
 * 3: 2 hours
 * 4: 6 hours
 * 5: 12 hours
 * 6: 1 day
 * 7: 7 days
 */
#define MOVEMENT_DEFAULT_LOW_ENERGY_INTERVAL 2

/* Set the led duration
 * Valid values are:
 * 0: No LED
 * 1: 1 second
 * 2: 3 seconds
 * 3: 5 seconds
 */
#define MOVEMENT_DEFAULT_LED_DURATION 1

/* Optionally debounce button presses (disable by default).
 * A value of 4 is a good starting point if you have issues
 * with multiple button presses firing.
 */
#define MOVEMENT_DEBOUNCE_TICKS 0

#endif // MOVEMENT_CONFIG_H_
