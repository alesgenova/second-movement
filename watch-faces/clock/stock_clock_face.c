/* SPDX-License-Identifier: MIT */

/*
 * MIT License
 *
 * Copyright © 2021-2023 Joey Castillo <joeycastillo@utexas.edu> <jose.castillo@gmail.com>
 * Copyright © 2022 David Keck <davidskeck@users.noreply.github.com>
 * Copyright © 2022 TheOnePerson <a.nebinger@web.de>
 * Copyright © 2023 Jeremy O'Brien <neutral@fastmail.com>
 * Copyright © 2023 Mikhail Svarichevsky <3@14.by>
 * Copyright © 2023 Wesley Aptekar-Cassels <me@wesleyac.com>
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

#include <stdlib.h>
#include <limits.h>
#include "stock_clock_face.h"
#include "watch.h"
#include "watch_utility.h"
#include "watch_common_display.h"
#include "watch_rtc.h"
#include "slcd.h"

// 2.4 volts seems to offer adequate warning of a low battery condition?
// refined based on user reports and personal observations; may need further adjustment.
#ifndef CLOCK_FACE_LOW_BATTERY_VOLTAGE_THRESHOLD
#define CLOCK_FACE_LOW_BATTERY_VOLTAGE_THRESHOLD 2400
#endif

static const uint8_t QUICK_TIMERS[] = {1, 3, 5, 10, 15, 20, 25, 30, 45, 60, 0};
static const uint8_t N_QUICK_TIMERS = sizeof(QUICK_TIMERS) / sizeof(uint8_t);
static uint8_t current_quick_timer = 0;

// Stopwatch status states
typedef enum
{
    SW_STATUS_IDLE = 0,
    SW_STATUS_RUNNING,
    SW_STATUS_RUNNING_LAPPING,
    SW_STATUS_STOPPED,
    SW_STATUS_STOPPED_LAPPING
} stopwatch_status_t;

// How quickly should the elapsing time be displayed?
static const uint8_t DISPLAY_RUNNING_RATE = 32;
static const uint8_t DISPLAY_RUNNING_RATE_SLOW = 2;

static void clock_display_all(watch_date_time_t date_time, bool skip_seconds);

static inline void _button_beep()
{
    // play a beep as confirmation for a button press (if applicable)
    if (movement_button_should_sound())
        watch_buzzer_play_note_with_volume(BUZZER_NOTE_C7, 50, movement_button_volume());
}

// Stopwatch helper functions

/// @brief Display minutes, seconds and fractions derived from 128 Hz tick counter
///        on the lcd.
/// @param ticks
static void _display_elapsed(stock_clock_state_t *state, uint32_t ticks)
{
    char buf[3];

    if (state->slow_refresh && (state->stopwatch_status == SW_STATUS_RUNNING || state->stopwatch_status == SW_STATUS_IDLE))
    {
        watch_display_character_lp_seconds(' ', 8);
        watch_display_character_lp_seconds(' ', 9);
    }
    else
    {
        uint8_t sec_100 = (ticks & 0x7F) * 100 / 128;

        watch_display_character_lp_seconds('0' + sec_100 / 10, 8);
        watch_display_character_lp_seconds('0' + sec_100 % 10, 9);
    }

    uint32_t seconds = ticks >> 7;

    if (seconds == state->old_display.seconds)
    {
        return;
    }

    state->old_display.seconds = seconds;

    sprintf(buf, "%02lu", seconds % 60);
    watch_display_text(WATCH_POSITION_MINUTES, buf);

    uint32_t minutes = seconds / 60;

    if (minutes == state->old_display.minutes)
    {
        return;
    }

    state->old_display.minutes = minutes;

    sprintf(buf, "%02lu", minutes % 60);
    watch_display_text(WATCH_POSITION_HOURS, buf);

    uint32_t hours = (minutes / 60) % 24;

    if (hours == state->old_display.hours)
    {
        return;
    }

    state->old_display.hours = hours;

    if (hours)
    {
        sprintf(buf, "%2lu", hours);
        watch_display_text(WATCH_POSITION_TOP_RIGHT, buf);
    }
    else
    {
        watch_display_text(WATCH_POSITION_TOP_RIGHT, "  ");
    }
}

static void _draw_stopwatch_indicators(stock_clock_state_t *state, movement_event_t event, uint32_t elapsed)
{
    uint8_t subsecond;
    bool tock;

    switch (state->stopwatch_status)
    {
    case SW_STATUS_RUNNING:
        subsecond = elapsed & 127;
        tock = subsecond >= 64;

        watch_clear_indicator(WATCH_INDICATOR_LAP);
        if (tock)
        {
            watch_clear_colon();
        }
        else
        {
            watch_set_colon();
        }

        return;

    case SW_STATUS_RUNNING_LAPPING:
        tock = event.subsecond > 0;

        if (tock)
        {
            watch_clear_indicator(WATCH_INDICATOR_LAP);
            watch_clear_colon();
        }
        else
        {
            watch_set_indicator(WATCH_INDICATOR_LAP);
            watch_set_colon();
        }

        return;

    case SW_STATUS_STOPPED_LAPPING:
        watch_set_indicator(WATCH_INDICATOR_LAP);
        watch_set_colon();

        return;

    case SW_STATUS_STOPPED:
    case SW_STATUS_IDLE:
    default:
        watch_clear_indicator(WATCH_INDICATOR_LAP);
        watch_set_colon();
        return;
    }
}

static uint8_t get_stopwatch_refresh_rate(stock_clock_state_t *state)
{
    switch (state->stopwatch_status)
    {
    case SW_STATUS_RUNNING:
        if (state->slow_refresh)
        {
            return DISPLAY_RUNNING_RATE_SLOW;
        }
        else
        {
            return DISPLAY_RUNNING_RATE;
        }
    case SW_STATUS_RUNNING_LAPPING:
        return 2;
    case SW_STATUS_STOPPED:
    case SW_STATUS_IDLE:
    default:
        return 1;
    }
}

static void stopwatch_state_transition(stock_clock_state_t *state, rtc_counter_t counter, movement_event_type_t event_type)
{
    switch (state->stopwatch_status)
    {
    case SW_STATUS_IDLE:
        switch (event_type)
        {
        case EVENT_ALARM_BUTTON_DOWN:
            state->stopwatch_status = SW_STATUS_RUNNING;
            state->start_counter = counter;
            movement_request_tick_frequency(get_stopwatch_refresh_rate(state));
            return;
        case EVENT_LIGHT_LONG_PRESS:
            state->slow_refresh = !state->slow_refresh;
            return;
        default:
            return;
        }

    case SW_STATUS_RUNNING:
        switch (event_type)
        {
        case EVENT_ALARM_BUTTON_DOWN:
            state->stopwatch_status = SW_STATUS_STOPPED;
            state->stop_counter = counter;
            movement_request_tick_frequency(get_stopwatch_refresh_rate(state));
            return;
        case EVENT_LIGHT_BUTTON_DOWN:
            state->stopwatch_status = SW_STATUS_RUNNING_LAPPING;
            state->lap_counter = counter;
            movement_request_tick_frequency(get_stopwatch_refresh_rate(state));
            return;
        default:
            return;
        }

    case SW_STATUS_RUNNING_LAPPING:
        switch (event_type)
        {
        case EVENT_ALARM_BUTTON_DOWN:
            state->stopwatch_status = SW_STATUS_STOPPED_LAPPING;
            state->stop_counter = counter;
            movement_request_tick_frequency(get_stopwatch_refresh_rate(state));
            return;
        case EVENT_LIGHT_BUTTON_DOWN:
            state->stopwatch_status = SW_STATUS_RUNNING;
            state->lap_counter = counter;
            movement_request_tick_frequency(get_stopwatch_refresh_rate(state));
            return;
        case EVENT_LIGHT_LONG_PRESS:
            state->stopwatch_status = SW_STATUS_RUNNING;
            state->slow_refresh = !state->slow_refresh;
            movement_request_tick_frequency(get_stopwatch_refresh_rate(state));
            return;
        default:
            return;
        }

    case SW_STATUS_STOPPED_LAPPING:
        switch (event_type)
        {
        case EVENT_ALARM_BUTTON_DOWN:
            state->stopwatch_status = SW_STATUS_RUNNING_LAPPING;
            state->start_counter = counter - state->stop_counter + state->start_counter;
            state->lap_counter = counter - state->stop_counter + state->lap_counter;
            movement_request_tick_frequency(get_stopwatch_refresh_rate(state));
            return;
        case EVENT_LIGHT_BUTTON_DOWN:
            state->stopwatch_status = SW_STATUS_STOPPED;
            return;
        default:
            return;
        }

    case SW_STATUS_STOPPED:
        switch (event_type)
        {
        case EVENT_ALARM_BUTTON_DOWN:
            state->stopwatch_status = SW_STATUS_RUNNING;
            state->start_counter = counter - state->stop_counter + state->start_counter;
            movement_request_tick_frequency(get_stopwatch_refresh_rate(state));
            return;
        case EVENT_LIGHT_BUTTON_DOWN:
            // Reset stopwatch - return to clock mode
            state->stopwatch_status = SW_STATUS_IDLE;
            state->stopwatch_mode = false;
            state->start_counter = 0;
            state->stop_counter = 0;
            state->lap_counter = 0;
            // Force full re-draw of clock
            state->date_time.previous.reg = 0xFFFFFFFF;
            movement_request_tick_frequency(1);
            return;
        default:
            return;
        }

    default:
        return;
    }
}

static uint32_t stopwatch_elapsed_time(stock_clock_state_t *state, rtc_counter_t counter)
{
    switch (state->stopwatch_status)
    {
    case SW_STATUS_IDLE:
        return 0;

    case SW_STATUS_RUNNING:
        return counter - state->start_counter;

    case SW_STATUS_RUNNING_LAPPING:
    case SW_STATUS_STOPPED_LAPPING:
        return state->lap_counter - state->start_counter;

    case SW_STATUS_STOPPED:
        return state->stop_counter - state->start_counter;

    default:
        return 0;
    }
}

// End of stopwatch helper functions

static void clock_indicate(watch_indicator_t indicator, bool on)
{
    if (on)
    {
        watch_set_indicator(indicator);
    }
    else
    {
        watch_clear_indicator(indicator);
    }
}

static void clock_indicate_alarm()
{
    clock_indicate(WATCH_INDICATOR_SIGNAL, movement_alarm_enabled());
}

static void clock_indicate_time_signal()
{
    clock_indicate(WATCH_INDICATOR_BELL, movement_signal_enabled());
}

static void clock_indicate_24h()
{
    clock_indicate(WATCH_INDICATOR_24H, !!movement_clock_mode_24h());
}

static bool clock_is_pm(watch_date_time_t date_time)
{
    return date_time.unit.hour >= 12;
}

static void clock_indicate_pm(watch_date_time_t date_time)
{
    if (movement_clock_mode_24h())
    {
        return;
    }
    clock_indicate(WATCH_INDICATOR_PM, clock_is_pm(date_time));
}

static void clock_indicate_low_available_power(stock_clock_state_t *state)
{
    // Set the low battery indicator if battery power is low
    if (watch_get_lcd_type() == WATCH_LCD_TYPE_CUSTOM)
    {
        // interlocking arrows imply "exchange" the battery.
        clock_indicate(WATCH_INDICATOR_ARROWS, state->battery_low);
    }
    else
    {
        // LAP indicator on classic LCD is an adequate fallback.
        clock_indicate(WATCH_INDICATOR_LAP, state->battery_low);
    }
}

static void clock_display_quick_timer(stock_clock_state_t *state, watch_date_time_t current, bool blink)
{
    char buf[3 + 1];
    blink = false;

    if (state->timer_active)
    {
        uint32_t now = movement_get_utc_timestamp();
        uint32_t remaining = (state->timer_target_timestamp - now);

        if (blink && remaining > 30 && current.unit.second % 2)
        {
            watch_display_text(WATCH_POSITION_TOP_RIGHT, "  ");
        }
        else
        {
            // display remaining seconds during the last 30 seconds, otherwise display minutes
            // if (remaining > 30)
            // {
            //     remaining = (remaining + 30) / 60;
            // }
            snprintf(buf, sizeof(buf), "%2d", remaining / 60);
            watch_display_text(WATCH_POSITION_TOP_RIGHT, buf);
            snprintf(buf, sizeof(buf), "%2d_", current_quick_timer);
            watch_display_text_with_fallback(WATCH_POSITION_TOP, buf, buf);
            remaining = remaining % 60;
            snprintf(buf, sizeof(buf), "%2d", remaining);
            watch_display_text(WATCH_POSITION_SECONDS, buf);
        }
    }
    else
    {
        clock_display_all(current, false);
    }
}

static void clock_disable_quick_timer(stock_clock_state_t *state)
{
    state->timer_active = false;
    state->timer_target_timestamp = 0;
    movement_cancel_background_task_for_face(state->watch_face_index);
}

static void clock_enable_quick_timer(stock_clock_state_t *state, uint32_t target_timestamp)
{
    state->timer_active = true;
    state->timer_target_timestamp = target_timestamp;
    watch_date_time_t target_date_time = watch_utility_date_time_from_unix_time(state->timer_target_timestamp, 0);
    movement_schedule_background_task_for_face(state->watch_face_index, target_date_time);
}

static void clock_increase_quick_timer(stock_clock_state_t *state, watch_date_time_t current)
{
    uint32_t now = movement_get_utc_timestamp();

    uint8_t target_quick_timer;

    if (state->timer_target_timestamp == 0)
    {
        // We are first enabling a quick timer
        target_quick_timer = QUICK_TIMERS[0];
        current_quick_timer = 0;
    }
    else
    {

        for (uint8_t i = 0; i < N_QUICK_TIMERS; i++)
        {
            target_quick_timer = QUICK_TIMERS[i];
            if (target_quick_timer > current_quick_timer)
            {
                break;
            }
        }
    }

    if (target_quick_timer == 0)
    {
        clock_disable_quick_timer(state);
        current_quick_timer = 0;
    }
    else
    {
        if (current_quick_timer == 0)
        {
            clock_enable_quick_timer(state, now + (target_quick_timer) * 60);
        }
        else
        {
            clock_enable_quick_timer(state, state->timer_target_timestamp + (target_quick_timer - current_quick_timer) * 60);
        }

        current_quick_timer = target_quick_timer;
    }
}

static void clock_reset_quick_timer(stock_clock_state_t *state, watch_date_time_t current)
{
    uint32_t now = movement_get_utc_timestamp();

    clock_enable_quick_timer(state, now + (current_quick_timer) * 60);
}

static watch_date_time_t clock_24h_to_12h(watch_date_time_t date_time)
{
    date_time.unit.hour %= 12;

    if (date_time.unit.hour == 0)
    {
        date_time.unit.hour = 12;
    }

    return date_time;
}

static void clock_check_battery_periodically(stock_clock_state_t *state, watch_date_time_t date_time)
{
    // check the battery voltage once a day
    if (date_time.unit.day == state->last_battery_check)
    {
        return;
    }

    state->last_battery_check = date_time.unit.day;

    uint16_t voltage = watch_get_vcc_voltage();

    state->battery_low = voltage < CLOCK_FACE_LOW_BATTERY_VOLTAGE_THRESHOLD;

    clock_indicate_low_available_power(state);
}

static void clock_display_all(watch_date_time_t date_time, bool skip_seconds)
{
    char buf[8 + 1];

    snprintf(
        buf,
        sizeof(buf),
        movement_clock_mode_24h() == MOVEMENT_CLOCK_MODE_024H ? "%02d%02d%02d%02d" : "%2d%2d%02d%02d",
        date_time.unit.day,
        date_time.unit.hour,
        date_time.unit.minute,
        date_time.unit.second);

    if (skip_seconds)
    {
        buf[6] = '\0';
        buf[0] = '\0';
    }

    watch_display_text_with_fallback(WATCH_POSITION_TOP_LEFT, watch_utility_get_long_weekday(date_time), watch_utility_get_weekday(date_time));
    watch_display_text(WATCH_POSITION_TOP_RIGHT, buf);
    watch_display_text(WATCH_POSITION_BOTTOM, buf + 2);
}

static bool clock_display_some(watch_date_time_t current, watch_date_time_t previous, bool skip_seconds)
{
    if ((current.reg >> 6) == (previous.reg >> 6))
    {
        // everything before seconds is the same, don't waste cycles setting those segments.

        if (!skip_seconds)
        {
            watch_display_character_lp_seconds('0' + current.unit.second / 10, 8);
            watch_display_character_lp_seconds('0' + current.unit.second % 10, 9);
        }

        return true;
    }
    else if ((current.reg >> 12) == (previous.reg >> 12))
    {
        // everything before minutes is the same.

        char buf[4 + 1];

        snprintf(
            buf,
            sizeof(buf),
            "%02d%02d",
            current.unit.minute,
            current.unit.second);

        watch_display_text(WATCH_POSITION_MINUTES, buf);

        if (!skip_seconds)
        {
            watch_display_text(WATCH_POSITION_SECONDS, buf + 2);
        }

        return true;
    }
    else
    {
        // other stuff changed; let's do it all.
        return false;
    }
}

static void clock_display_clock(stock_clock_state_t *state, watch_date_time_t current)
{
    if (!clock_display_some(current, state->date_time.previous, state->timer_active))
    {
        if (movement_clock_mode_24h() == MOVEMENT_CLOCK_MODE_12H)
        {
            clock_indicate_pm(current);
            current = clock_24h_to_12h(current);
        }
        clock_display_all(current, state->timer_active);
    }

    if (state->timer_active)
    {
        clock_display_quick_timer(state, current, true);
    }
}

static void clock_display_low_energy(watch_date_time_t date_time)
{
    if (movement_clock_mode_24h() == MOVEMENT_CLOCK_MODE_12H)
    {
        clock_indicate_pm(date_time);
        date_time = clock_24h_to_12h(date_time);
    }
    char buf[8 + 1];

    snprintf(
        buf,
        sizeof(buf),
        movement_clock_mode_24h() == MOVEMENT_CLOCK_MODE_024H ? "%02d%02d%02d  " : "%2d%2d%02d  ",
        date_time.unit.day,
        date_time.unit.hour,
        date_time.unit.minute);

    watch_display_text_with_fallback(WATCH_POSITION_TOP_LEFT, watch_utility_get_long_weekday(date_time), watch_utility_get_weekday(date_time));
    watch_display_text(WATCH_POSITION_TOP_RIGHT, buf);
    watch_display_text(WATCH_POSITION_BOTTOM, buf + 2);
}

static void clock_start_tick_tock_animation(void)
{
    if (!watch_sleep_animation_is_running())
    {
        watch_start_sleep_animation(500);
        watch_start_indicator_blink_if_possible(WATCH_INDICATOR_COLON, 500);
    }
}

static void clock_stop_tick_tock_animation(void)
{
    if (watch_sleep_animation_is_running())
    {
        watch_stop_sleep_animation();
        watch_stop_blink();
    }
}

static void clock_toggle_clock_mode(void)
{
}

void stock_clock_face_setup(uint8_t watch_face_index, void **context_ptr)
{
    (void)watch_face_index;

    if (*context_ptr == NULL)
    {
        *context_ptr = malloc(sizeof(stock_clock_state_t));
        stock_clock_state_t *state = (stock_clock_state_t *)*context_ptr;
        state->watch_face_index = watch_face_index;
        state->timer_active = false;
        state->timer_target_timestamp = 0;

        // Initialize stopwatch fields
        state->stopwatch_mode = false;
        state->alarm_down_counter = 0;
        state->start_counter = 0;
        state->stop_counter = 0;
        state->lap_counter = 0;
        state->stopwatch_status = SW_STATUS_IDLE;
        state->slow_refresh = false;
        state->old_display.seconds = UINT_MAX;
        state->old_display.minutes = UINT_MAX;
        state->old_display.hours = UINT_MAX;
    }
}

void stock_clock_face_activate(void *context)
{
    stock_clock_state_t *state = (stock_clock_state_t *)context;

    clock_stop_tick_tock_animation();

    if (state->stopwatch_mode)
    {
        // Activating in stopwatch mode
        watch_display_text_with_fallback(WATCH_POSITION_TOP_LEFT, "STW", "ST");
        // force full re-draw of stopwatch
        state->old_display.seconds = UINT_MAX;
        state->old_display.minutes = UINT_MAX;
        state->old_display.hours = UINT_MAX;
        movement_request_tick_frequency(get_stopwatch_refresh_rate(state));
    }
    else
    {
        // Activating in clock mode
        clock_indicate_time_signal();
        clock_indicate_alarm();
        clock_indicate_24h();
        watch_set_colon();
        // this ensures that none of the timestamp fields will match, so we can re-render them all.
        state->date_time.previous.reg = 0xFFFFFFFF;
    }
}

bool stock_clock_face_loop(movement_event_t event, void *context)
{
    stock_clock_state_t *state = (stock_clock_state_t *)context;
    watch_date_time_t current;

    // Handle stopwatch mode
    if (state->stopwatch_mode)
    {
        rtc_counter_t counter = watch_rtc_get_counter();

        stopwatch_state_transition(state, counter, event.event_type);
        rtc_counter_t elapsed = stopwatch_elapsed_time(state, counter);

        switch (event.event_type)
        {
        case EVENT_ACTIVATE:
            watch_display_text_with_fallback(WATCH_POSITION_TOP_LEFT, "STW", "ST");
            _draw_stopwatch_indicators(state, event, elapsed);
            _display_elapsed(state, elapsed);
            break;
        case EVENT_ALARM_BUTTON_DOWN:
        case EVENT_LIGHT_BUTTON_DOWN:
        case EVENT_LIGHT_LONG_PRESS:
            _button_beep();
            // Fall through
        case EVENT_TICK:
            _draw_stopwatch_indicators(state, event, elapsed);
            _display_elapsed(state, elapsed);

            // Check if we've returned to clock mode
            if (!state->stopwatch_mode)
            {
                // Reactivate to show clock
                stock_clock_face_activate(context);
                current = movement_get_local_date_time();
                clock_display_clock(state, current);
                state->date_time.previous = current;
            }
            break;
        default:
            movement_default_loop_handler(event);
            break;
        }

        return true;
    }

    // Handle clock mode
    switch (event.event_type)
    {
    case EVENT_LOW_ENERGY_UPDATE:
        clock_start_tick_tock_animation();
        clock_display_low_energy(movement_get_local_date_time());
        break;
    case EVENT_ALARM_BUTTON_DOWN:
        if (state->timer_active)
        {
            current = movement_get_local_date_time();
            clock_increase_quick_timer(state, current);
            clock_display_quick_timer(state, current, false);
        }
        else
        {
            // Record the time when alarm button is pressed
            state->alarm_down_counter = watch_rtc_get_counter();
        }
        break;
    case EVENT_ALARM_BUTTON_UP:
        if (!state->timer_active)
        {
            // Switch to stopwatch mode, starting from the time recorded at button down
            state->stopwatch_mode = true;
            state->stopwatch_status = SW_STATUS_RUNNING;
            state->start_counter = state->alarm_down_counter;
            state->old_display.seconds = UINT_MAX;
            state->old_display.minutes = UINT_MAX;
            state->old_display.hours = UINT_MAX;
            movement_request_tick_frequency(get_stopwatch_refresh_rate(state));

            // Display stopwatch immediately
            watch_display_text_with_fallback(WATCH_POSITION_TOP_LEFT, "STW", "ST");
            rtc_counter_t counter = watch_rtc_get_counter();
            rtc_counter_t elapsed = stopwatch_elapsed_time(state, counter);
            _draw_stopwatch_indicators(state, event, elapsed);
            _display_elapsed(state, elapsed);
            _button_beep();
        }
        break;
    case EVENT_ALARM_LONG_PRESS:
        current = movement_get_local_date_time();
        if (state->timer_active)
        {
            clock_disable_quick_timer(state);
        }
        else
        {
            clock_increase_quick_timer(state, current);
        }
        clock_display_quick_timer(state, current, false);
        break;
    case EVENT_LIGHT_LONG_PRESS:
        if (state->timer_active)
        {
            current = movement_get_local_date_time();
            clock_reset_quick_timer(state, current);
            clock_display_quick_timer(state, current, false);
        }
        break;
    case EVENT_TICK:
    case EVENT_ACTIVATE:
        current = movement_get_local_date_time();

        clock_display_clock(state, current);

        clock_check_battery_periodically(state, current);

        state->date_time.previous = current;

        break;
    case EVENT_BACKGROUND_TASK:
        movement_play_alarm();
        current = movement_get_local_date_time();
        clock_disable_quick_timer(state);
        clock_display_quick_timer(state, current, false);
        break;
    default:
        return movement_default_loop_handler(event);
    }

    return true;
}

void stock_clock_face_resign(void *context)
{
    (void)context;
}
