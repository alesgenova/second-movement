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
 *
 * Sunrise/sunset calculations are public domain code by Paul Schlyter, December 1992
 *
 */

#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <ctype.h>
#include "sunrise_sunset_face.h"
#include "watch.h"
#include "watch_utility.h"
#include "watch_common_display.h"
#include "sunriset.h"
#include "movement_location_data.h"

static void _sunrise_sunset_set_expiration(sunrise_sunset_state_t *state, watch_date_time_t next_rise_set)
{
    uint32_t timestamp = watch_utility_date_time_to_unix_time(next_rise_set, 0);
    state->rise_set_expires = watch_utility_date_time_from_unix_time(timestamp + 60, 0);
}

static void _format_location_code(const char *name, char out[3])
{
    out[0] = '?';
    out[1] = '?';
    out[2] = '\0';

    if (!name || !name[0])
        return;

    bool first_set = false;
    bool second_set = false;
    unsigned char previous = ' ';

    for (size_t i = 0; name[i] != '\0'; i++)
    {
        unsigned char ch = (unsigned char)name[i];
        if (isalpha(ch))
        {
            if (!first_set)
            {
                out[0] = (char)toupper(ch);
                first_set = true;
            }
            else if (!second_set && !isalpha(previous))
            {
                out[1] = (char)toupper(ch);
                second_set = true;
                break;
            }
        }
        previous = ch;
    }

    if (!second_set && first_set)
    {
        for (size_t i = 1; name[i] != '\0'; i++)
        {
            unsigned char ch = (unsigned char)name[i];
            if (isalpha(ch))
            {
                out[1] = (char)toupper(ch);
                second_set = true;
                break;
            }
        }
    }

    if (!first_set)
    {
        out[0] = '?';
    }
    if (!second_set)
    {
        out[1] = '?';
    }
}

typedef struct
{
    watch_date_time_t time;
    bool is_rise;
} sunrise_sunset_event_t;

#define SUNRISE_SUNSET_MAX_DAYS_LOOKAHEAD 366
#define SUNRISE_SUNSET_MAX_OFFSET (SUNRISE_SUNSET_MAX_DAYS_LOOKAHEAD * 2)

static watch_date_time_t _sunrise_sunset_day_start(watch_date_time_t date_time)
{
    date_time.unit.hour = 0;
    date_time.unit.minute = 0;
    date_time.unit.second = 0;
    return date_time;
}

static watch_date_time_t _sunrise_sunset_add_days(watch_date_time_t date_time, int16_t days)
{
    int64_t timestamp = (int64_t)watch_utility_date_time_to_unix_time(date_time, 0) + (int64_t)days * 86400;
    if (timestamp < 0)
        timestamp = 0;
    return watch_utility_date_time_from_unix_time((uint32_t)timestamp, 0);
}

static watch_date_time_t _sunrise_sunset_make_event_time(watch_date_time_t base_day, double event_hours)
{
    watch_date_time_t day_start = _sunrise_sunset_day_start(base_day);
    int32_t total_minutes = (int32_t)lround(event_hours * 60.0);
    int64_t timestamp = (int64_t)watch_utility_date_time_to_unix_time(day_start, 0) + (int64_t)total_minutes * 60;
    if (timestamp < 0)
        timestamp = 0;
    return watch_utility_date_time_from_unix_time((uint32_t)timestamp, 0);
}

static uint8_t _sunrise_sunset_compute_day_events(watch_date_time_t day, double lon, double lat, double hours_from_utc, sunrise_sunset_event_t events[2])
{
    double rise, set;
    uint8_t result = sun_rise_set(day.unit.year + WATCH_RTC_REFERENCE_YEAR, day.unit.month, day.unit.day, lon, lat, &rise, &set);

    if (result != 0)
        return result;

    rise += hours_from_utc;
    set += hours_from_utc;

    events[0].time = _sunrise_sunset_make_event_time(day, rise);
    events[0].is_rise = true;
    events[1].time = _sunrise_sunset_make_event_time(day, set);
    events[1].is_rise = false;

    return 0;
}

static bool _sunrise_sunset_find_future_event(
    watch_date_time_t utc_now,
    watch_date_time_t now_local,
    double lon,
    double lat,
    double hours_from_utc,
    int16_t target_offset,
    sunrise_sunset_event_t *target_event,
    sunrise_sunset_event_t *first_future_event,
    bool *has_first_future_event,
    uint8_t *initial_result,
    int16_t *max_index_reached)
{
    watch_date_time_t day_cursor = _sunrise_sunset_day_start(utc_now);
    int16_t current_index = -1;
    bool expiration_set = false;
    sunrise_sunset_event_t last_event;
    bool have_last_event = false;

    if (has_first_future_event)
        *has_first_future_event = false;
    if (initial_result)
        *initial_result = 0;

    for (int day = 0; day <= SUNRISE_SUNSET_MAX_DAYS_LOOKAHEAD; day++)
    {
        sunrise_sunset_event_t events[2];
        uint8_t result = _sunrise_sunset_compute_day_events(day_cursor, lon, lat, hours_from_utc, events);

        if (day == 0 && initial_result)
            *initial_result = result;

        if (result == 0)
        {
            for (int i = 0; i < 2; i++)
            {
                sunrise_sunset_event_t event = events[i];

                if (event.time.reg >= now_local.reg)
                {
                    if (!expiration_set && first_future_event)
                    {
                        *first_future_event = event;
                        expiration_set = true;
                        if (has_first_future_event)
                            *has_first_future_event = true;
                    }

                    current_index++;
                    last_event = event;
                    have_last_event = true;

                    if (current_index == target_offset)
                    {
                        if (target_event)
                            *target_event = event;
                        if (max_index_reached)
                            *max_index_reached = current_index;
                        return true;
                    }
                }
            }
        }

        day_cursor = _sunrise_sunset_add_days(day_cursor, 1);
    }

    if (max_index_reached)
        *max_index_reached = current_index;
    if (have_last_event && target_event)
        *target_event = last_event;
    if (has_first_future_event && !expiration_set)
        *has_first_future_event = false;

    return false;
}

static bool _sunrise_sunset_find_past_event(
    watch_date_time_t utc_now,
    watch_date_time_t now_local,
    double lon,
    double lat,
    double hours_from_utc,
    int16_t target_offset,
    sunrise_sunset_event_t *target_event,
    int16_t *max_offset_reached)
{
    watch_date_time_t day_cursor = _sunrise_sunset_day_start(utc_now);
    sunrise_sunset_event_t last_event;
    bool have_last_event = false;
    int16_t count = 0;

    for (int day = 0; day <= SUNRISE_SUNSET_MAX_DAYS_LOOKAHEAD; day++)
    {
        sunrise_sunset_event_t events[2];
        uint8_t result = _sunrise_sunset_compute_day_events(day_cursor, lon, lat, hours_from_utc, events);

        if (result == 0)
        {
            int first_idx = (events[0].time.reg >= events[1].time.reg) ? 0 : 1;
            int second_idx = first_idx == 0 ? 1 : 0;

            for (int iteration = 0; iteration < 2; iteration++)
            {
                int idx = (iteration == 0) ? first_idx : second_idx;
                sunrise_sunset_event_t event = events[idx];

                if (event.time.reg <= now_local.reg)
                {
                    count++;
                    last_event = event;
                    have_last_event = true;

                    if (count == target_offset)
                    {
                        if (target_event)
                            *target_event = event;
                        if (max_offset_reached)
                            *max_offset_reached = count;
                        return true;
                    }
                }
            }
        }

        day_cursor = _sunrise_sunset_add_days(day_cursor, -1);
    }

    if (max_offset_reached)
        *max_offset_reached = count;
    if (have_last_event && target_event)
        *target_event = last_event;

    return false;
}

static void _sunrise_sunset_display_none(uint8_t result, watch_date_time_t reference_time)
{
    char buf[14];

    watch_clear_colon();
    watch_clear_indicator(WATCH_INDICATOR_PM);
    watch_clear_indicator(WATCH_INDICATOR_24H);

    if (result == 1)
        watch_display_text_with_fallback(WATCH_POSITION_TOP_LEFT, "SET", "SE");
    else
        watch_display_text_with_fallback(WATCH_POSITION_TOP_LEFT, "RIS", "rI");

    sprintf(buf, "%2d", reference_time.unit.day);
    watch_display_text(WATCH_POSITION_TOP_RIGHT, buf);
    watch_display_text(WATCH_POSITION_BOTTOM, "None  ");
}

static void _sunrise_sunset_face_update(sunrise_sunset_state_t *state)
{
    char buf[14];
    movement_location_t movement_location = (movement_location_t)watch_get_backup_data(1);
    const location_long_lat_presets_t *location_preset = NULL;
    const location_long_lat_presets_t *display_location = NULL;
    char location_code[3];

    uint8_t timezone_index = movement_get_timezone_index();

    if (movement_location.reg != 0)
    {
        location_preset = movement_location_find_by_coordinates(movement_location.bit.latitude, movement_location.bit.longitude, timezone_index);
    }

    if (movement_location.reg == 0)
    {
        location_preset = movement_location_get_default_for_zone(timezone_index);
        if (location_preset)
        {
            movement_location.bit.latitude = location_preset->latitude;
            movement_location.bit.longitude = location_preset->longitude;
        }
    }

    display_location = location_preset ? location_preset : movement_location_get_default_for_zone(timezone_index);

    if (movement_location.reg == 0)
    {
        watch_display_text_with_fallback(WATCH_POSITION_TOP, "Sunri", "rI");
        watch_display_text_with_fallback(WATCH_POSITION_BOTTOM, "No LOC", "No Loc");
        return;
    }

    _format_location_code(display_location ? display_location->name : NULL, location_code);

    watch_date_time_t date_time = movement_get_local_date_time();
    watch_date_time_t utc_now = watch_utility_date_time_convert_zone(date_time, movement_get_current_timezone_offset(), 0);

    int16_t lat_centi = (int16_t)movement_location.bit.latitude;
    int16_t lon_centi = (int16_t)movement_location.bit.longitude;

    double lat = (double)lat_centi / 100.0;
    double lon = (double)lon_centi / 100.0;
    double hours_from_utc = ((double)movement_get_current_timezone_offset()) / 3600.0;

    int16_t offset = state->event_offset;

    sunrise_sunset_event_t future_candidate;
    sunrise_sunset_event_t first_future_event;
    bool has_first_future_event = false;
    uint8_t initial_result = 0;
    int16_t future_max_index = -1;

    bool future_found = _sunrise_sunset_find_future_event(
        utc_now,
        date_time,
        lon,
        lat,
        hours_from_utc,
        offset >= 0 ? offset : 0,
        &future_candidate,
        &first_future_event,
        &has_first_future_event,
        &initial_result,
        &future_max_index);

    sunrise_sunset_event_t event_to_display;
    bool have_event = false;

    if (offset < 0)
    {
        sunrise_sunset_event_t past_candidate;
        int16_t past_max_index = 0;
        bool past_found = _sunrise_sunset_find_past_event(
            utc_now,
            date_time,
            lon,
            lat,
            hours_from_utc,
            (int16_t)(-offset),
            &past_candidate,
            &past_max_index);

        if (!past_found)
        {
            if (past_max_index > 0)
            {
                state->event_offset = (int16_t)(-past_max_index);
                offset = state->event_offset;
                past_found = true;
            }
            else
            {
                state->event_offset = 0;
                offset = 0;
            }
        }

        if (past_found)
        {
            event_to_display = past_candidate;
            have_event = true;
        }
    }

    if (offset >= 0)
    {
        if (!has_first_future_event)
        {
            state->event_offset = 0;
            _sunrise_sunset_display_none(initial_result, date_time);
            state->rise_set_expires.reg = 0;
            return;
        }

        if (!future_found && future_max_index >= 0)
        {
            state->event_offset = future_max_index;
            offset = state->event_offset;
            future_found = true;
        }

        if (!have_event)
        {
            event_to_display = future_candidate;
            have_event = true;
        }
    }

    if (!have_event)
    {
        if (has_first_future_event)
        {
            state->event_offset = 0;
            event_to_display = future_candidate;
            have_event = true;
        }
        else
        {
            state->event_offset = 0;
            _sunrise_sunset_display_none(initial_result, date_time);
            state->rise_set_expires.reg = 0;
            return;
        }
    }

    if (has_first_future_event)
        _sunrise_sunset_set_expiration(state, first_future_event.time);
    else
        state->rise_set_expires.reg = 0;

    watch_set_colon();
    if (movement_clock_mode_24h())
        watch_set_indicator(WATCH_INDICATOR_24H);
    else
        watch_clear_indicator(WATCH_INDICATOR_24H);

    watch_date_time_t display_time = event_to_display.time;

    if (!movement_clock_mode_24h())
    {
        if (watch_utility_convert_to_12_hour(&display_time))
            watch_set_indicator(WATCH_INDICATOR_PM);
        else
            watch_clear_indicator(WATCH_INDICATOR_PM);
    }
    else
    {
        watch_clear_indicator(WATCH_INDICATOR_PM);
    }

    if (event_to_display.is_rise)
        watch_display_text_with_fallback(WATCH_POSITION_TOP_LEFT, "RIS", "rI");
    else
        watch_display_text_with_fallback(WATCH_POSITION_TOP_LEFT, "SET", "SE");

    sprintf(buf, "%2d", display_time.unit.day);
    watch_display_text(WATCH_POSITION_TOP_RIGHT, buf);
    sprintf(buf, "%2d%02d%2s", display_time.unit.hour, display_time.unit.minute, location_code);
    watch_display_text(WATCH_POSITION_BOTTOM, buf);
}

void sunrise_sunset_face_setup(uint8_t watch_face_index, void **context_ptr)
{
    (void)watch_face_index;
    if (*context_ptr == NULL)
    {
        *context_ptr = malloc(sizeof(sunrise_sunset_state_t));
        memset(*context_ptr, 0, sizeof(sunrise_sunset_state_t));
    }
}

void sunrise_sunset_face_activate(void *context)
{
    if (watch_sleep_animation_is_running())
        watch_stop_sleep_animation();
    (void)context;
}

bool sunrise_sunset_face_loop(movement_event_t event, void *context)
{
    sunrise_sunset_state_t *state = (sunrise_sunset_state_t *)context;

    switch (event.event_type)
    {
    case EVENT_ACTIVATE:
        _sunrise_sunset_face_update(state);
        break;
    case EVENT_LOW_ENERGY_UPDATE:
    case EVENT_TICK:
        // if entering low energy mode, start tick animation
        if (event.event_type == EVENT_LOW_ENERGY_UPDATE && !watch_sleep_animation_is_running())
            watch_start_sleep_animation(1000);
        // check if we need to update the display
        watch_date_time_t date_time = movement_get_local_date_time();
        if (state->rise_set_expires.reg != 0 && date_time.reg >= state->rise_set_expires.reg)
        {
            // and on the off chance that this happened before EVENT_TIMEOUT snapped us back to rise/set 0, go back now
            state->event_offset = 0;
            _sunrise_sunset_face_update(state);
        }
        break;
    case EVENT_LIGHT_BUTTON_DOWN:
        movement_illuminate_led();
        if (state->event_offset > -SUNRISE_SUNSET_MAX_OFFSET)
            state->event_offset--;
        _sunrise_sunset_face_update(state);
        break;
    case EVENT_ALARM_BUTTON_DOWN:
        if (state->event_offset < SUNRISE_SUNSET_MAX_OFFSET)
            state->event_offset++;
        _sunrise_sunset_face_update(state);
        break;
    case EVENT_TIMEOUT:
        state->event_offset = 0;
        movement_request_tick_frequency(1);
        _sunrise_sunset_face_update(state);
        break;
    default:
        return movement_default_loop_handler(event);
    }

    return true;
}

void sunrise_sunset_face_resign(void *context)
{
    sunrise_sunset_state_t *state = (sunrise_sunset_state_t *)context;
    state->event_offset = 0;
    state->rise_set_expires.reg = 0;
}
