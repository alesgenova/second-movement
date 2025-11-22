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
#include <math.h>
#include "sunrise_sunset_face.h"
#include "watch.h"
#include "watch_utility.h"
#include "watch_common_display.h"
#include "sunriset.h"

static const uint8_t _location_count = sizeof(longLatPresets) / sizeof(long_lat_presets_t);

static void _sunrise_sunset_set_expiration(sunrise_sunset_state_t *state, watch_date_time_t next_rise_set)
{
    uint32_t timestamp = watch_utility_date_time_to_unix_time(next_rise_set, 0);
    state->rise_set_expires = watch_utility_date_time_from_unix_time(timestamp + 60, 0);
}

static void _sunrise_sunset_face_update(sunrise_sunset_state_t *state)
{
    char buf[14];
    double rise, set, minutes, seconds;
    bool show_next_match = false;
    movement_location_t movement_location;

    // Get the current timezone index (same as set_time_face uses)
    uint8_t timezone_index = movement_get_timezone_index();

    // Use timezone index + 1 to access longLatPresets (index 0 is default)
    uint8_t preset_index = timezone_index + 1;

    // Bounds check
    if (preset_index >= _location_count)
    {
        watch_display_text_with_fallback(WATCH_POSITION_TOP, "Sunri", "rI");
        watch_display_text_with_fallback(WATCH_POSITION_BOTTOM, "No LOC", "No Loc");
        return;
    }

    movement_location.bit.latitude = longLatPresets[preset_index].latitude;
    movement_location.bit.longitude = longLatPresets[preset_index].longitude;

    if (movement_location.reg == 0)
    {
        watch_display_text_with_fallback(WATCH_POSITION_TOP, "Sunri", "rI");
        watch_display_text_with_fallback(WATCH_POSITION_BOTTOM, "No LOC", "No Loc");
        return;
    }

    watch_date_time_t date_time = movement_get_local_date_time();                                                           // the current local date / time
    watch_date_time_t utc_now = watch_utility_date_time_convert_zone(date_time, movement_get_current_timezone_offset(), 0); // the current date / time in UTC
    watch_date_time_t scratch_time;                                                                                         // scratchpad, contains different values at different times
    scratch_time.reg = utc_now.reg;

    // Weird quirky unsigned things were happening when I tried to cast these directly to doubles below.
    // it looks redundant, but extracting them to local int16's seemed to fix it.
    int16_t lat_centi = (int16_t)movement_location.bit.latitude;
    int16_t lon_centi = (int16_t)movement_location.bit.longitude;

    double lat = (double)lat_centi / 100.0;
    double lon = (double)lon_centi / 100.0;

    // sunriset returns the rise/set times as signed decimal hours in UTC.
    // this can mean hours below 0 or above 31, which won't fit into a watch_date_time_t struct.
    // to deal with this, we set aside the offset in hours, and add it back before converting it to a watch_date_time_t.
    double hours_from_utc = ((double)movement_get_current_timezone_offset()) / 3600.0;

    // we loop twice because if it's after sunset today, we need to recalculate to display values for tomorrow.
    for (int i = 0; i < 2; i++)
    {
        uint8_t result = sun_rise_set(scratch_time.unit.year + WATCH_RTC_REFERENCE_YEAR, scratch_time.unit.month, scratch_time.unit.day, lon, lat, &rise, &set);

        if (result != 0)
        {
            watch_clear_colon();
            watch_clear_indicator(WATCH_INDICATOR_PM);
            watch_clear_indicator(WATCH_INDICATOR_24H);
            if (result == 1)
                watch_display_text_with_fallback(WATCH_POSITION_TOP_LEFT, "SET", "SE");
            else
                watch_display_text_with_fallback(WATCH_POSITION_TOP_LEFT, "RIS", "rI");
            sprintf(buf, "%2d", scratch_time.unit.day);
            watch_display_text(WATCH_POSITION_TOP_RIGHT, buf);
            watch_display_text(WATCH_POSITION_BOTTOM, "None  ");
            return;
        }

        watch_set_colon();
        if (movement_clock_mode_24h())
            watch_set_indicator(WATCH_INDICATOR_24H);

        rise += hours_from_utc;
        set += hours_from_utc;

        minutes = 60.0 * fmod(rise, 1);
        seconds = 60.0 * fmod(minutes, 1);
        scratch_time.unit.hour = floor(rise);
        if (seconds < 30)
            scratch_time.unit.minute = floor(minutes);
        else
            scratch_time.unit.minute = ceil(minutes);

        // Handle hour overflow from timezone conversion
        while (scratch_time.unit.hour >= 24)
        {
            scratch_time.unit.hour -= 24;
            // Increment day (this will be handled by the date arithmetic)
            uint32_t timestamp = watch_utility_date_time_to_unix_time(scratch_time, 0);
            timestamp += 86400;
            scratch_time = watch_utility_date_time_from_unix_time(timestamp, 0);
        }

        if (scratch_time.unit.minute == 60)
        {
            scratch_time.unit.minute = 0;
            scratch_time.unit.hour = (scratch_time.unit.hour + 1) % 24;
        }

        if (date_time.reg < scratch_time.reg)
            _sunrise_sunset_set_expiration(state, scratch_time);

        if (date_time.reg < scratch_time.reg || show_next_match)
        {
            if (state->rise_index == 0 || show_next_match)
            {
                if (!movement_clock_mode_24h())
                {
                    if (watch_utility_convert_to_12_hour(&scratch_time))
                        watch_set_indicator(WATCH_INDICATOR_PM);
                    else
                        watch_clear_indicator(WATCH_INDICATOR_PM);
                }
                watch_display_text_with_fallback(WATCH_POSITION_TOP_LEFT, "RIS", "rI");
                sprintf(buf, "%2d", scratch_time.unit.day);
                watch_display_text(WATCH_POSITION_TOP_RIGHT, buf);
                sprintf(buf, "%2d%02d%2s", scratch_time.unit.hour, scratch_time.unit.minute, longLatPresets[preset_index].name);
                watch_display_text(WATCH_POSITION_BOTTOM, buf);
                return;
            }
            else
            {
                show_next_match = true;
            }
        }

        minutes = 60.0 * fmod(set, 1);
        seconds = 60.0 * fmod(minutes, 1);
        scratch_time.unit.hour = floor(set);
        if (seconds < 30)
            scratch_time.unit.minute = floor(minutes);
        else
            scratch_time.unit.minute = ceil(minutes);

        // Handle hour overflow from timezone conversion
        while (scratch_time.unit.hour >= 24)
        {
            scratch_time.unit.hour -= 24;
            // Increment day (this will be handled by the date arithmetic)
            uint32_t timestamp = watch_utility_date_time_to_unix_time(scratch_time, 0);
            timestamp += 86400;
            scratch_time = watch_utility_date_time_from_unix_time(timestamp, 0);
        }

        if (scratch_time.unit.minute == 60)
        {
            scratch_time.unit.minute = 0;
            scratch_time.unit.hour = (scratch_time.unit.hour + 1) % 24;
        }

        if (date_time.reg < scratch_time.reg)
            _sunrise_sunset_set_expiration(state, scratch_time);

        if (date_time.reg < scratch_time.reg || show_next_match)
        {
            if (state->rise_index == 0 || show_next_match)
            {
                if (!movement_clock_mode_24h())
                {
                    if (watch_utility_convert_to_12_hour(&scratch_time))
                        watch_set_indicator(WATCH_INDICATOR_PM);
                    else
                        watch_clear_indicator(WATCH_INDICATOR_PM);
                }
                watch_display_text_with_fallback(WATCH_POSITION_TOP_LEFT, "SET", "SE");
                sprintf(buf, "%2d", scratch_time.unit.day);
                watch_display_text(WATCH_POSITION_TOP_RIGHT, buf);
                sprintf(buf, "%2d%02d%2s", scratch_time.unit.hour, scratch_time.unit.minute, longLatPresets[preset_index].name);
                watch_display_text(WATCH_POSITION_BOTTOM, buf);
                return;
            }
            else
            {
                show_next_match = true;
            }
        }

        // it's after sunset. we need to display sunrise/sunset for tomorrow.
        uint32_t timestamp = watch_utility_date_time_to_unix_time(utc_now, 0);
        timestamp += 86400;
        scratch_time = watch_utility_date_time_from_unix_time(timestamp, 0);
    }
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
        if (date_time.reg >= state->rise_set_expires.reg)
        {
            // and on the off chance that this happened before EVENT_TIMEOUT snapped us back to rise/set 0, go back now
            state->rise_index = 0;
            _sunrise_sunset_face_update(state);
        }
        break;
    case EVENT_LIGHT_BUTTON_DOWN:
        movement_illuminate_led();
        break;
    case EVENT_ALARM_BUTTON_UP:
        state->rise_index = (state->rise_index + 1) % 2;
        _sunrise_sunset_face_update(state);
        break;
    case EVENT_TIMEOUT:
        state->rise_index = 0;
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
    state->rise_index = 0;
}
