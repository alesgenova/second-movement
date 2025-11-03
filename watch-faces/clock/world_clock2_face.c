/*
 * MIT License
 *
 * Copyright (c) 2023-2025 Konrad Rieck
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

#include <stdlib.h>
#include <string.h>
#include "world_clock2_face.h"
#include "watch_utility.h"
#include "watch_common_display.h"
#include "watch.h"
#include "zones.h"

static bool refresh_face;

/* Beep types */
typedef enum
{
    BEEP_BUTTON,
    BEEP_ENABLE,
    BEEP_DISABLE
} beep_type_t;

/* Simple macros for navigation */
#define FORWARD +1
#define BACKWARD -1

/* Constants */
#define NAME_DISPLAY_TIME 2

/* Pre-selected locations: New York City, London, Tokyo, Sydney.
 * NOTE: Previously this used a macro with arithmetic on signed 8-bit values
 * (e.g. 83 - 59) and stored them in an int8_t array with a -1 sentinel.
 * That caused indices > 127 to wrap to negative values when treated as int8_t.
 * To make this robust against changes in ordering and to eliminate signed
 * overflow, we now select by city name at setup time. */
static const char *const PRESELECTED_LOCATION_NAMES[] = {"New York City", "London", "Tokyo", "Sydney"};
#define NUM_PRESELECTED_LOCATIONS (sizeof(PRESELECTED_LOCATION_NAMES) / sizeof(PRESELECTED_LOCATION_NAMES[0]))

/* Modulo function */
static inline unsigned int mod(int a, int b)
{
    int r = a % b;
    return r < 0 ? r + b : r;
}

/* Get number of locations (use wider type to future-proof beyond 255) */
static inline uint16_t _get_num_locations(void)
{
    return movement_location_presets_count;
}

/* Find the next selected location */
static inline uint8_t _next_selected_location(world_clock2_state_t *state, int direction)
{
    uint16_t i = state->current_location;
    uint16_t num_locs = _get_num_locations();
    while (true)
    {
        i = mod(i + direction, num_locs);
        /* Return next selected location */
        if (state->locations[i].selected)
        {
            return (uint8_t)i; /* safe cast: i < num_locs <= MAX_LOCATIONS */
        }
        /* Could not find a selected location. Return first one */
        if (i == state->current_location)
        {
            return 0;
        }
    }
}

/* Find the next location in a specific region, ordered by longitude (west to east) */
static inline uint8_t _next_location_in_region(world_clock2_state_t *state, int direction)
{
    uint16_t current_idx = state->current_location;
    uint8_t region = state->current_region;
    int16_t current_lon = movement_location_presets[current_idx].longitude;
    uint16_t num_locs = _get_num_locations();

    uint16_t best_idx = current_idx;
    int16_t best_lon;
    bool found = false;

    if (direction == FORWARD)
    {
        // Find location with smallest longitude > current_lon
        best_lon = 18000; // Maximum longitude
        for (uint16_t i = 0; i < num_locs; i++)
        {
            if (movement_location_presets[i].region == region)
            {
                int16_t lon = movement_location_presets[i].longitude;
                if (lon > current_lon && lon < best_lon)
                {
                    best_lon = lon;
                    best_idx = i;
                    found = true;
                }
            }
        }
        // If none found, wrap to westernmost
        if (!found)
        {
            best_lon = 18000;
            for (uint16_t i = 0; i < num_locs; i++)
            {
                if (movement_location_presets[i].region == region && movement_location_presets[i].longitude < best_lon)
                {
                    best_lon = movement_location_presets[i].longitude;
                    best_idx = i;
                }
            }
        }
    }
    else
    {
        // Find location with largest longitude < current_lon
        best_lon = -18000; // Minimum longitude
        for (uint16_t i = 0; i < num_locs; i++)
        {
            if (movement_location_presets[i].region == region)
            {
                int16_t lon = movement_location_presets[i].longitude;
                if (lon < current_lon && lon > best_lon)
                {
                    best_lon = lon;
                    best_idx = i;
                    found = true;
                }
            }
        }
        // If none found, wrap to easternmost
        if (!found)
        {
            best_lon = -18000;
            for (uint16_t i = 0; i < num_locs; i++)
            {
                if (movement_location_presets[i].region == region && movement_location_presets[i].longitude > best_lon)
                {
                    best_lon = movement_location_presets[i].longitude;
                    best_idx = i;
                }
            }
        }
    }

    return (uint8_t)best_idx; /* safe cast */
}
static udatetime_t _movement_convert_date_time_to_udate(watch_date_time_t dt)
{
    /* *INDENT-OFF* */
    return (udatetime_t){
        .date.dayofmonth = dt.unit.day,
        .date.dayofweek = dayofweek(UYEAR_FROM_YEAR(dt.unit.year + WATCH_RTC_REFERENCE_YEAR), dt.unit.month, dt.unit.day),
        .date.month = dt.unit.month,
        .date.year = UYEAR_FROM_YEAR(dt.unit.year + WATCH_RTC_REFERENCE_YEAR),
        .time.hour = dt.unit.hour,
        .time.minute = dt.unit.minute,
        .time.second = dt.unit.second};
    /* *INDENT-ON* */
}

/* Play beep sound based on type */
static inline void _beep(beep_type_t beep_type)
{
    if (!movement_button_should_sound())
        return;

    switch (beep_type)
    {
    case BEEP_BUTTON:
        watch_buzzer_play_note(BUZZER_NOTE_C7, 50);
        break;

    case BEEP_ENABLE:
        watch_buzzer_play_note(BUZZER_NOTE_G7, 50);
        watch_buzzer_play_note(BUZZER_NOTE_REST, 75);
        watch_buzzer_play_note(BUZZER_NOTE_C8, 75);
        break;

    case BEEP_DISABLE:
        watch_buzzer_play_note(BUZZER_NOTE_C8, 50);
        watch_buzzer_play_note(BUZZER_NOTE_REST, 75);
        watch_buzzer_play_note(BUZZER_NOTE_G7, 75);
        break;
    }
}

/* Display location name/abbreviation on top */
static void _display_location_abbr(world_clock2_state_t *state, const char *abbr)
{
    char buf[11];

    if (watch_get_lcd_type() == WATCH_LCD_TYPE_CUSTOM)
    {
        /* Long abbreviation on custom LCD */
        sprintf(buf, "%-5s", abbr);
        watch_display_text_with_fallback(WATCH_POSITION_TOP, buf, buf);
    }
    else
    {
        /* Short abbreviation with location number on classic LCD */
        sprintf(buf, "%.2s%2d", abbr, state->current_location);
        watch_display_text(WATCH_POSITION_TOP_LEFT, buf);
        watch_display_text(WATCH_POSITION_TOP_RIGHT, buf + 2);
    }
}

/* Get abbreviation for current location's timezone */
static void _get_location_zone_info(world_clock2_state_t *state, char *abbr, uoffset_t *offset)
{
    uzone_t zone_info;
    udatetime_t dt;
    char ds;
    uint8_t zone_idx = movement_location_presets[state->current_location].timezone;

    watch_date_time_t utc_time = watch_rtc_get_date_time();
    unpack_zone(&zone_defns[zone_idx], "", &zone_info);
    dt = _movement_convert_date_time_to_udate(utc_time);
    ds = get_current_offset(&zone_info, &dt, offset);
    sprintf(abbr, zone_info.abrev_formatter, ds);
}

/* Efficient time display taken from world_clock_face.c */
static void _efficient_time_display(movement_event_t event, watch_date_time_t date_time,
                                    uint32_t previous_date_time, char *buf)
{
    if ((date_time.reg >> 6) == (previous_date_time >> 6) && event.event_type != EVENT_LOW_ENERGY_UPDATE)
    {
        // everything before seconds is the same, don't waste cycles setting those segments.
        watch_display_character_lp_seconds('0' + date_time.unit.second / 10, 8);
        watch_display_character_lp_seconds('0' + date_time.unit.second % 10, 9);
    }
    else if ((date_time.reg >> 12) == (previous_date_time >> 12) && event.event_type != EVENT_LOW_ENERGY_UPDATE)
    {
        // everything before minutes is the same.
        sprintf(buf, "%02d%02d", date_time.unit.minute, date_time.unit.second);
        watch_display_text(WATCH_POSITION_MINUTES, buf);
        watch_display_text(WATCH_POSITION_SECONDS, buf + 2);
    }
    else
    {
        // other stuff changed; let's do it all.
        if (!movement_clock_mode_24h())
        {
            // if we are in 12 hour mode, do some cleanup.
            if (date_time.unit.hour < 12)
            {
                watch_clear_indicator(WATCH_INDICATOR_PM);
            }
            else
            {
                watch_set_indicator(WATCH_INDICATOR_PM);
            }
            date_time.unit.hour %= 12;
            if (date_time.unit.hour == 0)
                date_time.unit.hour = 12;
        }

        /* Display colon and 24h indicator */
        watch_set_colon();
        if (movement_clock_mode_24h())
            watch_set_indicator(WATCH_INDICATOR_24H);

        /* Display day and time */
        sprintf(buf, "%02d%02d%02d", date_time.unit.hour, date_time.unit.minute, date_time.unit.second);
        watch_display_text(WATCH_POSITION_HOURS, buf + 0);
        watch_display_text(WATCH_POSITION_MINUTES, buf + 2);

        if (event.event_type == EVENT_LOW_ENERGY_UPDATE)
        {
            if (!watch_sleep_animation_is_running())
            {
                watch_display_text(WATCH_POSITION_SECONDS, "  ");
                watch_start_sleep_animation(500);
                watch_start_indicator_blink_if_possible(WATCH_INDICATOR_COLON, 500);
            }
        }
        else
        {
            watch_display_text(WATCH_POSITION_SECONDS, buf + 4);
        }
    }
}

static void _clock_display(movement_event_t event, world_clock2_state_t *state)
{
    char buf[11], zone_abbr[MAX_ZONE_NAME_LEN + 1];
    const char *location_name;
    watch_date_time_t date_time, utc_time;
    uint32_t previous_date_time;
    int32_t offset;
    uoffset_t zone_offset;
    uint8_t zone_idx;

    /* Update indicators and reset previous date time */
    if (refresh_face)
    {
        watch_clear_indicator(WATCH_INDICATOR_SIGNAL);
        state->previous_date_time = 0xFFFFFFFF;
        refresh_face = false;
    }

    /* Determine current time at location's timezone and store date/time */
    utc_time = watch_rtc_get_date_time();
    zone_idx = movement_location_presets[state->current_location].timezone;
    offset = movement_get_current_timezone_offset_for_zone(zone_idx);
    date_time = watch_utility_date_time_convert_zone(utc_time, 0, offset);
    previous_date_time = state->previous_date_time;
    state->previous_date_time = date_time.reg;

    if (state->show_zone_name > 0)
    {
        /* Check for first call to display location name */
        if (state->show_zone_name == NAME_DISPLAY_TIME)
        {
            watch_clear_colon();
            watch_clear_indicator(WATCH_INDICATOR_24H);
            watch_clear_indicator(WATCH_INDICATOR_PM);
            location_name = movement_location_presets[state->current_location].name;
            watch_display_text_with_fallback(WATCH_POSITION_BOTTOM, location_name, location_name);
        }
        state->show_zone_name--;
        /* Check for last call to display location name */
        if (state->show_zone_name == 0)
        {
            refresh_face = true;
        }
    }
    else
    {
        _efficient_time_display(event, date_time, previous_date_time, buf);
    }

    _get_location_zone_info(state, zone_abbr, &zone_offset);
    _display_location_abbr(state, zone_abbr);
}

/* Display region selection screen */
static void _region_selection_display(movement_event_t event, world_clock2_state_t *state)
{
    char buf[11];
    static const char *region_names[] = {"N America", "Asia", "Europe", "Africa", "S America", "Australia"};

    /* Update indicators */
    if (refresh_face)
    {
        watch_clear_colon();
        watch_clear_indicator(WATCH_INDICATOR_24H);
        watch_clear_indicator(WATCH_INDICATOR_PM);
        watch_clear_indicator(WATCH_INDICATOR_SIGNAL);
        refresh_face = false;
    }

    /* Display region numbers in world-map arrangement */
    /* Top row: 0(N.America), 2(Europe), 1(Asia) - northern continents */
    /* Bottom row: 4(S.America), 3(Africa), 5(Australia) - southern continents */
    /* Layout: Top: [ 0] [2 ] [ 1]   Bottom: [  4  3  5] */

    bool blink_state = (event.subsecond % 2) == 0;

    /* Display top row - northern continents: [0 2][ 1] or [02][ 1] */
    /* TOP_LEFT shows regions 0 and 2: "0 2" (3 chars) or "02" (2 chars) */
    /* TOP_RIGHT shows region 1: " 1" */

    /* Build top left string with regions 0 and 2 */
    char top_left_3[4], top_left_2[3];

    /* 3-char version: "0 2" with blinking */
    if (state->current_region == 0 && !blink_state)
    {
        sprintf(top_left_3, "  2");
    }
    else if (state->current_region == 2 && !blink_state)
    {
        sprintf(top_left_3, "0  ");
    }
    else
    {
        sprintf(top_left_3, "0 2");
    }

    /* 2-char version: "02" with blinking */
    if (state->current_region == 0 && !blink_state)
    {
        sprintf(top_left_2, " 2");
    }
    else if (state->current_region == 2 && !blink_state)
    {
        sprintf(top_left_2, "0 ");
    }
    else
    {
        sprintf(top_left_2, "02");
    }

    watch_display_text_with_fallback(WATCH_POSITION_TOP_LEFT, top_left_3, top_left_2);

    /* Region 1 (Asia) on top right */
    if (state->current_region == 1 && !blink_state)
    {
        watch_display_text(WATCH_POSITION_TOP_RIGHT, "  ");
    }
    else
    {
        watch_display_text(WATCH_POSITION_TOP_RIGHT, " 1");
    }

    /* Display bottom row: [4][3][5] using HOURS, MINUTES, SECONDS positions */

    /* Region 4 (S.America) in HOURS position */
    if (state->current_region == 4 && !blink_state)
    {
        watch_display_text(WATCH_POSITION_HOURS, "  ");
    }
    else
    {
        watch_display_text(WATCH_POSITION_HOURS, " 4");
    }

    /* Region 3 (Africa) in MINUTES position */
    if (state->current_region == 3 && !blink_state)
    {
        watch_display_text(WATCH_POSITION_MINUTES, "  ");
    }
    else
    {
        watch_display_text(WATCH_POSITION_MINUTES, " 3");
    }

    /* Region 5 (Australia) in SECONDS position */
    if (state->current_region == 5 && !blink_state)
    {
        watch_display_text(WATCH_POSITION_SECONDS, "  ");
    }
    else
    {
        watch_display_text(WATCH_POSITION_SECONDS, " 5");
    }

    /* Show region name briefly when switching */
    if (state->show_zone_name > 0)
    {
        watch_display_text_with_fallback(WATCH_POSITION_BOTTOM, region_names[state->current_region], region_names[state->current_region]);
        state->show_zone_name--;
    }
}

/* Display city selection screen */
static void _city_selection_display(movement_event_t event, world_clock2_state_t *state)
{
    char buf[11], zone_abbr[MAX_ZONE_NAME_LEN + 1];
    const char *location_name;
    uoffset_t zone_offset;

    /* Update indicators */
    if (refresh_face)
    {
        watch_clear_colon();
        watch_clear_indicator(WATCH_INDICATOR_24H);
        watch_clear_indicator(WATCH_INDICATOR_PM);
        refresh_face = false;
    }

    /* Mark selected location */
    if (state->locations[state->current_location].selected)
        watch_set_indicator(WATCH_INDICATOR_SIGNAL);
    else
        watch_clear_indicator(WATCH_INDICATOR_SIGNAL);

    /* Display location abbreviation on top */
    _get_location_zone_info(state, zone_abbr, &zone_offset);
    /* Blink abbreviation */
    if (event.subsecond % 2)
    {
        sprintf(zone_abbr, "%5s", "     ");
    }
    _display_location_abbr(state, zone_abbr);

    /* Display location name or offset on bottom */
    if (state->show_zone_name > 0)
    {
        location_name = movement_location_presets[state->current_location].name;
        watch_display_text_with_fallback(WATCH_POSITION_BOTTOM, location_name, location_name);
    }
    else
    {
        sprintf(buf, " %3d%02d", zone_offset.hours, zone_offset.minutes);
        watch_display_text_with_fallback(WATCH_POSITION_BOTTOM, buf, buf);
    }
}

static bool _clock_loop(movement_event_t event, world_clock2_state_t *state)
{
    switch (event.event_type)
    {
    case EVENT_ACTIVATE:
    case EVENT_TICK:
    case EVENT_LOW_ENERGY_UPDATE:
        _clock_display(event, state);
        break;
    case EVENT_ALARM_BUTTON_UP:
        refresh_face = true;
        state->current_location = _next_selected_location(state, FORWARD);
        state->show_zone_name = NAME_DISPLAY_TIME;
        _clock_display(event, state);
        break;
    case EVENT_LIGHT_BUTTON_DOWN:
        /* Do nothing. No light. */
        break;
    case EVENT_LIGHT_BUTTON_UP:
        refresh_face = true;
        state->current_location = _next_selected_location(state, BACKWARD);
        state->show_zone_name = NAME_DISPLAY_TIME;
        _clock_display(event, state);
        break;
    case EVENT_LIGHT_LONG_PRESS:
        /* Switch to region selection mode */
        state->current_mode = WORLD_CLOCK2_MODE_SETTINGS_REGION;
        /* Set current region based on the currently displayed location */
        state->current_region = movement_location_presets[state->current_location].region;
        state->show_zone_name = NAME_DISPLAY_TIME;
        refresh_face = true;
        movement_request_tick_frequency(4);
        _region_selection_display(event, state);
        _beep(BEEP_BUTTON);
        break;
    case EVENT_ALARM_REALLY_LONG_PRESS:
    {
        /* Get current watch timezone and ensure the location is in the list */
        uint8_t old_zone = movement_get_timezone_index();

        /* Find if any location uses the old timezone, if not add current location */
        bool old_zone_found = false;
        for (uint8_t i = 0; i < _get_num_locations(); i++)
        {
            if (movement_location_presets[i].timezone == old_zone && state->locations[i].selected)
            {
                old_zone_found = true;
                break;
            }
        }

        /* If old timezone not in selected locations, select current location's old zone */
        if (!old_zone_found)
        {
            for (uint8_t i = 0; i < _get_num_locations(); i++)
            {
                if (movement_location_presets[i].timezone == old_zone)
                {
                    state->locations[i].selected = true;
                    break;
                }
            }
        }

        /* Set the watch timezone to the currently displayed location's timezone */
        uint8_t new_zone = movement_location_presets[state->current_location].timezone;
        movement_set_timezone_index(new_zone);

        /* Persist the associated coordinates for sunrise/sunset and other faces */
        movement_location_t stored_location = {.reg = 0};
        stored_location.bit.latitude = movement_location_presets[state->current_location].latitude;
        stored_location.bit.longitude = movement_location_presets[state->current_location].longitude;
        watch_store_backup_data(stored_location.reg, 1);

        /* Play confirmation beep */
        _beep(BEEP_ENABLE);

        /* Refresh display */
        refresh_face = true;
        _clock_display(event, state);
    }
    break;
    default:
        return movement_default_loop_handler(event);
    }

    return true;
}

static bool _region_selection_loop(movement_event_t event, world_clock2_state_t *state)
{
    /* Region navigation order: 0 -> 4 -> 2 -> 3 -> 1 -> 5 -> 0 */
    static const uint8_t region_order[] = {0, 4, 2, 3, 1, 5};
    static const uint8_t num_regions = 6;

    switch (event.event_type)
    {
    case EVENT_ACTIVATE:
    case EVENT_TICK:
    case EVENT_LOW_ENERGY_UPDATE:
        _region_selection_display(event, state);
        break;
    case EVENT_ALARM_BUTTON_UP:
        /* Find current region in order and move to next */
        for (uint8_t i = 0; i < num_regions; i++)
        {
            if (region_order[i] == state->current_region)
            {
                state->current_region = region_order[(i + 1) % num_regions];
                break;
            }
        }
        state->show_zone_name = NAME_DISPLAY_TIME;
        _region_selection_display(event, state);
        break;
    case EVENT_LIGHT_BUTTON_UP:
        /* Find current region in order and move to previous */
        for (uint8_t i = 0; i < num_regions; i++)
        {
            if (region_order[i] == state->current_region)
            {
                state->current_region = region_order[(i + num_regions - 1) % num_regions];
                break;
            }
        }
        state->show_zone_name = NAME_DISPLAY_TIME;
        _region_selection_display(event, state);
        break;
    case EVENT_LIGHT_BUTTON_DOWN:
        /* Do nothing. No light. */
        break;
    case EVENT_ALARM_LONG_PRESS:
        /* Select region and enter city selection mode */
        /* Check if the current_location is in the selected region */
        if (movement_location_presets[state->current_location].region == state->current_region)
        {
            /* We're entering the region that contains the current location, stay on it */
            /* (current_location is already correct) */
        }
        else if (state->last_location_per_region[state->current_region] < _get_num_locations() &&
                 movement_location_presets[state->last_location_per_region[state->current_region]].region == state->current_region)
        {
            /* Use the last location we were at in this region */
            state->current_location = state->last_location_per_region[state->current_region];
        }
        else
        {
            /* Find first location in this region */
            for (uint8_t i = 0; i < _get_num_locations(); i++)
            {
                if (movement_location_presets[i].region == state->current_region)
                {
                    state->current_location = i;
                    break;
                }
            }
        }
        state->current_mode = WORLD_CLOCK2_MODE_SETTINGS_CITY;
        state->show_zone_name = NAME_DISPLAY_TIME;
        refresh_face = true;
        _city_selection_display(event, state);
        _beep(BEEP_BUTTON);
        break;
    case EVENT_LIGHT_LONG_PRESS:
        /* Return to clock mode */
        /* Find next selected location */
        if (!state->locations[state->current_location].selected)
            state->current_location = _next_selected_location(state, FORWARD);

        state->current_mode = WORLD_CLOCK2_MODE_CLOCK;
        state->show_zone_name = NAME_DISPLAY_TIME;
        refresh_face = true;
        movement_request_tick_frequency(1);
        _clock_display(event, state);
        _beep(BEEP_BUTTON);
        break;
    default:
        return movement_default_loop_handler(event);
    }

    return true;
}

static bool _city_selection_loop(movement_event_t event, world_clock2_state_t *state)
{
    uint8_t location;

    switch (event.event_type)
    {
    case EVENT_ACTIVATE:
    case EVENT_TICK:
    case EVENT_LOW_ENERGY_UPDATE:
        _city_selection_display(event, state);
        break;
    case EVENT_ALARM_BUTTON_UP:
        state->current_location = _next_location_in_region(state, FORWARD);
        /* Save this as the last location for this region */
        state->last_location_per_region[state->current_region] = state->current_location;
        state->show_zone_name = NAME_DISPLAY_TIME;
        _city_selection_display(event, state);
        break;
    case EVENT_LIGHT_BUTTON_UP:
        state->current_location = _next_location_in_region(state, BACKWARD);
        /* Save this as the last location for this region */
        state->last_location_per_region[state->current_region] = state->current_location;
        state->show_zone_name = NAME_DISPLAY_TIME;
        _city_selection_display(event, state);
        break;
    case EVENT_LIGHT_BUTTON_DOWN:
        /* Do nothing. No light. */
        break;
    case EVENT_ALARM_LONG_PRESS:
        /* Toggle selection of current location */
        location = state->current_location;
        state->locations[location].selected = !state->locations[location].selected;
        _city_selection_display(event, state);
        if (state->locations[location].selected)
        {
            _beep(BEEP_ENABLE);
        }
        else
        {
            _beep(BEEP_DISABLE);
        }
        break;
    case EVENT_LIGHT_LONG_PRESS:
        _beep(BEEP_BUTTON);
        break;
    case EVENT_LIGHT_LONG_UP:
        /* Return to region selection */
        /* Save current location as the last for this region before leaving */
        state->last_location_per_region[state->current_region] = state->current_location;
        state->current_mode = WORLD_CLOCK2_MODE_SETTINGS_REGION;
        state->show_zone_name = NAME_DISPLAY_TIME;
        refresh_face = true;
        _region_selection_display(event, state);
        break;
    case EVENT_LIGHT_REALLY_LONG_PRESS:
        /* Return to clock mode */
        /* Save current location as the last for this region before leaving */
        state->last_location_per_region[state->current_region] = state->current_location;
        /* Find next selected location */
        if (!state->locations[state->current_location].selected)
            state->current_location = _next_selected_location(state, FORWARD);

        state->current_mode = WORLD_CLOCK2_MODE_CLOCK;
        state->show_zone_name = NAME_DISPLAY_TIME;
        refresh_face = true;
        movement_request_tick_frequency(1);
        _clock_display(event, state);
        _beep(BEEP_BUTTON);
        break;
    default:
        return movement_default_loop_handler(event);
    }

    return true;
}

void world_clock2_face_setup(uint8_t watch_face_index, void **context_ptr)
{
    (void)watch_face_index;
    /* We derive preselected indices dynamically from names to avoid index drift. */

    if (*context_ptr == NULL)
    {
        *context_ptr = malloc(sizeof(world_clock2_state_t));
        memset(*context_ptr, 0, sizeof(world_clock2_state_t));

        /* Start in clock mode */
        world_clock2_state_t *state = (world_clock2_state_t *)*context_ptr;
        state->current_mode = WORLD_CLOCK2_MODE_CLOCK;
        state->current_location = 0; /* temporary until we find default city */
        state->current_region = 0;
        state->show_zone_name = NAME_DISPLAY_TIME;

        uint8_t timezone_index = movement_get_timezone_index();
        movement_location_t stored_location = {.reg = watch_get_backup_data(1)};
        bool location_found = false;

        if (stored_location.reg != 0)
        {
            uint16_t num_locations = _get_num_locations();
            for (uint16_t i = 0; i < num_locations; i++)
            {
                if (movement_location_presets[i].latitude == stored_location.bit.latitude &&
                    movement_location_presets[i].longitude == stored_location.bit.longitude)
                {
                    state->current_location = i;
                    location_found = true;
                    break;
                }
            }
        }

        if (!location_found)
        {
            int16_t preselected_index = movement_location_index_by_name(PRESELECTED_LOCATION_NAMES[0]);
            if (preselected_index >= 0)
            {
                state->current_location = (uint16_t)preselected_index;
                location_found = true;
            }
        }

        if (!location_found)
        {
            const location_long_lat_presets_t *default_location = movement_location_get_default_for_zone(timezone_index);
            if (default_location)
            {
                state->current_location = (uint16_t)(default_location - movement_location_presets);
                location_found = true;
            }
        }

        if (!location_found)
        {
            /* Final fallback to first entry */
            state->current_location = 0;
        }

        state->current_region = movement_location_presets[state->current_location].region;

        /* Initialize last_location_per_region to invalid values */
        /* Initialize per-region last location markers */
        for (uint8_t r = 0; r < NUM_REGIONS; r++)
        {
            state->last_location_per_region[r] = 0xFF; // Invalid
        }
        state->last_location_per_region[state->current_region] = state->current_location;
        /* Initialize location selection flags */
        for (uint16_t i = 0; i < _get_num_locations(); i++)
        {
            state->locations[i].selected = false;
        }

        /* Select predefined cities by name */
        for (uint8_t n = 0; n < NUM_PRESELECTED_LOCATIONS; n++)
        {
            const char *wanted = PRESELECTED_LOCATION_NAMES[n];
            int16_t idx = movement_location_index_by_name(wanted);
            if (idx >= 0)
            {
                state->locations[(uint16_t)idx].selected = true;
            }
        }
    }
}

void world_clock2_face_activate(void *context)
{
    world_clock2_state_t *state = (world_clock2_state_t *)context;

    switch (state->current_mode)
    {
    case WORLD_CLOCK2_MODE_CLOCK:
        /* Normal tick frequency */
        movement_request_tick_frequency(1);
        break;
    case WORLD_CLOCK2_MODE_SETTINGS_REGION:
    case WORLD_CLOCK2_MODE_SETTINGS_CITY:
        /* Faster frequency for blinking effect */
        movement_request_tick_frequency(4);
        break;
    }

    /* Set initial state */
    refresh_face = true;
}

bool world_clock2_face_loop(movement_event_t event, void *context)
{
    world_clock2_state_t *state = (world_clock2_state_t *)context;
    switch (state->current_mode)
    {
    case WORLD_CLOCK2_MODE_CLOCK:
        return _clock_loop(event, state);
    case WORLD_CLOCK2_MODE_SETTINGS_REGION:
        return _region_selection_loop(event, state);
    case WORLD_CLOCK2_MODE_SETTINGS_CITY:
        return _city_selection_loop(event, state);
    }
    return false;
}

void world_clock2_face_resign(void *context)
{
    (void)context;
}
