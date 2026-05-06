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

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "astronomy_face.h"
#include "watch_utility.h"
#include "watch_common_display.h"
#include "sunrise_sunset_face.h"

#define NUM_AVAILABLE_BODIES 7
#define _location_count (sizeof(longLatPresets) / sizeof(long_lat_presets_t))

static const char astronomy_face_bodies[NUM_AVAILABLE_BODIES] = {
    ASTRO_BODY_SUN,
    ASTRO_BODY_MERCURY,
    ASTRO_BODY_VENUS,
    ASTRO_BODY_MOON,
    ASTRO_BODY_MARS,
    ASTRO_BODY_JUPITER,
    ASTRO_BODY_SATURN,
};

static const char astronomy_face_body_names[NUM_AVAILABLE_BODIES][3] = {
    "SO",   // Sol
    "ME",   // Mercury
    "VE",   // Venus
    "LU",   // Moon (Luna)
    "MA",   // Mars
    "JU",   // Jupiter
    "SA",   // Saturn
};

/* Returns false if no valid location is configured for the current timezone. */
static bool _astronomy_face_get_location(float *lat, float *lon) {
    uint8_t preset_index = movement_get_timezone_index() + 1;
    if (preset_index >= _location_count)
        return false;

    movement_location_t loc;
    loc.bit.latitude  = longLatPresets[preset_index].latitude;
    loc.bit.longitude = longLatPresets[preset_index].longitude;
    if (loc.reg == 0)
        return false;

    *lat = (float)(int16_t)loc.bit.latitude  / 100.0;
    *lon = (float)(int16_t)loc.bit.longitude / 100.0;
    return true;
}

static void _astronomy_face_recalculate(astronomy_state_t *state) {
    float lat, lon;
    if (!_astronomy_face_get_location(&lat, &lon)) {
        state->valid = false;
        return;
    }
    state->valid = true;

    watch_date_time_t date_time = watch_rtc_get_date_time();
    uint32_t timestamp = watch_utility_date_time_to_unix_time(date_time, movement_get_current_timezone_offset());
    date_time = watch_utility_date_time_from_unix_time(timestamp, 0);
    float jd = astro_convert_date_to_julian_date(date_time.unit.year + WATCH_RTC_REFERENCE_YEAR,
                                                   date_time.unit.month, date_time.unit.day,
                                                   date_time.unit.hour, date_time.unit.minute, date_time.unit.second);

    float lat_r = astro_degrees_to_radians(lat);
    float lon_r = astro_degrees_to_radians(lon);

    astro_equatorial_coordinates_t radec_precession = astro_get_ra_dec(jd, astronomy_face_bodies[state->active_body_index], lat_r, lon_r, true);
    astro_horizontal_coordinates_t horiz = astro_ra_dec_to_alt_az(jd, lat_r, lon_r, radec_precession.right_ascension, radec_precession.declination);
    astro_equatorial_coordinates_t radec = astro_get_ra_dec(jd, astronomy_face_bodies[state->active_body_index], lat_r, lon_r, false);

    state->altitude = astro_radians_to_degrees(horiz.altitude);
    state->azimuth = astro_radians_to_degrees(horiz.azimuth);
    state->right_ascension = astro_radians_to_hms(radec.right_ascension);
    state->declination = astro_radians_to_dms(radec.declination);
    state->distance = radec.distance;
}

static void _astronomy_face_update(movement_event_t event, astronomy_state_t *state) {
    char buf[14];
    switch (state->mode) {
        case ASTRONOMY_MODE_SELECTING_BODY:
            watch_clear_colon();
            watch_display_text(WATCH_POSITION_BOTTOM, " Astro");
            if (event.subsecond % 2) {
                watch_display_text(WATCH_POSITION_TOP_LEFT, (char *)astronomy_face_body_names[state->active_body_index]);
            } else {
                watch_display_text(WATCH_POSITION_TOP_LEFT, "  ");
            }
            if (event.subsecond == 0) {
                watch_display_text(WATCH_POSITION_TOP_RIGHT, "  ");
                switch (state->animation_state) {
                    case 0:
                        watch_set_pixel(0, 7);
                        watch_set_pixel(2, 6);
                        break;
                    case 1:
                        watch_set_pixel(1, 7);
                        watch_set_pixel(2, 9);
                        break;
                    case 2:
                        watch_set_pixel(2, 7);
                        watch_set_pixel(0, 9);
                        break;
                }
                state->animation_state = (state->animation_state + 1) % 3;
            }
            break;
        case ASTRONOMY_MODE_CALCULATING:
            watch_clear_display();
            watch_start_character_blink('C', 100);
            _astronomy_face_recalculate(state);
            watch_stop_blink();
            if (!state->valid) {
                watch_display_text_with_fallback(WATCH_POSITION_TOP, "Astro", "AS");
                watch_display_text_with_fallback(WATCH_POSITION_BOTTOM, "No LOC", "No Loc");
                state->mode = ASTRONOMY_MODE_SELECTING_BODY;
                movement_request_tick_frequency(4);
                break;
            }
            state->mode = ASTRONOMY_MODE_DISPLAYING_ALT;
            // fall through
        case ASTRONOMY_MODE_DISPLAYING_ALT:
            watch_clear_colon();
            watch_display_text(WATCH_POSITION_TOP_LEFT, (char *)astronomy_face_body_names[state->active_body_index]);
            watch_display_text(WATCH_POSITION_TOP_RIGHT, "aL");
            sprintf(buf, "%6d", (int16_t)round(state->altitude * 100));
            watch_display_text(WATCH_POSITION_BOTTOM, buf);
            break;
        case ASTRONOMY_MODE_DISPLAYING_AZI:
            watch_display_text(WATCH_POSITION_TOP_LEFT, (char *)astronomy_face_body_names[state->active_body_index]);
            watch_display_text(WATCH_POSITION_TOP_RIGHT, "aZ");
            sprintf(buf, "%6d", (int16_t)round(state->azimuth * 100));
            watch_display_text(WATCH_POSITION_BOTTOM, buf);
            break;
        case ASTRONOMY_MODE_DISPLAYING_RA:
            watch_set_colon();
            watch_display_text(WATCH_POSITION_TOP_LEFT, "ra");
            watch_display_text(WATCH_POSITION_TOP_RIGHT, " H");
            sprintf(buf, "%02d%02d%02d", state->right_ascension.hours, state->right_ascension.minutes, state->right_ascension.seconds);
            watch_display_text(WATCH_POSITION_BOTTOM, buf);
            break;
        case ASTRONOMY_MODE_DISPLAYING_DEC:
            watch_clear_colon();
            watch_display_text(WATCH_POSITION_TOP_LEFT, "de");
            watch_display_text(WATCH_POSITION_TOP_RIGHT, "  ");
            sprintf(buf, "%3d%2d%2d", state->declination.degrees, state->declination.minutes, state->declination.seconds);
            watch_display_text(WATCH_POSITION_BOTTOM, buf);
            break;
        case ASTRONOMY_MODE_DISPLAYING_DIST:
            watch_clear_colon();
            watch_display_text(WATCH_POSITION_TOP_LEFT, "di");
            if (state->distance >= 0.00668456) {
                watch_display_text(WATCH_POSITION_TOP_RIGHT, "AU");
                sprintf(buf, "%6d", (uint16_t)round(state->distance * 100));
            } else {
                watch_display_text(WATCH_POSITION_TOP_RIGHT, " K");
                sprintf(buf, "%6ld", (uint32_t)round(state->distance * 149597871.0));
            }
            watch_display_text(WATCH_POSITION_BOTTOM, buf);
            break;
        case ASTRONOMY_MODE_NUM_MODES:
            break;
    }
}

void astronomy_face_setup(uint8_t watch_face_index, void **context_ptr) {
    (void) watch_face_index;
    if (*context_ptr == NULL) {
        *context_ptr = malloc(sizeof(astronomy_state_t));
        memset(*context_ptr, 0, sizeof(astronomy_state_t));
    }
}

void astronomy_face_activate(void *context) {
    (void)context;
    movement_request_tick_frequency(4);
}

bool astronomy_face_loop(movement_event_t event, void *context) {
    astronomy_state_t *state = (astronomy_state_t *)context;

    switch (event.event_type) {
        case EVENT_ACTIVATE:
        case EVENT_TICK:
            _astronomy_face_update(event, state);
            break;
        case EVENT_ALARM_BUTTON_UP:
            switch (state->mode) {
                case ASTRONOMY_MODE_SELECTING_BODY:
                    state->active_body_index = (state->active_body_index + 1) % NUM_AVAILABLE_BODIES;
                    break;
                case ASTRONOMY_MODE_CALCULATING:
                    break;
                case ASTRONOMY_MODE_DISPLAYING_DIST:
                    state->mode = ASTRONOMY_MODE_DISPLAYING_ALT;
                    break;
                default:
                    state->mode++;
                    break;
            }
            _astronomy_face_update(event, state);
            break;
        case EVENT_ALARM_LONG_PRESS:
            if (state->mode == ASTRONOMY_MODE_SELECTING_BODY) {
                state->mode = ASTRONOMY_MODE_CALCULATING;
                movement_request_tick_frequency(1);
                _astronomy_face_update(event, state);
            } else if (state->mode != ASTRONOMY_MODE_CALCULATING) {
                state->mode = ASTRONOMY_MODE_SELECTING_BODY;
                movement_request_tick_frequency(4);
                _astronomy_face_update(event, state);
            }
            break;
        case EVENT_TIMEOUT:
            movement_move_to_page(0);
            break;
        case EVENT_LOW_ENERGY_UPDATE:
            break;
        default:
            movement_default_loop_handler(event);
            break;
    }

    return true;
}

void astronomy_face_resign(void *context) {
    astronomy_state_t *state = (astronomy_state_t *)context;
    state->mode = ASTRONOMY_MODE_SELECTING_BODY;
}
