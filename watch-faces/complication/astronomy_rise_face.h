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

#ifndef ASTRONOMY_RISE_FACE_H_
#define ASTRONOMY_RISE_FACE_H_

/*
 * ASTRONOMY RISE/SET face
 *
 * Shows a summary view plus per-body rise/set information for major solar
 * system bodies.
 *
 * In the summary view, the top row indicates whether Venus, Mars, Jupiter and
 * Saturn are above the horizon, and the time fields show the next solar rise
 * and set pair for the current location.
 * E.g
 *    V_ _S
 * 19:33 05
 * Means Venus and Saturn are above the horizon, Mars and Jupiter are below.
 * and the next sunset is at 19:33
 * The next sunrise at 05:xx.
 *
 * Short Alarm enters body browsing and cycles through available bodies. Long
 * Alarm opens the selected body's detail screens. Once in the detail screens,
 * short Alarm cycles through:
 *
 *     OV - Overview: rise hour, set hour, sun-object angle, and percent of the
 *          body's visible face illuminated.
 *     CO - Constellation: approximate constellation the body is currently in.
 *     rI - Rise time (local): the next local rise time for the selected body.
 *          Lap indicator says that it is tomorrow.
 *     SE - Set time (local): the next local set time for the selected body.
 *          Lap indicator says that it is tomorrow.
 *     SA - Sun angle: the angular separation between the Sun and the selected
 *          body, plus a trend indicator: ^ or u for rising or falling.
 *     FA - Face illuminated: the percent of the selected body's visible face
 *          illuminated from Earth, plus a trend indicator.
 *
 * Long Alarm returns to body selection.
 * Rise/set times use the body's RA/Dec at local noon as the position estimate,
 * which is accurate to a few minutes for planets and the Sun.
 *
 * Notice that accuracy was chosen so that times are accurate to within 10 minutes.
 */

#include "movement.h"
#include "astrolib.h"

#define ASTRONOMY_RISE_NUM_BODIES 7

typedef enum {
    ASTRONOMY_RISE_MODE_SELECTING_BODY = 0,
    ASTRONOMY_RISE_MODE_CALCULATING,
    ASTRONOMY_RISE_MODE_DISPLAYING_OVERVIEW,
    ASTRONOMY_RISE_MODE_DISPLAYING_CONSTELLATION,
    ASTRONOMY_RISE_MODE_DISPLAYING_RISE,
    ASTRONOMY_RISE_MODE_DISPLAYING_SET,
    ASTRONOMY_RISE_MODE_DISPLAYING_SUN_ANGLE,
    ASTRONOMY_RISE_MODE_DISPLAYING_FACE_LIT,
    ASTRONOMY_RISE_MODE_NUM_MODES
} astronomy_rise_mode_t;

typedef struct {
    bool valid;
    bool has_linear_rate;
    uint8_t year;
    uint8_t month;
    uint8_t day;
    uint32_t computed_at_local_ts;
    double jd_noon_utc;
    float right_ascension;
    float declination;
    float vx;
    float vy;
    float vz;
    float vx_rate_per_day;
    float vy_rate_per_day;
    float vz_rate_per_day;
} astronomy_rise_body_cache_t;

typedef struct {
    astronomy_rise_mode_t mode;
    uint8_t active_body_index;
    bool valid;
    bool circumpolar;
    bool never_rises;
    bool rise_is_tomorrow;
    bool set_is_tomorrow;
    bool browsing_objects;
    double altitude;
    uint8_t recalc_mask;
    uint8_t recalc_remaining;
    watch_date_time_t rise_time;
    watch_date_time_t set_time;
    uint8_t constellation_index;
    float sun_angle_deg;
    float face_lit_percent;
    int8_t sun_angle_trend;
    int8_t face_lit_trend;
    astronomy_rise_body_cache_t body_cache[ASTRONOMY_RISE_NUM_BODIES];
} astronomy_rise_state_t;

void astronomy_rise_face_setup(uint8_t watch_face_index, void **context_ptr);
void astronomy_rise_face_activate(void *context);
bool astronomy_rise_face_loop(movement_event_t event, void *context);
void astronomy_rise_face_resign(void *context);

#define astronomy_rise_face ((const watch_face_t){ \
    astronomy_rise_face_setup,                     \
    astronomy_rise_face_activate,                  \
    astronomy_rise_face_loop,                      \
    astronomy_rise_face_resign,                    \
    NULL,                                          \
})

#endif // ASTRONOMY_RISE_FACE_H_
