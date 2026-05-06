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

/*
 * File outline
 *
 * Body metadata and lookup:
 * - _astronomy_rise_body_index:                      Map an astronomy body constant to its table index.
 *
 * Constellations and observer location:
 * - _astronomy_rise_constellation:                  Convert right ascension to a constellation table entry.
 * - _astronomy_rise_get_location:                   Load the current preset latitude and longitude.
 *
 * Time and geometry helpers:
 * - _astronomy_rise_compute_rise_set:              Estimate rise and set times from RA/Dec and observer position.
 * - _astronomy_rise_to_unix:                       Convert a watch date-time to a Unix timestamp.
 * - _astronomy_rise_from_unix:                     Convert a Unix timestamp back to a watch date-time.
 * - _astronomy_rise_local_to_utc:                  Reinterpret a local time as its UTC equivalent.
 * - _astronomy_rise_shift_days_utc:                Move a UTC date-time forward or backward by whole days.
 * - _astronomy_rise_local_noon:                    Build a local date-time pinned to 12:00 noon.
 * - _astronomy_rise_wrap_angle_pm_pi:              Wrap an angle into the [-pi, pi] range.
 * - _astronomy_rise_wrap_angle_0_2pi:              Wrap an angle into the [0, 2pi) range.
 * - _astronomy_rise_radec_to_unit_vector:          Convert right ascension and declination to XYZ.
 * - _astronomy_rise_unit_vector_to_radec:          Convert an XYZ direction back to RA/Dec.
 * - _astronomy_rise_max_motion_deg_per_hour:       Return a conservative drift rate for a body.
 * - _astronomy_rise_prediction_window_days:        Return how far cached linear prediction may be trusted.
 * - _astronomy_rise_cache_ttl_seconds:             Compute cache lifetime for a body's noon coordinates.
 * - _astronomy_rise_julian_from_local:             Convert a local watch time to a Julian date.
 * - _astronomy_rise_hours_ut_to_local:             Turn UTC fractional hours on a date into local wall time.
 * - _astronomy_rise_next_two_sun_events:           Find the next two sunrise or sunset events after now.
 *
 * Cache and recalculation:
 * - _astronomy_rise_get_noon_radec_cached:         Fetch or predict noon RA/Dec and refresh the cache.
 * - _astronomy_rise_cache_fresh_for_body_on_date:  Check whether one body's cache is fresh for a date.
 * - _astronomy_rise_cache_fresh_for_summary:       Check whether summary mode can reuse cached body data.
 * - _astronomy_rise_refresh_recalc_mask:           Mark which summary bodies still need recalculation.
 * - _astronomy_rise_recalculate_body:              Recompute one body without changing the user's selection.
 * - _astronomy_rise_recalculate:                   Run the full input, model, and state update pipeline.
 *
 * Model pipeline:
 * - _astronomy_rise_build_model_input:             Gather current body, time, timezone, and location inputs.
 * - _astronomy_rise_compute_model:                 Compute altitude, rise/set, phase, and constellation outputs.
 * - _astronomy_rise_apply_model_output:            Copy computed model results back into face state.
 *
 * Display helpers and render modes:
 * - _astronomy_rise_display_body_name:             Paint the active body's label across the top row.
 * - _astronomy_rise_display_time:                  Paint a rise/set time with labels and tomorrow indicator.
 * - _astronomy_rise_render_summary:                Draw the summary screen with visibility letters and sun timing.
 * - _astronomy_rise_render_selecting_body:         Draw the body-selection prompt while browsing objects.
 * - _astronomy_rise_render_calculating:            Show recalculation progress and process pending bodies.
 * - _astronomy_rise_render_status:                 Draw the shared circumpolar "always up" status view.
 * - _astronomy_rise_render_not_rising:             Draw the shared "never rises" status view.
 * - _astronomy_rise_render_rise:                   Draw the rise-time view or an edge-case status instead.
 * - _astronomy_rise_render_set:                    Draw the set-time view or an edge-case status instead.
 * - _astronomy_rise_render_constellation:          Draw the active body's current constellation name.
 * - _astronomy_rise_render_overview:               Draw a compact overview of phase and event telemetry.
 * - _astronomy_rise_render_sun_angle:              Draw the Sun-object angle with a trend marker.
 * - _astronomy_rise_render_face_lit:               Draw illuminated-face percent with a trend marker.
 * - _astronomy_rise_render_display_mode:           Dispatch to the active display-mode renderer.
 *
 * Watch face lifecycle:
 * - _astronomy_rise_update:                        Refresh the screen for the current mode and state.
 * - astronomy_rise_face_setup:                     Allocate and zero the persistent face state.
 * - astronomy_rise_face_activate:                  Handle face activation with no extra setup required.
 * - astronomy_rise_face_loop:                      Process button, tick, and timeout events for the face.
 * - astronomy_rise_face_resign:                    Reset mode flags when leaving the face.
 */

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include "astronomy_rise_face.h"
#include "watch_utility.h"
#include "watch_common_display.h"
#include "sunrise_sunset_face.h"
#include "sunriset.h"
#include "astro_trig.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define NUM_AVAILABLE_BODIES ASTRONOMY_RISE_NUM_BODIES
#define _location_count (sizeof(longLatPresets) / sizeof(long_lat_presets_t))
#define _recalc_valid_mask ((uint8_t)((1u << NUM_AVAILABLE_BODIES) - 1u))

typedef struct {
    char body;
    char short_name[5];
    char long_name[6];
    char summary_letter;
    bool include_in_summary_recalc;
    float max_motion_deg_per_hour;
    float prediction_window_days;
    uint32_t fixed_ttl_seconds;
} astronomy_rise_body_info_t;

static const astronomy_rise_body_info_t astronomy_rise_body_info[NUM_AVAILABLE_BODIES] = {
    // body,              short, long,    letter sum?   recalc, max mo, pred window
    { ASTRO_BODY_MERCURY, "MERC", "Mercu", '\0', false, 0.12f, 3.0f,  24u * 3600u },
    { ASTRO_BODY_VENUS,   "VEnu", "Venus", 'V',  true,  0.08f, 3.0f,  24u * 3600u },
    { ASTRO_BODY_MOON,    "MOOn", "Moon ", '\0', false, 0.70f, 1.0f,  0u },
    { ASTRO_BODY_MARS,    "MArS", "Mars ", 'M',  true,  0.04f, 5.0f,  24u * 3600u },
    { ASTRO_BODY_JUPITER, "JuPt", "Jupit", 'J',  true,  0.01f, 30.0f, 0u },
    { ASTRO_BODY_SATURN,  "SAtu", "Satur", 'S',  true,  0.005f,45.0f, 0u },
    { ASTRO_BODY_SUN,     "SUN ", "Sun  ", '\0', true,  0.05f, 7.0f,  0u },
};

static uint8_t _astronomy_rise_body_index(char body) {
    for (uint8_t i = 0; i < NUM_AVAILABLE_BODIES; i++) {
        if (astronomy_rise_body_info[i].body == body) return i;
    }
    return 0;
}

typedef struct { float ra_start; const char name[7]; } constellation_entry_t;
typedef struct {
    uint8_t active_body_index;
    float latitude_deg;
    float longitude_deg;
    int32_t timezone_offset_seconds;
    watch_date_time_t local_now;
} astronomy_rise_model_input_t;

typedef struct {
    bool valid;
    bool circumpolar;
    bool never_rises;
    bool rise_is_tomorrow;
    bool set_is_tomorrow;
    float altitude_deg;
    watch_date_time_t rise_time;
    watch_date_time_t set_time;
    uint8_t constellation_index;
    float sun_angle_deg;
    float face_lit_percent;
    int8_t sun_angle_trend;
    int8_t face_lit_trend;
} astronomy_rise_model_output_t;

static void _astronomy_rise_recalculate(astronomy_rise_state_t *state);

// Ecliptic constellations ordered by RA start (hours). Covers full 0-24h circle.
static const constellation_entry_t _constellations[] = {
    {  0.0f / 12.0 * M_PI, "Pisces" },
    {  1.9f / 12.0 * M_PI, "Aries " },
    {  3.3f / 12.0 * M_PI, "Taurus" },
    {  5.7f / 12.0 * M_PI, "Gemini" },
    {  7.5f / 12.0 * M_PI, "Cancer" },
    {  9.1f / 12.0 * M_PI, "Leo   " },
    { 11.6f / 12.0 * M_PI, "Virgo " },
    { 14.3f / 12.0 * M_PI, "Libra " },
    { 15.8f / 12.0 * M_PI, "Scorpi" },
    { 16.5f / 12.0 * M_PI, "Ophiuc" },
    { 17.9f / 12.0 * M_PI, "Sagitt" },
    { 20.1f / 12.0 * M_PI, "Capric" },
    { 21.7f / 12.0 * M_PI, "Aquari" },
    { 23.3f / 12.0 * M_PI, "Pisces" },
};

#define QUICK_IN_RANGE(r, range) { \
    while (r < 0) r += range; \
    while (r >= (range)) r -= range; \
}

const uint8_t NUM_CONSTELLATIONS = (sizeof(_constellations) / sizeof(_constellations[0])) ;

static uint8_t _astronomy_rise_constellation(float ra_rad) {
    const float two_pi = 2.0 * M_PI;
    const float bin_size = 24.0 / (NUM_CONSTELLATIONS-1) / (12.0 / M_PI) ;

    QUICK_IN_RANGE(ra_rad, two_pi);

    uint8_t idx = (ra_rad / bin_size); // seed guess

    while (
        (idx > 0) &&
        (_constellations[idx].ra_start > ra_rad)
    ) idx--;
    while (
        (idx < NUM_CONSTELLATIONS - 1 ) &&
        (_constellations[idx + 1].ra_start <= ra_rad)
    ) idx++;

    return idx;
}

static bool _astronomy_rise_get_location(float *lat, float *lon) {
    uint8_t preset_index = movement_get_timezone_index() + 1;
    if (preset_index >= _location_count)
        return false;
    movement_location_t loc;
    if( (longLatPresets[preset_index].latitude  == 0 ) &&
        (longLatPresets[preset_index].longitude == 0 )    )
        return false ;
    *lat = (float)(int16_t)longLatPresets[preset_index].latitude  / 100.0f;
    *lon = (float)(int16_t)longLatPresets[preset_index].longitude / 100.0f;
    return true;
}

/*
 * Compute rise/set times for a body given its RA/Dec (in radians) and observer lat/lon (degrees).
 * Uses the standard hour-angle formula: cos(H) = (sin(altit) - sin(lat)*sin(dec)) / (cos(lat)*cos(dec))
 * altit = -35/60 degrees (atmospheric refraction + limb, matching sun_rise_set convention).
 * Returns false if body is circumpolar or never rises (sets *circumpolar accordingly).
 * rise_ut and set_ut are hours in UTC on the given date.
 */
static bool _astronomy_rise_compute_rise_set(float ra_rad, float dec_rad, float lat_deg, float lon_deg,
                               int year, int month, int day,
                               float *rise_ut, float *set_ut,
                               bool *circumpolar, bool *never_rises) {
    const float altit = -35.0 / 60.0;  // degrees, horizon with refraction
    float lat_r = lat_deg * M_PI / 180.0;
    float dec_r = dec_rad;

    float cos_H = (astro_sinf(altit * M_PI / 180.0) - astro_sinf(lat_r) * astro_sinf(dec_r))
                   / (astro_cosf(lat_r) * astro_cosf(dec_r));

    *circumpolar = false;
    *never_rises = false;

    if (cos_H < -1.0) {
        *circumpolar = true;
        return false;
    }
    if (cos_H > 1.0) {
        *never_rises = true;
        return false;
    }

    float H = acos(cos_H) * 180.0 / M_PI;  // hour angle at rise, in degrees

    // GMST at 0h UT on the given date (using sunriset's GMST0)
    // Days since J2000.0 at 0h UT
    float d = 367.0 * year - (int)(7.0 * (year + (int)((month + 9) / 12)) / 4)
               + (int)(275.0 * month / 9) + day - 730530.0;

    float gmst0_hours = GMST0(d) / 15.0;  // GMST0 returns degrees, convert to hours
    float ra_hours = ra_rad * 180.0 / M_PI / 15.0;  // RA in hours

    // Local sidereal time at transit
    float transit_ut = ra_hours - gmst0_hours - lon_deg / 15.0;

    // Normalize to [0, 24)
    QUICK_IN_RANGE(transit_ut, 24.0);

    float H_hours = H / 15.0;

    *rise_ut = transit_ut - H_hours;
    *set_ut  = transit_ut + H_hours;

    QUICK_IN_RANGE(*rise_ut, 24.0);
    QUICK_IN_RANGE(*set_ut, 24.0);

    return true;
}

static uint32_t _astronomy_rise_to_unix(watch_date_time_t dt, int32_t timezone_offset_seconds) {
    return watch_utility_date_time_to_unix_time(dt, timezone_offset_seconds);
}

static watch_date_time_t _astronomy_rise_from_unix(uint32_t unix_time, int32_t timezone_offset_seconds) {
    return watch_utility_date_time_from_unix_time(unix_time, timezone_offset_seconds);
}

static watch_date_time_t _astronomy_rise_local_to_utc(watch_date_time_t local_dt, int32_t timezone_offset_seconds) {
    return _astronomy_rise_from_unix(_astronomy_rise_to_unix(local_dt, timezone_offset_seconds), 0);
}

static watch_date_time_t _astronomy_rise_shift_days_utc(watch_date_time_t utc_dt, int day_offset) {
    if (day_offset == 0) return utc_dt;
    int64_t shifted = (int64_t)_astronomy_rise_to_unix(utc_dt, 0) + (int64_t)day_offset * 86400;
    if (shifted < 0) shifted = 0;
    return _astronomy_rise_from_unix((uint32_t)shifted, 0);
}

static watch_date_time_t _astronomy_rise_local_noon(watch_date_time_t local_dt) {
    watch_date_time_t noon = {0};
    noon.unit.year = local_dt.unit.year;
    noon.unit.month = local_dt.unit.month;
    noon.unit.day = local_dt.unit.day;
    noon.unit.hour = 12;
    return noon;
}

static float _astronomy_rise_wrap_angle_pm_pi(float x) {
    while (x > (float)M_PI) x -= (float)(2.0 * M_PI);
    while (x < (float)(-M_PI)) x += (float)(2.0 * M_PI);
    return x;
}

static float _astronomy_rise_wrap_angle_0_2pi(float x) {
    QUICK_IN_RANGE(x, (float)(2.0 * M_PI));
    return x;
}

static void _astronomy_rise_radec_to_unit_vector(float ra, float dec, float *x, float *y, float *z) {
    float cdec = astro_cosf(dec);
    *x = cdec * astro_cosf(ra);
    *y = cdec * astro_sinf(ra);
    *z = astro_sinf(dec);
}

static void _astronomy_rise_unit_vector_to_radec(float x, float y, float z, float *ra, float *dec) {
    float norm2 = x * x + y * y + z * z;
    if (norm2 <= 0.0f) {
        *ra = 0.0f;
        *dec = 0.0f;
        return;
    }
    float inv_norm = 1.0f / sqrtf(norm2);
    x *= inv_norm;
    y *= inv_norm;
    z *= inv_norm;
    if (z > 1.0f) z = 1.0f;
    if (z < -1.0f) z = -1.0f;
    *dec = asinf(z);
    *ra = _astronomy_rise_wrap_angle_0_2pi(atan2f(y, x));
}

static float _astronomy_rise_max_motion_deg_per_hour(uint8_t body_index) {
    // Conservative max apparent geocentric drift rates against stars.
    // Used for cache validity, not for rendering.
    float value = astronomy_rise_body_info[body_index].max_motion_deg_per_hour;
    return value > 0.0f ? value : 0.05f;
}

static float _astronomy_rise_prediction_window_days(uint8_t body_index) {
    float value = astronomy_rise_body_info[body_index].prediction_window_days;
    return value > 0.0f ? value : 5.0f;
}

static uint32_t _astronomy_rise_cache_ttl_seconds(uint8_t body_index) {
    uint32_t fixed_ttl_seconds = astronomy_rise_body_info[body_index].fixed_ttl_seconds;
    if (fixed_ttl_seconds > 0) {
        return fixed_ttl_seconds;
    }

    const float max_allowed_drift_deg = 2.0f;
    float motion_deg_per_hour = _astronomy_rise_max_motion_deg_per_hour(body_index);
    float ttl_hours = max_allowed_drift_deg / motion_deg_per_hour;
    uint32_t ttl_seconds = (uint32_t)(ttl_hours * 3600.0f);
    if (ttl_seconds < 300) ttl_seconds = 300;               // avoid thrashing
    if (ttl_seconds > 14 * 24 * 3600) ttl_seconds = 14 * 24 * 3600;
    return ttl_seconds;
}

static double _astronomy_rise_julian_from_local(watch_date_time_t local_dt, int32_t timezone_offset_seconds) {
    watch_date_time_t utc = _astronomy_rise_local_to_utc(local_dt, timezone_offset_seconds);
    return astro_convert_date_to_julian_date(utc.unit.year + WATCH_RTC_REFERENCE_YEAR,
                                             utc.unit.month, utc.unit.day,
                                             utc.unit.hour, utc.unit.minute, utc.unit.second);
}

static watch_date_time_t _astronomy_rise_hours_ut_to_local(float hours_ut, int year, int month, int day, int32_t timezone_offset_seconds) {
    float hours_local = hours_ut + (float)timezone_offset_seconds / 3600.0;

    // Handle day rollover — adjust the date if we crossed midnight
    int day_offset = 0;
    while (hours_local < 0)   { hours_local += 24.0; day_offset--; }
    while (hours_local >= 24) { hours_local -= 24.0; day_offset++; }

    // Apply day offset via unix timestamp to handle month/year boundaries
    watch_date_time_t base = {0};
    base.unit.year  = year - WATCH_RTC_REFERENCE_YEAR;
    base.unit.month = month;
    base.unit.day   = day;
    if (day_offset != 0) {
        base = _astronomy_rise_shift_days_utc(base, day_offset);
    }

    watch_date_time_t dt = {0};
    dt.unit.hour  = (uint8_t)hours_local;
    float minutes = 60.0 * fmod(hours_local, 1.0);
    dt.unit.minute = (uint8_t)minutes;
    dt.unit.second = (uint8_t)round(60.0 * fmod(minutes, 1.0));
    if (dt.unit.second >= 60) { dt.unit.second = 0; dt.unit.minute++; }
    if (dt.unit.minute >= 60) { dt.unit.minute = 0; dt.unit.hour++; }
    if (dt.unit.hour >= 24) {
        dt.unit.hour = 0;
        base = _astronomy_rise_shift_days_utc(base, 1);
    }
    dt.unit.year  = base.unit.year;
    dt.unit.month = base.unit.month;
    dt.unit.day   = base.unit.day;
    return dt;
}

typedef struct {
    bool valid;
    watch_date_time_t first;
    watch_date_time_t second;
} astronomy_rise_next_sun_events_t;

static astronomy_rise_next_sun_events_t _astronomy_rise_next_two_sun_events(
    watch_date_time_t local_now, int32_t timezone_offset_seconds, float lat_deg, float lon_deg
) {
    astronomy_rise_next_sun_events_t out = {0};
    watch_date_time_t now_utc = _astronomy_rise_local_to_utc(local_now, timezone_offset_seconds);
    uint32_t now_ts = _astronomy_rise_to_unix(now_utc, 0);
    uint32_t best1_ts = UINT32_MAX;
    uint32_t best2_ts = UINT32_MAX;

    for (int day_offset = 0; day_offset <= 7; day_offset++) {
        uint32_t day_ts_local = _astronomy_rise_to_unix(local_now, timezone_offset_seconds) + (uint32_t)(day_offset * 86400);
        watch_date_time_t day_local = _astronomy_rise_from_unix(day_ts_local, timezone_offset_seconds);
        int year = day_local.unit.year + WATCH_RTC_REFERENCE_YEAR;
        int month = day_local.unit.month;
        int day = day_local.unit.day;

        double rise_ut = 0.0;
        double set_ut = 0.0;
        uint8_t result = sun_rise_set(year, month, day, lon_deg, lat_deg, &rise_ut, &set_ut);
        if (result != 0) continue;

        watch_date_time_t rise_local = _astronomy_rise_hours_ut_to_local((float)rise_ut, year, month, day, timezone_offset_seconds);
        watch_date_time_t set_local = _astronomy_rise_hours_ut_to_local((float)set_ut, year, month, day, timezone_offset_seconds);
        watch_date_time_t rise_utc = _astronomy_rise_local_to_utc(rise_local, timezone_offset_seconds);
        watch_date_time_t set_utc = _astronomy_rise_local_to_utc(set_local, timezone_offset_seconds);
        uint32_t rise_ts = _astronomy_rise_to_unix(rise_utc, 0);
        uint32_t set_ts = _astronomy_rise_to_unix(set_utc, 0);

        if (rise_ts > now_ts) {
            if (rise_ts < best1_ts) {
                best2_ts = best1_ts;
                out.second = out.first;
                best1_ts = rise_ts;
                out.first = rise_local;
                out.valid = true;
            } else if (rise_ts < best2_ts) {
                best2_ts = rise_ts;
                out.second = rise_local;
            }
        }

        if (set_ts > now_ts) {
            if (set_ts < best1_ts) {
                best2_ts = best1_ts;
                out.second = out.first;
                best1_ts = set_ts;
                out.first = set_local;
                out.valid = true;
            } else if (set_ts < best2_ts) {
                best2_ts = set_ts;
                out.second = set_local;
            }
        }
    }

    if (best1_ts == UINT32_MAX || best2_ts == UINT32_MAX) {
        out.valid = false;
    }
    return out;
}

static astro_equatorial_coordinates_t _astronomy_rise_get_noon_radec_cached(
    astronomy_rise_state_t *state,
    const astronomy_rise_model_input_t *input,
    int calc_year, int calc_month, int calc_day,
    float lat_r, float lon_r
) {
    astronomy_rise_body_cache_t *cache = &state->body_cache[input->active_body_index];
    uint32_t now_ts = _astronomy_rise_to_unix(input->local_now, input->timezone_offset_seconds);
    uint32_t ttl_seconds = _astronomy_rise_cache_ttl_seconds(input->active_body_index);
    watch_date_time_t local_calc = {0};
    local_calc.unit.year = calc_year - WATCH_RTC_REFERENCE_YEAR;
    local_calc.unit.month = calc_month;
    local_calc.unit.day = calc_day;
    watch_date_time_t noon_utc = _astronomy_rise_local_to_utc(
        _astronomy_rise_local_noon(local_calc), input->timezone_offset_seconds);
    double jd_noon = astro_convert_date_to_julian_date(noon_utc.unit.year + WATCH_RTC_REFERENCE_YEAR,
                                                       noon_utc.unit.month, noon_utc.unit.day,
                                                       noon_utc.unit.hour, noon_utc.unit.minute, noon_utc.unit.second);

    bool same_date = cache->valid &&
                     cache->year == (uint8_t)(calc_year - WATCH_RTC_REFERENCE_YEAR) &&
                     cache->month == (uint8_t)calc_month &&
                     cache->day == (uint8_t)calc_day;
    bool fresh = same_date && ((now_ts - cache->computed_at_local_ts) <= ttl_seconds);
    if (fresh) {
        astro_equatorial_coordinates_t cached = {0};
        cached.right_ascension = cache->right_ascension;
        cached.declination = cache->declination;
        return cached;
    }

    if (cache->valid && cache->has_linear_rate) {
        float max_dt_days = _astronomy_rise_prediction_window_days(input->active_body_index);
        double dt = jd_noon - cache->jd_noon_utc;
        if (fabs(dt) <= max_dt_days) {
            astro_equatorial_coordinates_t predicted = {0};
            float dtf = (float)dt;
            float x = cache->vx + cache->vx_rate_per_day * dtf;
            float y = cache->vy + cache->vy_rate_per_day * dtf;
            float z = cache->vz + cache->vz_rate_per_day * dtf;
            _astronomy_rise_unit_vector_to_radec(x, y, z, &predicted.right_ascension, &predicted.declination);
            return predicted;
        }
    }

    bool old_valid = cache->valid;
    double old_jd_noon = cache->jd_noon_utc;
    float old_vx = cache->vx;
    float old_vy = cache->vy;
    float old_vz = cache->vz;
    astro_equatorial_coordinates_t radec_noon = astro_get_ra_dec(
        jd_noon, astronomy_rise_body_info[input->active_body_index].body, lat_r, lon_r, false);
    float new_vx, new_vy, new_vz;
    _astronomy_rise_radec_to_unit_vector(radec_noon.right_ascension, radec_noon.declination, &new_vx, &new_vy, &new_vz);

    if (old_valid) {
        double dt = jd_noon - old_jd_noon;
        if (fabs(dt) > 0.01 && fabs(dt) < 120.0) {
            float dtf = (float)dt;
            cache->vx_rate_per_day = (new_vx - old_vx) / dtf;
            cache->vy_rate_per_day = (new_vy - old_vy) / dtf;
            cache->vz_rate_per_day = (new_vz - old_vz) / dtf;
            cache->has_linear_rate = true;
        }
    } else {
        cache->has_linear_rate = false;
    }

    cache->valid = true;
    cache->year = (uint8_t)(calc_year - WATCH_RTC_REFERENCE_YEAR);
    cache->month = (uint8_t)calc_month;
    cache->day = (uint8_t)calc_day;
    cache->computed_at_local_ts = now_ts;
    cache->jd_noon_utc = jd_noon;
    cache->right_ascension = radec_noon.right_ascension;
    cache->declination = radec_noon.declination;
    cache->vx = new_vx;
    cache->vy = new_vy;
    cache->vz = new_vz;
    return radec_noon;
}

static bool _astronomy_rise_cache_fresh_for_body_on_date(
    const astronomy_rise_state_t *state,
    uint8_t body_index,
    const watch_date_time_t *local_now,
    int32_t timezone_offset_seconds,
    uint8_t year, uint8_t month, uint8_t day
) {
    const astronomy_rise_body_cache_t *cache = &state->body_cache[body_index];
    if (!cache->valid) return false;
    if (cache->year != year || cache->month != month || cache->day != day) return false;
    uint32_t now_ts = _astronomy_rise_to_unix(*local_now, timezone_offset_seconds);
    uint32_t ttl_seconds = _astronomy_rise_cache_ttl_seconds(body_index);
    return (now_ts - cache->computed_at_local_ts) <= ttl_seconds;
}

static bool _astronomy_rise_cache_fresh_for_summary(
    const astronomy_rise_state_t *state,
    uint8_t body_index,
    const watch_date_time_t *local_now,
    int32_t timezone_offset_seconds
) {
    if (_astronomy_rise_cache_fresh_for_body_on_date(
            state, body_index, local_now, timezone_offset_seconds,
            local_now->unit.year, local_now->unit.month, local_now->unit.day)) {
        return true;
    }

    uint32_t tomorrow_ts = _astronomy_rise_to_unix(*local_now, timezone_offset_seconds) + 86400;
    watch_date_time_t tomorrow = _astronomy_rise_from_unix(tomorrow_ts, timezone_offset_seconds);
    return _astronomy_rise_cache_fresh_for_body_on_date(
        state, body_index, local_now, timezone_offset_seconds,
        tomorrow.unit.year, tomorrow.unit.month, tomorrow.unit.day);
}

static void _astronomy_rise_refresh_recalc_mask(astronomy_rise_state_t *state) {
    watch_date_time_t now = movement_get_local_date_time();
    int32_t tz = movement_get_current_timezone_offset();
    uint8_t mask = 0;
    uint8_t count = 0;
    for (uint8_t body_index = 0; body_index < NUM_AVAILABLE_BODIES; body_index++) {
        if (!astronomy_rise_body_info[body_index].include_in_summary_recalc) continue;
        bool fresh = _astronomy_rise_cache_fresh_for_summary(state, body_index, &now, tz);
        if (!fresh) {
            mask |= (1 << body_index);
            count++;
        }
    }
    state->recalc_mask = (uint8_t)(mask & _recalc_valid_mask);
    state->recalc_remaining = count;
}

static bool _astronomy_rise_recalculate_body(astronomy_rise_state_t *state, uint8_t body_index) {
    uint8_t old_body = state->active_body_index;
    state->active_body_index = body_index;
    _astronomy_rise_recalculate(state);
    state->active_body_index = old_body;
    return state->valid;
}

static bool _astronomy_rise_build_model_input(const astronomy_rise_state_t *state, astronomy_rise_model_input_t *input) {
    if (!_astronomy_rise_get_location(&input->latitude_deg, &input->longitude_deg)) {
        return false;
    }
    input->active_body_index = state->active_body_index;
    input->timezone_offset_seconds = movement_get_current_timezone_offset();
    input->local_now = movement_get_local_date_time();
    return true;
}

static void _astronomy_rise_compute_model(astronomy_rise_state_t *state, const astronomy_rise_model_input_t *input, astronomy_rise_model_output_t *output) {
    output->valid = true;

    watch_date_time_t utc = _astronomy_rise_local_to_utc(input->local_now, input->timezone_offset_seconds);

    double jd = astro_convert_date_to_julian_date(utc.unit.year + WATCH_RTC_REFERENCE_YEAR,
                                                  utc.unit.month, utc.unit.day,
                                                  utc.unit.hour, utc.unit.minute, utc.unit.second);

    float lat_r = astro_degrees_to_radians(input->latitude_deg);
    float lon_r = astro_degrees_to_radians(input->longitude_deg);

    astro_equatorial_coordinates_t radec_p = astro_get_ra_dec(jd, astronomy_rise_body_info[input->active_body_index].body, lat_r, lon_r, true);
    astro_horizontal_coordinates_t horiz = astro_ra_dec_to_alt_az(jd, lat_r, lon_r, radec_p.right_ascension, radec_p.declination);
    output->altitude_deg = astro_radians_to_degrees(horiz.altitude);

    astro_equatorial_coordinates_t obj_now = astro_get_ra_dec(jd, astronomy_rise_body_info[input->active_body_index].body, lat_r, lon_r, false);
    astro_equatorial_coordinates_t sun_now = astro_get_ra_dec(jd, ASTRO_BODY_SUN, lat_r, lon_r, false);
    float cos_elong = astro_sinf(obj_now.declination) * astro_sinf(sun_now.declination) +
                      astro_cosf(obj_now.declination) * astro_cosf(sun_now.declination) * astro_cosf(obj_now.right_ascension - sun_now.right_ascension);
    if (cos_elong > 1.0f) cos_elong = 1.0f;
    if (cos_elong < -1.0f) cos_elong = -1.0f;
    float elong = acosf(cos_elong);
    output->sun_angle_deg = astro_radians_to_degrees(elong);

    float es = sun_now.distance;
    float eo = obj_now.distance;
    float so2 = es * es + eo * eo - 2.0f * es * eo * cos_elong;
    float so = (so2 > 0.0f) ? sqrtf(so2) : 0.0f;
    float face_lit = 100.0f;
    if (eo > 0.0f && so > 0.0f) {
        float cos_phase = (so * so + eo * eo - es * es) / (2.0f * so * eo);
        if (cos_phase > 1.0f) cos_phase = 1.0f;
        if (cos_phase < -1.0f) cos_phase = -1.0f;
        face_lit = 50.0f * (1.0f + cos_phase);
    }
    if (astronomy_rise_body_info[input->active_body_index].body == ASTRO_BODY_SUN) face_lit = 100.0f;
    if (face_lit < 0.0f) face_lit = 0.0f;
    if (face_lit > 100.0f) face_lit = 100.0f;
    output->face_lit_percent = face_lit;
    output->sun_angle_trend = 0;
    output->face_lit_trend = 0;

    // Current trend: compare now vs now+1h.
    double jd_next = jd + (1.0 / 24.0);
    astro_equatorial_coordinates_t obj_next = astro_get_ra_dec(jd_next, astronomy_rise_body_info[input->active_body_index].body, lat_r, lon_r, false);
    astro_equatorial_coordinates_t sun_next = astro_get_ra_dec(jd_next, ASTRO_BODY_SUN, lat_r, lon_r, false);
    float cos_elong_next = astro_sinf(obj_next.declination) * astro_sinf(sun_next.declination) +
                           astro_cosf(obj_next.declination) * astro_cosf(sun_next.declination) * astro_cosf(obj_next.right_ascension - sun_next.right_ascension);
    if (cos_elong_next > 1.0f) cos_elong_next = 1.0f;
    if (cos_elong_next < -1.0f) cos_elong_next = -1.0f;
    float elong_next_deg = astro_radians_to_degrees(acosf(cos_elong_next));

    float face_lit_next = 100.0f;
    float es_next = sun_next.distance;
    float eo_next = obj_next.distance;
    float so2_next = es_next * es_next + eo_next * eo_next - 2.0f * es_next * eo_next * cos_elong_next;
    float so_next = (so2_next > 0.0f) ? sqrtf(so2_next) : 0.0f;
    if (eo_next > 0.0f && so_next > 0.0f) {
        float cos_phase_next = (so_next * so_next + eo_next * eo_next - es_next * es_next) / (2.0f * so_next * eo_next);
        if (cos_phase_next > 1.0f) cos_phase_next = 1.0f;
        if (cos_phase_next < -1.0f) cos_phase_next = -1.0f;
        face_lit_next = 50.0f * (1.0f + cos_phase_next);
    }
    if (astronomy_rise_body_info[input->active_body_index].body == ASTRO_BODY_SUN) face_lit_next = 100.0f;
    if (face_lit_next < 0.0f) face_lit_next = 0.0f;
    if (face_lit_next > 100.0f) face_lit_next = 100.0f;

    float d_sun_angle = elong_next_deg - output->sun_angle_deg;
    float d_face_lit = face_lit_next - output->face_lit_percent;
    const float trend_eps = 0.01f;
    if (d_sun_angle > trend_eps) output->sun_angle_trend = 1;
    else if (d_sun_angle < -trend_eps) output->sun_angle_trend = -1;
    if (d_face_lit > trend_eps) output->face_lit_trend = 1;
    else if (d_face_lit < -trend_eps) output->face_lit_trend = -1;

    float rise_ut, set_ut;
    bool circ, never;
    int calc_year = input->local_now.unit.year + WATCH_RTC_REFERENCE_YEAR;
    int calc_month = input->local_now.unit.month;
    int calc_day = input->local_now.unit.day;
    astro_equatorial_coordinates_t radec_noon = _astronomy_rise_get_noon_radec_cached(
        state, input, calc_year, calc_month, calc_day, lat_r, lon_r);

    bool ok = _astronomy_rise_compute_rise_set(radec_noon.right_ascension, radec_noon.declination,
                                                input->latitude_deg, input->longitude_deg, calc_year, calc_month, calc_day,
                                                &rise_ut, &set_ut, &circ, &never);

    if (ok) {
        watch_date_time_t set_local = _astronomy_rise_hours_ut_to_local(
            set_ut, calc_year, calc_month, calc_day, input->timezone_offset_seconds);
        uint32_t now_ts = _astronomy_rise_to_unix(input->local_now, input->timezone_offset_seconds);
        uint32_t set_ts = _astronomy_rise_to_unix(set_local, input->timezone_offset_seconds);
        if (set_ts <= now_ts) {
            uint32_t tomorrow_ts = now_ts + 86400;
            watch_date_time_t tomorrow = _astronomy_rise_from_unix(tomorrow_ts, input->timezone_offset_seconds);
            calc_year = tomorrow.unit.year + WATCH_RTC_REFERENCE_YEAR;
            calc_month = tomorrow.unit.month;
            calc_day = tomorrow.unit.day;
            astro_equatorial_coordinates_t radec_tom = _astronomy_rise_get_noon_radec_cached(
                state, input, calc_year, calc_month, calc_day, lat_r, lon_r);
            ok = _astronomy_rise_compute_rise_set(radec_tom.right_ascension, radec_tom.declination,
                                                  input->latitude_deg, input->longitude_deg, calc_year, calc_month, calc_day,
                                                  &rise_ut, &set_ut, &circ, &never);
        }
    }

    // Moon-only refinement: refine rise and set independently so one event
    // does not bias the other.
    if (ok && astronomy_rise_body_info[input->active_body_index].body == ASTRO_BODY_MOON) {
        float rise_ut_refined = rise_ut;
        float set_ut_refined = set_ut;

        // Refine rise from Moon position at estimated rise time.
        watch_date_time_t rise_guess_local = _astronomy_rise_hours_ut_to_local(
            rise_ut_refined, calc_year, calc_month, calc_day, input->timezone_offset_seconds);
        double jd_rise_guess = _astronomy_rise_julian_from_local(rise_guess_local, input->timezone_offset_seconds);
        astro_equatorial_coordinates_t radec_rise = astro_get_ra_dec(
            jd_rise_guess, ASTRO_BODY_MOON, lat_r, lon_r, false);
        float rise_candidate, set_dummy;
        bool rise_circ, rise_never;
        if (_astronomy_rise_compute_rise_set(radec_rise.right_ascension, radec_rise.declination,
                                             input->latitude_deg, input->longitude_deg, calc_year, calc_month, calc_day,
                                             &rise_candidate, &set_dummy, &rise_circ, &rise_never)) {
            rise_ut_refined = rise_candidate;
        }

        // Refine set from Moon position at estimated set time.
        watch_date_time_t set_guess_local = _astronomy_rise_hours_ut_to_local(
            set_ut_refined, calc_year, calc_month, calc_day, input->timezone_offset_seconds);
        double jd_set_guess = _astronomy_rise_julian_from_local(set_guess_local, input->timezone_offset_seconds);
        astro_equatorial_coordinates_t radec_set = astro_get_ra_dec(
            jd_set_guess, ASTRO_BODY_MOON, lat_r, lon_r, false);
        float rise_dummy, set_candidate;
        bool set_circ, set_never;
        if (_astronomy_rise_compute_rise_set(radec_set.right_ascension, radec_set.declination,
                                             input->latitude_deg, input->longitude_deg, calc_year, calc_month, calc_day,
                                             &rise_dummy, &set_candidate, &set_circ, &set_never)) {
            set_ut_refined = set_candidate;
        }

        rise_ut = rise_ut_refined;
        set_ut = set_ut_refined;
    }

    output->circumpolar = circ;
    output->never_rises = never;
    if (ok) {
        output->rise_time = _astronomy_rise_hours_ut_to_local(
            rise_ut, calc_year, calc_month, calc_day, input->timezone_offset_seconds);
        output->set_time = _astronomy_rise_hours_ut_to_local(
            set_ut, calc_year, calc_month, calc_day, input->timezone_offset_seconds);
        output->rise_is_tomorrow = (output->rise_time.unit.day != (uint8_t)input->local_now.unit.day);
        output->set_is_tomorrow = (output->set_time.unit.day != (uint8_t)input->local_now.unit.day);
    }
    output->constellation_index = _astronomy_rise_constellation(radec_noon.right_ascension);
}

static void _astronomy_rise_apply_model_output(astronomy_rise_state_t *state, const astronomy_rise_model_output_t *output) {
    state->valid = output->valid;
    state->circumpolar = output->circumpolar;
    state->never_rises = output->never_rises;
    state->rise_is_tomorrow = output->rise_is_tomorrow;
    state->set_is_tomorrow = output->set_is_tomorrow;
    state->altitude = output->altitude_deg;
    state->rise_time = output->rise_time;
    state->set_time = output->set_time;
    state->constellation_index = output->constellation_index;
    state->sun_angle_deg = output->sun_angle_deg;
    state->face_lit_percent = output->face_lit_percent;
    state->sun_angle_trend = output->sun_angle_trend;
    state->face_lit_trend = output->face_lit_trend;
}

static void _astronomy_rise_recalculate(astronomy_rise_state_t *state) {
    astronomy_rise_model_input_t input;
    if (!_astronomy_rise_build_model_input(state, &input)) {
        state->valid = false;
        return;
    }
    astronomy_rise_model_output_t output = {0};
    _astronomy_rise_compute_model(state, &input, &output);
    _astronomy_rise_apply_model_output(state, &output);
}

static void _astronomy_rise_display_body_name(const char *name_long, const char *name_short) {
    watch_display_text_with_fallback(WATCH_POSITION_TOP_LEFT, name_long, name_short);
    watch_display_text(WATCH_POSITION_TOP_RIGHT, name_long + 3);
}

static void _astronomy_rise_display_time(watch_date_time_t t, const char *name_long, const char *name_short, const char *seconds_label, bool is_tomorrow) {
    watch_set_colon();
    _astronomy_rise_display_body_name(name_long, name_short);
    watch_display_text(WATCH_POSITION_SECONDS, seconds_label);
    if (is_tomorrow)
        watch_set_indicator(WATCH_INDICATOR_LAP);
    else
        watch_clear_indicator(WATCH_INDICATOR_LAP);
    char hrbuf[3], minbuf[3];
    if (movement_clock_mode_24h()) {
        watch_set_indicator(WATCH_INDICATOR_24H);
        snprintf(hrbuf, sizeof hrbuf, "%02d", t.unit.hour);
        snprintf(minbuf, sizeof minbuf, "%02d", t.unit.minute);
    } else {
        watch_date_time_t t12 = t;
        if (watch_utility_convert_to_12_hour(&t12))
            watch_set_indicator(WATCH_INDICATOR_PM);
        else
            watch_clear_indicator(WATCH_INDICATOR_PM);
        snprintf(hrbuf, sizeof hrbuf, "%2d", t12.unit.hour);
        snprintf(minbuf, sizeof minbuf, "%02d", t12.unit.minute);
    }
    watch_display_text(WATCH_POSITION_HOURS, hrbuf);
    watch_display_text(WATCH_POSITION_MINUTES, minbuf);
}

/*
 * Render helpers:
 * - Each helper paints one user-visible astronomy mode or status on the LCD.
 * - Shared formatting (body labels, HH:MM layout, indicators) is centralized so
 *   mode switches remain lightweight and visually consistent.
 * - Render outline:
 *   _astronomy_rise_render_summary: summary watch view with visible-body letters and sun timing.
 *   _astronomy_rise_render_selecting_body: body-selection prompt/state.
 *   _astronomy_rise_render_calculating: progress UI while refreshing cached body data.
 *   _astronomy_rise_render_status: shared "always up" circumpolar status.
 *   _astronomy_rise_render_not_rising: shared "never rises" status.
 *   _astronomy_rise_render_rise: rise-time mode (or shared status fallback).
 *   _astronomy_rise_render_set: set-time mode (or shared status fallback).
 *   _astronomy_rise_render_constellation: current constellation for active body.
 *   _astronomy_rise_render_overview: compact telemetry (phase, rise/set hour pair, sun angle).
 *   _astronomy_rise_render_sun_angle: sun-angle metric with trend indicator.
 *   _astronomy_rise_render_face_lit: illuminated-face percent with trend indicator.
 *   _astronomy_rise_render_display_mode: dispatcher for active render mode + altitude indicator.
 */
/*
 * Render watch-mode summary.
 * Shows four tracked-body visibility markers split across top-left/top-right:
 * each letter means body is above the horizon, '_' means below. If Sun data is
 * available, HH:MM:SS is repurposed as first-event hour:first-event minute:
 * second-event hour (rise/set order). Bottom row shows pending recalc count.
 */
static void _astronomy_rise_render_summary(astronomy_rise_state_t *state) {
    float lat, lon;
    if (!_astronomy_rise_get_location(&lat, &lon)) {
        watch_display_text_with_fallback(WATCH_POSITION_TOP, "Astro", "AS");
        watch_display_text_with_fallback(WATCH_POSITION_BOTTOM, "No LOC", "No Loc");
        return;
    }
    watch_date_time_t now = movement_get_local_date_time();
    int32_t tz = movement_get_current_timezone_offset();
    double jd = _astronomy_rise_julian_from_local(now, tz);
    float lat_r = astro_degrees_to_radians(lat);
    float lon_r = astro_degrees_to_radians(lon);

    uint8_t sun_index = _astronomy_rise_body_index(ASTRO_BODY_SUN);
    char summary_letters[4] = {' ', ' ', ' ', ' '};
    uint8_t summary_letter_count = 0;
    char left[3] = {' ', ' ', '\0'};
    char right[3] = {' ', ' ', '\0'};
    for (uint8_t idx = 0; idx < NUM_AVAILABLE_BODIES; idx++) {
        char letter = astronomy_rise_body_info[idx].summary_letter;
        if (letter == '\0') continue;
        if (summary_letter_count >= 4) break;

        const astronomy_rise_body_cache_t *cache = &state->body_cache[idx];
        char ch = ' ';
        if (!cache->valid) {
            ch = ' ';
        } else {
            astro_horizontal_coordinates_t h = astro_ra_dec_to_alt_az(jd, lat_r, lon_r, cache->right_ascension, cache->declination);
            ch = (h.altitude > 0) ? letter : '_';
        }
        summary_letters[summary_letter_count++] = ch;
    }

    for (uint8_t i = 0; i < 4; i++) {
        if (i < 2) left[i] = summary_letters[i];
        else right[i - 2] = summary_letters[i];
    }
    watch_set_colon();
    watch_display_text(WATCH_POSITION_TOP_LEFT, left);
    watch_display_text(WATCH_POSITION_TOP_RIGHT, right);

    if (_astronomy_rise_recalculate_body(state, sun_index)) {
        watch_date_time_t rise = state->rise_time;
        watch_date_time_t set = state->set_time;
        uint32_t rise_ts = _astronomy_rise_to_unix(rise, tz);
        uint32_t set_ts = _astronomy_rise_to_unix(set, tz);
        watch_date_time_t first = rise;
        watch_date_time_t second = set;
        if (set_ts < rise_ts) {
            first = set;
            second = rise;
        }

        watch_clear_indicator(WATCH_INDICATOR_PM);
        watch_clear_indicator(WATCH_INDICATOR_24H);
        int first_hour = first.unit.hour;
        int second_hour = second.unit.hour;
        if (movement_clock_mode_24h()) {
            watch_set_indicator(WATCH_INDICATOR_24H);
        } else {
            watch_date_time_t t12_first = first;
            if (watch_utility_convert_to_12_hour(&t12_first)) watch_set_indicator(WATCH_INDICATOR_PM);
            first_hour = t12_first.unit.hour;
            watch_date_time_t t12_second = second;
            watch_utility_convert_to_12_hour(&t12_second);
            second_hour = t12_second.unit.hour;
        }
        char hours[3];
        char minutes[3];
        char seconds[3];
        snprintf(hours, sizeof hours, "%2d", first_hour);
        snprintf(minutes, sizeof minutes, "%02d", first.unit.minute);
        snprintf(seconds, sizeof seconds, "%02d", second_hour);
        watch_display_text(WATCH_POSITION_HOURS, hours);
        watch_display_text(WATCH_POSITION_MINUTES, minutes);
        watch_display_text(WATCH_POSITION_SECONDS, seconds);
    } else {
        // Fallback so summary screen is never blank in time fields.
        watch_display_text(WATCH_POSITION_SECONDS, astronomy_rise_body_info[state->active_body_index].short_name);
    }

    _astronomy_rise_refresh_recalc_mask(state);
    if (state->recalc_remaining > 0) {
        char bottom[7];
        snprintf(bottom, sizeof bottom, "Calc %d", state->recalc_remaining);
        watch_display_text(WATCH_POSITION_BOTTOM, bottom);
    }
}

/*
 * Render body-selection idle screen.
 * Clears transient indicators from previous modes, keeps body name visible on
 * top row, and prints "Astro" on bottom as the selection/browse prompt.
 */
static void _astronomy_rise_render_selecting_body(const char *name_long, const char *name_short) {
    watch_clear_indicator(WATCH_INDICATOR_SIGNAL);
    watch_clear_indicator(WATCH_INDICATOR_LAP);
    watch_clear_indicator(WATCH_INDICATOR_PM);
    watch_clear_indicator(WATCH_INDICATOR_24H);
    watch_clear_colon();
    _astronomy_rise_display_body_name(name_long, name_short);
    watch_display_text(WATCH_POSITION_BOTTOM, " Astro");
}

/*
 * Render and run batch recalculation.
 * Displays "Calc N" progress on the bottom row while iterating through the
 * recalc bitmask, recomputing one body at a time, and clearing completed bits.
 * Returns with mode forced back to SELECTING_BODY.
 */
static bool _astronomy_rise_render_calculating(astronomy_rise_state_t *state) {
    state->recalc_mask &= _recalc_valid_mask;
    state->recalc_remaining = 0;
    for (uint8_t i = 0; i < NUM_AVAILABLE_BODIES; i++) {
        if (state->recalc_mask & (1 << i)) state->recalc_remaining++;
    }

    if (state->recalc_remaining == 0 || state->recalc_mask == 0) {
        state->mode = ASTRONOMY_RISE_MODE_SELECTING_BODY;
        return false;
    }

    watch_clear_display();
    while (state->recalc_remaining > 0 && state->recalc_mask != 0) {
        char bottom[7];
        snprintf(bottom, sizeof bottom, "Calc %d", state->recalc_remaining);
        watch_display_text(WATCH_POSITION_BOTTOM, bottom);

        uint8_t body_index = 0;
        while (body_index < NUM_AVAILABLE_BODIES && ((state->recalc_mask & (1 << body_index)) == 0)) body_index++;
        if (body_index >= NUM_AVAILABLE_BODIES) {
            state->recalc_mask &= _recalc_valid_mask;
            break;
        }

        _astronomy_rise_recalculate_body(state, body_index);
        state->recalc_mask &= (uint8_t)~(1 << body_index);
        if (state->recalc_remaining > 0) {
            state->recalc_remaining--;
        }
    }

    state->mode = ASTRONOMY_RISE_MODE_SELECTING_BODY;
    return false;
}

/*
 * Render circumpolar status.
 * Shows active body name on top and an "always up" message on bottom when the
 * target body never sets for the current latitude/date model.
 */
static void _astronomy_rise_render_status(const char *name_long, const char *name_short) {
    watch_clear_colon();
    _astronomy_rise_display_body_name(name_long, name_short);
    watch_display_text_with_fallback(WATCH_POSITION_BOTTOM, "AlwAysup", "ALuP  ");
}

/*
 * Render never-rises status.
 * Shows active body name on top and a "never" message on bottom when the body
 * does not rise above the horizon for the current latitude/date model.
 */
static void _astronomy_rise_render_not_rising(const char *name_long, const char *name_short) {
    watch_clear_colon();
    _astronomy_rise_display_body_name(name_long, name_short);
    watch_display_text_with_fallback(WATCH_POSITION_BOTTOM, "NEvEr ", "nEvr  ");
}

/*
 * Render rise-time mode.
 * Uses shared status views for circumpolar/never-rises edge cases; otherwise
 * displays rise HH:MM with "rI" seconds label and LAP indicator when event is
 * on the following local day.
 */
static void _astronomy_rise_render_rise(const astronomy_rise_state_t *state, const char *name_long, const char *name_short) {
    if (state->circumpolar) {
        _astronomy_rise_render_status(name_long, name_short);
    } else if (state->never_rises) {
        _astronomy_rise_render_not_rising(name_long, name_short);
    } else {
        _astronomy_rise_display_time(state->rise_time, name_long, name_short, "rI", state->rise_is_tomorrow);
    }
}

/*
 * Render set-time mode.
 * Uses shared status views for circumpolar/never-rises edge cases; otherwise
 * displays set HH:MM with "SE" seconds label and LAP indicator when event is
 * on the following local day.
 */
static void _astronomy_rise_render_set(const astronomy_rise_state_t *state, const char *name_long, const char *name_short) {
    if (state->circumpolar) {
        _astronomy_rise_render_status(name_long, name_short);
    } else if (state->never_rises) {
        _astronomy_rise_render_not_rising(name_long, name_short);
    } else {
        _astronomy_rise_display_time(state->set_time, name_long, name_short, "SE", state->set_is_tomorrow);
    }
}

/*
 * Render constellation mode.
 * Shows active body name on top and the 6-character ecliptic constellation
 * label derived from cached constellation index on the bottom row.
 */
static void _astronomy_rise_render_constellation(const astronomy_rise_state_t *state, const char *name_long, const char *name_short) {
    watch_clear_colon();
    _astronomy_rise_display_body_name(name_long, name_short);
    char constellation[7];
    memcpy(constellation, _constellations[state->constellation_index].name, sizeof(constellation));
    watch_display_text(WATCH_POSITION_BOTTOM, constellation);
}

/*
 * Render compact overview telemetry.
 * Top-left shows body name; top-right shows face-lit percent (0-99, 2 chars).
 * HH and MM show rise-hour and set-hour. SS shows rounded sun-angle when
 * in-range, otherwise "--" to signal out-of-display-range values.
 */
static void _astronomy_rise_render_overview(const astronomy_rise_state_t *state, const char *name_long, const char *name_short) {
    watch_clear_colon();
    watch_display_text_with_fallback(WATCH_POSITION_TOP_LEFT, name_long, name_short);

    // top-right: face percent (2 chars)
    int pct = (int)roundf(state->face_lit_percent);
    if (pct < 0) pct = 0;
    if (pct > 99) pct = 99;
    char top_right[3];
    snprintf(top_right, sizeof top_right, "%2d", pct);
    watch_display_text(WATCH_POSITION_TOP_RIGHT, top_right);

    // bottom HH:MM shows rise hour:set hour
    char hours[3], minutes[3];
    snprintf(hours, sizeof hours, "%02d", state->rise_time.unit.hour);
    snprintf(minutes, sizeof minutes, "%02d", state->set_time.unit.hour);
    watch_display_text(WATCH_POSITION_HOURS, hours);
    watch_display_text(WATCH_POSITION_MINUTES, minutes);

    // seconds: sun angle integer if <= 99, otherwise '--'
    int angle = (int)roundf(state->sun_angle_deg);
    char seconds[3];
    if (angle >= 0 && angle <= 99) {
        snprintf(seconds, sizeof seconds, "%2d", angle);
    } else {
        snprintf(seconds, sizeof seconds, "--");
    }
    watch_display_text(WATCH_POSITION_SECONDS, seconds);
}

/*
 * Render sun-angle detail mode.
 * Seconds label is "SA". HH:MM fields encode a trend marker plus a clamped
 * 0-180 degree integer using fixed-width formatting for stable LCD alignment.
 */
static void _astronomy_rise_render_sun_angle(const astronomy_rise_state_t *state, const char *name_long, const char *name_short) {
    watch_clear_colon();
    _astronomy_rise_display_body_name(name_long, name_short);
    watch_display_text(WATCH_POSITION_SECONDS, "SA");
    char value[5];
    int whole = (int)roundf(state->sun_angle_deg);
    if (whole < 0) whole = 0;
    if (whole > 180) whole = 180;
    char trend = ' ';
    if (state->sun_angle_trend > 0) trend = '^';
    else if (state->sun_angle_trend < 0) trend = 'u';
    snprintf(value, sizeof value, "%c%3d", trend, whole);
    char hours[3] = { value[0], value[1], '\0' };
    char minutes[3] = { value[2], value[3], '\0' };
    watch_display_text(WATCH_POSITION_HOURS, hours);
    watch_display_text(WATCH_POSITION_MINUTES, minutes);
}

/*
 * Render illuminated-face detail mode.
 * Seconds label is "FA". HH:MM fields encode trend marker plus clamped
 * 0-100 percent value so waxing/waning direction and magnitude fit compactly.
 */
static void _astronomy_rise_render_face_lit(const astronomy_rise_state_t *state, const char *name_long, const char *name_short) {
    watch_clear_colon();
    _astronomy_rise_display_body_name(name_long, name_short);
    watch_display_text(WATCH_POSITION_SECONDS, "FA");
    char value[5];
    int pct = (int)roundf(state->face_lit_percent);
    if (pct < 0) pct = 0;
    if (pct > 100) pct = 100;
    char trend = ' ';
    if (state->face_lit_trend > 0) trend = '^';
    else if (state->face_lit_trend < 0) trend = 'u';
    snprintf(value, sizeof value, "%c%3d", trend, pct);
    char hours[3] = { value[0], value[1], '\0' };
    char minutes[3] = { value[2], value[3], '\0' };
    watch_display_text(WATCH_POSITION_HOURS, hours);
    watch_display_text(WATCH_POSITION_MINUTES, minutes);
}

/*
 * Render dispatcher for active display mode.
 * Maintains SIGNAL indicator as an above-horizon flag from current altitude,
 * then routes to the selected render helper based on the state mode enum.
 */
static void _astronomy_rise_render_display_mode(const astronomy_rise_state_t *state, const char *name_long, const char *name_short) {
    if (state->altitude > 0)
        watch_set_indicator(WATCH_INDICATOR_SIGNAL);
    else
        watch_clear_indicator(WATCH_INDICATOR_SIGNAL);

    switch (state->mode) {
        case ASTRONOMY_RISE_MODE_DISPLAYING_OVERVIEW:
            _astronomy_rise_render_overview(state, name_long, name_short);
            break;
        case ASTRONOMY_RISE_MODE_DISPLAYING_CONSTELLATION:
            _astronomy_rise_render_constellation(state, name_long, name_short);
            break;
        case ASTRONOMY_RISE_MODE_DISPLAYING_RISE:
            _astronomy_rise_render_rise(state, name_long, name_short);
            break;
        case ASTRONOMY_RISE_MODE_DISPLAYING_SET:
            _astronomy_rise_render_set(state, name_long, name_short);
            break;
        case ASTRONOMY_RISE_MODE_DISPLAYING_SUN_ANGLE:
            _astronomy_rise_render_sun_angle(state, name_long, name_short);
            break;
        case ASTRONOMY_RISE_MODE_DISPLAYING_FACE_LIT:
            _astronomy_rise_render_face_lit(state, name_long, name_short);
            break;
        default:
            break;
    }

}

static void _astronomy_rise_update(movement_event_t event, astronomy_rise_state_t *state) {
    (void)event;
    const char *name_long = astronomy_rise_body_info[state->active_body_index].long_name;
    const char *name_short = astronomy_rise_body_info[state->active_body_index].short_name;
    switch (state->mode) {
        case ASTRONOMY_RISE_MODE_SELECTING_BODY:
            if (!state->browsing_objects) {
                _astronomy_rise_refresh_recalc_mask(state);
                if (state->recalc_remaining > 0) {
                    state->mode = ASTRONOMY_RISE_MODE_CALCULATING;
                    _astronomy_rise_render_calculating(state);
                    if (state->mode == ASTRONOMY_RISE_MODE_SELECTING_BODY) {
                        if (state->browsing_objects) {
                            _astronomy_rise_render_selecting_body(name_long, name_short);
                        } else {
                            _astronomy_rise_render_summary(state);
                        }
                    } else {
                        _astronomy_rise_render_display_mode(state, name_long, name_short);
                    }
                    break;
                }
            }
            if (state->browsing_objects) {
                _astronomy_rise_render_selecting_body(name_long, name_short);
            } else {
                _astronomy_rise_render_summary(state);
            }
            break;
        case ASTRONOMY_RISE_MODE_CALCULATING:
            _astronomy_rise_render_calculating(state);
            if (state->mode == ASTRONOMY_RISE_MODE_SELECTING_BODY) {
                if (state->browsing_objects) {
                    _astronomy_rise_render_selecting_body(name_long, name_short);
                } else {
                    _astronomy_rise_render_summary(state);
                }
            } else {
                _astronomy_rise_render_display_mode(state, name_long, name_short);
            }
            break;
        case ASTRONOMY_RISE_MODE_DISPLAYING_RISE:
        case ASTRONOMY_RISE_MODE_DISPLAYING_SET:
        case ASTRONOMY_RISE_MODE_DISPLAYING_CONSTELLATION:
        case ASTRONOMY_RISE_MODE_DISPLAYING_OVERVIEW:
        case ASTRONOMY_RISE_MODE_DISPLAYING_SUN_ANGLE:
        case ASTRONOMY_RISE_MODE_DISPLAYING_FACE_LIT:
            _astronomy_rise_render_display_mode(state, name_long, name_short);
            break;
        case ASTRONOMY_RISE_MODE_NUM_MODES:
            break;
    }
}

void astronomy_rise_face_setup(uint8_t watch_face_index, void **context_ptr) {
    (void)watch_face_index;
    if (*context_ptr == NULL) {
        *context_ptr = malloc(sizeof(astronomy_rise_state_t));
        memset(*context_ptr, 0, sizeof(astronomy_rise_state_t));
    }
}

void astronomy_rise_face_activate(void *context) {
    (void)context;
}

bool astronomy_rise_face_loop(movement_event_t event, void *context) {
    astronomy_rise_state_t *state = (astronomy_rise_state_t *)context;

    switch (event.event_type) {
        case EVENT_ACTIVATE:
            _astronomy_rise_update(event, state);
            break;
        case EVENT_TICK:
            break;
        case EVENT_ALARM_BUTTON_UP:
            switch (state->mode) {
                case ASTRONOMY_RISE_MODE_SELECTING_BODY:
                    if (!state->browsing_objects) {
                        state->browsing_objects = true;
                        state->active_body_index = 0;  // first object: Sun
                    } else {
                        state->active_body_index = (state->active_body_index + 1) % NUM_AVAILABLE_BODIES;
                    }
                    break;
                case ASTRONOMY_RISE_MODE_CALCULATING:
                    break;
                case ASTRONOMY_RISE_MODE_DISPLAYING_FACE_LIT:
                    state->mode = ASTRONOMY_RISE_MODE_DISPLAYING_OVERVIEW;
                    break;
                default:
                    state->mode++;
                    break;
            }
            _astronomy_rise_update(event, state);
            break;
        case EVENT_ALARM_LONG_PRESS:
            if (state->mode == ASTRONOMY_RISE_MODE_SELECTING_BODY) {
                if (state->browsing_objects) {
                    state->mode = ASTRONOMY_RISE_MODE_DISPLAYING_OVERVIEW;
                    _astronomy_rise_recalculate_body(state, state->active_body_index);
                    _astronomy_rise_update(event, state);
                } else {
                    state->browsing_objects = true;
                    state->active_body_index = 0;
                    _astronomy_rise_update(event, state);
                }
            } else if (state->mode == ASTRONOMY_RISE_MODE_CALCULATING) {
                // Let long-press escape auto-calc and return to object browsing.
                state->mode = ASTRONOMY_RISE_MODE_SELECTING_BODY;
                state->browsing_objects = true;
                _astronomy_rise_update(event, state);
            } else if (state->mode != ASTRONOMY_RISE_MODE_CALCULATING) {
                state->mode = ASTRONOMY_RISE_MODE_SELECTING_BODY;
                state->browsing_objects = true;
                _astronomy_rise_update(event, state);
            }
            break;
        case EVENT_ALARM_LONG_UP:
            // Ignore release event to avoid immediately undoing LONG_PRESS transitions.
            _astronomy_rise_update(event, state);
            break;
        case EVENT_ALARM_REALLY_LONG_PRESS:
            // Ignore very long hold event; keep current display mode stable.
            _astronomy_rise_update(event, state);
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

void astronomy_rise_face_resign(void *context) {
    astronomy_rise_state_t *state = (astronomy_rise_state_t *)context;
    state->mode = ASTRONOMY_RISE_MODE_SELECTING_BODY;
    state->browsing_objects = false;
}
