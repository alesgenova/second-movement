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

#ifndef SUNRISE_SUNSET_FACE_H_
#define SUNRISE_SUNSET_FACE_H_

/*
 * SUNRISE & SUNSET FACE
 *
 * Uses the current timezone and location for sunrise/sunset calculations.
 *
 * Refer to the wiki for usage instructions:
 *  https://www.sensorwatch.net/docs/watchfaces/complication/#sunrisesunset
 */

#include "movement.h"

typedef struct
{
    uint8_t rise_index;
    watch_date_time_t rise_set_expires;
} sunrise_sunset_state_t;

void sunrise_sunset_face_setup(uint8_t watch_face_index, void **context_ptr);
void sunrise_sunset_face_activate(void *context);
bool sunrise_sunset_face_loop(movement_event_t event, void *context);
void sunrise_sunset_face_resign(void *context);

#define sunrise_sunset_face ((const watch_face_t){ \
    sunrise_sunset_face_setup,                     \
    sunrise_sunset_face_activate,                  \
    sunrise_sunset_face_loop,                      \
    sunrise_sunset_face_resign,                    \
    NULL,                                          \
})

typedef struct
{
    char name[3];
    int16_t latitude;
    int16_t longitude;
} long_lat_presets_t;

static const long_lat_presets_t longLatPresets[] =
    {
        {.name = "  "},                                         // Default, the long and lat get replaced by what's set in the watch
        {.name = "PP", .latitude = -1428, .longitude = -17068}, // Pago Pago
        {.name = "HN", .latitude = 2132, .longitude = -15786},  // Honolulu
        {.name = "AN", .latitude = 6122, .longitude = -14980},  // Anchorage
        {.name = "LA", .latitude = 3405, .longitude = -11824},  // Los Angeles
        {.name = "DN", .latitude = 3975, .longitude = -10499},  // Denver
        {.name = "PH", .latitude = 3345, .longitude = -11201},  // Phoenix
        {.name = "CH", .latitude = 4188, .longitude = -8763},   // Chicago
        {.name = "RG", .latitude = 5045, .longitude = -10467},  // Regina
        {.name = "NY", .latitude = 4072, .longitude = -7401},   // New York
        {.name = "HX", .latitude = 4465, .longitude = -6358},   // Halifax
        {.name = "MN", .latitude = -306, .longitude = -6015},   // Manaus
        {.name = "SC", .latitude = -3345, .longitude = -7064},  // Santiago
        {.name = "SP", .latitude = -2355, .longitude = -4663},  // Sao Paulo
        {.name = "SJ", .latitude = 4756, .longitude = -5274},   // St. Johns
        {.name = "NK", .latitude = 6417, .longitude = -5174},   // Nuuk
        {.name = "UT", .latitude = 0, .longitude = 0},          // UTC
        {.name = "LN", .latitude = 5151, .longitude = -1},      // London
        {.name = "LG", .latitude = 638, .longitude = 339},      // Lagos
        {.name = "BR", .latitude = 5252, .longitude = 1340},    // Berlin
        {.name = "CA", .latitude = 3003, .longitude = 3125},    // Cairo
        {.name = "MP", .latitude = -2588, .longitude = 3262},   // Maputo
        {.name = "JR", .latitude = 3178, .longitude = 3522},    // Jerusalem
        {.name = "HE", .latitude = 6017, .longitude = 2495},    // Helsinki
        {.name = "NR", .latitude = -128, .longitude = 3682},    // Nairobi
        {.name = "RY", .latitude = 2469, .longitude = 4677},    // Riyadh
        {.name = "MC", .latitude = 5575, .longitude = 3761},    // Moscow
        {.name = "TE", .latitude = 3570, .longitude = 5138},    // Tehran
        {.name = "DB", .latitude = 2527, .longitude = 5523},    // Dubai
        {.name = "KO", .latitude = 2252, .longitude = 8852},    // Kolkata
        {.name = "KT", .latitude = 2772, .longitude = 8522},    // Kathmandu
        {.name = "YG", .latitude = 1677, .longitude = 9603},    // Yangon
        {.name = "BK", .latitude = 1376, .longitude = 10052},   // Bangkok
        {.name = "SH", .latitude = 3123, .longitude = 12123},   // Shanghai
        {.name = "HK", .latitude = 2232, .longitude = 11411},   // Hong Kong
        {.name = "SI", .latitude = 135, .longitude = 10385},    // Singapore
        {.name = "PE", .latitude = -3195, .longitude = 11585},  // Perth
        {.name = "TK", .latitude = 3569, .longitude = 13969},   // Tokyo
        {.name = "SE", .latitude = 3757, .longitude = 12698},   // Seoul
        {.name = "DR", .latitude = -1246, .longitude = 13084},  // Darwin
        {.name = "AD", .latitude = -3493, .longitude = 13885},  // Adelaide
        {.name = "BN", .latitude = -2747, .longitude = 15302},  // Brisbane
        {.name = "HB", .latitude = -4288, .longitude = 14732},  // Hobart
        {.name = "SY", .latitude = -3387, .longitude = 15121},  // Sydney
        {.name = "GU", .latitude = 1347, .longitude = 14447},   // Guam
        {.name = "TR", .latitude = 143, .longitude = 17302},    // Tarawa
        {.name = "AK", .latitude = -3685, .longitude = 17487}   // Auckland
};

#endif // SUNRISE_SUNSET_FACE_H_
