/*
 * Shared location presets for world clock and astronomical faces.
 *
 * This table centralizes the latitude/longitude presets so that
 * multiple watch faces can rely on the same source of truth when
 * looking up cities or default coordinates for a timezone.
 */

#ifndef MOVEMENT_LOCATION_DATA_H_
#define MOVEMENT_LOCATION_DATA_H_

#include <stdint.h>
#include "zones.h"

#ifdef __cplusplus
extern "C"
{
#endif

  typedef struct
  {
    char name[16];
    int16_t latitude;  /* hundredths of degrees */
    int16_t longitude; /* hundredths of degrees */
    uint8_t region;    /* 0-5: North America, Asia, Europe, Africa, South America, Oceania */
    uint8_t timezone;  /* index into zone_defns */
  } location_long_lat_presets_t;

  extern const location_long_lat_presets_t movement_location_presets[];
  extern const uint16_t movement_location_presets_count;

  const location_long_lat_presets_t *movement_location_find_by_name(const char *name);
  int16_t movement_location_index_by_name(const char *name);
  const location_long_lat_presets_t *movement_location_find_by_coordinates(int16_t latitude, int16_t longitude, uint8_t timezone);
  const location_long_lat_presets_t *movement_location_get_default_for_zone(uint8_t timezone);

#ifdef __cplusplus
}
#endif

#endif /* MOVEMENT_LOCATION_DATA_H_ */
