#include "movement_location_data.h"

#include <string.h>

/* Location database - sorted west to east within each region */
const location_long_lat_presets_t movement_location_presets[] = {
    // North America (west to east by longitude)
    {.name = "Anchorage", .latitude = 6122, .longitude = -14980, .region = 0, .timezone = UTZ_ANCHORAGE},       // Anchorage, United States
    {.name = "Honolulu", .latitude = 2132, .longitude = -15786, .region = 0, .timezone = UTZ_HONOLULU},         // Honolulu, United States
    {.name = "Fairbanks", .latitude = 6488, .longitude = -14772, .region = 0, .timezone = UTZ_ANCHORAGE},       // Fairbanks, United States
    {.name = "Vancouver", .latitude = 4926, .longitude = -12311, .region = 0, .timezone = UTZ_LOS_ANGELES},     // Vancouver, Canada
    {.name = "Seattle", .latitude = 4761, .longitude = -12234, .region = 0, .timezone = UTZ_LOS_ANGELES},       // Seattle, United States
    {.name = "Los Angeles", .latitude = 3405, .longitude = -11824, .region = 0, .timezone = UTZ_LOS_ANGELES},   // Los Angeles, United States
    {.name = "San Francisco", .latitude = 3778, .longitude = -12242, .region = 0, .timezone = UTZ_LOS_ANGELES}, // San Francisco, United States
    {.name = "Phoenix", .latitude = 3345, .longitude = -11201, .region = 0, .timezone = UTZ_PHOENIX},           // Phoenix, United States
    {.name = "Denver", .latitude = 3977, .longitude = -10492, .region = 0, .timezone = UTZ_DENVER},             // Denver, United States
    {.name = "Edmonton", .latitude = 5355, .longitude = -11349, .region = 0, .timezone = UTZ_DENVER},           // Edmonton, Canada
    {.name = "Regina", .latitude = 5045, .longitude = -10467, .region = 0, .timezone = UTZ_REGINA},             // Regina, Canada
    {.name = "Ciudad Juarez", .latitude = 3174, .longitude = -10649, .region = 0, .timezone = UTZ_DENVER},      // Ciudad Juarez, Mexico
    {.name = "Culiacan", .latitude = 2480, .longitude = -10739, .region = 0, .timezone = UTZ_PHOENIX},          // Culiacan, Mexico
    {.name = "Mexico City", .latitude = 1932, .longitude = -9915, .region = 0, .timezone = UTZ_CHICAGO},        // Mexico City, Mexico
    {.name = "Winnipeg", .latitude = 4990, .longitude = -9714, .region = 0, .timezone = UTZ_CHICAGO},           // Winnipeg, Canada
    {.name = "Houston", .latitude = 2976, .longitude = -9537, .region = 0, .timezone = UTZ_CHICAGO},            // Houston, United States
    {.name = "Chicago", .latitude = 4188, .longitude = -8762, .region = 0, .timezone = UTZ_CHICAGO},            // Chicago, United States
    {.name = "GuatemalaCity", .latitude = 1464, .longitude = -9051, .region = 0, .timezone = UTZ_CHICAGO},      // Guatemala City, Guatemala
    {.name = "Detroit", .latitude = 4233, .longitude = -8304, .region = 0, .timezone = UTZ_NEW_YORK},           // Detroit, United States
    {.name = "Havana", .latitude = 2314, .longitude = -8236, .region = 0, .timezone = UTZ_NEW_YORK},            // Havana, Cuba
    {.name = "Jacksonville", .latitude = 3033, .longitude = -8166, .region = 0, .timezone = UTZ_NEW_YORK},      // Jacksonville, United States
    {.name = "Toronto", .latitude = 4365, .longitude = -7938, .region = 0, .timezone = UTZ_NEW_YORK},           // Toronto, Canada
    {.name = "Halifax", .latitude = 4465, .longitude = -6358, .region = 0, .timezone = UTZ_HALIFAX},            // Halifax, Canada
    {.name = "St. Johns", .latitude = 4756, .longitude = -5274, .region = 0, .timezone = UTZ_ST_JOHNS},         // St. Johns, Canada
    {.name = "Nuuk", .latitude = 6417, .longitude = -5174, .region = 0, .timezone = UTZ_NUUK},                  // Nuuk, Greenland
    {.name = "Panama City", .latitude = 897, .longitude = -7953, .region = 0, .timezone = UTZ_CHICAGO},         // Panama City, Panama
    {.name = "Miami", .latitude = 2576, .longitude = -8019, .region = 0, .timezone = UTZ_NEW_YORK},             // Miami, United States
    {.name = "Raleigh", .latitude = 3578, .longitude = -7864, .region = 0, .timezone = UTZ_NEW_YORK},           // Raleigh, United States
    {.name = "New York City", .latitude = 4071, .longitude = -7401, .region = 0, .timezone = UTZ_NEW_YORK},     // New York City, United States
    {.name = "Montreal", .latitude = 4551, .longitude = -7362, .region = 0, .timezone = UTZ_NEW_YORK},          // Montreal, Canada
    {.name = "Santo Domingo", .latitude = 1847, .longitude = -6989, .region = 0, .timezone = UTZ_CHICAGO},      // Santo Domingo, Dominican Republic

    // Europe (west to east by longitude)
    {.name = "Reykjavik", .latitude = 6410, .longitude = -2195, .region = 2, .timezone = UTZ_UTC},    // Reykjavik, Iceland
    {.name = "Dublin", .latitude = 5335, .longitude = -627, .region = 2, .timezone = UTZ_LONDON},     // Dublin, Ireland
    {.name = "London", .latitude = 5150, .longitude = -13, .region = 2, .timezone = UTZ_LONDON},      // London, United Kingdom
    {.name = "Glasgow", .latitude = 5586, .longitude = -425, .region = 2, .timezone = UTZ_LONDON},    // Glasgow, United Kingdom
    {.name = "Lisbon", .latitude = 3871, .longitude = -912, .region = 2, .timezone = UTZ_LONDON},     // Lisbon, Portugal
    {.name = "Madrid", .latitude = 4042, .longitude = -370, .region = 2, .timezone = UTZ_BERLIN},     // Madrid, Spain
    {.name = "Paris", .latitude = 4885, .longitude = 235, .region = 2, .timezone = UTZ_BERLIN},       // Paris, France
    {.name = "Amsterdam", .latitude = 5237, .longitude = 488, .region = 2, .timezone = UTZ_BERLIN},   // Amsterdam, Netherlands
    {.name = "Brussels", .latitude = 5085, .longitude = 435, .region = 2, .timezone = UTZ_BERLIN},    // Brussels, Belgium
    {.name = "Zurich", .latitude = 4737, .longitude = 853, .region = 2, .timezone = UTZ_BERLIN},      // Zurich, Switzerland
    {.name = "Rome", .latitude = 4189, .longitude = 1248, .region = 2, .timezone = UTZ_BERLIN},       // Rome, Italy
    {.name = "Berlin", .latitude = 5252, .longitude = 1340, .region = 2, .timezone = UTZ_BERLIN},     // Berlin, Germany
    {.name = "Vienna", .latitude = 4821, .longitude = 1637, .region = 2, .timezone = UTZ_BERLIN},     // Vienna, Austria
    {.name = "Stockholm", .latitude = 5933, .longitude = 1807, .region = 2, .timezone = UTZ_BERLIN},  // Stockholm, Sweden
    {.name = "Athens", .latitude = 3798, .longitude = 2372, .region = 2, .timezone = UTZ_CAIRO},      // Athens, Greece
    {.name = "Helsinki", .latitude = 6017, .longitude = 2495, .region = 2, .timezone = UTZ_HELSINKI}, // Helsinki, Finland
    {.name = "Istanbul", .latitude = 4101, .longitude = 2898, .region = 2, .timezone = UTZ_MOSCOW},   // Istanbul, Turkey
    {.name = "Murmansk", .latitude = 6897, .longitude = 3305, .region = 2, .timezone = UTZ_MOSCOW},   // Murmansk, Russia // Just for arctic circle testing
    {.name = "Moscow", .latitude = 5575, .longitude = 3761, .region = 2, .timezone = UTZ_MOSCOW},     // Moscow, Russia

    // Asia (west to east by longitude)
    {.name = "Aleppo", .latitude = 3620, .longitude = 3716, .region = 1, .timezone = UTZ_MOSCOW},          // Aleppo, Syria
    {.name = "Baghdad", .latitude = 3333, .longitude = 4442, .region = 1, .timezone = UTZ_RIYADH},         // Baghdad, Iraq
    {.name = "Najaf", .latitude = 3200, .longitude = 4433, .region = 1, .timezone = UTZ_RIYADH},           // Najaf, Iraq
    {.name = "Sanaa", .latitude = 1535, .longitude = 4420, .region = 1, .timezone = UTZ_RIYADH},           // Sanaa, Yemen
    {.name = "Yerevan", .latitude = 4018, .longitude = 4451, .region = 1, .timezone = UTZ_DUBAI},          // Yerevan, Armenia
    {.name = "Riyadh", .latitude = 2333, .longitude = 4533, .region = 1, .timezone = UTZ_RIYADH},          // Riyadh, Saudi Arabia
    {.name = "Tehran", .latitude = 3569, .longitude = 5139, .region = 1, .timezone = UTZ_TEHRAN},          // Tehran, Iran
    {.name = "Dubai", .latitude = 2507, .longitude = 5519, .region = 1, .timezone = UTZ_DUBAI},            // Dubai, United Arab Emirates
    {.name = "Jerusalem", .latitude = 3178, .longitude = 3518, .region = 1, .timezone = UTZ_JERUSALEM},    // Jerusalem, Israel
    {.name = "Karachi", .latitude = 2485, .longitude = 6702, .region = 1, .timezone = UTZ_KOLKATA},        // Karachi, Pakistan
    {.name = "Kabul", .latitude = 3453, .longitude = 6919, .region = 1, .timezone = UTZ_KATHMANDU},        // Kabul, Afghanistan
    {.name = "Kathmandu", .latitude = 2772, .longitude = 8522, .region = 1, .timezone = UTZ_KATHMANDU},    // Kathmandu, Nepal
    {.name = "Tashkent", .latitude = 4131, .longitude = 6928, .region = 1, .timezone = UTZ_KOLKATA},       // Tashkent, Uzbekistan
    {.name = "Astana", .latitude = 5113, .longitude = 7143, .region = 1, .timezone = UTZ_KOLKATA},         // Astana, Kazakhstan
    {.name = "Mumbai", .latitude = 1905, .longitude = 7287, .region = 1, .timezone = UTZ_KOLKATA},         // Mumbai, India
    {.name = "Delhi", .latitude = 2863, .longitude = 7722, .region = 1, .timezone = UTZ_KOLKATA},          // Delhi, India
    {.name = "Bangalore", .latitude = 1298, .longitude = 7759, .region = 1, .timezone = UTZ_KOLKATA},      // Bangalore, India
    {.name = "Kolkata", .latitude = 2252, .longitude = 8852, .region = 1, .timezone = UTZ_KOLKATA},        // Kolkata, India
    {.name = "Urumqi", .latitude = 4382, .longitude = 8761, .region = 1, .timezone = UTZ_KOLKATA},         // Urumqi, China
    {.name = "Dhaka", .latitude = 2376, .longitude = 9039, .region = 1, .timezone = UTZ_KOLKATA},          // Dhaka, Bangladesh
    {.name = "Yangon", .latitude = 1677, .longitude = 9603, .region = 1, .timezone = UTZ_YANGON},          // Yangon, Myanmar
    {.name = "Bangkok", .latitude = 1375, .longitude = 10049, .region = 1, .timezone = UTZ_BANGKOK},       // Bangkok, Thailand
    {.name = "Kuala Lumpur", .latitude = 315, .longitude = 10170, .region = 1, .timezone = UTZ_SINGAPORE}, // Kuala Lumpur, Malaysia
    {.name = "Lanzhou", .latitude = 3647, .longitude = 10373, .region = 1, .timezone = UTZ_SHANGHAI},      // Lanzhou, China
    {.name = "Hanoi", .latitude = 2103, .longitude = 10585, .region = 1, .timezone = UTZ_BANGKOK},         // Hanoi, Vietnam
    {.name = "Jakarta", .latitude = -618, .longitude = 10683, .region = 1, .timezone = UTZ_BANGKOK},       // Jakarta, Indonesia
    {.name = "Ulaanbaatar", .latitude = 4792, .longitude = 10692, .region = 1, .timezone = UTZ_SHANGHAI},  // Ulaanbaatar, Mongolia
    {.name = "Chongqing", .latitude = 3006, .longitude = 10787, .region = 1, .timezone = UTZ_SHANGHAI},    // Chongqing, China
    {.name = "Singapore", .latitude = 123, .longitude = 10385, .region = 1, .timezone = UTZ_SINGAPORE},    // Singapore
    {.name = "Guangzhou", .latitude = 2313, .longitude = 11326, .region = 1, .timezone = UTZ_SHANGHAI},    // Guangzhou, China
    {.name = "Hong Kong", .latitude = 2232, .longitude = 11418, .region = 1, .timezone = UTZ_HONG_KONG},   // Hong Kong
    {.name = "Beijing", .latitude = 4019, .longitude = 11641, .region = 1, .timezone = UTZ_SHANGHAI},      // Beijing, China
    {.name = "Makassar", .latitude = -513, .longitude = 11941, .region = 1, .timezone = UTZ_SINGAPORE},    // Makassar, Indonesia
    {.name = "Taizhong", .latitude = 2416, .longitude = 12065, .region = 1, .timezone = UTZ_SHANGHAI},     // Taizhong, Taiwan
    {.name = "Manila", .latitude = 1459, .longitude = 12098, .region = 1, .timezone = UTZ_SHANGHAI},       // Manila, Philippines
    {.name = "Shanghai", .latitude = 3123, .longitude = 12147, .region = 1, .timezone = UTZ_SHANGHAI},     // Shanghai, China
    {.name = "Daqing", .latitude = 4632, .longitude = 12456, .region = 1, .timezone = UTZ_SHANGHAI},       // Daqing, China
    {.name = "Davao City", .latitude = 706, .longitude = 12561, .region = 1, .timezone = UTZ_SHANGHAI},    // Davao City, Philippines
    {.name = "Seoul", .latitude = 3757, .longitude = 12698, .region = 1, .timezone = UTZ_SEOUL},           // Seoul, South Korea
    {.name = "Tokyo", .latitude = 3568, .longitude = 13976, .region = 1, .timezone = UTZ_TOKYO},           // Tokyo, Japan

    // Africa (west to east by longitude)
    {.name = "Dakar", .latitude = 1469, .longitude = -1745, .region = 3, .timezone = UTZ_UTC},            // Dakar, Senegal
    {.name = "Monrovia", .latitude = 633, .longitude = -1080, .region = 3, .timezone = UTZ_UTC},          // Monrovia, Liberia
    {.name = "Casablanca", .latitude = 3359, .longitude = -762, .region = 3, .timezone = UTZ_LONDON},     // Casablanca, Morocco
    {.name = "Abidjan", .latitude = 532, .longitude = -402, .region = 3, .timezone = UTZ_UTC},            // Abidjan, Ivory Coast
    {.name = "Ouagadougou", .latitude = 1237, .longitude = -153, .region = 3, .timezone = UTZ_UTC},       // Ouagadougou, Burkina Faso
    {.name = "Kano", .latitude = 1199, .longitude = 852, .region = 3, .timezone = UTZ_LAGOS},             // Kano, Nigeria
    {.name = "Tunis", .latitude = 3384, .longitude = 940, .region = 3, .timezone = UTZ_LAGOS},            // Tunis, Tunisia
    {.name = "Lagos", .latitude = 646, .longitude = 339, .region = 3, .timezone = UTZ_LAGOS},             // Lagos, Nigeria
    {.name = "Yaounde", .latitude = 387, .longitude = 1152, .region = 3, .timezone = UTZ_LAGOS},          // Yaounde, Cameroon
    {.name = "Lubango", .latitude = -1492, .longitude = 1349, .region = 3, .timezone = UTZ_LAGOS},        // Lubango, Angola
    {.name = "Kinshasa", .latitude = -430, .longitude = 1531, .region = 3, .timezone = UTZ_LAGOS},        // Kinshasa, DR Congo
    {.name = "Maputo", .latitude = -2588, .longitude = 3262, .region = 3, .timezone = UTZ_MAPUTO},        // Maputo, Mozambique
    {.name = "Cape Town", .latitude = -3393, .longitude = 1842, .region = 3, .timezone = UTZ_MAPUTO},     // Cape Town, South Africa
    {.name = "Bangui", .latitude = 436, .longitude = 1858, .region = 3, .timezone = UTZ_LAGOS},           // Bangui, Central African Republic
    {.name = "Banghazi", .latitude = 3212, .longitude = 2009, .region = 3, .timezone = UTZ_MAPUTO},       // Banghazi, Libya
    {.name = "Mbuji-Mayi", .latitude = -613, .longitude = 2360, .region = 3, .timezone = UTZ_MAPUTO},     // Mbuji-Mayi, DR Congo
    {.name = "Nyala", .latitude = 1228, .longitude = 2477, .region = 3, .timezone = UTZ_MAPUTO},          // Nyala, Sudan
    {.name = "Kisangani", .latitude = 52, .longitude = 2521, .region = 3, .timezone = UTZ_MAPUTO},        // Kisangani, DR Congo
    {.name = "Johannesburg", .latitude = -2620, .longitude = 2805, .region = 3, .timezone = UTZ_MAPUTO},  // Johannesburg, South Africa
    {.name = "Cairo", .latitude = 3004, .longitude = 3124, .region = 3, .timezone = UTZ_CAIRO},           // Cairo, Egypt
    {.name = "Kampala", .latitude = 32, .longitude = 3258, .region = 3, .timezone = UTZ_NAIROBI},         // Kampala, Uganda
    {.name = "Addis Ababa", .latitude = 904, .longitude = 3875, .region = 3, .timezone = UTZ_NAIROBI},    // Addis Ababa, Ethiopia
    {.name = "Nampula", .latitude = -1497, .longitude = 3927, .region = 3, .timezone = UTZ_MAPUTO},       // Nampula, Mozambique
    {.name = "Dar es Salaam", .latitude = -682, .longitude = 3928, .region = 3, .timezone = UTZ_NAIROBI}, // Dar es Salaam, Tanzania
    {.name = "Nairobi", .latitude = -129, .longitude = 3682, .region = 3, .timezone = UTZ_NAIROBI},       // Nairobi, Kenya
    {.name = "Mogadishu", .latitude = 203, .longitude = 4534, .region = 3, .timezone = UTZ_NAIROBI},      // Mogadishu, Somalia
    {.name = "Antananarivo", .latitude = -1891, .longitude = 4753, .region = 3, .timezone = UTZ_NAIROBI}, // Antananarivo, Madagascar

    // South America (west to east by longitude)
    {.name = "Guayaquil", .latitude = -229, .longitude = -8010, .region = 4, .timezone = UTZ_CHICAGO},         // Guayaquil, Ecuador
    {.name = "Lima", .latitude = -1205, .longitude = -7703, .region = 4, .timezone = UTZ_CHICAGO},             // Lima, Peru
    {.name = "Bogota", .latitude = 465, .longitude = -7408, .region = 4, .timezone = UTZ_CHICAGO},             // Bogota, Colombia
    {.name = "Manaus", .latitude = -306, .longitude = -6015, .region = 4, .timezone = UTZ_MANAUS},             // Manaus, Brazil
    {.name = "Santiago", .latitude = -3344, .longitude = -7065, .region = 4, .timezone = UTZ_SANTIAGO},        // Santiago, Chile
    {.name = "La Paz", .latitude = -1650, .longitude = -6813, .region = 4, .timezone = UTZ_CHICAGO},           // La Paz, Bolivia
    {.name = "Caracas", .latitude = 1051, .longitude = -6691, .region = 4, .timezone = UTZ_CHICAGO},           // Caracas, Venezuela
    {.name = "San Miguel de", .latitude = -2683, .longitude = -6520, .region = 4, .timezone = UTZ_SAO_PAULO},  // San Miguel de Tucuman, Argentina
    {.name = "Buenos Aires", .latitude = -3461, .longitude = -5839, .region = 4, .timezone = UTZ_SAO_PAULO},   // Buenos Aires, Argentina
    {.name = "Asuncion", .latitude = -2528, .longitude = -5763, .region = 4, .timezone = UTZ_SAO_PAULO},       // Asuncion, Paraguay
    {.name = "Brasilia", .latitude = -1033, .longitude = -5320, .region = 4, .timezone = UTZ_SAO_PAULO},       // Brasilia, Brazil
    {.name = "Porto Alegre", .latitude = -3003, .longitude = -5123, .region = 4, .timezone = UTZ_SAO_PAULO},   // Porto Alegre, Brazil
    {.name = "Sao Paulo", .latitude = -2355, .longitude = -4663, .region = 4, .timezone = UTZ_SAO_PAULO},      // Sao Paulo, Brazil
    {.name = "Rio de Janeiro", .latitude = -2292, .longitude = -4323, .region = 4, .timezone = UTZ_SAO_PAULO}, // Rio de Janeiro, Brazil
    {.name = "Recife", .latitude = -806, .longitude = -3488, .region = 4, .timezone = UTZ_SAO_PAULO},          // Recife, Brazil

    // Oceania (west to east by longitude)
    {.name = "Pago Pago", .latitude = -1428, .longitude = -17068, .region = 5, .timezone = UTZ_PAGO_PAGO}, // Pago Pago, American Samoa
    {.name = "Perth", .latitude = -3196, .longitude = 11586, .region = 5, .timezone = UTZ_PERTH},          // Perth, Australia
    {.name = "Darwin", .latitude = -1246, .longitude = 13084, .region = 5, .timezone = UTZ_DARWIN},        // Darwin, Australia
    {.name = "Adelaide", .latitude = -3493, .longitude = 13860, .region = 5, .timezone = UTZ_ADELAIDE},    // Adelaide, Australia
    {.name = "Melbourne", .latitude = -3781, .longitude = 14496, .region = 5, .timezone = UTZ_SYDNEY},     // Melbourne, Australia
    {.name = "Hobart", .latitude = -4288, .longitude = 14732, .region = 5, .timezone = UTZ_HOBART},        // Hobart, Australia
    {.name = "Sydney", .latitude = -3387, .longitude = 15121, .region = 5, .timezone = UTZ_SYDNEY},        // Sydney, Australia
    {.name = "Brisbane", .latitude = -2747, .longitude = 15302, .region = 5, .timezone = UTZ_BRISBANE},    // Brisbane, Australia
    {.name = "Guam", .latitude = 1347, .longitude = 14447, .region = 5, .timezone = UTZ_GUAM},             // Guam
    {.name = "Tarawa", .latitude = 143, .longitude = 17302, .region = 5, .timezone = UTZ_TARAWA},          // Tarawa, Kiribati
    {.name = "Auckland", .latitude = -3685, .longitude = 17476, .region = 5, .timezone = UTZ_AUCKLAND},    // Auckland, New Zealand
    {.name = "Wellington", .latitude = -4129, .longitude = 17478, .region = 5, .timezone = UTZ_AUCKLAND},  // Wellington, New Zealand
};

const uint16_t movement_location_presets_count = (uint16_t)(sizeof(movement_location_presets) / sizeof(movement_location_presets[0]));

#if defined(__STDC_VERSION__) && __STDC_VERSION__ >= 201112L
_Static_assert((sizeof(movement_location_presets) / sizeof(movement_location_presets[0])) <= 160, "Movement location table exceeds MAX_LOCATIONS");
#endif

const location_long_lat_presets_t *movement_location_find_by_name(const char *name)
{
  if (!name)
    return NULL;
  for (uint16_t i = 0; i < movement_location_presets_count; i++)
  {
    if (strcmp(movement_location_presets[i].name, name) == 0)
    {
      return &movement_location_presets[i];
    }
  }
  return NULL;
}

int16_t movement_location_index_by_name(const char *name)
{
  if (!name)
    return -1;
  for (uint16_t i = 0; i < movement_location_presets_count; i++)
  {
    if (strcmp(movement_location_presets[i].name, name) == 0)
    {
      return (int16_t)i;
    }
  }
  return -1;
}

const location_long_lat_presets_t *movement_location_find_by_coordinates(int16_t latitude, int16_t longitude, uint8_t timezone)
{
  for (uint16_t i = 0; i < movement_location_presets_count; i++)
  {
    if ((timezone == movement_location_presets[i].timezone) &&
        movement_location_presets[i].latitude == latitude &&
        movement_location_presets[i].longitude == longitude)
    {
      return &movement_location_presets[i];
    }
  }
  return NULL;
}

const location_long_lat_presets_t *movement_location_get_default_for_zone(uint8_t timezone)
{
  if (timezone >= NUM_ZONE_NAMES)
  {
    return NULL;
  }
  for (uint16_t i = 0; i < movement_location_presets_count; i++)
  {
    if (movement_location_presets[i].timezone == timezone)
    {
      return &movement_location_presets[i];
    }
  }
  return NULL;
}
