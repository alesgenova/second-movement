/*
 * Partial C port of Greg Miller's public domain astro library (gmiller@gregmiller.net) 2019
 * https://github.com/gmiller123456/astrogreg
 *
 * Ported by Joey Castillo for Sensor Watch
 * https://github.com/joeycastillo/Sensor-Watch/
 *
 * Public Domain
 *
 * THIS SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include "astrolib.h"
#include "vsop87a_milli.h"
#include "astro_trig.h"

double astro_convert_utc_to_tt(double jd) ;
double astro_get_GMST(double ut1);
astro_cartesian_coordinates_t astro_subtract_cartesian(astro_cartesian_coordinates_t a, astro_cartesian_coordinates_t b);
astro_cartesian_coordinates_t astro_rotate_from_vsop_to_J2000(astro_cartesian_coordinates_t c);
astro_matrix_t astro_get_x_rotation_matrix(float r);
astro_matrix_t astro_get_y_rotation_matrix(float r);
astro_matrix_t astro_get_z_rotation_matrix(float r);
astro_matrix_t astro_transpose_matrix(astro_matrix_t m);
astro_matrix_t astro_dot_product(astro_matrix_t a, astro_matrix_t b);
astro_matrix_t astro_get_precession_matrix(double jd);
astro_cartesian_coordinates_t astro_matrix_multiply(astro_cartesian_coordinates_t v, astro_matrix_t m);
astro_cartesian_coordinates_t astro_convert_geodedic_latlon_to_ITRF_XYZ(float lat, float lon, float height);
astro_cartesian_coordinates_t astro_convert_ITRF_to_GCRS(astro_cartesian_coordinates_t r, double ut1);
astro_cartesian_coordinates_t astro_convert_coordinates_from_meters_to_AU(astro_cartesian_coordinates_t c);
astro_cartesian_coordinates_t astro_get_observer_geocentric_coords(double jd, float lat, float lon);
astro_cartesian_coordinates_t astro_get_body_coordinates(astro_body_t bodyNum, float et);
astro_cartesian_coordinates_t astro_get_body_coordinates_light_time_adjusted(astro_body_t body, astro_cartesian_coordinates_t origin, float t);
astro_equatorial_coordinates_t astro_convert_cartesian_to_polar(astro_cartesian_coordinates_t xyz);
static void _astro_get_moon_coordinates(float et, float coords[3]);

/* Tiny Earth ephemeris cache: many call paths request Earth repeatedly for the same et. */
static bool _astro_earth_cache_valid = false;
static float _astro_earth_cache_et = 0.0f;
static astro_cartesian_coordinates_t _astro_earth_cache_coords = {0};
static bool _astro_precession_cache_valid = false;
static double _astro_precession_cache_jdtt = 0.0;
static astro_matrix_t _astro_precession_cache_matrix = {0};

typedef struct {
    bool valid;
    bool has_rate;
    float t;
    astro_cartesian_coordinates_t pos;
    astro_cartesian_coordinates_t vel_per_t;
} _astro_body_linear_cache_t;

static _astro_body_linear_cache_t _astro_jupiter_linear_cache = {0};
static _astro_body_linear_cache_t _astro_saturn_linear_cache = {0};
static _astro_body_linear_cache_t _astro_mars_linear_cache = {0};
typedef struct {
    bool valid;
    bool has_rate;
    float t;
    astro_cartesian_coordinates_t pos;
    astro_cartesian_coordinates_t vel_per_t;
} _astro_earth_linear_cache_t;
static _astro_earth_linear_cache_t _astro_earth_linear_cache = {0};

// We do linear or quadratic predictions to the planets. Below is how often we need to recalc the exact values.
// It is set so that their prediods don't have common denominbators so we don't need to recalc on the same day,

#ifndef ASTRO_ENABLE_OUTER_PLANET_LINEAR_PREDICTOR
#define ASTRO_ENABLE_OUTER_PLANET_LINEAR_PREDICTOR 1
#endif

#ifndef ASTRO_ENABLE_EARTH_QUADRATIC_PREDICTOR
#define ASTRO_ENABLE_EARTH_QUADRATIC_PREDICTOR 1
#endif

#ifndef ASTRO_JUPITER_PREDICTION_DAYS
#define ASTRO_JUPITER_PREDICTION_DAYS 150.0f
#endif

#ifndef ASTRO_SATURN_PREDICTION_DAYS
#define ASTRO_SATURN_PREDICTION_DAYS 171.0f
#endif

#ifndef ASTRO_EARTH_PREDICTION_DAYS
#define ASTRO_EARTH_PREDICTION_DAYS 7.0f
#endif

#ifndef ASTRO_MARS_PREDICTION_DAYS
#define ASTRO_MARS_PREDICTION_DAYS 13.0f
#endif

//Special "Math.floor()" function used by convertDateToJulianDate()
static double _astro_special_floor(double d) {
    if(d > 0) {
        return floor(d);
    }
    return floor(d) - 1;
}

double astro_convert_date_to_julian_date(uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second) {
    if (month < 3){
        year = year - 1;
        month = month + 12;
    }

    double b = 0;
    if (!(year < 1582 || (year == 1582 && (month < 10 || (month == 10 && day < 5))))) {
        double a = _astro_special_floor(year / 100.0);
        b = 2 - a + _astro_special_floor(a / 4.0);
    }

    double jd = _astro_special_floor(365.25 * (year + 4716)) + _astro_special_floor(30.6001 * (month + 1)) + day + b - 1524.5;
    jd += hour / 24.0;
    jd += minute / 24.0 / 60.0;
    jd += second / 24.0 / 60.0 / 60.0;

    return jd;
}

//Return all values in radians.
//The positions are adjusted for the parallax of the Earth, and the offset of the observer from the Earth's center
//All input and output angles are in radians!
astro_equatorial_coordinates_t astro_get_ra_dec(double jd, astro_body_t body, float lat, float lon, bool calculate_precession) {
    double jdTT = astro_convert_utc_to_tt(jd);
    float t = (float)astro_convert_jd_to_julian_millenia_since_j2000(jdTT);

    /* Moon coords from astro_get_body_coordinates are already geocentric equatorial J2000.
       Skip the heliocentric subtraction and VSOP rotation used for planets. */
    if (body == ASTRO_BODY_MOON) {
        astro_cartesian_coordinates_t moon_coords = astro_get_body_coordinates(ASTRO_BODY_MOON, t);
        astro_cartesian_coordinates_t observerXYZ = astro_get_observer_geocentric_coords(jdTT, lat, lon);
        moon_coords = astro_subtract_cartesian(moon_coords, observerXYZ);
        astro_equatorial_coordinates_t retval = astro_convert_cartesian_to_polar(moon_coords);
        retval.declination = M_PI/2.0 - retval.declination;
        if (retval.right_ascension < 0) retval.right_ascension += 2*M_PI;
        return retval;
    }

    // Get current position of Earth and the target body
    astro_cartesian_coordinates_t earth_coords = astro_get_body_coordinates(ASTRO_BODY_EARTH, t);
    astro_cartesian_coordinates_t body_coords = astro_get_body_coordinates_light_time_adjusted(body, earth_coords, t);

    // Convert to Geocentric coordinate
    body_coords = astro_subtract_cartesian(body_coords, earth_coords);

    //Rotate ecliptic coordinates to J2000 coordinates
    body_coords = astro_rotate_from_vsop_to_J2000(body_coords);

    astro_matrix_t precession;
    // TODO: rotate body for precession, nutation and bias
    if(calculate_precession) {
        if (_astro_precession_cache_valid && jdTT == _astro_precession_cache_jdtt) {
            precession = _astro_precession_cache_matrix;
        } else {
            precession = astro_get_precession_matrix(jdTT);
            _astro_precession_cache_valid = true;
            _astro_precession_cache_jdtt = jdTT;
            _astro_precession_cache_matrix = precession;
        }
        body_coords = astro_matrix_multiply(body_coords, precession);
    }

    //Convert to topocentric
    astro_cartesian_coordinates_t observerXYZ = astro_get_observer_geocentric_coords(jdTT, lat, lon);

    if(calculate_precession) {
        //TODO: rotate observerXYZ for precession, nutation and bias
        astro_matrix_t precessionInv = astro_transpose_matrix(precession);
        observerXYZ = astro_matrix_multiply(observerXYZ, precessionInv);
    }

    body_coords = astro_subtract_cartesian(body_coords, observerXYZ);

    //Convert to topocentric RA DEC by converting from cartesian coordinates to polar coordinates
    astro_equatorial_coordinates_t retval = astro_convert_cartesian_to_polar(body_coords);

    retval.declination = M_PI/2.0 - retval.declination;  //Dec.  Offset to make 0 the equator, and the poles +/-90 deg
    if(retval.right_ascension < 0) retval.right_ascension += 2*M_PI; //Ensure RA is positive

    return retval;
}

//Converts a Julian Date in UTC to Terrestrial Time (TT)
double astro_convert_utc_to_tt(double jd) {
    //Leap seconds are hard coded, should be updated from the IERS website for other times

    //TAI = UTC + leap seconds (e.g. 32)
    //TT=TAI + 32.184

    //return jd + (32.0 + 32.184) / 24.0 / 60.0 / 60.0;
    return jd + (37.0 + 32.184) / 24.0 / 60.0 / 60.0;

    /*
    https://data.iana.org/time-zones/tzdb-2018a/leap-seconds.list
    2272060800  10  # 1 Jan 1972
    2287785600  11  # 1 Jul 1972
    2303683200  12  # 1 Jan 1973
    2335219200  13  # 1 Jan 1974
    2366755200  14  # 1 Jan 1975
    2398291200  15  # 1 Jan 1976
    2429913600  16  # 1 Jan 1977
    2461449600  17  # 1 Jan 1978
    2492985600  18  # 1 Jan 1979
    2524521600  19  # 1 Jan 1980
    2571782400  20  # 1 Jul 1981
    2603318400  21  # 1 Jul 1982
    2634854400  22  # 1 Jul 1983
    2698012800  23  # 1 Jul 1985
    2776982400  24  # 1 Jan 1988
    2840140800  25  # 1 Jan 1990
    2871676800  26  # 1 Jan 1991
    2918937600  27  # 1 Jul 1992
    2950473600  28  # 1 Jul 1993
    2982009600  29  # 1 Jul 1994
    3029443200  30  # 1 Jan 1996
    3076704000  31  # 1 Jul 1997
    3124137600  32  # 1 Jan 1999
    3345062400  33  # 1 Jan 2006
    3439756800  34  # 1 Jan 2009
    3550089600  35  # 1 Jul 2012
    3644697600  36  # 1 Jul 2015
    3692217600  37  # 1 Jan 2017
    */
}

double astro_convert_jd_to_julian_millenia_since_j2000(double jd) {
    return (jd - 2451545.0) / 365250.0;
}

astro_cartesian_coordinates_t astro_subtract_cartesian(astro_cartesian_coordinates_t a, astro_cartesian_coordinates_t b) {
    astro_cartesian_coordinates_t retval;

    retval.x = a.x - b.x;
    retval.y = a.y - b.y;
    retval.z = a.z - b.z;

    return retval;
}

// Performs the rotation from ecliptic coordinates to J2000 coordinates for the given vector x
astro_cartesian_coordinates_t astro_rotate_from_vsop_to_J2000(astro_cartesian_coordinates_t c) {
    /* From VSOP87.doc
        X        +1.000000000000  +0.000000440360  -0.000000190919   X
        Y     =  -0.000000479966  +0.917482137087  -0.397776982902   Y
        Z FK5     0.000000000000  +0.397776982902  +0.917482137087   Z VSOP87A
    */
    astro_cartesian_coordinates_t t;
    t.x = c.x + c.y * 0.000000440360 + c.z * -0.000000190919;
    t.y = c.x * -0.000000479966 + c.y * 0.917482137087 + c.z * -0.397776982902;
    t.z = c.y * 0.397776982902 + c.z * 0.917482137087;

    return t;
}

double astro_get_GMST(double ut1) {
    double D = ut1 - 2451545.0;
    double T = D/36525.0;
    double gmst = fmod((280.46061837 + 360.98564736629*D + 0.000387933*T*T - T*T*T/38710000.0), 360.0);

    if(gmst<0) {
        gmst+=360;
    }

    return gmst/15;
}

static astro_matrix_t _astro_get_empty_matrix() {
    astro_matrix_t t;
    for(uint8_t i = 0; i < 3 ; i++) {
        for(uint8_t j = 0 ; j < 3 ; j++) {
            t.elements[i][j] = 0;
        }
    }
    return t;
}

//Gets a rotation matrix about the x axis.  Angle R is in radians
astro_matrix_t astro_get_x_rotation_matrix(float r) {
    astro_matrix_t t = _astro_get_empty_matrix();

    t.elements[0][0]=1;
    t.elements[0][1]=0;
    t.elements[0][2]=0;
    t.elements[1][0]=0;
    t.elements[1][1]=astro_cosf((float)(r));
    t.elements[1][2]=astro_sinf((float)(r));
    t.elements[2][0]=0;
    t.elements[2][1]=-astro_sinf((float)(r));
    t.elements[2][2]=astro_cosf((float)(r));

    return t;
}

//Gets a rotation matrix about the y axis.  Angle R is in radians
astro_matrix_t astro_get_y_rotation_matrix(float r) {
    astro_matrix_t t = _astro_get_empty_matrix();

    t.elements[0][0]=astro_cosf((float)(r));
    t.elements[0][1]=0;
    t.elements[0][2]=-astro_sinf((float)(r));
    t.elements[1][0]=0;
    t.elements[1][1]=1;
    t.elements[1][2]=0;
    t.elements[2][0]=astro_sinf((float)(r));
    t.elements[2][1]=0;
    t.elements[2][2]=astro_cosf((float)(r));

    return t;
}

//Gets a rotation matrix about the z axis.  Angle R is in radians
astro_matrix_t astro_get_z_rotation_matrix(float r) {
    astro_matrix_t t = _astro_get_empty_matrix();

    t.elements[0][0]=astro_cosf((float)(r));
    t.elements[0][1]=astro_sinf((float)(r));
    t.elements[0][2]=0;
    t.elements[1][0]=-astro_sinf((float)(r));
    t.elements[1][1]=astro_cosf((float)(r));
    t.elements[1][2]=0;
    t.elements[2][0]=0;
    t.elements[2][1]=0;
    t.elements[2][2]=1;

    return t;
}


astro_matrix_t astro_dot_product(astro_matrix_t a, astro_matrix_t b) {
    astro_matrix_t retval;

    for(uint8_t i = 0; i < 3 ; i++) {
        for(uint8_t j = 0 ; j < 3 ; j++) {
            float temp = 0;
            for(uint8_t k = 0; k < 3 ; k++) {
                temp += a.elements[i][k] * b.elements[k][j];
            }
            retval.elements[i][j]=temp;
        }
    }

    return retval;
}

astro_matrix_t astro_transpose_matrix(astro_matrix_t m) {
    astro_matrix_t retval;
    for(uint8_t i = 0; i < 3 ; i++) {
        for(uint8_t j = 0 ; j < 3 ; j++) {
            retval.elements[i][j] = m.elements[j][i];
        }
    }
    return retval;
}

astro_matrix_t astro_get_precession_matrix(double jd) {
    //2006 IAU Precession.  Implemented from IERS Technical Note No 36 ch5.
    //https://www.iers.org/SharedDocs/Publikationen/EN/IERS/Publications/tn/TechnNote36/tn36_043.pdf?__blob=publicationFile&v=1

    float t = (float)((jd - 2451545.0) / 36525.0);  //5.2
    const float Arcsec2Radians = M_PI/180.0/60.0/60.0; //Converts arc seconds used in equations below to radians

    float e0 = 84381.406 * Arcsec2Radians; //5.6.4
    float omegaA = e0 + ((-0.025754 + (0.0512623 +	(-0.00772503 + (-0.000000467 + 0.0000003337*t) * t) * t) * t) * t) * Arcsec2Radians; //5.39
    float psiA = ((5038.481507 +	(-1.0790069 + (-0.00114045 + (0.000132851 - 0.0000000951*t) * t) * t) * t) * t) * Arcsec2Radians; //5.39
    float chiA = ((10.556403 + (-2.3814292 + (-0.00121197 + (0.000170663 - 0.0000000560*t) * t) * t) * t) * t) * Arcsec2Radians; //5.40
    //Rotation matrix from 5.4.5
    //(R1(−e0) · R3(psiA) · R1(omegaA) · R3(−chiA))
    //Above eq rotates from "of date" to J2000, so we reverse the signs to go from J2000 to "of date"
    astro_matrix_t m1 = astro_get_x_rotation_matrix(e0);
    astro_matrix_t m2 = astro_get_z_rotation_matrix(-psiA);
    astro_matrix_t m3 = astro_get_x_rotation_matrix(-omegaA);
    astro_matrix_t m4 = astro_get_z_rotation_matrix(chiA);

    astro_matrix_t m5 = astro_dot_product(m4, m3);
    astro_matrix_t m6 = astro_dot_product(m5, m2);
    astro_matrix_t precessionMatrix = astro_dot_product(m6, m1);

    return precessionMatrix;
}

astro_cartesian_coordinates_t astro_matrix_multiply(astro_cartesian_coordinates_t v, astro_matrix_t m) {
    astro_cartesian_coordinates_t t;

    t.x = v.x*m.elements[0][0] + v.y*m.elements[0][1] + v.z*m.elements[0][2];
    t.y = v.x*m.elements[1][0] + v.y*m.elements[1][1] + v.z*m.elements[1][2];
    t.z = v.x*m.elements[2][0] + v.y*m.elements[2][1] + v.z*m.elements[2][2];

    return t;
}

//Converts cartesian XYZ coordinates to polar (e.g. J2000 xyz to Right Accention and Declication)
astro_equatorial_coordinates_t astro_convert_cartesian_to_polar(astro_cartesian_coordinates_t xyz) {
    astro_equatorial_coordinates_t t;

    t.distance = sqrt(xyz.x * xyz.x + xyz.y * xyz.y + xyz.z * xyz.z);
    t.declination = acos(xyz.z / t.distance);
    t.right_ascension = atan2(xyz.y, xyz.x);

    if(t.declination < 0) t.declination += 2 * M_PI;

    if(t.right_ascension < 0) t.right_ascension += 2 * M_PI;

    return t;
}

//Convert Geodedic Lat Lon to geocentric XYZ position vector
//All angles are input as radians
astro_cartesian_coordinates_t astro_convert_geodedic_latlon_to_ITRF_XYZ(float lat, float lon, float height) {
    //Algorithm from Explanatory Supplement to the Astronomical Almanac 3rd ed. P294
    const float a = 6378136.6;
    const float f = 1 / 298.25642;

    const float C = sqrt(((astro_cosf((float)(lat))*astro_cosf((float)(lat))) + (1.0-f)*(1.0-f) * (astro_sinf((float)(lat))*astro_sinf((float)(lat)))));

    const float S = (1-f)*(1-f)*C;

    float h = height;

    astro_cartesian_coordinates_t r;
    r.x = (a*C+h) * astro_cosf((float)(lat)) * astro_cosf((float)(lon));
    r.y = (a*C+h) * astro_cosf((float)(lat)) * astro_sinf((float)(lon));
    r.z = (a*S+h) * astro_sinf((float)(lat));

    return r;
}

//Convert position vector to celestial "of date" system.
//g(t)=R3(-GAST) r
//(Remember to use UT1 for GAST, not ET)
//All angles are input and output as radians
astro_cartesian_coordinates_t astro_convert_ITRF_to_GCRS(astro_cartesian_coordinates_t r, double ut1) {
    //This is a simple rotation matrix implemenation about the Z axis, rotation angle is -GMST

    float GMST = (float)astro_get_GMST(ut1);
    GMST =- GMST * 15.0 * M_PI / 180.0;

    astro_matrix_t m = astro_get_z_rotation_matrix(GMST);
    astro_cartesian_coordinates_t t = astro_matrix_multiply(r, m);

    return t;
}

astro_cartesian_coordinates_t astro_convert_coordinates_from_meters_to_AU(astro_cartesian_coordinates_t c) {
    astro_cartesian_coordinates_t t;

    t.x = c.x / 1.49597870691E+11;
    t.y = c.y / 1.49597870691E+11;
    t.z = c.z / 1.49597870691E+11;

    return t;
}

astro_cartesian_coordinates_t astro_get_observer_geocentric_coords(double jd, float lat, float lon) {
    astro_cartesian_coordinates_t r = astro_convert_geodedic_latlon_to_ITRF_XYZ(lat, lon,0);
    r = astro_convert_ITRF_to_GCRS(r, jd);
    r = astro_convert_coordinates_from_meters_to_AU(r);

    return r;
}

static void _astro_get_moon_coordinates(float et, float coords[3]) {
    /* Low-precision geocentric Moon position (Meeus ch47, ~0.3 deg accuracy).
       Returns geocentric ecliptic cartesian in AU, J2000 frame approximation. */
    float T = et * 10.0;  // et is Julian millennia, T is Julian centuries
    float L0 = 218.3164477 + 481267.88123421*T;
    float M  = 134.9633964 + 477198.8675055*T;
    float Ms = 357.5291092 + 35999.0502909*T;
    float F  = 93.2720950  + 483202.0175233*T;
    float D  = 297.8501921 + 445267.1114034*T;
    float r2d = M_PI / 180.0;
    float lam = L0
        + 6.288774 * astro_sinf(M  * r2d)
        + 1.274027 * astro_sinf((2*D - M) * r2d)
        + 0.658314 * astro_sinf(2*D * r2d)
        + 0.213618 * astro_sinf(2*M * r2d)
        - 0.185116 * astro_sinf(Ms * r2d)
        - 0.114332 * astro_sinf(2*F * r2d);
    float beta =
        + 5.128122 * astro_sinf(F * r2d)
        + 0.280602 * astro_sinf((M + F) * r2d)
        + 0.277693 * astro_sinf((M - F) * r2d)
        + 0.173237 * astro_sinf((2*D - F) * r2d);
    /* Distance in AU (mean ~0.00257 AU) */
    float dist_km = 385000.56
        - 20905.355 * astro_cosf(M * r2d)
        - 3699.111  * astro_cosf((2*D - M) * r2d)
        - 2955.968  * astro_cosf(2*D * r2d);
    float dist_au = dist_km / 149597870.7;
    /* Convert ecliptic lon/lat to J2000 equatorial cartesian */
    float eps = (23.439291 - 0.013004*T) * r2d;
    float lam_r  = lam  * r2d;
    float beta_r = beta * r2d;
    coords[0] = dist_au * astro_cosf(beta_r) * astro_cosf(lam_r);
    coords[1] = dist_au * (astro_cosf(beta_r)*astro_sinf(lam_r)*astro_cosf(eps) - astro_sinf(beta_r)*astro_sinf(eps));
    coords[2] = dist_au * (astro_cosf(beta_r)*astro_sinf(lam_r)*astro_sinf(eps) + astro_sinf(beta_r)*astro_cosf(eps));
}

static bool _astro_linear_prediction_window_ok(astro_body_t body, float dt_t) {
    float abs_dt_t = fabsf(dt_t);
    /* t is Julian millennia; convert window days to t by dividing by 365250. */
    switch (body) {
        case ASTRO_BODY_MARS:    return abs_dt_t <= (ASTRO_MARS_PREDICTION_DAYS / 365250.0f);
        case ASTRO_BODY_JUPITER: return abs_dt_t <= (ASTRO_JUPITER_PREDICTION_DAYS / 365250.0f);
        case ASTRO_BODY_SATURN:  return abs_dt_t <= (ASTRO_SATURN_PREDICTION_DAYS / 365250.0f);
        default:                 return false;
    }
}

static astro_cartesian_coordinates_t _astro_linear_predict(astro_body_t body, float t, bool *used_prediction) {
    _astro_body_linear_cache_t *cache = 0;
    switch (body) {
        case ASTRO_BODY_MARS: cache = &_astro_mars_linear_cache; break;
        case ASTRO_BODY_JUPITER: cache = &_astro_jupiter_linear_cache; break;
        case ASTRO_BODY_SATURN: cache = &_astro_saturn_linear_cache; break;
        default: break;
    }
    astro_cartesian_coordinates_t out = {0};
    *used_prediction = false;
    if (cache == 0) return out;

    if (cache->valid && cache->has_rate) {
        float dt_t = t - cache->t;
        if (_astro_linear_prediction_window_ok(body, dt_t)) {
            out.x = cache->pos.x + cache->vel_per_t.x * dt_t;
            out.y = cache->pos.y + cache->vel_per_t.y * dt_t;
            out.z = cache->pos.z + cache->vel_per_t.z * dt_t;
            *used_prediction = true;
            return out;
        }
    }

    return out;
}

static bool _astro_seed_velocity_from_mean_motion(astro_body_t body, const astro_cartesian_coordinates_t *pos, astro_cartesian_coordinates_t *vel_per_t) {
    /* Mean angular rates in radians per Julian millennia (coarse initializer). */
    float omega = 0.0f;
    switch (body) {
        case ASTRO_BODY_MARS:
            omega = (float)(2.0 * M_PI * 1000.0 / 1.8808476);   /* sidereal period years */
            break;
        case ASTRO_BODY_JUPITER:
            omega = (float)(2.0 * M_PI * 1000.0 / 11.862615);  /* sidereal period years */
            break;
        case ASTRO_BODY_SATURN:
            omega = (float)(2.0 * M_PI * 1000.0 / 29.447498);  /* sidereal period years */
            break;
        default:
            return false;
    }

    /* Assume prograde motion approximately tangent in the ecliptic XY plane. */
    vel_per_t->x = -omega * pos->y;
    vel_per_t->y =  omega * pos->x;
    vel_per_t->z = 0.0f;
    return true;
}

static void _astro_linear_update(astro_body_t body, float t, const float coords[3]) {
    _astro_body_linear_cache_t *cache = 0;
    switch (body) {
        case ASTRO_BODY_MARS: cache = &_astro_mars_linear_cache; break;
        case ASTRO_BODY_JUPITER: cache = &_astro_jupiter_linear_cache; break;
        case ASTRO_BODY_SATURN: cache = &_astro_saturn_linear_cache; break;
        default: break;
    }
    if (cache == 0) return;
    astro_cartesian_coordinates_t new_pos = {coords[0], coords[1], coords[2]};

    if (cache->valid) {
        float dt_t = t - cache->t;
        if (fabsf(dt_t) > 1e-9f && fabsf(dt_t) < (120.0f / 365250.0f)) {
            cache->vel_per_t.x = (new_pos.x - cache->pos.x) / dt_t;
            cache->vel_per_t.y = (new_pos.y - cache->pos.y) / dt_t;
            cache->vel_per_t.z = (new_pos.z - cache->pos.z) / dt_t;
            cache->has_rate = true;
        }
    } else {
        astro_cartesian_coordinates_t seeded_vel = {0};
        if (_astro_seed_velocity_from_mean_motion(body, &new_pos, &seeded_vel)) {
            cache->vel_per_t = seeded_vel;
            cache->has_rate = true;
        }
    }

    cache->valid = true;
    cache->t = t;
    cache->pos = new_pos;
}

static bool _astro_earth_prediction_window_ok(float dt_t) {
    return fabsf(dt_t) <= (ASTRO_EARTH_PREDICTION_DAYS / 365250.0f);
}

static bool _astro_seed_earth_velocity_from_mean_motion(const astro_cartesian_coordinates_t *pos, astro_cartesian_coordinates_t *vel_per_t) {
    /* Earth sidereal period ~365.256363 days. Convert to rad per Julian millennia. */
    float omega = (float)(2.0 * M_PI * (365250.0 / 365.256363));
    vel_per_t->x = -omega * pos->y;
    vel_per_t->y =  omega * pos->x;
    vel_per_t->z = 0.0f;
    return true;
}

static bool _astro_earth_predict(float t, astro_cartesian_coordinates_t *out) {
#if ASTRO_ENABLE_EARTH_QUADRATIC_PREDICTOR
    if (_astro_earth_linear_cache.valid && _astro_earth_linear_cache.has_rate) {
        float dt = t - _astro_earth_linear_cache.t;
        if (_astro_earth_prediction_window_ok(dt)) {
            out->x = _astro_earth_linear_cache.pos.x + _astro_earth_linear_cache.vel_per_t.x * dt;
            out->y = _astro_earth_linear_cache.pos.y + _astro_earth_linear_cache.vel_per_t.y * dt;
            out->z = _astro_earth_linear_cache.pos.z + _astro_earth_linear_cache.vel_per_t.z * dt;
            return true;
        }
    }
#endif
    return false;
}

static void _astro_earth_update_model_from_exact(float t, const float coords[3]) {
    astro_cartesian_coordinates_t p = {coords[0], coords[1], coords[2]};
    _astro_earth_linear_cache.valid = true;
    _astro_earth_linear_cache.t = t;
    _astro_earth_linear_cache.pos = p;
    _astro_earth_linear_cache.has_rate =
        _astro_seed_earth_velocity_from_mean_motion(&p, &_astro_earth_linear_cache.vel_per_t);
}

//Returns a body's cartesian coordinates centered on the Sun.
//Requires vsop87a_milli_js, if you wish to use a different version of VSOP87, replace the class name vsop87a_milli below
astro_cartesian_coordinates_t astro_get_body_coordinates(astro_body_t body, float et) {
    astro_cartesian_coordinates_t retval = {0};
    float coords[3];
    float et_f = (float)et;
    switch(body) {
        case ASTRO_BODY_SUN:
            return retval; //Sun is at the center for vsop87a
        case ASTRO_BODY_MERCURY:
             vsop87a_milli_getMercury(et_f, coords);
             break;
        case ASTRO_BODY_VENUS:
             vsop87a_milli_getVenus(et_f, coords);
             break;
        case ASTRO_BODY_EARTH:
             if (_astro_earth_cache_valid && et_f == _astro_earth_cache_et) {
                 return _astro_earth_cache_coords;
             }
            {
                astro_cartesian_coordinates_t p = {0};
                if (_astro_earth_predict(et_f, &p)) {
                    coords[0] = p.x;
                    coords[1] = p.y;
                    coords[2] = p.z;
                } else {
                    vsop87a_milli_getEarth(et_f, coords);
                    _astro_earth_update_model_from_exact(et_f, coords);
                }
            }
             _astro_earth_cache_valid = true;
             _astro_earth_cache_et = et_f;
             _astro_earth_cache_coords.x = coords[0];
             _astro_earth_cache_coords.y = coords[1];
             _astro_earth_cache_coords.z = coords[2];
             break;
        case ASTRO_BODY_MARS:
            {
#if ASTRO_ENABLE_OUTER_PLANET_LINEAR_PREDICTOR
                bool used_prediction = false;
                astro_cartesian_coordinates_t p = _astro_linear_predict(ASTRO_BODY_MARS, et_f, &used_prediction);
                if (used_prediction) {
                    coords[0] = p.x;
                    coords[1] = p.y;
                    coords[2] = p.z;
                } else {
                    vsop87a_milli_getMars(et_f, coords);
                    _astro_linear_update(ASTRO_BODY_MARS, et_f, coords);
                }
#else
                vsop87a_milli_getMars(et_f, coords);
#endif
            }
             break;
        case ASTRO_BODY_JUPITER:
            {
#if ASTRO_ENABLE_OUTER_PLANET_LINEAR_PREDICTOR
                bool used_prediction = false;
                astro_cartesian_coordinates_t p = _astro_linear_predict(ASTRO_BODY_JUPITER, et_f, &used_prediction);
                if (used_prediction) {
                    coords[0] = p.x;
                    coords[1] = p.y;
                    coords[2] = p.z;
                } else {
                    vsop87a_milli_getJupiter(et_f, coords);
                    _astro_linear_update(ASTRO_BODY_JUPITER, et_f, coords);
                }
#else
                vsop87a_milli_getJupiter(et_f, coords);
#endif
            }
             break;
        case ASTRO_BODY_SATURN:
            {
#if ASTRO_ENABLE_OUTER_PLANET_LINEAR_PREDICTOR
                bool used_prediction = false;
                astro_cartesian_coordinates_t p = _astro_linear_predict(ASTRO_BODY_SATURN, et_f, &used_prediction);
                if (used_prediction) {
                    coords[0] = p.x;
                    coords[1] = p.y;
                    coords[2] = p.z;
                } else {
                    vsop87a_milli_getSaturn(et_f, coords);
                    _astro_linear_update(ASTRO_BODY_SATURN, et_f, coords);
                }
#else
                vsop87a_milli_getSaturn(et_f, coords);
#endif
            }
             break;
        case ASTRO_BODY_EMB:
             vsop87a_milli_getEmb(et_f, coords);
             break;
        case ASTRO_BODY_MOON:
            _astro_get_moon_coordinates(et_f, coords);
            break;
    }

    retval.x = coords[0];
    retval.y = coords[1];
    retval.z = coords[2];

    return retval;
}

astro_cartesian_coordinates_t astro_get_body_coordinates_light_time_adjusted(astro_body_t body, astro_cartesian_coordinates_t origin, float t) {
    //Get current position of body
    astro_cartesian_coordinates_t body_coords = astro_get_body_coordinates(body, t);

    float newT = t;

    for(uint8_t i = 0 ; i < 2 ; i++) {
        //Calculate light time to body
        body_coords = astro_subtract_cartesian(body_coords, origin);
        float distance = sqrt(body_coords.x*body_coords.x + body_coords.y*body_coords.y + body_coords.z*body_coords.z);
        distance *= 1.496e+11; //Convert from AU to meters
        float lightTime = distance / 299792458.0;

        //Convert light time to Julian Millenia, and subtract it from the original value of t
        newT -= lightTime / 24.0 / 60.0 / 60.0 / 365250.0;
        //Recalculate body position adjusted for light time
        body_coords = astro_get_body_coordinates(body, newT);
    }

    return body_coords;
}

astro_horizontal_coordinates_t astro_ra_dec_to_alt_az(double jd, float lat, float lon, float ra, float dec) {
    float GMST = (float)(astro_get_GMST(jd) * M_PI/180.0 * 15.0);
    float h = GMST + lon - ra;

    float sina = astro_sinf((float)(dec))*astro_sinf((float)(lat)) + astro_cosf((float)(dec))*astro_cosf((float)(h))*astro_cosf((float)(lat));
    float a = asin(sina);

    float cosAz = (astro_sinf((float)(dec))*astro_cosf((float)(lat)) - astro_cosf((float)(dec))*astro_cosf((float)(h))*astro_sinf((float)(lat))) / astro_cosf((float)(a));
    float Az = acos(cosAz);

    if(astro_sinf((float)(h)) > 0) Az = 2.0*M_PI - Az;

    astro_horizontal_coordinates_t retval;
    retval.altitude = a;
    retval.azimuth = Az;

    return retval;
}

float astro_degrees_to_radians(float degrees) {
    return degrees * M_PI / 180;
}

float astro_radians_to_degrees(float radians) {
    return radians * 180.0 / M_PI;
}

astro_angle_dms_t astro_radians_to_dms(float radians) {
    astro_angle_dms_t retval;
    int8_t sign = (radians < 0) ? -1 : 1;
    float degrees = fabs(astro_radians_to_degrees(radians));

    retval.degrees = (uint16_t)degrees;
    float temp = 60.0 * (degrees - retval.degrees);
    retval.minutes = (uint8_t)temp;
    retval.seconds = (uint8_t)round(60.0 * (temp - retval.minutes));

    if (retval.seconds > 59) {
        retval.seconds = 0.0;
        retval.minutes++;
    }

    if (retval.minutes > 59) {
        retval.minutes = 0;
        retval.degrees++;
    }

    degrees *= sign;

    return retval;
}

astro_angle_hms_t astro_radians_to_hms(float radians) {
    astro_angle_hms_t retval;
    float degrees = astro_radians_to_degrees(radians);
    float temp = degrees / 15.0;

    retval.hours = (uint8_t)temp;
    temp = 60.0 * (temp - retval.hours);
    retval.minutes = (uint8_t)temp;
    retval.seconds = (uint8_t)round(60.0 * (temp - retval.minutes));

    if (retval.seconds > 59) {
        retval.seconds = 0;
        retval.minutes++;
    }

    if (retval.minutes > 59) {
        retval.minutes = 0;
        retval.hours++;
    }

    return retval;
}
