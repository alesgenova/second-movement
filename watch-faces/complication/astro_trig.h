#pragma once
#include <math.h>
#include <stdint.h>

/*
 * Fast cosine approximation using a minimax polynomial on [0, pi/2]
 * with range reduction via symmetry.
 *
 * ASTRO_COS_TERMS controls the number of polynomial terms (2-6):
 *   2 terms: very coarse
 *   3 terms: coarse
 *   4 terms: max error ~8.9e-4
 *   5 terms: max error ~2.5e-5
 *   6 terms: max error ~1.8e-7 (near float32 precision)
 *
 * Default is 5. Override by defining ASTRO_COS_TERMS before including this header.
 */
#ifndef ASTRO_COS_TERMS
#define ASTRO_COS_TERMS 5
#endif

#if (ASTRO_COS_TERMS < 2) || (ASTRO_COS_TERMS > 6)
#error "ASTRO_COS_TERMS must be an integer in [2, 6]"
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* Shared range reduction: maps x to [0, pi/2], returns sign for cosine symmetry. */
static inline int8_t astro_cosf_reduce_(float *x) {
    *x = *x - (float)(2.0 * M_PI) * floorf(*x * (float)(1.0 / (2.0 * M_PI)));  /* [0, 2pi) */

    int8_t sign = 1;
    if (*x > (float)M_PI) {
        *x = (float)(2.0 * M_PI) - *x;  /* [0, pi] */
    }
    if (*x > (float)(M_PI / 2.0)) {
        *x = (float)M_PI - *x;          /* [0, pi/2] */
        sign = -1;
    }
    return sign;
}

#define ASTRO_TERM_(min_terms, coeff, terms) (((terms) >= (min_terms)) ? (coeff) : 0.0f)

#define DEFINE_ASTRO_COSF_CN_(N) \
static inline float astro_cosf_C##N(float x) { \
    int8_t sign = astro_cosf_reduce_(&x); \
    float x2 = x * x; \
    float r = 1.0f + x2 * ( \
        ASTRO_TERM_(2, -0.4999999963f,   N) + x2 * ( \
        ASTRO_TERM_(3,  0.416666418e-1f, N) + x2 * ( \
        ASTRO_TERM_(4, -0.13888397e-2f,  N) + x2 * ( \
        ASTRO_TERM_(5,  0.24801587e-4f,  N) + x2 * ( \
        ASTRO_TERM_(6, -0.275573e-6f,    N) \
    ))))); \
    return sign * r; \
}

#define DEFINE_ASTRO_SINF_CN_(N) \
static inline float astro_sinf_C##N(float x) { \
    return astro_cosf_C##N(x - (float)(M_PI / 2.0)); \
}

DEFINE_ASTRO_COSF_CN_(2)
DEFINE_ASTRO_COSF_CN_(3)
DEFINE_ASTRO_COSF_CN_(4)
DEFINE_ASTRO_COSF_CN_(5)
DEFINE_ASTRO_COSF_CN_(6)

DEFINE_ASTRO_SINF_CN_(2)
DEFINE_ASTRO_SINF_CN_(3)
DEFINE_ASTRO_SINF_CN_(4)
DEFINE_ASTRO_SINF_CN_(5)
DEFINE_ASTRO_SINF_CN_(6)

#define ASTRO_CAT2_(a, b) a##b
#define ASTRO_CAT2(a, b) ASTRO_CAT2_(a, b)

static inline float astro_cosf(float x) {
    return ASTRO_CAT2(astro_cosf_C, ASTRO_COS_TERMS)(x);
}

static inline float astro_sinf(float x) {
    return ASTRO_CAT2(astro_sinf_C, ASTRO_COS_TERMS)(x);
}

#undef ASTRO_CAT2
#undef ASTRO_CAT2_
#undef DEFINE_ASTRO_SINF_CN_
#undef DEFINE_ASTRO_COSF_CN_
#undef ASTRO_TERM_
