#ifndef _FIXEDPOINTTYPES_H_
#define _FIXEDPOINTTYPES_H_

#include "FM_DataTypes.h"

#ifdef __cplusplus
extern "C" {
#endif


typedef int16_t NT; /* 16 bit, Q12 fixed point! */
typedef int32_t NTPRECISE; /* 32 bit, Q24 fixed point */
typedef int32_t NTDELTATIME; /* 32 bit, Q24 fixed point.  Useful when you're just working with time deltas and only want to use a 32 bit number  */
typedef int32_t NTEXTENDED; /* 32 bit, Q12 fixed point */
typedef uint32_t TIMECOEFFICIENT; /* U32Q32 which is a coefficient to convert counter tics to NTTIME */

typedef int64_t NTTIME; /* 64 bit, Q24 fixed point */

/* for fixed point
 *
 *  Fixed point = S16Q12
 *  Fixed point precise = S32Q24
 *  Fixed point time = S64Q24
 */

#define K_NUM_FIXED_POINT_BITS (16)
#define QFIXEDPOINT 12
#define QFIXEDPOINTPRECISE 24
#define QFIXEDPOINTEXTENDED 12
#define QFIXEDPOINTTIME 24
#define QFIXEDPOINTDELTATIME 24


/*
 *  Conversion between different formats
 *
 */

/* fm_float_t to arbitrary fixed point number */
#define TOFIX_CUSTOM(x,q)  ((int32_t)(((fm_float_t)x) * (fm_float_t)(1 << (q)) ))

/* fm_float_t to fixed point */
#define CONST(x)  ((NT)(((fm_float_t)x) * (fm_float_t)(1UL << QFIXEDPOINT) ))
#define TOFIX(x)  ((NT)(((fm_float_t)x) * (fm_float_t)(1UL << QFIXEDPOINT) ))

/* fm_float_t to fixed point precise */
#define CONST_PRECISE(x) ((NTPRECISE)(((fm_float_t)x) * (fm_float_t)(1UL << QFIXEDPOINTPRECISE) ))
#define TOFIX_PRECISE(x) ((NTPRECISE)(((fm_float_t)x) * (fm_float_t)(1UL << QFIXEDPOINTPRECISE) ))


/* fm_float_t to fixed point time */
#define CONST_TIME(x) ((NTTIME)(((fm_dbl_t)x) * (fm_dbl_t)(1UL << QFIXEDPOINTTIME)))
#define TOFIX_TIME(x) ((NTTIME)(((fm_dbl_t)x) * (fm_dbl_t)(1UL << QFIXEDPOINTTIME)))

/* Converion from float to time coefficient, valid range is between 0 and 0.999999 */
#define TOFIX_TIMECOEFFICIENT(x) ((uint32_t)(((fm_dbl_t)x) * (fm_dbl_t)(4294967296.0) ))

/* fm_float_t to fixed point delta time */
#define CONST_DELTATIME(x) ((NTDELTATIME)(((fm_float_t)x) * (fm_float_t)(1UL << QFIXEDPOINTDELTATIME)))
#define TOFIX_DELTATIME(x) ((NTDELTATIME)(((fm_float_t)x) * (fm_float_t)(1UL << QFIXEDPOINTDELTATIME)))

/*fm_float_t to fixed point extended */
#define CONST_EXTENDED(x) ((NTEXTENDED)(((fm_float_t)x) * (fm_float_t)(1UL << QFIXEDPOINTEXTENDED) ))
#define TOFIX_EXTENDED(x) ((NTEXTENDED)(((fm_float_t)x) * (fm_float_t)(1UL << QFIXEDPOINTEXTENDED) ))

/* fixed point to fixed point precise  */
#define NT_TO_NTPRECISE(A) (((NTPRECISE)(A) << (QFIXEDPOINTPRECISE-QFIXEDPOINT)))

/* fixed point precise to fixed point */
#define NTPRECISE_TO_NT(A) ((NT)(((NTPRECISE)(A) + FIXEDPOINT_ROUNDING_VALUE) >> (QFIXEDPOINTPRECISE-QFIXEDPOINT)))

/* fixed point to fixed point time */
#define NT_TO_NTTIME(A) ((NTPRECISE)(A) << (QFIXEDPOINTTIME - QFIXEDPOINT))

/* fixed point time to fixed point  */
#define NTTIME_TO_NT(A) ((NT)(((A) + FIXEDPOINT_ROUNDING_VALUE) >> (QFIXEDPOINTTIME - QFIXEDPOINT)))

/* fixed point time to fixed point extended */
#define NTTIME_TO_NTEXTENDED(A) ((NTEXTENDED)(((A) + FIXEDPOINT_ROUNDING_VALUE) >> (QFIXEDPOINTTIME - QFIXEDPOINTEXTENDED)))

/* fixed point precise to fixed point time */
#define NTPRECISE_TO_NTTIME(A) ((NTPRECISE)((A) >> (QFIXEDPOINTPRECISE-QFIXEDPOINTTIME)))

/* fixed point time to fixed point precise */
#define NTTIME_TO_NTPRECISE(A) ((NTPRECISE)((A) << (QFIXEDPOINTPRECISE-QFIXEDPOINTTIME)))

/*fixed point extended to fixed point precise*/
#define NTEXTENDED_TO_NTPRECISE(A) ((NTPRECISE)((A) << (QFIXEDPOINTPRECISE-QFIXEDPOINTEXTENDED)))

/* fixed point precise to fixed point extended*/
#define NTPRECISE_TO_NTEXTENDED(A) ((NTEXTENDED)((A) + FIXEDPOINT_ROUNDING_VALUE) >> (QFIXEDPOINTPRECISE-QFIXEDPOINTEXTENDED))

/*fixed point extended to fixed point precise*/
#define NTEXTENDED_TO_NT(A) ((NT)((A) << (QFIXEDPOINT-QFIXEDPOINTEXTENDED)))

/*fixed point extended to fixed point precise*/
#define NT_TO_NTEXTENDED(A) ((NTEXTENDED)((A))  >> (QFIXEDPOINT-QFIXEDPOINTEXTENDED))

/*fixed point delta time to fixed point time  */
#define NTDELTATIME_TO_NTTIME(A) ((NTTIME)((A) << (QFIXEDPOINTTIME-QFIXEDPOINTDELTATIME)))

#define NTTIME_TO_NTDELTATIME(A) ((NTDELTATIME)((A) >> (QFIXEDPOINTTIME-QFIXEDPOINTDELTATIME)))

/*fixed point number of arbitrary q to a floating point number  */
#define TOFLT_CUSTOM(x,q) ((fm_float_t)(x) / (fm_float_t)(1UL << (q)))

/* to floating point from fixed point */
#define TOFLT(x) ((fm_float_t)(x) / (fm_float_t)(1 << QFIXEDPOINT))

/* to floating point from fixed point precise */
#define TOFLT_PRECISE(x) ((fm_float_t)(x) / (fm_float_t)(1UL << QFIXEDPOINTPRECISE))

/* to floating point from fixed point time */
#define TOFLT_TIME(x) ((fm_dbl_t)(x) / (fm_dbl_t)(1UL << QFIXEDPOINTTIME))

/* to floating point from fixed point delta time */
#define TOFLT_DELTATIME(x) ((fm_float_t)(x) / (fm_float_t)(1UL << QFIXEDPOINTDELTATIME))

/*to floating point from fixed point extended */
#define TOFLT_EXTENDED(x) ((fm_float_t)(x) / (fm_float_t)(1UL << QFIXEDPOINTEXTENDED))

/*  int to fixed point conversion */ 
#define INT_TO_PRECISE(x)  ( (int32_t) (x) << QFIXEDPOINTPRECISE )
#define INT_TO_NT(x)       ( (int32_t) (x) << QFIXEDPOINT)
#define INT_TO_EXTENDED(x) ( (int32_t) (x) << QFIXEDPOINTEXTENDED )

/*  fixed point to int conversion (no rounding, returns integer portion) */ 
#define PRECISE_TO_INT(x)  ( (int32_t) (x) >> QFIXEDPOINTPRECISE )
#define NT_TO_INT(x)       ( (int32_t) (x) >> QFIXEDPOINT)
#define EXTENDED_TO_INT(x) ( (int32_t) (x) >> QFIXEDPOINTEXTENDED )

/*-------------------------------------------------------------------------*/


#ifdef __cplusplus
}
#endif

#endif
