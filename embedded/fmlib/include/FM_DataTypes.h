/*****************************************************************************
 *                                                                           *
 *                       Sensor Platforms Inc.                               *
 *                   2860 Zanker Road, Suite 210                             *
 *                        San Jose, CA 95134                                 *
 *                                                                           *
 *****************************************************************************
 *                                                                           *
 *               Copyright (c) 2012 Sensor Platforms Inc.                    *
 *                       All Rights Reserved                                 *
 *                                                                           *
 *                   Proprietary and Confidential                            *
 *             Use only under license described in EULA.txt                  *
 *                                                                           *
 ****************************************************************************/
/*! \file                                                                    *
 *                                                                           *
 *  @author Sensor Platforms Inc: http://www.sensorplatforms.com             *
 *  @author        Support Email: support@sensorplatforms.com                *
 *
 *  \ingroup core
 *
 *  \brief Define portable data types to be used by all SPI library code
 *
 ****************************************************************************/
#ifndef FM_DATATYPES_H
#define FM_DATATYPES_H

/* We shall use STDINT types such as uint32_t, int8_t, etc. for integer types in all of our code.
 Since FM_Platform.h file is included in all the library modules these typedefs and defines have
 been moved from FM_EmbeddedTypes.h */
#include <stdint.h>

#ifndef TRUE
# define TRUE 1
#endif

#ifndef FALSE
# define FALSE (!TRUE)
#endif

/* Other types that are not provided by stdint */
typedef char            fm_char_t;
/* Note that char cannot be replaced by int8_t or uint8_t as most compiler treats
   char, signed char and unsigned char as different because char itself maybe signed or unsigned */
typedef unsigned char   fm_byte_t;
typedef double          fm_dbl_t;
typedef float           fm_float_t;

#ifdef __cplusplus
  typedef bool          fm_bool_t;
#else
  typedef char          fm_bool_t;
#endif
typedef unsigned short fm_size_t;
typedef int fm_status_t;

#endif //FM_DATATYPES_H

