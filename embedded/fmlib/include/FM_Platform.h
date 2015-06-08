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
 *  \brief redefine LOGI and other target platform specifics so they integrate more naturally
 *
 *   For example: on linux you might want LOGE to go to stderr and LOGI to
 *   stdout, but on Android you want them both to go to LogCat.
 *
 ****************************************************************************/
#ifndef FM_PLATFORM_H
#define FM_PLATFORM_H

#include "FM_DataTypes.h"
#include "FM_Constants.h"

/* Calibration matrix storage type. Note - must occur before the platform specific
   include files */
typedef struct {
    uint32_t magicSize;
    fm_float_t magSkr[3][3];
    fm_float_t magOffset[3];
    fm_float_t magCalQuality[2];
    fm_float_t xlSkr[3][3];
    fm_float_t xlOffset[3];
    fm_float_t gyrSkr[3][3];
    fm_float_t gyrOffset[3];
    uint16_t magCalSource;
    uint32_t checksum;
} StoredCalData_t;

#define NVM_CHECK_MAGIC_NUMBER 0xa5

#define USE_GYRO_SCALE_CALIBRATION



#include "FM_Platform_lpc.h"   

/** Variable argument FM_ASSERT implementation using macros */
#define VA_NUM_ARGS(...)                        VA_NUM_ARGS_IMPL(__VA_ARGS__, 5,4,3,2,1)
#define VA_NUM_ARGS_IMPL(_1,_2,_3,_4,_5,N,...)  N

#define macro_dispatcher(func, ...)     macro_dispatcher_(func, VA_NUM_ARGS(__VA_ARGS__))
#define macro_dispatcher_(func, nargs)  macro_dispatcher__(func, nargs)
#define macro_dispatcher__(func, nargs) func ## nargs

#ifndef FM_ASSERT
#define FM_ASSERT(...) macro_dispatcher(FM_ASSERT, __VA_ARGS__)(__VA_ARGS__)
#endif
/* Now we just need to define the macros conrresponding to the number of arguments expected:
#define FM_ASSERT1(condition)      <macro definition here>
#define FM_ASSERT2(condition, msg) <macro definition here>
*/


#ifndef FM_ASSERT1
# define FM_ASSERT1(x) assert(x)
#endif

#ifndef FM_ASSERT2
# define FM_ASSERT2(cond, msg)       \
    {                                \
        if(!(cond)){ LOG_Err(msg); } \
        assert(cond);                \
    }
#endif


#ifndef DEFAULT_GYROSCOPE_NOISE
# define DEFAULT_GYROSCOPE_NOISE        (0.0079f)
#ifndef FM_PLATFORM_LINUX
# warning DEFAULT_GYROSCOPE_NOISE not defined for the platform, using default values
#endif
#endif

#ifndef DEFAULT_MAGNETOMETER_NOISE
# define DEFAULT_MAGNETOMETER_NOISE     (2.5f)
#ifndef FM_PLATFORM_LINUX
# warning DEFAULT_MAGNETOMETER_NOISE not defined for the platform, using default values
#endif
#endif

#ifndef DEFAULT_GYROSCOPE_SCALE_NOISE
# define DEFAULT_GYROSCOPE_SCALE_NOISE  (0.05f)
#ifndef FM_PLATFORM_LINUX
# warning DEFAULT_GYROSCOPE_SCALE_NOISE not defined for the platform, using default values
#endif
#endif

#ifndef DEFAULT_ACCELEROMETER_NOISE
# define DEFAULT_ACCELEROMETER_NOISE    (1.0f/M_SI_EARTH_GRAVITY)
#ifndef FM_PLATFORM_LINUX
# warning DEFAULT_ACCELEROMETER_NOISE not defined for the platform, using default values
#endif
#endif

// Define expected field norm
#ifndef EXPECTED_MAG_NORM
#define EXPECTED_MAG_NORM              (50.0f)
#ifndef FM_PLATFORM_LINUX
# warning EXPECTED_MAG_NORM not defined for the platform, using default values
#endif
#endif

// Define saturation limits if not defined
#ifndef GYROSCOPE_MAX_RANGE
#define GYROSCOPE_MAX_RANGE             (34.9)  //rad/sec (2000dps)
#endif

#ifndef ACCELEROMETER_MAX_RANGE
#define ACCELEROMETER_MAX_RANGE         (50.0)  //m/s^2
#endif

#ifndef MAGNETOMETER_MAX_RANGE
#define MAGNETOMETER_MAX_RANGE          (2000)  //uT
#endif

#ifndef DEFAULT_GYRO_BIAS_STABILITY
#define DEFAULT_GYRO_BIAS_STABILITY          (1e-5f)  //dps / root second
#endif


#endif //FM_PLATFORM_H
