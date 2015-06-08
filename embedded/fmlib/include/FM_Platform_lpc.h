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
 *  \ingroup embedded
 *
 ****************************************************************************/
#ifndef FM_PLATFORM_LPC43XX_EVM_H
#define FM_PLATFORM_LPC43XX_EVM_H

#include <assert.h>

#define ACC_BMC150_I2C
#define GYRO_BMG160_I2C
#define MAG_BMC150_I2C



#ifdef FM_INCL_ACCEL_MAG_MOTION_CONTEXT
	#undef FM_INCL_ACCEL_MAG_MOTION_CONTEXT
#endif

#ifdef FM_INCL_ACCEL_MAG_EXTERNAL_MAG_CONTEXT
	#undef FM_INCL_ACCEL_MAG_EXTERNAL_MAG_CONTEXT
#endif

#ifdef FM_INCL_CONTEXT_CARRY
	#undef FM_INCL_CONTEXT_CARRY
#endif

#ifdef FM_INCL_CONTEXT_POSTURE
	#undef FM_INCL_CONTEXT_POSTURE
#endif

#ifdef FM_INCL_CONTEXT_POSTURE_SIT_STAND
#undef FM_INCL_CONTEXT_POSTURE_SIT_STAND
#endif

#define FM_INCL_SENSITIVE_STEP_DETECTOR
#define FM_INCL_SENSITIVE_STEP_COUNTER
#define FM_INCL_SIGNIFICANT_MOTION


#ifndef LIBRARY_VERSION
# define LIBRARY_VERSION 0x0
# define LIBRARY_VERSION_STRING "FM_Lib version 0.0.00.xxxxxx"
#endif

#define ENABLE_SPI_FIXED_POINT_FMLIB
#ifndef SIMULATOR
//# define ENABLE_FLASH_STORE
#endif

#ifdef RESET_ON_ASSERT
# define _SysRESET()         NVIC_GenerateSystemReset()
#else
# define _SysRESET()         while(1)
#endif


//Platform Customizations
#define FM_MODULE_MASK      0xFFFFFFFF

#define _AssertIndication()        /* Nothing for now */
#define _FlushUart()               /* Do nothing */

#define FM_UNUSED(x)        (void)x;

/* UART debug related */
#define ASSERT_MESG_SZ              200

/* Define the assert macros corresponding to the number of arguments expected */
#define FM_ASSERT1(condition)        \
    if (!(condition))                                                                      \
    {                                                                                      \
        extern char _assertMsgBuff[];                                                      \
        __disable_irq();                                                                   \
        _AssertIndication();                                                               \
        _FlushUart();                                                                      \
        snprintf(_assertMsgBuff, ASSERT_MESG_SZ, "ASSERT: %s(%d) - [%s]", __MODULE__,      \
            __LINE__, #condition);                                                         \
        printf("%s\r\n", _assertMsgBuff);                                                  \
        _SysRESET();                                                                       \
    }

#define FM_ASSERT2(condition, message)        \
    if (!(condition))                                                                      \
    {                                                                                      \
        extern char _assertMsgBuff[];                                                      \
        __disable_irq();                                                                   \
        _AssertIndication();                                                               \
        _FlushUart();                                                                      \
        snprintf(_assertMsgBuff, ASSERT_MESG_SZ, "ASSERT: %s(%d) - [%s], MSG:%.100s",      \
            __MODULE__, __LINE__, #condition, message);                                    \
        printf("%s\r\n", _assertMsgBuff);                                                  \
        _SysRESET();                                                                       \
    }

int _dprintf( uint8_t dbgLvl, const char *fmt, ...);
extern uint32_t g_logging;

/* Helper macros needed for logging */
#define LOG_Err_Helper(fmt, ...)    _dprintf(0, fmt " (%s :: %s)\r\n", __VA_ARGS__)
#define LOG_Info_Helper(fmt, ...)   {if (g_logging & 0x10) _dprintf(0, fmt " (MId:%08X, GId:%02X)\r\n", __VA_ARGS__);}


#ifndef FM_MODULE_ID
# define FM_MODULE_ID   FMID_PLATFORM
#endif


/* The following macros can also be defined as functions so to ensure that when defined as functions
   the default macro is not invoked, we MUST set the corresponding define to 1 (even if you have it
   implemented as a macro */
/* Macro to avoid warnings for unused vars */
#define M_MatrixUnusedFunc(x,y,z,a)    (void)(x);(void)(y);(void)(z);(void)(a);

#define SetDebugMatrix(x,y,z,a)         M_MatrixUnusedFunc(x,y,z,a)
#define _SetDebugMatrix_DEFINED_        1 /* MUST DO THIS FOR ALL DebugMatix functions */

#define SetDebugMatrixPrecise(x,y,z,a)  M_MatrixUnusedFunc(x,y,z,a)
#define _SetDebugMatrixPrecise_DEFINED_ 1 /* MUST DO THIS FOR ALL DebugMatix functions */

#define SetDebugMatrixShort(x,y,z,a)    M_MatrixUnusedFunc(x,y,z,a)
#define _SetDebugMatrixShort_DEFINED_   1 /* MUST DO THIS FOR ALL DebugMatix functions */

#define SetDebugMatrixTime(x,y,z,a)     M_MatrixUnusedFunc(x,y,z,a)
#define _SetDebugMatrixTime_DEFINED_    1 /* MUST DO THIS FOR ALL DebugMatix functions */

int32_t mul_precise( int32_t x, int32_t y );
#define MUL_PRECISE(x, y)               mul_precise((x), (y))

int16_t mul_precise_round( int32_t x, int32_t y );

#define MUL_PRECISE_ROUND(x, y) mul_precise_round( x, y )

int32_t FixedPoint32BitMult(int32_t x, int32_t y,int32_t q);
#define MUL_EXTENDED(A,B) (FixedPoint32BitMult((A),(B),QFIXEDPOINT))

#define STATS_RESET(x)
#define STATS_START(x)
#define STATS_STOP(x)
#define STATS_COLLECT(x,y)

#define DEFAULT_GYROSCOPE_NOISE        (0.007f)
#define DEFAULT_MAGNETOMETER_NOISE     (1.5f)
#define DEFAULT_GYROSCOPE_SCALE_NOISE  (0.05f)
#define DEFAULT_ACCELEROMETER_NOISE    (1.0f/M_SI_EARTH_GRAVITY)

#define EXPECTED_MAG_NORM              (50.0f)

/* Sensor specific data flags */
#define FLAG_FACTORY_MAG_CAL            0x0100

/* Time tick conversion constant */
/* !!!! WARNING !!!! This must match "US_PER_RTC_TICK" definition from hw_setup_xxxx.h file */
#define SYSTEM_SECONDS_PER_TICK         (0.000024f) /* !!!! WARNING !!!! */

/* The following is the default calibration matrix specific to NXP platform */
#ifdef _DEFINE_DEFAULT_CAL_
#define _DEFAULT_CAL_DEFINED_
fm_char_t _assertMsgBuff[ASSERT_MESG_SZ];
StoredCalData_t _calData = {
    (NVM_CHECK_MAGIC_NUMBER << 8) | sizeof(StoredCalData_t),
    {{1.0f, 0.0f, 0.0f},
    {0.0f, 1.0f, 0.0f},
    {0.0f, 0.0f, 1.0f}},
    {0.0f, 0.0f, 0.0f},
    {0.0f, 0.0f},
    {{1.0f, 0.0f, 0.0f},
    {0.0f, 1.0f, 0.0f},
    {0.0f, 0.0f, 1.0f}},
    {0.0f, 0.0f, 0.0f},
    {{1.0f, 0.0f, 0.0f},
    {0.0f, 1.0f, 0.0f},
    {0.0f, 0.0f, 1.0f}},
    {0.0f, 0.0f, 0.0f},
    (NVM_CHECK_MAGIC_NUMBER << 8 )| NVM_CHECK_MAGIC_NUMBER,
};
#endif //_DEFINE_DEFAULT_CAL_

#endif //FM_PLATFORM_LPC43XX_EVM_H
