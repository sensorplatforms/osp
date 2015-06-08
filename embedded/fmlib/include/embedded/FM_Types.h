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
 *                                                                           *
 ****************************************************************************/

#ifndef FM_TYPES_H
#define FM_TYPES_H

#include "FM_DataTypes.h"
//#include "core/FixedPointTypes.h"
#include "osp-fixedpoint-types.h"
typedef char FM_STATUS_t;

#define FM_STATUS_IDLE                           ((FM_STATUS_t)  1)
#define FM_STATUS_OK                             ((FM_STATUS_t)  0)
#define FM_STATUS_UNSPECIFIED_ERROR              ((FM_STATUS_t) -1)
#define FM_STATUS_UNKNOWN_INPUT                  ((FM_STATUS_t) -2)
#define FM_STATUS_UNKNOWN_REQUEST                ((FM_STATUS_t) -3)
#define FM_STATUS_RESULT_NOT_POSSIBLE            ((FM_STATUS_t) -4)
#define FM_STATUS_INVALID_TIMESTAMP              ((FM_STATUS_t) -5)
#define FM_STATUS_BUFFER_TOO_SMALL               ((FM_STATUS_t) -6)
#define FM_STATUS_STORED_CAL_NOT_VALID           ((FM_STATUS_t) -7)
#define FM_STATUS_QUEUE_FULL                     ((FM_STATUS_t) -8)
#define FM_STATUS_ALREADY_SUBSCRIBED             ((FM_STATUS_t) -9)
#define FM_STATUS_NOT_SUBSCRIBED                 ((FM_STATUS_t) -10)
#define FM_STATUS_SENSOR_IN_USE                  ((FM_STATUS_t) -11)
#define FM_STATUS_SENSOR_ALREADY_REGISTERED      ((FM_STATUS_t) -12)
#define FM_STATUS_SENSOR_NOT_REGISTERED          ((FM_STATUS_t) -13)
#define FM_STATUS_SENSOR_INVALID_DESCRIPTOR      ((FM_STATUS_t) -14)
#define FM_STATUS_SENSOR_INVALID_TYPE            ((FM_STATUS_t) -15)
#define FM_STATUS_SENSOR_UNSUPPORTED_SAMPLE_RATE ((FM_STATUS_t)-16)
#define FM_STATUS_RESULT_IN_USE                  ((FM_STATUS_t) -17)
#define FM_STATUS_RESULT_INVALID_DESCRIPTOR      ((FM_STATUS_t) -18)
#define FM_STATUS_NO_MORE_HANDLES                ((FM_STATUS_t) -19)
#define FM_STATUS_NULL_POINTER                   ((FM_STATUS_t) -20)
#define FM_STATUS_INVALID_HANDLE                 ((FM_STATUS_t) -21)
#define FM_STATUS_SYSTEM_INVALID_DESCRIPTOR      ((FM_STATUS_t) -22)

#define FM_SENSOR_TYPE_ACCELEROMETER              1
#define FM_SENSOR_TYPE_MAGNETIC_FIELD             2
#define FM_SENSOR_TYPE_GYROSCOPE                  3
#define FM_SENSOR_TYPE_ORIENTATION                4
#define FM_SENSOR_TYPE_LIGHT                      5
#define FM_SENSOR_TYPE_PRESSURE                   6
#define FM_SENSOR_TYPE_TEMPERATURE                7
#define FM_SENSOR_TYPE_PROXIMITY                  8
#define FM_SENSOR_TYPE_GRAVITY                    9
#define FM_SENSOR_TYPE_LINEAR_ACCELERATION       10
#define FM_SENSOR_TYPE_ROTATION_VECTOR           11
#define FM_SENSOR_TYPE_RELATIVE_HUMIDITY         12

#define FM_RESULT_TYPE_INCLINOMETER             (1<<1)
#define FM_RESULT_TYPE_ROTATION_VECTOR          (1<<2)
#define FM_RESULT_TYPE_CALIBRATED_MAGNETOMETER  (1<<3)
#define FM_RESULT_TYPE_MAG_FIELD_ANOMALY        (1<<4)
#define FM_RESULT_TYPE_ROTATION_MATRIX          (1<<5)
#define FM_RESULT_TYPE_COMPASS                  (1<<6)
#define FM_RESULT_TYPE_WIN8_COMPASS             (1<<7)
#define FM_RESULT_TYPE_MOTION_CONTEXT           (1<<8)

#define FM_CONSTRAINT_POWER                     (1<<1)
#define FM_CONSTRAINT_PERFORMANCE               (1<<2)
#define FM_CONSTRAINT_FORGO_CALIBRATION         (1<<3)

#define FM_NUMBER_OF_CALIBRATION_TYPES          (4)

//This must always start at 0, as we are using these for indices
typedef enum {
       CalTypeAccelerometer = 0,
       CalTypeMagnetometer = 1,
       CalTypeGyroscope = 2,
       CalTypeGyroscopeExternal = 3
} EnumCalType_t;

typedef enum {
       CalibratorDefault = 0,
       CalibratorInit = 1,
       CalibratorRegular = 2,
       CalibratorPremium = 3
} EnumCalSource_t;

typedef struct {
    NTPRECISE delayInSeconds;
    NTPRECISE noise[3]; //always as NTPRECISE, and in units of the sensor
    NTEXTENDED saturationlimits_max[3];
    NTEXTENDED saturationlimits_min[3];
    NTPRECISE factoryskr[3][3]; //factory cal
    int32_t factoryoffset[3];  //factory cal
    NT nonlineareffects[4][3]; //could be nonlinear calibration, or other means of dealing with sensor nonlinearities
    NTPRECISE biasStability[3]; //how badly the bias wanders of time
    NTPRECISE repeatability[3];
    NT tempco[3][2]; //2 temperature coefficients on each axis
    NT shake[3]; //g^2 effects on gyroscopes
    NTEXTENDED expectednorm; //40 uT for magnetometer, 9.805 for accel, and 0 for gyro

    int8_t sensortype;
} FM_SensorProperties_t ;

typedef enum {
     CommandTurnOnSensor = 1,
     CommandTurnOffSensor = 2,
} EnumResourceCommand_t ;


typedef void (* FM_NotificationCallback_t)(void * obj);
typedef void (* FM_NotificationWithDataCallback_t)(void * obj, void * extradata);
typedef void (* FM_ResourceCommandCallback_t)(void * obj, uint32_t sensorid, EnumResourceCommand_t command);


#endif // FM_TYPES_H
