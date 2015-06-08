/****************************************************************************************************
 *                                                                                                  *
 *                                    Sensor Platforms Inc.                                         *
 *                                    2860 Zanker Road, Suite 210                                   *
 *                                    San Jose, CA 95134                                            *
 *                                                                                                  *
 ****************************************************************************************************
 *                                                                                                  *
 *                                Copyright (c) 2012 Sensor Platforms Inc.                          *
 *                                        All Rights Reserved                                       *
 *                                                                                                  *
 ***************************************************************************************************/
/**
 * @file spi-sensor-hub-priv.h
 * Definitions and declarations for I2C Slave interface for Sensor Hub functionality
 *
 ***************************************************************************************************/
#if !defined (__SPI_SPI_SH_SENSOR_ID_HUB_PRIV_H__)
#define   __SPI_SPI_SH_SENSOR_ID_HUB_PRIV_H__

#ifndef __KERNEL__
#include <stdint.h>
#endif

/*-------------------------------------------------------------------------------------------------*\
 |    I N C L U D E   F I L E S
\*-------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------*\
 |    C O N S T A N T S   &   M A C R O S
\*-------------------------------------------------------------------------------------------------*/
#ifdef __KERNEL__
#   define SPI_SH_SUSPEND_DELAY 100        /* msec suspend delay*/
#endif

#   define SPI_SH_SENSOR_ID_NO_DELTA_MASK 0x00
#   define SPI_SH_SENSOR_ID_DELTA_TIME4_MASK 0x80
#   define SPI_SH_SENSOR_ID_DELTA_TIME2_MASK 0x40
#   define SPI_SH_SENSOR_ID_DELTA_TIME1_MASK 0x20
#   define SPI_SH_SENSOR_ID_DELTA_DATA_MASK 0x10

#   define SPI_SH_SENSOR_ID_DELTA_TIME1_DATA_MASK ( \
		SPI_SH_SENSOR_ID_DELTA_TIME1_MASK | \
		SPI_SH_SENSOR_ID_DELTA_DATA_MASK)

#   define SPI_SH_SENSOR_ID_DELTA_TIME2_DATA_MASK ( \
		SPI_SH_SENSOR_ID_DELTA_TIME2_MASK | \
		SPI_SH_SENSOR_ID_DELTA_DATA_MASK)

#   define SPI_SH_SENSOR_ID_DELTA_TIME4_DATA_MASK ( \
		SPI_SH_SENSOR_ID_DELTA_TIME4_MASK | \
		SPI_SH_SENSOR_ID_DELTA_DATA_MASK)

#   define SPI_SH_SENSOR_ID_DELTA_TIME5_DATA_MASK ( \
		SPI_SH_SENSOR_ID_DELTA_DATA_MASK)

#   define SPI_SH_SENSOR_ID_DELTA_MASK ( \
		SPI_SH_SENSOR_ID_DELTA_TIME1_MASK | \
		SPI_SH_SENSOR_ID_DELTA_TIME2_MASK | \
		SPI_SH_SENSOR_ID_DELTA_TIME4_MASK | \
		SPI_SH_SENSOR_ID_DELTA_DATA_MASK)



enum SPI_SH_SENSOR_ID {
		SPI_SH_SENSOR_ID_FIRST = 0,

    SPI_SH_SENSOR_ID_ACCELEROMETER = SPI_SH_SENSOR_ID_FIRST, //!< calibrated accelerometer data
    SPI_SH_SENSOR_ID_ACCELEROMETER_UNCALIBRATED = 1, 		//!< uncalibrated accelerometer data
    SPI_SH_SENSOR_ID_MAGNETIC_FIELD = 2, 					//!< calibrated magnetometer data
    SPI_SH_SENSOR_ID_MAGNETIC_FIELD_UNCALIBRATED = 3, 		//!< uncalibrated magnetometer data
    SPI_SH_SENSOR_ID_GYROSCOPE = 4, 						//!< calibrated gyroscope data
    SPI_SH_SENSOR_ID_GYROSCOPE_UNCALIBRATED = 5, 			//!< uncalibrated gyroscope data
    SPI_SH_SENSOR_ID_LIGHT = 6, 							//!< light data
    SPI_SH_SENSOR_ID_PRESSURE = 7, 							//!< barometer pressure data
    SPI_SH_SENSOR_ID_PROXIMITY = 8, 						//!< proximity data
    SPI_SH_SENSOR_ID_RELATIVE_HUMIDITY = 9, 				//!< relative humidity data
    SPI_SH_SENSOR_ID_AMBIENT_TEMPERATURE = 10, 				//!< ambient temperature data
    SPI_SH_SENSOR_ID_GRAVITY = 11, 							//!< gravity part of acceleration in body frame
    SPI_SH_SENSOR_ID_LINEAR_ACCELERATION = 12, 				//!< dynamic acceleration
    SPI_SH_SENSOR_ID_ORIENTATION = 13, 						//!< yaw, pitch, roll (also use this for Win8 Inclinometer)
    SPI_SH_SENSOR_ID_AUG_REALITY_COMPASS = 14,				//!< heading which switches to aug-reality mode when camera towards horizon (Win8 compass)
    SPI_SH_SENSOR_ID_ROTATION_VECTOR = 15, 					//!< accel+mag+gyro quaternion
    SPI_SH_SENSOR_ID_GEOMAGNETIC_ROTATION_VECTOR = 16, 		//!< accel+mag quaternion
    SPI_SH_SENSOR_ID_GAME_ROTATION_VECTOR = 17,			 	//!< accel+gyro quaternion
    SPI_SH_SENSOR_ID_VIRTUAL_GYROSCOPE = 18, 				//!< virtual gyroscope data from accel+mag
    SPI_SH_SENSOR_ID_STEP_DETECTOR = 19, 					//!< precise time a step occured
    SPI_SH_SENSOR_ID_STEP_COUNTER = 20, 					//!< count of sensitive steps
    SPI_SH_SENSOR_ID_SENSITIVE_STEP_DETECTOR = 21, 			//!< precise time a sensitive step occured
    SPI_SH_SENSOR_ID_SENSITIVE_STEP_COUNTER = 22, 			//!< count of steps
    SPI_SH_SENSOR_ID_SIGNIFICANT_MOTION = 23,
    SPI_SH_SENSOR_ID_CONTEXT_DEVICE_MOTION = 24, 			//!< context of device relative to world frame
    SPI_SH_SENSOR_ID_CONTEXT_CARRY = 25, 					//!< context of device relative to user
    SPI_SH_SENSOR_ID_CONTEXT_POSTURE = 26, 					//!< context of user relative to world frame
    SPI_SH_SENSOR_ID_CONTEXT_TRANSPORT = 27, 				//!< context of environment relative to world frame
    SPI_SH_SENSOR_ID_CONTEXT_CHANGE_DETECTOR = 28,			//!< low compute trigger for seeing if context may have changed
    SPI_SH_SENSOR_ID_STEP_SEGMENT_DETECTOR = 29,			//!< low compute trigger for analyzing if step may have occured
    SPI_SH_SENSOR_ID_GESTURE_EVENT = 30, 					//!< gesture event such as a double-tap or shake
    SPI_SH_SENSOR_ID_NEED_CALIBRATION = 31, 				//!< boolean indication that the user should perform a calibration sequence
    SPI_SH_SENSOR_ID_MESSAGE = 32, 							//!< warnings from the library: e.g. excessive timestamp jitter
    SPI_SH_SENSOR_ID_RGB_LIGHT = 33, 						//!< RGB light data
    SPI_SH_SENSOR_ID_UV_LIGHT = 34, 						//!< UV light data
    SPI_SH_SENSOR_ID_HEART_RATE = 35, 						//!< heart-rate data
    SPI_SH_SENSOR_ID_BLOOD_OXYGEN_LEVEL = 36, 				//!< blood-oxygen level data
    SPI_SH_SENSOR_ID_SKIN_HYDRATION_LEVEL = 37, 			//!< skin-hydration level data
    SPI_SH_SENSOR_ID_BREATHALYZER = 38, 					//!< breathalyzer data

	SPI_SH_SENSOR_ID_COUNT	/* may not exceed SPI_SH_SENSOR_ID_TYPE_MASK */
};



/*-------------------------------------------------------------------------------------------------*\
 |    T Y P E   D E F I N I T I O N S
\*-------------------------------------------------------------------------------------------------*/

#define spi_pack __attribute__ ((__packed__))

#ifndef PC_EMULATOR 
#define SPI_SH_MAX_BROADCAST_BUFFER_SIZE 256
#else
#define SPI_SH_MAX_BROADCAST_BUFFER_SIZE 60 // Temporary workaround for USB I2C bridge limit
#endif

struct spi_pack TimeStamp40 {
	uint32_t    timeStamp32;
	uint8_t     timeStamp40;
};

uint64_t timestamp40_to_timestamp64(struct TimeStamp40 *timeStamp);

void timestamp40_minus_timestamp40(
	struct TimeStamp40 *timeStamp1,
	struct TimeStamp40 *timeStamp2,
	struct TimeStamp40 *timeStampResult);

void timestamp40_plus_timestamp8(
	struct TimeStamp40 *timeStamp1,
	int8_t timeStamp2,
	struct TimeStamp40 *timeStampResult);

void timestamp40_plus_timestamp16(
	struct TimeStamp40 *timeStamp1,
	int16_t timeStamp2,
	struct TimeStamp40 *timeStampResult);

void timestamp40_plus_timestamp32(
	struct TimeStamp40 *timeStamp1,
	int32_t timeStamp2,
	struct TimeStamp40 *timeStampResult);


struct spi_pack spi_sh_motion_sensor_broadcast_node {
	/*
	 * raw time stamp in sensor time capture ticks 
	 */
	struct TimeStamp40 timeStamp;
	int16_t Data[3];	/* Raw sensor data */
};

struct spi_pack spi_sh_motion_uncal_sensor_broadcast_node {
	/*
	 * raw time stamp in sensor time capture ticks 
	 */
	struct TimeStamp40 timeStamp;
	int16_t Data[3];	/* Raw sensor data */
	int16_t Bias[3];	/* Raw sensor bias */
};

struct spi_pack spi_sh_segment_broadcast_node {
	int64_t endTime;	/* in NTTIME  */
	int64_t duration;	/* in NTTIME  */
	uint8_t type;
};


struct spi_pack spi_sh_significant_motion_broadcast_node {
	struct TimeStamp40 timeStamp;
	unsigned char  significantMotionDetected;	/* bool */

};
 
struct spi_pack spi_sh_orientation_broadcast_node {
	/*
	 * raw time stamp in sensor time capture ticks 
	 */
	struct TimeStamp40 timeStamp;
	int32_t Data[3];	/* Raw sensor data in NTEXTENDED*/
};

struct spi_pack spi_sh_step_sensitive_sensor_broadcast {
	struct TimeStamp40 timeStamp;
    uint16_t numStepsTotal;
};

struct spi_pack sh_quaternion_data {
    /*
     * raw time stamp in sensor time capture ticks
     */
	struct TimeStamp40 timeStamp;
    int32_t W;	                 /* w/x/y/z/e_est Raw sensor data  in NTPRECISE*/
    int32_t X;  
    int32_t Y;	
    int32_t Z;
    int32_t E_EST;
} ;
struct spi_pack spi_sh_motion_sensor_broadcast_delta_time4_node {
	uint32_t timeStamp;	/* raw time stamp in sensor time capture ticks */
	int16_t Data[3];	/* Raw sensor data */
};

struct spi_pack spi_sh_motion_sensor_broadcast_delta_time2_node {
	uint16_t timeStamp;	/* raw time stamp in sensor time capture  ticks */
	int16_t Data[3];	/* Raw sensor data */
};

struct spi_pack spi_sh_motion_sensor_broadcast_delta_time1_node {
	uint8_t timeStamp;	/* raw time stamp in sensor time capture  ticks */
	int16_t Data[3];	/* Raw sensor data */
};

struct spi_pack spi_sh_motion_sensor_broadcast_delta_data_node {
	/*
	 * raw time stamp in sensor time capture  ticks 
	 */
	struct TimeStamp40 timeStamp;
	int8_t Data[3];		/* Raw sensor data */
};

struct spi_pack spi_sh_motion_sensor_broadcast_delta_time4_data_node {
	uint32_t timeStamp;	/* raw time stamp in sensor time capture  ticks */
	int8_t Data[3];		/* Raw sensor data */
};

struct spi_pack spi_sh_motion_sensor_broadcast_delta_time2_data_node {
	uint16_t timeStamp;	/* raw time stamp in sensor time capture  ticks */
	int8_t Data[3];		/* Raw sensor data */
};

struct spi_pack spi_sh_motion_sensor_broadcast_delta_time1_data_node {
	uint8_t timeStamp;	/* raw time stamp in sensor time capture  ticks */
	int8_t Data[3];		/* Raw sensor data */
};

 struct spi_pack spi_sh_sensor_broadcast_node {
	uint8_t sensorId;	/* enum SPI_SH_SENSOR_ID_ID */
    uint8_t compression;
	union spi_pack {
		struct spi_sh_motion_sensor_broadcast_node  sensorData;
		struct spi_sh_motion_uncal_sensor_broadcast_node uncal_sensorData;
		struct spi_sh_segment_broadcast_node        segmentData;
        struct sh_quaternion_data                   quaternionData;
        struct spi_sh_orientation_broadcast_node       orientationData;
        struct spi_sh_step_sensitive_sensor_broadcast  stepSensitiveData;
		struct spi_sh_significant_motion_broadcast_node significantMotionData;
	} data;
};

struct spi_pack spi_sh_sensor_broadcast_delta_time4_node {
	/*
	 * enum SPI_SH_SENSOR_ID_ID | SPI_SH_SENSOR_ID_DELTA_TIME4_MASK
	 */
	uint8_t sensorId;
	struct spi_sh_motion_sensor_broadcast_delta_time4_node data;
};

struct spi_pack spi_sh_sensor_broadcast_delta_time2_node {
	/*
	 * enum SPI_SH_SENSOR_ID_ID | SPI_SH_SENSOR_ID_DELTA_TIME2_MASK
	 */
	uint8_t sensorId;
	struct spi_sh_motion_sensor_broadcast_delta_time2_node data;
};

struct spi_pack spi_sh_sensor_broadcast_delta_time1_node {
	/*
	 * enum SPI_SH_SENSOR_ID_ID | SPI_SH_SENSOR_ID_DELTA_TIME1_MASK
	 */
	uint8_t sensorId;
	struct spi_sh_motion_sensor_broadcast_delta_time1_node data;
};


struct spi_pack spi_sh_sensor_broadcast_delta_data_node {
	/*
	 * enum SPI_SH_SENSOR_ID_ID | SPI_SH_SENSOR_ID_DELTA_DATA_MASK
	 */
	uint8_t sensorId;
	struct spi_sh_motion_sensor_broadcast_delta_data_node data;
};

struct spi_pack spi_sh_sensor_broadcast_delta_time4_data_node {
	/*
	 * enum SPI_SH_SENSOR_ID_ID | SPI_SH_SENSOR_ID_DELTA_DATA_MASK |
	 * SPI_SH_SENSOR_ID_DELTA_TIME4_MASK
	 */
	uint8_t sensorId;
	struct spi_sh_motion_sensor_broadcast_delta_time4_data_node data;
};

struct spi_pack spi_sh_sensor_broadcast_delta_time2_data_node {
	/*
	 * enum SPI_SH_SENSOR_ID_ID | SPI_SH_SENSOR_ID_DELTA_DATA_MASK |
	 * SPI_SH_SENSOR_ID_DELTA_TIME2_MASK
	 */
	uint8_t sensorId;
	struct spi_sh_motion_sensor_broadcast_delta_time2_data_node data;
};

struct spi_pack spi_sh_sensor_broadcast_delta_time1_data_node {
	/*
	 * enum SPI_SH_SENSOR_ID_ID |
	 SPI_SH_SENSOR_ID_DELTA_TIME1_MASK |
	 SPI_SH_SENSOR_ID_DELTA_DATA_MASK
	 */
	uint8_t sensorId;
	struct spi_sh_motion_sensor_broadcast_delta_time1_data_node data;
};

struct spi_pack ShCmdGetHeader_get_8bits_param_t {
    uint8_t param;
} ;

struct spi_pack ShCmdGetHeader_get_16bits_param_t {
    uint16_t param;
} ;

enum SPI_SH_HUB_COMMANDS {
    SPI_SH_GET_WHO_AM_I = 0x00,                 /* gets 8 bits Device ID */
    SPI_SH_GET_VERSION,                         /* gets 16 bits version number on following read */
    SPI_SH_RESET,
    
    /* there three commands most be gnerated */
    /* atomicly, in this sequence */
	SPI_SH_GET_BROADCAST_LENGTH,            /* gets 16 bit of broadcast length */
	SPI_SH_GET_BROADCAST_DATA,              /* gets as many bytes as broadcast length read */
    
} ;


struct spi_pack ShHubCmdHeader_t {
    uint8_t command;	/* enum SPI_SH_HUB_COMMANDS */
};

struct spi_pack ShHubCmdHeader_8bits_param_t {
    uint8_t command;	/* enum SPI_SH_HUB_COMMANDS */
    uint8_t param;
} ;


enum SPI_SH_SENSOR_COMMANDS {
	SPI_SH_SENSOR_SET_ENABLE = 0x20,
	SPI_SH_SENSOR_GET_ENABLE,

	SPI_SH_SENSOR_SET_DELAY,
	SPI_SH_SENSOR_GET_DELAY,

#   if defined TRANSMIT_CAL_TO_SH

	SPI_SH_SENSOR_SET_CALIBRATE,
	SPI_SH_SENSOR_GET_CALIBRATE,
#   endif

} ;



struct spi_pack ShSensorCmdHeader_t {
    uint8_t command;	/* enum SPI_SH_SENSOR_ID_COMMANDS */
    uint8_t sensorId;	/* enum SPI_SH_SENSOR_ID_ID */
} ;

struct spi_pack ShSensorSetCmdHeader_8bits_param_t {
    uint8_t command;	/* enum SPI_SH_SENSOR_ID_COMMANDS */
    uint8_t sensorId;	/* enum SPI_SH_SENSOR_ID_ID */
    uint8_t param;
} ;

struct spi_pack ShSensorSetCmdHeader_16bits_param_t {
    uint8_t command;	/* enum SPI_SH_SENSOR_ID_COMMANDS */
    uint8_t sensorId;	/* enum SPI_SH_SENSOR_ID_ID */
    uint16_t param;
} ;


union spi_pack ShCmdHeaderUnion{
    struct ShSensorCmdHeader_t command;
    struct ShSensorSetCmdHeader_8bits_param_t sensor_cmd_8bits_param;
    struct ShSensorSetCmdHeader_16bits_param_t sensor_cmd_16bits_param;
    struct ShHubCmdHeader_t hubCmdHeader;
    struct ShHubCmdHeader_8bits_param_t hub_cmd_8bits_param;
} ;



/*-------------------------------------------------------------------------------------------------*\
 |    E X T E R N A L   V A R I A B L E S   &   F U N C T I O N S
\*-------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------*\
 |    P U B L I C   V A R I A B L E S   D E F I N I T I O N S
\*-------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------*\
 |    P U B L I C   F U N C T I O N   D E C L A R A T I O N S
\*-------------------------------------------------------------------------------------------------*/


#endif /* __SPI_SPI_SH_SENSOR_ID_HUB_PRIV_H__ */
/*-------------------------------------------------------------------------------------------------*\
 |    E N D   O F   F I L E
\*-------------------------------------------------------------------------------------------------*/

