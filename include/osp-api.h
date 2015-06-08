/* Open Sensor Platform Project
 * https://github.com/sensorplatforms/open-sensor-platform
 *
 * Copyright (C) 2013 Sensor Platforms Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef OSP_API_H__
#define OSP_API_H__

#ifdef __cplusplus
extern "C" {
#endif

/*-------------------------------------------------------------------------------------------------*\
 |    I N C L U D E   F I L E S
\*-------------------------------------------------------------------------------------------------*/
#include "osp-types.h"
#include "osp-sensors.h"
#include "osp-fixedpoint-types.h"

/*-------------------------------------------------------------------------------------------------*\
 |    C O N S T A N T S   &   M A C R O S
\*-------------------------------------------------------------------------------------------------*/
/// flags to pass into sensor descriptors
#define OSP_NO_SENSOR_CONTROL_CALLBACK  ((OSP_SensorControlCallback_t)NULL)
#define OSP_NO_NVM_WRITE_CALLBACK       ((OSP_WriteCalDataCallback_t)NULL)
#define OSP_NO_OUTPUT_READY_CALLBACK    ((OSP_OutputReadyCallback_t)NULL)
#define OSP_32BIT_DATA                  (0xFFFFFFFFL)
#define OSP_NO_OPTIONAL_DATA            ((void*)NULL)

/* Flags defining sensor attributes */
#define OSP_NO_FLAGS                    (0)
#define OSP_FLAGS_INPUT_SENSOR          (1 << 0)
#define OSP_FLAGS_UNCALIBRATED          (1 << 1)
#define OSP_FLAGS_CALIBRATED            (1 << 2)

/*-------------------------------------------------------------------------------------------------*\
 |    T Y P E   D E F I N I T I O N S
\*-------------------------------------------------------------------------------------------------*/

//! used to swap axes or conventions from sensor frame to body frame in a SensorDescriptor_t
/*!
 *  this is most often used:
 *  - when the different sensors on a board were not physically aligned to the same coordinate system
 *  - to convert from a left-handed system magnetometer into a right-handed system
 */
typedef enum {
    AXIS_MAP_UNUSED     = 0,
    AXIS_MAP_POSITIVE_X = 1,
    AXIS_MAP_NEGATIVE_X = 2,
    AXIS_MAP_POSITIVE_Y = 3,
    AXIS_MAP_NEGATIVE_Y = 4,
    AXIS_MAP_POSITIVE_Z = 5,
    AXIS_MAP_NEGATIVE_Z = 6,
    AXIS_MAP_ENUM_COUNT
} AxisMapType_t ;


//! handle type returned by OSP_RegisterInputSensor() 
typedef void* InputSensorHandle_t;            

//! handle type returned by OSP_SubscribeSensorResult() or OSP_UnsubscribeSensorResult()
typedef void* ResultHandle_t;

//! data passed back via OSP_SensorControlCallback_t to tell the sensor driver to change the operation of its physical sensors
typedef enum {
    SENSOR_CONTROL_SENSOR_OFF      = 0,         //!< turn off sensor
    SENSOR_CONTROL_SENSOR_SLEEP    = 1,         //!< put sensor in low power sleep mode w/ fast turn on
    SENSOR_CONTROL_SENSOR_ON       = 2,         //!< turn on sensor
    SENSOR_CONTROL_SET_SAMPLE_RATE = 3,         //!< sample sensor at new rate, Data = sample time in seconds, NTPRECISE
    SENSOR_CONTROL_SET_LPF_FREQ    = 4,         //!< set Low pass filter 3db cutoff frequency (in Hz) 0 = turn off filter
    SENSOR_CONTROL_SET_HPF_FREQ    = 5,         //!< set High pass filter 3db cutoff frequency (in Hz) 0 = turn off filter
    SENSOR_CONTROL_ENUM_COUNT
} SensorControlCommand_t;

//! how enable/disable/setDelay type commands and data are passed back to the sensor driver
typedef struct  {
#if (0)   // replace handle with actual InputSensor_t 	
    InputSensorHandle_t Handle;                 //!< handle that was returned from OSP_RegisterInputSensor()
#else 
	InputSensor_t  SensorIdx;                   //!< sensor enum value
#endif 
    SensorControlCommand_t Command;             //!< command to sensor (power on/off, change rate, etc...)
    int32_t Data;                               //!< as need and appropriate for each command: e.g. high pass frequency in Hz
} SensorControl_t;

// Gesture results

//! Used for all gesture results in conjunction with the enum GestureType_t
typedef struct {
    NTTIME TimeStamp;                  //!< Time in seconds
    NT *Probability;                   //!< Probability vector.  index into this with the appropriate GestureType_t enum
} GestureEventOutputData_t;

// Context results
//! Used for all context results. \sa ContextMotionType_t, ContextPostureType_t, ContextCarryType_t
typedef struct {
    NTTIME TimeStamp;                           //!< Time in seconds
    NT *Probability;                            //!< Probability vector. Num elements corresponds to each context type
} ContextResultData_t;


//! calibrated acceleration in m/s^2. note positive Z when flat on table.
typedef struct {
    NTTIME TimeStamp;                           //!< Time in seconds
    NTPRECISE X;                                //!< X axis 32Q24 fixed point data
    NTPRECISE Y;                                //!< Y axis 32Q24 fixed point data
    NTPRECISE Z;                                //!< Z axis 32Q24 fixed point data
} Android_CalibratedAccelResultData_t;

//! calibrated magnetometer in uT  +Y axis points out the top edge of the device
typedef struct {
    NTTIME TimeStamp;                           //!< Time in seconds
    NTEXTENDED X;                               //!< X axis 32Q12 fixed point data
    NTEXTENDED Y;                               //!< Y axis 32Q12 fixed point data
    NTEXTENDED Z;                               //!< Z axis 32Q12 fixed point data
} Android_CalibratedMagResultData_t;

//! calibrated rotation rate in rad/s right handed +Y axis points out the top edge of the device
typedef struct {
    NTTIME TimeStamp;                           //!< Time in seconds
    NTPRECISE X;                                //!< X axis 32Q24 fixed point data
    NTPRECISE Y;                                //!< Y axis 32Q24 fixed point data
    NTPRECISE Z;                                //!< Z axis 32Q24 fixed point data
} Android_CalibratedGyroResultData_t;


//! uncalibrated acceleration in m/s^2. note positive Z when flat on table.
typedef struct {
    NTTIME TimeStamp;              //!< Time in seconds
    NTPRECISE X;                   //!< 32Q24 fixed point data 
    NTPRECISE Y;                   //!< 32Q24 fixed point data 
    NTPRECISE Z;                   //!< 32Q24 fixed point data 
    NTPRECISE X_offset;            //!< 32Q24 fixed point data 
    NTPRECISE Y_offset;            //!< 32Q24 fixed point data 
    NTPRECISE Z_offset;            //!< 32Q24 fixed point data 
} Android_UncalibratedAccelResultData_t;

//! uncalibrated rotation rate in rad/s.  Note positive Z when spin counter-clockwise on the table (right handed).
typedef struct {
    NTTIME TimeStamp;              //!< Time in seconds
    NTPRECISE X;                   //!< 32Q24 fixed point data representing rad/sec.
    NTPRECISE Y;                   //!< 32Q24 fixed point data representing rad/sec.
    NTPRECISE Z;                   //!< 32Q24 fixed point data representing rad/sec.
    NTPRECISE X_drift_offset;      //!< 32Q24 fixed point data representing rad/sec.
    NTPRECISE Y_drift_offset;      //!< 32Q24 fixed point data representing rad/sec.
    NTPRECISE Z_drift_offset;      //!< 32Q24 fixed point data representing rad/sec.
} Android_UncalibratedGyroResultData_t;

//! uncalibrated magnetometer in uT.  Note positive Y when top edge points towards magnetic North.
typedef struct {
    NTTIME TimeStamp;               //!< Time in seconds
    NTEXTENDED X;                   //!< 32Q12 fixed point data representing uT.
    NTEXTENDED Y;                   //!< 32Q12 fixed point data representing uT.
    NTEXTENDED Z;                   //!< 32Q12 fixed point data representing uT.
    NTEXTENDED X_hardIron_offset;   //!< 32Q12 fixed point data representing uT
    NTEXTENDED Y_hardIron_offset;   //!< 32Q12 fixed point data representing uT
    NTEXTENDED Z_hardIron_offset;   //!< 32Q12 fixed point data representing uT
} Android_UncalibratedMagResultData_t;

//! yaw pitch roll
typedef struct {
    NTTIME TimeStamp;                           //!< Time in seconds
    NTEXTENDED Pitch;                           //!< pitch in degrees
    NTEXTENDED Roll;                            //!< roll in degrees
    NTEXTENDED Yaw;                             //!< yaw in degrees   
} Android_OrientationResultData_t;

//! as defined in the Android Sensors HAL 1.0
typedef struct {
    NTTIME TimeStamp;                     //!< Time in seconds
    osp_bool_t significantMotionDetected;  //!< always set to true, indicating a step was taken
} Android_SignificantMotionResultData_t;

//! indicates when each step is taken
typedef struct {
    NTTIME TimeStamp;              //!< Time in seconds
    osp_bool_t StepDetected;             //!< always set to true, indicating a step was taken
} Android_StepDetectorResultData_t;

//! Android style step counter, but note that host driver must bookkeep between 
//! sensorhub power on/off to to meet android requirement
typedef struct {
	NTTIME TimeStamp;                //!< Time in seconds
	uint32_t StepCount;              //!< steps since sensorhub power on.
                                     //!< this is an important distinction from 
                                     //!< full android requirement
} Android_StepCounterResultData_t; 

//! positive, normalized quaternion used for the various flavors of ROTATION_VECTOR
typedef struct {
    NTTIME TimeStamp;              //!< Time in seconds
    NTPRECISE X;                   //!< X component of normalized quaternion in 32Q24 fixed point
    NTPRECISE Y;                   //!< Y component of normalized quaternion in 32Q24 fixed point
    NTPRECISE Z;                   //!< Z component of normalized quaternion in 32Q24 fixed point
    NTPRECISE W;                   //!< W component of normalized quaternion in 32Q24 fixed point
    NTPRECISE ErrorEst;            //!< estimated heading Accuracy in radians in 32Q24 fixed point (-1 if unavailable)
} Android_RotationVectorResultData_t;

//! calibrated acceleration in m/s^2. Note positive Z when flat on table.
typedef struct {
    NTTIME TimeStamp;              //!< Time in seconds
    NTPRECISE X;                   //!< X axis 32Q24 fixed point data
    NTPRECISE Y;                   //!< Y axis 32Q24 fixed point data
    NTPRECISE Z;                   //!< Z axis 32Q24 fixed point data
} Android_GenericTriAxisResultData_t; 

	
#if (1) // TBD: SHOULD THIS BE DEFINED IN OSP ????
//! true if device is in a magnetic field anomaly
typedef struct {
    NTTIME TimeStamp;               //!< Time in seconds
    osp_bool_t MagAnomalyStatus;    //!< true if in magnetic field anomaly
} Android_MagAnomalyResultData_t;
#endif 

#if (1)  // Context change detector 
typedef struct {
	NTTIME startTime;
	NTTIME endTime;
	NTDELTATIME duration;
	uint32_t type;         // 0=transition, 1=steady state, 2=transition step, 3=steady state step, 4=event
	uint32_t flags;              // What is flags?
	uint32_t sequenceNumber;     // What is the purpose of this sequence number? Who provides this?
} ContextChangeDetectorResultData_t;
#endif 


//! callback type used when the library needs to do an atomic operation
/*!
 *  This is absolutely necessary in systems that do background calibration.
 */
typedef void (*OSP_CriticalSectionCallback_t)(void);

//! Callback type used for controlling sensor operation (e.g. on/off/sleep control)
typedef OSP_STATUS_t (* OSP_SensorControlCallback_t)(SensorControl_t* SensorControlCommand);


//! describes system wide settings
/*!
 *  This cannot change after the call initialize.
 *  If there is a smart sensor with its own timestamp, it should be converted into system timestamp units.
 *  The API will not handle 32-bit rollover and clients need to worry about this.
 *
 *  \warning on a multi-threaded system if Enter and Exit Critical are NULL then there are no guarantees on data consistency
 *
 */
typedef struct  {
    TIMECOEFFICIENT TstampConversionToSeconds;   //!< 1 count = this many seconds
    OSP_CriticalSectionCallback_t EnterCritical; //!< callback for entering a critical section of code (i.e. no task switch), NULL if not implemented
    OSP_CriticalSectionCallback_t ExitCritical;  //!< callback for exiting a critical section of code (i.e. task switch ok now), NULL if not implemented
    OSP_SensorControlCallback_t SensorsControl;  //!< callback (NULL if not used) used to request sensor control (i.e. On/Off, rate, etc...).
#if (0)  
    CalibrationConfig_t CalConfig;              //!< Bitfield - if setup, configures which calibrations will be disabled or how frequently they are allowed to run
    ResultInit_t *pResultsTable;                //!< Pointer to results table used to track result subscription (resource management)
	SensorInit_t *pSensorInitTable;             //!< Pointer to the table containing physical sensor descriptor/driver information
#endif 
} SystemDescriptor_t;


//! called by calibration routines when there is new calibration coefficients that should be written to non-volatile memory.
/*!
 *  \warning SensorHandle can change from run to run. Do store the SensorHandle value into NVM to tag this cal data
 *
 *  \param SensorHandle INPUT the sensor this cal data belongs to.
 *  \param CalData      INPUT array of bytes
 *  \param TimeStamp    INPUT timestamp of when this calibration was calculated
 */
typedef void  (* OSP_WriteCalDataCallback_t)(InputSensorHandle_t SensorHandle, void * CalData, NTTIME TimeStamp);

//! called by the algorithms to notify the sensor hub that there is a new data item ready
/*!
 *  cast the pData to the expected sensor type. If you have a general callback handler use the
 *  ResultHandle to lookup what sensor type this should be cast to.
 *
 * \warning the data pointed to by pData is only guaranteed to be valid during the lifetime of this callback
 * \param ResultHandle a handle returned from OSP_SubscribeSensorResult()
 * \param pData the computed value you are interested in.  Cast as appropriate for each result, e.g. TriAxisData for CALIBRATED_GYRO_DATA,
 */
typedef void  (* OSP_ResultReadyCallback_t)(ResultHandle_t ResultHandle, void* pData);


//! describes either a physical or logical sensor and its configuration
/*!
 * Convert sensor data straight from the sensor into the system conventions of orientation and units
 * data is converted are applied in the order of AxisMapping, ConversionOffset, and ConversionScale.
 * Allows conversion from LHS sensor to RHS sensor hub system.
 * Must re-register sensors to change orientation.

 * Sensor conversions convert native binary format into the following units:
      - Accelerometer        - Units are in m/sec/sec
      - Magnetic field       - Units are in uT
      - Gyroscope            - Units are in radians/sec
      - Pressure             - Units are in hPa
      - Temperature          - Units are in degrees Celsius
      - Light (Illumination) - Units are in lux
      - Proximity            - Units are in cm
      - Relative Humidity    - Units are in Percent R.H. (%RH)

 * There is no callback for the library to read stored calibration parameters for 
   each of the physical sensors Acel, Gyro and Mag. The sensor hub is responsibled 
   to read these calibrations from NVM before registering/re-registerating this 
   sensor. If there is no stored calibration data available on the sensor hub, 
   then pass a NULL value for the field pCalibrationData.
   When the fusion library computed a new calibration data, it will update the
   data structure and call the function pointer pOptionalWriteDataCallback() if 
   it is not a NULL pointer.  Sensor hub is responsible to store the updated 
   calibration data to NVM.
*/

typedef struct  {
    InputSensor_t SensorType;                   //!< physical sensor accel, gyro, etc input to fusion algorithm
    uint32_t DataWidthMask;                     //!< how much of the data word that is sent significant
    AxisMapType_t AxisMapping[3];               //!< e.g. use to switch from left handed sensor to right handed system  or swap X and Y axes of a two axis sensor
    int32_t ConversionOffset[3];                //!< an offset for X,Y & Z to compensate for sensor origin (0,0,0)
    NTPRECISE ConversionScale[3];               //!< multipliers to convert the raw sensor data into physical unit ( e.g. m/s^2 for accel).
    NTEXTENDED MaxValue;                        //!< in engineering units. i.e. a +/- 2000 dps gyro would have a max value of 34.907 rad/sec
    NTEXTENDED MinValue;                        //!< in engineering units. i.e. a +/- 2000 dps gyro would have a min value of -34.907 rad/sec
    void * pCalibrationData;                    //!< pointer to stored sensor calibration data (can be NULL if no store calibration is implemented )
    OSP_WriteCalDataCallback_t pOptionalWriteDataCallback; //!< this routine will be called when new cal data is ready to write to NVM (NULL if not used)
	NTPRECISE Noise[3];                         //!< noise based on Power Spectral Density test results
    NTPRECISE NominalSampleRateInSeconds;       //!< rate in which data is sent to the library (for accel, 50Hz and 100 Hz are the only supported rates)
    NTPRECISE SensorDelay;                      //!< time (in seconds) from sensor signal acquisition to time stamp (including filter propagation delay)
    uint32_t On_Time;                           //!< time (in microseconds) when going from OFF mode to ON mode to valid data available
    uint32_t Wake_Time;                         //!< time (in microseconds) when going from SLEEP mode to ON mode to valid data available
    uint16_t Hpf_Cutoff;                        //!< high pass filter 3db cutoff in Hz (0 if none)
    uint16_t Lpf_Cutoff;                        //!< low pass filter 3db cutoff in Hz (0 if none)
    char* SensorName;                           //!< short human readable description, Null terminated
    uint32_t VendorId;                          //!< sensor vendor ID, as assigned by SPI
    uint32_t ProdId;                            //!< sensor product ID, as defined by each vendor
    uint32_t Version;                           //!< sensor version
    uint32_t PlatformId;                        //!< platform ID, as defined by each vendor
    NTPRECISE xyzPositionFromPlatformOrigin[3]; //!< in meters (NTPRECISE gives us sub-micron resolution)
    uint16_t Flags;                             //!< defined on a per sensor type basis
    NTPRECISE factoryskr[3][3];                 //!< linear factory cal, default is {{TOFIX_PRECISE(1.0f),0,0},{0,TOFIX_PRECISE(1.0f),0},{0,0,TOFIX_PRECISE(1.0f)}
    int32_t factoryoffset[3];                   //!< linear factory cal, default is {0,0,0}

    NT nonlineareffects[4][3];                  //!< could be nonlinear calibration, or other means of dealing with sensor nonlinearities
                                                //!< default is {{0,0,0},{0,0,0},{0,0,0},{0,0,0}}
                                                //!< for poorer performing gyroscopes such as the ST or panasonic,
                                                //!< try this default: {{TOFIX(0.075f),0,0},{0,TOFIX(0.075f),0},{0,0,TOFIX(0.075f)},{0,0,0}}

    NTPRECISE biasStability[3];                 //!< how badly the bias wanders in time,
                                                //!< default to {0,0,0} for mag and accel
                                                //!< default for gyro is {TOFIX_PRECISE(DEFAULT_GYRO_BIAS_STABILITY),
                                                //!< TOFIX_PRECISE(DEFAULT_GYRO_BIAS_STABILITY),TOFIX_PRECISE(DEFAULT_GYRO_BIAS_STABILITY)}
                                                //!< set DEFAULT_GYRO_BIAS_STABILITY to 5e-3 for a poor performing gyro such as the Panasonic
    NTPRECISE repeatability[3];                 //!< bias repeatability, in sensor units.  Default is {0,0,0}.
    NT tempco[3][2];                            //!< 2 temperature coefficients on each axis.  Default is {{0,0},{0,0},{0,0}}
    NT shake[3];                                //!< susceptability to shaking.  Default to 0.
                                                //!< Poorer performing gyroscopes such as the ST gyroscope
                                                //!< should get a value of {TOFIX(0.0085f),TOFIX(0.0085f),TOFIX(0.0085f)}

    NTEXTENDED expectednorm;                    //!< TOFIX_EXTENDED(EXPECTED_MAG_NORM)  for magnetometer,
                                                //!< TOFIX_EXTENDED(9.805f) for accel,
                                                //!< and 0 for gyro

    void* ptrSensorSpecificData;                //!< used in conjunction with Flags
} SensorDescriptor_t;

#if (0)  // REMOVE THIS SINCE IT IS NOT PART OF FUSION LIBRARY INTERFACE. 
//Rajiv,  
/* This structure is used to hold information about the physical sensors in the system. This is
* the base unit of the SensorInitTable[]. TODO NOTE: that this must hold only things that are known at
* compile time as the SensorInitTable array is marked as constant i.e. in ROM.
*/
typedef struct _SensorInit
{
    SensorDescriptor_t *pDescriptor;
    SensorDriverIf_t   *pIF;
    uint16_t           Flags;   //TODO: legacy implementation support - need to re-evaluate
    uint8_t            UserCnt; //Number of subscribed results that depend on this sensor
} SensorInit_t;
#endif 

//! use to send raw sensor data from a driver into this API
/*!
 *  \note even if you have single axis or 2 axis data this is the type to use
 */
typedef struct  {
    uint32_t TimeStamp;                         //!< Raw time stamp
    int32_t Data[3];                            //!< Raw sensor data
} TriAxisSensorRawData_t;


  // QUOC: OSP-0.9 named this structure as SensorDescriptor. Should we named it as SensorResultDescriptor and 
  // the corresponding API as OSP_SubscribeSensorResult() ?
//! use to configure format and notifications when requesting a sensor fusion result via OSP_SubscribeResult()
/*!
 *
 */
typedef struct  {
    ASensorType_t SensorType;                        //!<  sensor type from either the ASensorType_t or PSensorType_t list
    OSP_ResultReadyCallback_t pResultReadyCallback; //!<  This routine will be called when a new result is ready
    NTPRECISE OutputRateInSeconds;              //!<  0 = Output ASAP when new data is available
    NTPRECISE NotifySensitivityPositive;        //!<  positive change sensitivity, plus and minus allows asymmetry, but does not require it
    NTPRECISE NotifySensitivityNegative;        //!<  negative change sensitivity
    uint16_t OptionFlags;                       //!<  defined on a result specific basis
    void* OptionData;                           //!<  used in conjunction with Flags
} ResultDescriptor_t;

#if (0)    // not need for OSP 
//! Data stucture used to track result subscriptions
typedef struct _ResultInit
{
    ResultDescriptor_t *pRDescrip;
    uint16_t    Flags;
    uint8_t     SubsCnt;
} ResultInit_t;
#endif 


//! numeric and string formated version data, used with FM_GetVersion()
typedef struct  {
    uint32_t VersionNumber;                     //!< x.x.x.x one byte for major, minor, bugfix, and stamp
    char *VersionString;                        //!< Human readable null terminated string
    char *buildTime;                            //!< Build date and time string (null terminated)
} OSP_Library_Version_t;


/*-------------------------------------------------------------------------------------------------*\
 |    E X T E R N A L   V A R I A B L E S   &   F U N C T I O N S
\*-------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------*\
 |    P U B L I C   V A R I A B L E S   D E F I N I T I O N S
\*-------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------*\
 |    A P I   F U N C T I O N   D E C L A R A T I O N S
\*-------------------------------------------------------------------------------------------------*/

//! Call immediately at startup to initialize the Open-Sensor-Platform Library and inform it of system characteristics
/*!
*  It is imperative to call this at cold boot, or after any time RAM has been lost, and before calling anything else.
*
*  Does internal initializations that the library requires.
*
*  \param pSystemDescriptor - INPUT pointer to a struct that describes things like time tick conversion value. This must not change
*         but may reside in read only memory.
*
*  \return status as specified in OSP_Types.h
*/

OSP_STATUS_t     OSP_Initialize(const SystemDescriptor_t* pSystemDesc);


//! Call at startup for each physical sensor in the system that will feed data into OSP
/*!
 *  Tells the Open-Sensor-Platform Library what kind of sensor inputs it has to work with so its Resource Manager
 *  can choose the most appropriate algorithms to execute.
 *
 *  In a standard sensorhub use case input sensors are registered once.
 *  In a convertible tablet use case where a sensor's physical location changes, this will be called as the physical placement changes.
 *
 *  \note it is not necessary to register/unregister input sensors when the host requests a change in output data rate.  
 *
 *  \warning the caller must preserve the data pointed to by pSensorDescriptor after this call
 *
 *  \param pSensorDescriptor INPUT pointer to data which describes all the details of this sensor and its
 *      current operating mode; e.g. sensor type, SI unit conversion factor
 *  \param pReturnedHandle OUTPUT a handle to use when feeding data in via OSP_SetInputData()
 *
 *  \return status as specified in OSP_Types.h
*/
OSP_STATUS_t     OSP_RegisterInputSensor(SensorDescriptor_t *pSensorDescriptor, InputSensorHandle_t *pReturnedHandle);


//! use to change a sensors operating mode (output rate, position, etc...)
/*!
 *  Tells the Fusion Library to use this sensor instead of the previously registered sensor of the same type
 *
 *  Use Case: Standard Sensor Hub
 *     - after a change request interval/setDelay command from the host, create a new sensor descriptor with
 *       the updated sensor data rate and pass it FM_ReplaceSensor()
 *
 *  Use Case: Transformer Ultra Book
 *     - when the host detects a change in device configuration (Tablet Mode --> Laptop Mode) which could change
 *       sensor position (e.g. sensors in screen have been rotated 180 degrees) or sensor
 *       environment (e.g. screen now interferes with magnetometer)
 *
 *  \warning the caller must preserve the data pointed to by pSensorDescriptor after this call
 *
 *  \param pSensorDescriptor IN pointer to data which describes all the details of the new sensor and its
 *      current operating mode; e.g. sensor type, SI unit conversion factor
*/
OSP_STATUS_t     OSP_ReplaceInputSensor(SensorDescriptor_t *pSensorDescriptor, InputSensorHandle_t *pSensorHandle);

//! queues sensor data which will be processed by OSP_DoForegroundProcessing() and OSP_DoBackgroundProcessing()
/*!
 *
 *  Queuing data for un-registered sensors (or as sensors that). 
 *  Queue size defaults to 8, though is implementation dependent and available via SENSOR_FG_DATA_Q_SIZE.
 *
 *  \param sensorHandle INPUT requires a valid handle as returned by OSP_RegisterInputSensor()
 *  \param data INPUT pointer to timestamped raw sensor data
 *
 *  \return status. Will always be OSP_STATUS_OK. If there is no room in the queue,
 *   The last data will be overwritten and a warning will be triggered if you subscribe to RESULT_WARNING
*/
OSP_STATUS_t     OSP_SetInputData(InputSensorHandle_t SensorHandle, TriAxisSensorRawData_t *data);


//! triggers computation for primary algorithms  e.g ROTATION_VECTOR
/*!
 *  Separating OSP_DoForegroundProcessing and OSP_DoBackgroundProcessing calls allows for computation to happen in different thread contexts
 *  - Call at least as often as your fastest registered result output rate
 *  - Call from a medium priority task to ensure computation happens in a reasonable time
 *  
 *  Guideline: the foreground task should not compute for more than 10ms on any platform
 *
 *  \note What algorithms get computed in the foreground versus the background are implementation dependent
 *
 *  \return status as specified in OSP_Types.h
*/
OSP_STATUS_t     OSP_DoForegroundProcessing(void);


//! triggers computation for less time critical background algorithms, e.g. sensor calibration
/*!
 *  Separating OSP_DoForegroundProcessing() and OSP_DoBackgroundProcessing() calls allows for computation to happen in different thread contexts
 *  - Call at least as often as your slowest registered sensor input
 *  - Call from the lowest priority task to ensure that more time critical functions can happen.
 *
 *  \warning may execute for tens of milliseconds (depending on clock rate and results chosen)
 *  \note What algorithms get computed in the foreground versus the background are implementation dependent
 *
 *  \return status as specified in OSP_Types.h
 */
OSP_STATUS_t     OSP_DoBackgroundProcessing(void);

//! call for each Open-Sensor-Platform result (STEP_COUNT, ROTATION_VECTOR, etc) you want computed and output
/*!
 *  Use Case: Standard Sensor Hub
 *     - after each enable or setDelay command from the host, create a new descriptor with
 *       the desired sensor type result and output data rate and pass it to OSP_SubscribeSensorResult()
 *     
 *
 *  Use Case: Sensor augmented GPS + Android Sensor Hub
 *     - as GPS requires it, create result descriptors requesting SENSOR_ACCELEROMETER, 
 *       SENSOR_MAGNETOMETER, SENSOR_GYROSCOPE
 *     - as host requests come in, create result descriptors per a standard Sensor Hub
 *
 *
 *  \sa OSP_SubscribeSensorResult
 *
 *  \param pResultDescriptor INPUT pointer to data which describes the details of how the fusion should be
 *         computed: e.g output rate, sensors to use, etc.
 *  \param pResultHandle OUTPUT a handle to be used for OSP_UnsubscribeSensorResult()
 *
 *  \return status as specified in OSP_Types.h. OSP_UNSUPPORTED_FEATURE for results that aren't available or licensed
 */
//OSP_STATUS_t     OSP_SubscribeSensorResult(ResultDescriptor_t const *pResultDescriptor, ResultHandle_t *pResultHandle);
OSP_STATUS_t     OSP_SubscribeSensorResult(ResultDescriptor_t *pResultDescriptor, ResultHandle_t *pResultHandle);


//! stops the chain of computation for a registered result
/*!
 *  Use Case: Standard Sensor Hub
 *      - assume you are currently subscribed to ROTATION_VECTOR at 100Hz
 *      - a change request interval from the host for ROTATION_VECTOR at 50Hz
 *      - call OSP_UnsubscribeSensorResult() with the current result handle
 *      - modify the descriptor with the 50Hz output data rate and pass it to OSP_SubscribeSensorResult()
 *
 *  \param ResultHandle INPUT that was received from OSP_SubscribeSensorResult()
 *
 *  \return status as specified in OSP_Types.h
 */
OSP_STATUS_t     OSP_UnsubscribeSensorResult(ResultHandle_t ResultHandle);


#if (1)   //TODO: Do we need this API?? 
// Use to get calibrated sensor data in cases where desired output rate is not an even multiple of input rate.
/*
 *  the current calibrated data for the requested sensor
 *
 *  \param sensorHandle IN requires a valid handle as returned by FM_RegisterInputSensor()
 *  \param data OUT pointer to calibrated sensor data
 *
 *
 *  \return status as FM_STATUS_IDLE if data has not exceeded the change sensitivity since the last
 *      calibrated result callback value for this sensor, or the last time that FM_GetCalibratedSensor returned FM_STATUS_OK.
 *      i.e. notification change is calculated from the last time we notified that a sensitivity change occurred.
 *      Note, if not subscribed to a calibrated result for this sensor, return will always be FM_STATUS_IDLE.
 *      If data has exceeded the change sensitivity, will return FM_STATUS_OK. All other errors apply.
 */

OSP_STATUS_t     OSP_GetCalibratedSensor(InputSensorHandle_t SensorHandle, int32_t *data);
#endif 



//! provides version number and version string of the library implementation
/*!
 *
 *  \param pVersionStruct OUTPUT pointer to a pointer that will receive the version data.
 *
 *  \return status as specified in OSP_Types.h
 */

// TODO:  Should we name this API to something else to distinguish between OSP framework and fusion library?
OSP_STATUS_t     OSP_GetVersion(const OSP_Library_Version_t **ppVersionStruct);


#ifdef __cplusplus
}
#endif

#endif // OSP_API_H__

/*-------------------------------------------------------------------------------------------------*\
 |    E N D   O F   F I L E
\*-------------------------------------------------------------------------------------------------*/
