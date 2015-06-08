#ifndef _FMEMBEDDEDALGCALLS_H_
#define _FMEMBEDDEDALGCALLS_H_

#include "FM_Types.h"
#include "context/ContextDataTypes.h"
#include "calibration/CalibrationTypes.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "FixedPointTypes.h"



/*-----------
 * Flags / Modes
 *------------*/

//for attitude mode (switch between e-compass/gyrocompass)
#define ALGMODE_ATTITUDE_DEFAULT         (0x00000000)
#define ALGMODE_ATTITUDE_POWERSAVE_ON    (0x00000001)



/*-----------
 * Callback definitions
 *------------*/

typedef void (*FMAlg_ResultCallback_t)(NTTIME time, NTPRECISE * data, int16_t len);
typedef void (*FMAlg_ContextDeviceMotionCallback_t)(NTTIME time, NT * data, int16_t len);
typedef void (*FMAlg_CalibrationCallback_t)(NTTIME time, FM_CalStorageStruct * cal);
typedef void (*FMAlg_CalibratedSensorCallback_t)(uint8_t sensortype, NTTIME time, int32_t calibratedData[3]);
typedef void (*FMAlg_CriticalSectionCallback_t)(void);
typedef void (*FMAlg_ResourceControlCallback_t)(uint32_t sensorid, EnumResourceCommand_t command, uint32_t data);


/*-----------
 * Declarations
 *------------*/

/*
 * Order of operations for starting up the algs
 *
 * On boot, call
 *
 * 1) FM_InitializeAllAlgorithms (REQUIRED)
 * 2) FM_SetCalibration    (OPTIONAL -- IF THERE IS A CALIBRATION IN FLASH)
 * 3) FM_SetSensorProperty (KIND OF NOT OPTIONAL, DEFAULTS EXIST THOUGH)
 * 4) FM_ResetAllAlgorithms (REQUIRED)
 *
 * When setting re-setting sensor property (because of slider phone / ultrabook lid)
 * do items 2 and 3 as required (per sensor), and then 4 as above.
 */



/* FM_SetSensorProperty sets things like sensor noise and saturation limits */
void FM_SetSensorProperty(FM_SensorProperties_t * pProperties);

/* FM_SetCalibration is meant to set data from persistent storage to the background calibrator */
void FM_SetCalibration(FM_CalStorageStruct * pCal,fm_bool_t ValidCalData) ;

/* FM_InitializeAllAlgorithms is meant to initialize all algs to a clean slate. DO THIS ONCE*/
void FM_InitializeAllAlgorithms(FMAlg_ResourceControlCallback_t resourceControlCallback, FMAlg_CriticalSectionCallback_t lock, FMAlg_CriticalSectionCallback_t unlock);

void FM_ResetAllAlgorithms(void);


/******************************************************
 These will register result callbacks
*************************/

void FM_RegisterAttitudeCallback(uint32_t modeFlags, FMAlg_ResultCallback_t fpCallback);
void FM_RegisterMagAnomalyCallback(FMAlg_ResultCallback_t fpCallback);
#ifdef FM_SIX_AXIS_ONLY
void FM_RegisterVirtualGyroRateCallback(FMAlg_ResultCallback_t fpCallback);
#endif
void FM_RegisterCalibrationCallback(FMAlg_CalibrationCallback_t fpCallback);
void FM_RegisterCalibratedSensorDataCallback(FMAlg_CalibratedSensorCallback_t fpCallback);


/* FM_RegisterDeviceMotionContextCallback
 * 1) There are different sensitivities of motion context, so look at the enum.
 * 2) The callback will return you an NT array.  Each index in the array has an enum... for example, array[isTranslating] will get you
 *    the probability of translating (a number between 0 and 1 in NT fixed point format).
 */
void FM_RegisterDeviceMotionContextCallback(EContextDeviceMotionSensitivty sensitivity, FMAlg_ContextDeviceMotionCallback_t fpCallback);



/******************************************************
 Set sensor data here
*************************/
/* The setting of foreground sensor data */
void FM_SetForegroundAccelerometerMeasurement(NTTIME time, const NTPRECISE measurementInMetersPerSecondSquare[3]) ;
void FM_SetForegroundMagnetometerMeasurement(NTTIME time, const NTEXTENDED measurementInMicroTesla[3]) ;
void FM_SetForegroundGyroscopeMeasurement(NTTIME time, const NTPRECISE measurementInRadiansPerSecond[3]) ;

/* The setting of foreground background data */
void FM_SetBackgroundAccelerometerMeasurement(NTTIME time, const NTPRECISE measurementInMetersPerSecondSquare[3]) ;
void FM_SetBackgroundMagnetometerMeasurement(NTTIME time, const NTEXTENDED measurementInMicroTesla[3]) ;
void FM_SetBackgroundGyroscopeMeasurement(NTTIME time,const NTPRECISE measurementInRadiansPerSecond[3]) ;


/******************************************************
 Misc stuff that needs to be exported
*************************/

void FM_ReturnCalibratedAccelerometerMeasurement(NTPRECISE measurementInMetersPerSecondSquare[3],NTPRECISE * calibratedAccelerometerMeasurement);
void FM_ReturnCalibratedMagnetometerMeasurement(NTEXTENDED measurementInMicroTesla[3],NTEXTENDED * calibratedMagnetometerMeasurement);
void FM_ReturnCalibratedGyroscopeMeasurement(NTPRECISE measurementInRadiansPerSecond[3],NTPRECISE * calibratedGyroscopeMeasurement);



#ifdef __cplusplus
}
#endif


#endif //_FMEMBEDDEDALGCALLS_H_
