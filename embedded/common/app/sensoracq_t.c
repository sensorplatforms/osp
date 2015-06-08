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
/*------------------------------------------------------------------*\
 |    I N C L U D E   F I L E S
\*------------------------------------------------------------------*/
#include "common.h"
#include "acc_common.h"
#include "mag_common.h"
#include "gyro_common.h"
#include "pressure_common.h"
#include "osp-api.h"
#include "osp-sensors.h"
#include "sensacq_i2c.h"

/*-------------------------------------------------------------------*\
 |    E X T E R N A L   V A R I A B L E S   &   F U N C T I O N S
\*-------------------------------------------------------------------*/
void WaitForHostSync(void);

/*-------------------------------------------------------------------*\
 |    P U B L I C   V A R I A B L E S   D E F I N I T I O N S
\*-------------------------------------------------------------------*/

/*-------------------------------------------------------------------*\
 |    P R I V A T E   C O N S T A N T S   &   M A C R O S
\*-------------------------------------------------------------------*/

/*-------------------------------------------------------------------*\
 |    P R I V A T E   T Y P E   D E F I N I T I O N S
\*-------------------------------------------------------------------*/

/*-------------------------------------------------------------------*\
 |    S T A T I C   V A R I A B L E S   D E F I N I T I O N S
\*-------------------------------------------------------------------*/
#ifndef INTERRUPT_BASED_SAMPLING
static AsfTimer sSensorTimer = NULL_TIMER;
#endif
static AsfTimer sPressureTimer = NULL_TIMER;
/*-------------------------------------------------------------------*\
 |    F O R W A R D   F U N C T I O N   D E C L A R A T I O N S
\*-------------------------------------------------------------------*/
static void SensorDataHandler(InputSensor_t sensorId, uint32_t timeStamp);
/*-------------------------------------------------------------------*\
 |    P R I V A T E     F U N C T I O N S
\*-------------------------------------------------------------------*/

/*********************************************************************
 * @fn      HandleTimers
 *          Handles the MSG_TIMER_EXPIRY messages
 *
 *********************************************************************/
static void HandleTimers(MsgTimerExpiry *pTimerExp)
{
	uint32_t timeStamp;

	switch (pTimerExp->userValue) {
	case TIMER_REF_PRESSURE_READ:
		/* Schedule the next sampling */
		ASFTimerStart(SENSOR_ACQ_TASK_ID, TIMER_REF_PRESSURE_READ,
				PRESSURE_SAMPLE_PERIOD, &sPressureTimer);

		timeStamp = GetCurrentTime();
		SensorDataHandler(PRESSURE_INPUT_SENSOR, timeStamp);
		break;
#ifndef INTERRUPT_BASED_SAMPLING
	case TIMER_REF_SENSOR_READ:
		/* Schedule the next sampling */
		ASFTimerStart(SENSOR_ACQ_TASK_ID, TIMER_REF_SENSOR_READ,
				SENSOR_SAMPLE_PERIOD, &sSensorTimer);

		/* Call data handler for each sensors */
		timeStamp = RTC_GetCounter();
		SensorDataHandler(ACCEL_INPUT_SENSOR, timeStamp);
		SensorDataHandler(MAG_INPUT_SENSOR, timeStamp);
		SensorDataHandler(GYRO_INPUT_SENSOR, timeStamp);
		break;
#endif
	default:
		D1_printf("Unknown Timer! Reference %d\r\n", pTimerExp->userValue);
		break;
	}
}

#ifdef ENABLE_FLASH_STORE
/***********************************************************************
 * @fn      StoreCalibrationData
 *          Stores calibration data. Why in this task?. Doing it in
 *          Sensor acq makes it unlikely to interfere with the
 *          sensor data reads over I2C.
 *
 ***********************************************************************/
static void StoreCalibrationData( CalEvent_t event )
{
}
#endif

/***********************************************************************
 * @fn      SensorDataHandler
 *          Handle data ready indications from ISR for various sensors
 *
 ***********************************************************************/
static uint32_t queue_err = 0;

static void SensorDataHandler(InputSensor_t sensorId, uint32_t timeStamp)
{
	MsgAccelData accelData;
	MsgMagData magData;
	MsgGyroData gyroData;
	MsgPressData pressData;
	static uint8_t sMagDecimateCount = 0;
	static uint8_t gyroSampleCount = 0;
	static uint8_t accSampleCount = 0;

#if defined ALGORITHM_TASK
	MessageBuffer *pMagSample = NULLP;
	MessageBuffer *pAccSample = NULLP;
	MessageBuffer *pGyroSample = NULLP;
	MessageBuffer *pPressSample = NULLP;
#endif

	switch (sensorId) {
	case MAG_INPUT_SENSOR:
		/* Read mag Data - reading would clear interrupt also */
		Mag_ReadData(&magData);           
       
		if ((sMagDecimateCount++ % MAG_DECIMATE_FACTOR) == 0) {
			/* Replace time stamp with that captured
			   by interrupt handler */
			magData.timeStamp = timeStamp;           
#ifdef ALGORITHM_TASK
			ASF_assert(ASFCreateMessage(MSG_MAG_DATA, 
					sizeof(MsgMagData),
					&pMagSample) == ASF_OK);
			pMagSample->msg.msgMagData = magData;
			if (!(ASFSendMessage(ALGORITHM_TASK_ID,
					pMagSample) == ASF_OK)) {
				queue_err++;
			}
#endif                     
		}
		break;

	case GYRO_INPUT_SENSOR:
		Gyro_ReadData(&gyroData); //Reads also clears DRDY interrupt

		/* Replace time stamp with that captured by interrupt handler */
    
		if ((gyroSampleCount++ % GYRO_SAMPLE_DECIMATE) == 0) {
			/* Read Gyro Data - reading typically clears interrupt as well */
			gyroData.timeStamp = timeStamp;

#ifdef ALGORITHM_TASK
			if (ASFCreateMessage(MSG_GYRO_DATA, 
					sizeof(MsgGyroData),
					&pGyroSample) == ASF_OK) {
				pGyroSample->msg.msgGyroData = gyroData;
				if (!(ASFSendMessage(ALGORITHM_TASK_ID, 
					pGyroSample) == ASF_OK)) {
					queue_err++;
				}
			}
#endif                     
          //  D0_printf("Gyro ADC: %d, %d, %d\r\n", gyroData.X, gyroData.Y, gyroData.Z);                    
		}
		break;
	case ACCEL_INPUT_SENSOR:
#if defined TRIGGERED_MAG_SAMPLING
		if (accSampleCount % MAG_TRIGGER_RATE_DECIMATE == 0) {
			Mag_TriggerDataAcq(); //Mag is triggered relative to Accel to avoid running separate timer
		}
#endif   

		/* Read Accel Data - reading typically clears interrupt as well */
		Accel_ReadData(&accelData);

		if (accSampleCount++ % ACCEL_SAMPLE_DECIMATE == 0) {
			/* Replace time stamp with that captured by interrupt handler */
			accelData.timeStamp = timeStamp;

#ifdef ALGORITHM_TASK
			if (ASFCreateMessage(MSG_ACC_DATA, sizeof(MsgAccelData),
                               &pAccSample) == ASF_OK) {
				pAccSample->msg.msgAccelData = accelData;
				if (!(ASFSendMessage(ALGORITHM_TASK_ID,
					pAccSample) == ASF_OK)) {
					queue_err++;
				}
			}                                        
#endif
            // D0_printf("Accel ADC: %d, %d, %d\r\n", accelData.X, accelData.Y, accelData.Z); 
		}
		break;
	case PRESSURE_INPUT_SENSOR:
		/* Read Accel Data - reading typically clears interrupt as well */
		Pressure_ReadData(&pressData);

		/* Replace time stamp with that captured by interrupt handler */
		pressData.timeStamp = timeStamp;
    
		/* To do: Send the pressure sensor out */
#ifdef ALGORITHM_TASK
		ASF_assert(ASFCreateMessage(MSG_PRESS_DATA,
				sizeof(MsgPressData),
				&pPressSample) == ASF_OK);
		pPressSample->msg.msgPressData = pressData;
		ASFSendMessage(ALGORITHM_TASK_ID, pPressSample);
#endif
          
       // D0_printf("Pressure ADC: P = %d,  T = %d\r\n", pressData.X, pressData.Z);                
		break;
	default:
		D0_printf("Input Sensor ID %d is not supported\r\n", sensorId);
		break;
	}
}

// Uncomment this to enable libfm to turn on/off sensor based on resource usage
//#define SENSOR_CONTROL_ENABLE  

/***********************************************************************
 * @fn      SensorControlCmdHandler
 *          Handle sensor parameter setting request 
 *
 ***********************************************************************/
void SensorControlCmdHandler(MsgSensorControlData *pData)
{
	InputSensor_t sensorIdx;
    SensorControlCommand_t command; 
	sensorIdx = pData->sensorIdx;

    command = (SensorControlCommand_t) pData->command;
	switch(command){
	case SENSOR_CONTROL_SENSOR_OFF:		
		// e.g. put sensor into power saving mode 
		D0_printf("Sensor Control set sensor idx %d OFF\r\n", sensorIdx);
        switch ( sensorIdx){
        case ACCEL_INPUT_SENSOR:
#ifdef SENSOR_CONTROL_ENABLE
            Accel_ConfigDataInt(false);         // do not disable accel so watch window input will work 
#endif 
            break;
        case MAG_INPUT_SENSOR:
#ifdef SENSOR_CONTROL_ENABLE
            Mag_ConfigDataInt(false);
#endif
            break;
       case  GYRO_INPUT_SENSOR:
#ifdef SENSOR_CONTROL_ENABLE
          Gyro_ConfigDataInt(false);
#endif 
            break;
        }
		break;
	case SENSOR_CONTROL_SENSOR_SLEEP:
		// put sensor into sleep mode 
		D0_printf("Sensor Control set sensor idx %d to SLEEP mode\r\n", sensorIdx);
		break;
	case SENSOR_CONTROL_SENSOR_ON:
		// Turn sensor into normal operating mode
		D0_printf("Sensor Control set sensor idx %d ON\r\n", sensorIdx);
        switch ( sensorIdx){
        case ACCEL_INPUT_SENSOR:
#ifdef SENSOR_CONTROL_ENABLE
            Accel_ConfigDataInt(true);
#endif 
            break;
        case MAG_INPUT_SENSOR:
#ifdef SENSOR_CONTROL_ENABLE
            Mag_ConfigDataInt(true);
#endif 
            break;
       case  GYRO_INPUT_SENSOR:
#ifdef SENSOR_CONTROL_ENABLE            
            Gyro_ConfigDataInt(true);
#endif
            break;
        }
		break;
	case SENSOR_CONTROL_SET_SAMPLE_RATE:
		// Update the sensor output rate 
		D0_printf("Sensor Control set sensor idx %d sampe rate to %d\r\n", 
				  sensorIdx, pData->data);
		break;
	case SENSOR_CONTROL_SET_LPF_FREQ:
		D0_printf("Sensor Control set sensor idx %d LPF to %d\r\n",
				  sensorIdx, pData->data);
		break;
	case SENSOR_CONTROL_SET_HPF_FREQ:
		D0_printf("Sensor Control set sensor idx %d HPF to %d\r\n",
				  sensorIdx, pData->data);		
		break;
	default:
		D0_printf("Invalid sensor control command value (%d)\r\n", pData->command);
	}
}

/*-------------------------------------------------------------------*\
 |    P U B L I C     F U N C T I O N S
\*-------------------------------------------------------------------*/
/*********************************************************************
 * @fn      SendDataReadyIndication
 * @brief  This helper function sends data ready indication to
 *         Sensor Acq task. Called from ISR.
 *         Best effort to send the message. If buffer or queue space is
 *         not available, the message is dropped!
 * @param  sensorId: Sensor identifier whose data is ready to be read
 * @return None
 *
 *********************************************************************/
void SendDataReadyIndication(uint8_t sensorId, uint32_t timeStamp)
{
	MessageBuffer *pSendMsg = NULLP;
	if (ASFCreateMessage(MSG_SENSOR_DATA_RDY,
			sizeof(MsgSensorDataRdy), &pSendMsg) == ASF_OK) {
		pSendMsg->msg.msgSensorDataRdy.sensorId = sensorId;
		pSendMsg->msg.msgSensorDataRdy.timeStamp = timeStamp;
        
        if ( ASFSendMessage(SENSOR_ACQ_TASK_ID, pSendMsg) != ASF_OK ) {
            D0_printf("Error sending sensoracq message for senosr id %d\r\n", sensorId);           
        }        
    } else {
        D0_printf("Error creating sensoracq message for sensor id %d\r\n", sensorId);
    }

}

/*******************************************************************
 * @fn      SensorAcqTask
 *          This task is responsible for acquiring data from and
 *          controlling the sensors in the system.
 *
 * @param   none
 *
 * @return  none
 *
 *******************************************************************/
/* Sensor data flow:
 * 1. Sensor interrupts on data ready.
 * 2. IRQ handler is solely expected to call SendDataReadyIndication.
 * 3. SensorAcqTask will see a message data is available.
 * 4. Data is read. Sensor driver is expected to clear 
 *    the interrupt from the read function.
 */
ASF_TASK void SensorAcqTask(ASF_TASK_ARG)
{
	MessageBuffer *rcvMsg = NULLP;
	volatile uint8_t  i;

#ifndef WAIT_FOR_HOST_SYNC
	os_dly_wait(MSEC_TO_TICS(50)); /* Allow startup time for sensors */
#else
	WaitForHostSync(); //This also allows for startup time for sensors
#endif
	/* Setup I2C bus here? */
	dev_i2c_init();

	/* Setup interface for the Magnetometer */
	Mag_HardwareSetup(true);
	Mag_Initialize();

	/* Setup interface for the accelerometers */
	Accel_HardwareSetup(true);
	Accel_Initialize(INIT_NORMAL);     

	/* Setup Gyro */
	Gyro_HardwareSetup(true);
	Gyro_Initialize();
    D0_printf("Gyro init done:\r\n");


	/* Setup Pressure */
	Pressure_HardwareSetup(true);
	Pressure_Initialize();
	ASFTimerStart(SENSOR_ACQ_TASK_ID, TIMER_REF_PRESSURE_READ,
			PRESSURE_SAMPLE_PERIOD, &sPressureTimer);

#ifndef INTERRUPT_BASED_SAMPLING
	/* Start sample period timer */
	ASFTimerStart(SENSOR_ACQ_TASK_ID, TIMER_REF_SENSOR_READ,
			SENSOR_SAMPLE_PERIOD, &sSensorTimer);
#else
	/* Enable Sensor interrupts */
	Mag_ConfigDataInt(true);
	Accel_ConfigDataInt(true);    
	Gyro_ConfigDataInt(true);

    # ifdef TRIGGERED_MAG_SAMPLING
        D0_printf("Set mag to low power mode\r\n");
        Mag_SetLowPowerMode(); //Low power mode until triggered
    # endif
#endif
	
    D0_printf("%s initialized\r\n", __func__);  

    /* Magnetometer sensor does not re-generate interrupt if its outputs are not read. */
    Mag_ClearDataInt();

	while (1) {		
		ASFReceiveMessage(SENSOR_ACQ_TASK_ID, &rcvMsg);               
      //  D0_printf("rcvMsg->msgId = %d\r\n",rcvMsg->msgId);
 
		switch (rcvMsg->msgId) {
		case MSG_TIMER_EXPIRY:
			HandleTimers(&rcvMsg->msg.msgTimerExpiry);
			break;
		case MSG_CAL_EVT_NOTIFY:
#ifdef ENABLE_FLASH_STORE
			StoreCalibrationData((CalEvent_t)rcvMsg->msg.msgCalEvtNotify.byte);
#else
			D0_printf("#### WARNING - NV Storage Not Implemented! #####\r\n");
#endif
			break;
		case MSG_SENSOR_DATA_RDY:
            //D0_printf("MSG_SENSOR_DATA_RDY msg id %d\r\n", rcvMsg->msgId); 
          
#ifdef INTERRUPT_BASED_SAMPLING
			SensorDataHandler((InputSensor_t)rcvMsg->msg.msgSensorDataRdy.sensorId,
					rcvMsg->msg.msgSensorDataRdy.timeStamp);
#endif
			break;
		case MSG_SENSOR_CONTROL: 
            //D0_printf("MSG_SENSOR_CONTROL msg id %d\r\n",rcvMsg->msgId);
			SensorControlCmdHandler(&rcvMsg->msg.msgSensorControlData);
			break;
		default:
			/* Unhandled messages */
			D2_printf("SensorAcqTask:!!!UNHANDLED MESSAGE:%d!!!\r\n", rcvMsg->msgId);
			break;
		}
	}
}

/*----------------------------------------------------------------------*\
 |    E N D   O F   F I L E
\*----------------------------------------------------------------------*/
