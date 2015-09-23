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

/*-------------------------------------------------------------------------*\
 |    I N C L U D E   F I L E S
\*-------------------------------------------------------------------------*/
#include "board.h"
#include "common.h"
#ifdef ANDROID_COMM_TASK
#include "hostinterface.h"
#include "osp-sensors.h"
#include <string.h>
#include "sensorhub.h"
#include "SensorPackets.h"
#include "Queue.h"
#include "MQErrorCodes.h"
#include "osp_i2c_map.h"
#include "algorithm-api.h"    //Algorithm API to subscribe sensor result
#include "rtc_api.h"

#define HIF_PACKET_SIZE	M_CalcBufferSize(sizeof(HostIFPackets_t))

#define FASTDATA_NUM_Q	3
#define FASTDATA_LEN_Q	4096

struct DataBuf {
	uint8_t	DataChunk[FASTDATA_LEN_Q];
	volatile int length;
	volatile int state;
};

volatile struct FastData {
	struct DataBuf Databuf[FASTDATA_NUM_Q];
	volatile uint8_t write;
	volatile uint8_t read;
} FDC;

static uint8_t PacketMem[HIF_PACKET_SIZE];

// FIXME: Since OSP_CONFIG command is not implemented in the host, we need to set this value to 1 so FastData buffer is properly initialized on startup. 
static volatile uint8_t ClearQueue = 1;        // Original code set this to 0. 


void Hostif_Init(void);
void Hostif_StartTx(uint8_t *pBuf, uint16_t size, int magic);
void CHostif_StartTxChained(uint8_t *pBuf, uint16_t size, uint8_t *pBuf_next, uint16_t size_next, int magic);
int FastData_add(volatile struct FastData *FD, Buffer_t *pHIFPkt);

/*-------------------------------------------------------------------------*\
 |    E X T E R N A L   V A R I A B L E S   &   F U N C T I O N S
\*-------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------*\
 |    P R I V A T E   C O N S T A N T S   &   M A C R O S
\*-------------------------------------------------------------------------*/
#define SH_WHO_AM_I                 0x54
#define SH_VERSION0                 0x01
#define SH_VERSION1                 0x24

static uint32_t SensorState[2];
static uint32_t PSensorState[2];

// TODO: Need to call algorithm module to subscribe the enable sensor. 
static void SensorEnable(ASensorType_t sen, int space)
{
	int v = (int)sen;
	int set;
	if (v & SENSOR_DEVICE_PRIVATE_BASE) {
		space = 1;
		v &= ~SENSOR_DEVICE_PRIVATE_BASE;
	}
	if (space == 0) {
		if (v < 32) {
			set = (1<<v);
			SensorState[0] |= set;
		} else {
			set = (1<<(v-32));
			SensorState[1] |= set;
		}
	    // Why hardcode significant motion enable ?
		SensorState[0] |= 1 << SENSOR_SIGNIFICANT_MOTION;
	} else if (space == 1) {
		if (v < 32) {
			set = (1<<v);
			PSensorState[0] |= set;
		} else {
			set = (1<<(v-32));
			PSensorState[1] |= set;
		}
		sen |= SENSOR_DEVICE_PRIVATE_BASE;
	}

	/* Now subscribe to the algorithm to enable this sensor */
	Algorithm_SubscribeSensor(sen);
}

// TODO: Need to call the algorithm to unsubscribe the sensor
static void SensorDisable(ASensorType_t sen, int space)
{
	int v = (int)sen;
	int set;

	if (v & SENSOR_DEVICE_PRIVATE_BASE) {
		space = 1;
		v &= ~SENSOR_DEVICE_PRIVATE_BASE;
	}
	if (space == 0) {
		if (v < 32) {
			set = (1<<v);
			SensorState[0] &= ~set;
		} else {
			set = (1<<(v-32));
			SensorState[1] &= ~set;
		}
	} else if (space == 1) {
		if (v < 32) {
			set = (1<<v);
			PSensorState[0] &= ~set;
		} else {
			set = (1<<(v-32));
			PSensorState[1] &= ~set;
		}
		sen |= SENSOR_DEVICE_PRIVATE_BASE;
	}

	// Why hardcode this sensor?
	SensorState[0] |= 1 << SENSOR_SIGNIFICANT_MOTION;

	// Now un-subscribe this sensor from the algorithm 
	Algorithm_UnsubscribeSensor(sen);   
}

static int GetSensorState(ASensorType_t sen, int space)
{
	int v = (int)sen;
	int set;

	if (v & SENSOR_DEVICE_PRIVATE_BASE) {
		v &= ~SENSOR_DEVICE_PRIVATE_BASE;
		space = 1;
	}

	if (space == 0) {
		if (v < 32) {
       			/* Standard Android sensor enum value */
			set = (1<<v);
			if (SensorState[0] & set) return 1;
			return 0;
		} else {
			set = (1<<(v-32));
			if (SensorState[1] & set) return 1;
			return 0;
		}
	} else if (space == 1) {
		if (v < 32) {
       			/* Standard Android sensor enum value */
			set = (1<<v);
			if (PSensorState[0] & set) return 1;
			return 0;
		} else {
			set = (1<<(v-32));
			if (PSensorState[1] & set) return 1;
			return 0;
		}
	}
}

#if (0) // obsolete
static ASensorType_t ResultToAndroidTypeMap(uint8_t sen)
{
	switch ((SensorType_t) sen) {
	case SENSOR_ACCELEROMETER_UNCALIBRATED:  
		return AP_PSENSOR_ACCELEROMETER_UNCALIBRATED;
						
	case SENSOR_ACCELEROMETER_CALIBRATED:
		return A_SENSOR_ACCELEROMETER;
	case SENSOR_MAGNETIC_FIELD_UNCALIBRATED:
		return A_SENSOR_MAGNETIC_FIELD_UNCALIBRATED;
	case SENSOR_MAGNETIC_FIELD_CALIBRATED:
		return A_SENSOR_MAGNETIC_FIELD;
	case SENSOR_GYROSCOPE_UNCALIBRATED:
		return A_SENSOR_GYROSCOPE_UNCALIBRATED;
	case SENSOR_GYROSCOPE_CALIBRATED:
		return A_SENSOR_GYROSCOPE;
	case SENSOR_LIGHT:
		return A_SENSOR_LIGHT;
	case SENSOR_PRESSURE:
		return A_SENSOR_PRESSURE;
	case SENSOR_PROXIMITY:
		return A_SENSOR_PROXIMITY;
	case SENSOR_RELATIVE_HUMIDITY:
		return A_SENSOR_RELATIVE_HUMIDITY;
	case SENSOR_AMBIENT_TEMPERATURE:
		return A_SENSOR_AMBIENT_TEMPERATURE;
	case SENSOR_GRAVITY:
		return A_SENSOR_GRAVITY;
	case SENSOR_LINEAR_ACCELERATION:
		return A_SENSOR_LINEAR_ACCELERATION;
	case SENSOR_ORIENTATION:
		return A_SENSOR_ORIENTATION;
	case SENSOR_ROTATION_VECTOR:
		return A_SENSOR_ROTATION_VECTOR;
	case SENSOR_GEOMAGNETIC_ROTATION_VECTOR:
		return A_SENSOR_GEOMAGNETIC_ROTATION_VECTOR;
	case SENSOR_GAME_ROTATION_VECTOR:
		return A_SENSOR_GAME_ROTATION_VECTOR;
	case SENSOR_VIRTUAL_GYROSCOPE:
		return A_SENSOR_GYROSCOPE;
	case SENSOR_STEP_DETECTOR:
		return A_SENSOR_STEP_DETECTOR;
	case SENSOR_STEP_COUNTER:
		return A_SENSOR_STEP_COUNTER;
	case SENSOR_CONTEXT_DEVICE_MOTION:
		return AP_PSENSOR_CONTEXT_DEVICE_MOTION;
	case SENSOR_CONTEXT_CARRY:
		return AP_PSENSOR_CONTEXT_CARRY;
	case SENSOR_CONTEXT_POSTURE:
		return AP_PSENSOR_CONTEXT_POSTURE;
	case SENSOR_CONTEXT_TRANSPORT:
		return AP_PSENSOR_CONTEXT_TRANSPORT;
	case SENSOR_CONTEXT_CHANGE_DETECTOR:
		return AP_PSENSOR_CONTEXT_DEVICE_MOTION;
	case SENSOR_STEP_SEGMENT_DETECTOR:
		return AP_PSENSOR_STEP;
	case SENSOR_SIGNIFICANT_MOTION:
		return A_SENSOR_SIGNIFICANT_MOTION;
	default:
		return A_SENSOR_META_DATA;
	}
}
#endif 

/*-------------------------------------------------------------------------*\
 |    P R I V A T E   T Y P E   D E F I N I T I O N S
\*-------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------*\
 |    S T A T I C   V A R I A B L E S   D E F I N I T I O N S
\*-------------------------------------------------------------------------*/
static SH_RegArea_t SlaveRegMap;
static uint8_t QueueOverFlow = 0;
/*-------------------------------------------------------------------------*\
 |    F O R W A R D   F U N C T I O N   D E C L A R A T I O N S
\*-------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------*\
 |    P U B L I C   V A R I A B L E S   D E F I N I T I O N S
\*-------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------*\
 |    P R I V A T E     F U N C T I O N S
\*-------------------------------------------------------------------------*/

static void FastData_init(struct FastData *FD)
{
	int i;
	FD->write = 0;
	FD->read = 0;
	for (i = 0; i < FASTDATA_NUM_Q; i++) {
		FD->Databuf[i].length = 0;
		FD->Databuf[i].state = 0;
	}
}

int drops = 0;
static int FastData_add(volatile struct FastData *FD, Buffer_t *pHIFPkt)
{
	volatile struct DataBuf *DB;
	static int state = 0;
	static uint32_t lastTimeStamp = 0;
	uint32_t curTime;

	if (ClearQueue) {
		FastData_init(FD);
		ClearQueue = 0;
	}

	DB = &(FD->Databuf[FD->write]);
	if (pHIFPkt->Header.Length+DB->length > FASTDATA_LEN_Q) {
		if (((FD->write+1)%FASTDATA_NUM_Q) == FD->read) {
			drops++;
			state = 1;
			return 1;	/* No more Q's available */
		} else {
			if (state == 1) {
				D0_printf("drops = %i\r\n", drops);
			}
			state = 0;
			FD->write++;
			FD->write %= FASTDATA_NUM_Q;
			DB = &(FD->Databuf[FD->write]);
			DB->length = 0;
			DB->state = 0;
		}

	}
	memcpy(DB->DataChunk + DB->length, &(pHIFPkt->DataStart), pHIFPkt->Header.Length);
	DB->length += pHIFPkt->Header.Length;
	DB->state++;
    curTime = rtc_read();

	if (lastTimeStamp != 0) {
		if ((curTime - lastTimeStamp) > 1000) {
			if (((FD->write+1)%FASTDATA_NUM_Q) == FD->read) {
				return 1;
			} else {
				FD->write++;
				FD->write%=FASTDATA_NUM_Q;
				DB = &(FD->Databuf[FD->write]);
				DB->length = 0;
				DB->state = 0;
			}
		}
	}

	lastTimeStamp = curTime;
	if (DB->state > 300) {
		if (((FD->write+1)%FASTDATA_NUM_Q) == FD->read) {
			return 1;
		} else {
			FD->write++;
			FD->write%=FASTDATA_NUM_Q;
			DB = &(FD->Databuf[FD->write]);
			DB->length = 0;
			DB->state = 0;
		}
	}
	return 0;
}

static int FastData_get(struct FastData *FD)
{
	if (ClearQueue) return -1;		/* Clear in progress */
	if (FD->read == FD->write) return -1;	/* No data available */
	return FD->read;
}

/************************************************************************
 * @fn      SH_Slave_init
 *          Initialize the Sensor Hub I2C Slave register interface
 *
 ************************************************************************/
static void SH_Slave_init(void)
{
    memset(&SlaveRegMap, 0, sizeof(SlaveRegMap));
    SlaveRegMap.version0 = SH_VERSION0;
    SlaveRegMap.version1 = SH_VERSION1;
    SlaveRegMap.whoami   = SH_WHO_AM_I;

    SlaveRegMap.irq_cause = SH_MSG_IRQCAUSE_NEWMSG;
    SlaveRegMap.read_len = 0;
    SlaveRegMap.rd_mem[0] = SH_MSG_TYPE_ABS_ACCEL;
}

/***************************************************************************
 * @fn      SendSensorBoolData
 *          Sends boolean sensor data over the I2C slave interface

 * Enqueues data only.
 *
 ***************************************************************************/

static void SendSensorBoolData(ASensorType_t sensorType, MsgSensorBoolData *pMsg)
{
	Buffer_t	*pHifPacket;
	uint8_t		*pPayload;
	UncalibratedFixP_t	UnCalFixPData;
 

	/* Create packet and place in queue */

	pHifPacket = (Buffer_t *)PacketMem;

	pPayload = M_GetBufferDataStart(pHifPacket);
#if 0  // Result to Android is now obsoleted in new OSP API
	sensorType = ResultToAndroidTypeMap(sensorId);     
#endif 
    
    // Do not send data if host did not activate this sensor type
	if (GetSensorState(sensorType, 0) == 0)
		return;

	SensorHubAssertInt();	/* Assert interrupt to host */

	/* Process sensor and format into packet */
	switch (sensorType) {
	case SENSOR_STEP_DETECTOR:
	case SENSOR_SIGNIFICANT_MOTION:
		UnCalFixPData.TimeStamp.TS64 = pMsg->timeStamp;
		UnCalFixPData.Axis[0]	= pMsg->active;
		UnCalFixPData.Axis[1]	= 0;
		UnCalFixPData.Axis[2]	= 0;
		UnCalFixPData.Offset[0]	= 0;
		UnCalFixPData.Offset[1]	= 0;
		UnCalFixPData.Offset[2]	= 0;

		pHifPacket->Header.Length = FormatUncalibratedPktFixP(pPayload,
			&UnCalFixPData, META_DATA_UNUSED, sensorType);

		break;
	default:
		return;
	}
	QueueOverFlow = FastData_add(&FDC, pHifPacket);
}

/***************************************************************************
 * @fn      SendSensorData
 *          Sends 3-axis sensor data over the I2C slave interface

 * Enqueues data only.
 *
 ***************************************************************************/
static void SendSensorData(ASensorType_t sensorType, MsgSensorData *pMsg)
{
	Buffer_t	*pHifPacket;
	uint8_t		*pPayload;
	UncalibratedFixP_t	UnCalFixPData;
	CalibratedFixP_t	CalFixPData;
	QuaternionFixP_t	QuatFixPData;
	
	/* Create packet and place in queue */

	pHifPacket = (Buffer_t *)PacketMem;

	pPayload = M_GetBufferDataStart(pHifPacket);

//	sensorType = ResultToAndroidTypeMap(sensorId);
	if (GetSensorState(sensorType) == 0)
		return;

	SensorHubAssertInt();	/* Assert interrupt to host */

	/* Process sensor and format into packet */
	switch (sensorType) {
	case AP_PSENSOR_ACCELEROMETER_UNCALIBRATED:
	case SENSOR_MAGNETIC_FIELD_UNCALIBRATED:
	case SENSOR_GYROSCOPE_UNCALIBRATED:
	case SENSOR_PRESSURE:
	case SENSOR_STEP_COUNTER:
		UnCalFixPData.TimeStamp.TS64 = pMsg->timeStamp;
		UnCalFixPData.Axis[0]	= pMsg->X;
		UnCalFixPData.Axis[1]	= pMsg->Y;
		UnCalFixPData.Axis[2]	= pMsg->Z;
		UnCalFixPData.Offset[0]	= 0;
		UnCalFixPData.Offset[1]	= 0;
		UnCalFixPData.Offset[2]	= 0;

		pHifPacket->Header.Length = FormatUncalibratedPktFixP(pPayload,
			&UnCalFixPData, META_DATA_UNUSED, sensorType);
		break;

	case SENSOR_ACCELEROMETER:
	case SENSOR_MAGNETIC_FIELD:
	case SENSOR_GYROSCOPE:
		CalFixPData.TimeStamp.TS64 = pMsg->timeStamp;
		CalFixPData.Axis[0]	= pMsg->X;
		CalFixPData.Axis[1]	= pMsg->Y;
		CalFixPData.Axis[2]	= pMsg->Z;
		
		pHifPacket->Header.Length = FormatCalibratedPktFixP(pPayload,
			&CalFixPData, sensorType);
		break;
	case SENSOR_ROTATION_VECTOR:
	case SENSOR_GEOMAGNETIC_ROTATION_VECTOR:
	case SENSOR_GAME_ROTATION_VECTOR:
		QuatFixPData.TimeStamp.TS64 = pMsg->timeStamp;
		QuatFixPData.Quat[0]	= pMsg->W;
		QuatFixPData.Quat[1]	= pMsg->X;
		QuatFixPData.Quat[2]	= pMsg->Y;
		QuatFixPData.Quat[3]	= pMsg->Z;

		pHifPacket->Header.Length = FormatQuaternionPktFixP(pPayload,
			&QuatFixPData, sensorType);
		break;
	case SENSOR_ORIENTATION:
	case SENSOR_GRAVITY:
	case SENSOR_LINEAR_ACCELERATION:
		CalFixPData.TimeStamp.TS64 = pMsg->timeStamp;
		CalFixPData.Axis[0]	= pMsg->X;
		CalFixPData.Axis[1]	= pMsg->Y;
		CalFixPData.Axis[2]	= pMsg->Z;

		pHifPacket->Header.Length = FormatCalibratedPktFixP(pPayload,
			&CalFixPData, sensorType);
		break;
	default:
		return;
	}

	/* Enqueue packet in HIF queue */
	/* status = EnQueue(_HiFQueue, pHifPacket); */
	QueueOverFlow = FastData_add(&FDC, pHifPacket);
}

extern volatile int i2cdoneflag;

/*------------------------------------------------------------------------*\
 |    P U B L I C     F U N C T I O N S
\*------------------------------------------------------------------------*/
static int proc_stat = 0;
uint8_t cmdlog[512];
int	cmdidx = 0;
uint8_t process_command(uint8_t *rx_buf, uint16_t length)
{
	uint8_t remain  = 0;
	int16_t		status;
	SensorPacketTypes_t Out;
	int		pack_sz;
	int		fdq;
	static 		int space = 0;

	if (length < 1) return 0;
	cmdlog[cmdidx] = rx_buf[0];
	cmdidx++;
	cmdidx%=512;
	switch (rx_buf[0]) {
#if 0
	case OSP_DATA_LEN:	/* Get length */
		/* Note - Hostif_StartTx retains the pointer,
		 * i.e. it does NOT copy the data
		 */
		Hostif_StartTx((uint8_t *)(&SlaveRegMap.read_len), sizeof(SlaveRegMap.read_len), __LINE__);

		break;
#endif
	case OSP_DATA_LEN_L:
#if 0
		Hostif_StartTx((uint8_t *)(&SlaveRegMap.read_len2), sizeof(SlaveRegMap.read_len2), __LINE__);
#endif
		CHostif_StartTxChained((uint8_t *)&(SlaveRegMap.read_len2), sizeof(SlaveRegMap.read_len2), 
					(uint8_t *)SlaveRegMap.rd_mem, SlaveRegMap.read_len2, __LINE__);

		break;
	case OSP_DATA_LEN_H:
#if 0
		Hostif_StartTx((uint8_t *)(&SlaveRegMap.read_len2)+1, sizeof(SlaveRegMap.read_len2)-1, __LINE__);
#endif
		CHostif_StartTxChained((uint8_t *)&(SlaveRegMap.read_len2)+1, sizeof(SlaveRegMap.read_len2)-1, 
					(uint8_t *)SlaveRegMap.rd_mem, SlaveRegMap.read_len2, __LINE__);

		break;
	case OSP_INT_LEN:
		if (QueueOverFlow) {
			SlaveRegMap.intlen = OSP_INT_OVER;
		} else {
			SlaveRegMap.intlen = OSP_INT_NONE;
		}
		pack_sz = 0;
		fdq = FastData_get(&FDC);
		if (fdq >= 0) {
			SlaveRegMap.intlen = OSP_INT_DRDY;

			memcpy(SlaveRegMap.rd_mem, (void *)(FDC.Databuf[fdq].DataChunk), FDC.Databuf[fdq].length);
			pack_sz = FDC.Databuf[fdq].length;
			if (pack_sz < 2) {
				__ASM volatile("BKPT #01");
			}
			FDC.read++;
			FDC.read %= FASTDATA_NUM_Q;
		} else {
            // No data to send so remove host interrupt assert signal 
			SensorHubDeAssertInt();        
		}
		SlaveRegMap.intlen |= (pack_sz << 4);
		SlaveRegMap.read_len2 = pack_sz;
		if (pack_sz == 0) {
			Hostif_StartTx((uint8_t *)&(SlaveRegMap.intlen), sizeof(SlaveRegMap.intlen), __LINE__);
		} else {

			CHostif_StartTxChained((uint8_t *)&(SlaveRegMap.intlen), sizeof(SlaveRegMap.intlen), 
					(uint8_t *)SlaveRegMap.rd_mem, pack_sz, __LINE__);
#if 0
			D0_printf("Sending - len %i, pack %i intlen %x state %08x %08x\r\n", 
				pack_sz, FDC.Databuf[fdq].state, SlaveRegMap.intlen, SensorState[0], SensorState[1]);
#endif
		}
		break;
	case OSP_INT_REASON:
		if (QueueOverFlow) {
			SlaveRegMap.irq_cause = OSP_INT_OVER;
		} else {
			SlaveRegMap.irq_cause = OSP_INT_NONE;
		}
		pack_sz = 0;
		fdq = FastData_get(&FDC);
		if (fdq >= 0) {
			SlaveRegMap.irq_cause = OSP_INT_DRDY;
			Hostif_StartTx((uint8_t *)&(SlaveRegMap.irq_cause), sizeof(SlaveRegMap.irq_cause), __LINE__);

			memcpy(SlaveRegMap.rd_mem, (void *)(FDC.Databuf[fdq].DataChunk), FDC.Databuf[fdq].length);
			pack_sz = FDC.Databuf[fdq].length;
			if (pack_sz < 2) {
				__ASM volatile("BKPT #01");
			}
			FDC.read++;
			FDC.read %= FASTDATA_NUM_Q;
		} else {
			Hostif_StartTx((uint8_t *)&(SlaveRegMap.irq_cause), sizeof(SlaveRegMap.irq_cause), __LINE__);
			SensorHubDeAssertInt();
		}
		SlaveRegMap.read_len = pack_sz;
		SlaveRegMap.read_len2 = pack_sz;

		break;
	case OSP_WHOAMI:		/* Who am */
		Hostif_StartTx((uint8_t *)(&SlaveRegMap.whoami), sizeof(SlaveRegMap.whoami), __LINE__);
		break;
	case OSP_VERSION0:
		Hostif_StartTx((uint8_t *)(&SlaveRegMap.version0), sizeof(SlaveRegMap.version0), __LINE__);
		break;
	case OSP_VERSION1:
		Hostif_StartTx((uint8_t *)(&SlaveRegMap.version1), sizeof(SlaveRegMap.version1), __LINE__);
		break;

	case OSP_CONFIG:		/* Reset, not implemented yet */
		/* Need to flush queue somewhere */
		QueueOverFlow = 0;
		ClearQueue = 1;
		break;
	case OSP_DATA_OUT:		/* Read data */
		if (SlaveRegMap.read_len2 == 0)
			break;
		if (SlaveRegMap.read_len2 < 3 && SlaveRegMap.read_len2 > 0) {
			__ASM volatile("BKPT #01");
		}
#if 0
		Hostif_StartTx((uint8_t *)SlaveRegMap.rd_mem, SlaveRegMap.read_len2, __LINE__);
#endif
		break;
	case OSP_DATA_IN:		/* Write data */
		/* Host has written a packet */
		ParseHostIntefacePkt(&Out, &rx_buf[1], length-1);
		break;
	case 0x1f:
		space = 0;
		break;
	case 0x1e:
		space = 1;
		break;
	default:
		if (rx_buf[0] >= 0x20 && rx_buf[0] < 0x50) {
			/* ENABLE */
			SensorEnable((ASensorType_t)(rx_buf[0]-0x20), space);
			D0_printf("Enable %i\r\n", rx_buf[0] - 0x20);
		} else if (rx_buf[0] >= 0x50 && rx_buf[0] < 0x80) {
			/* DISABLE */
			SensorDisable((ASensorType_t)(rx_buf[0]-0x50), space);
			D0_printf("Disable %i\r\n", rx_buf[0] - 0x50);
		}
		break;
	}
	return remain;
}

/**********************************************************************
 * @fn      I2CCommTask
 *          This tasks primary goal is to serialize the communication
 *	    request (sensor results) going over I2C
 *
 * @param   none
 *
 * @return  none
 *
 **********************************************************************/
ASF_TASK void I2CCommTask(ASF_TASK_ARG)
{
	MessageBuffer *rcvMsg = NULLP;

	FastData_init(&FDC);

	Hostif_Init();
	/* Active high int, set to in active */
	SensorHubDeAssertInt();
	/* Init register area for slave */
	SH_Slave_init();

	SensorState[0] = 0;
	SensorState[1] = 0;
	SensorState[0] = 1 << SENSOR_SIGNIFICANT_MOTION;

    D0_printf("%s-> I2C Slave ready\r\n", __FUNCTION__);
	while(1) {
		ASFReceiveMessage(I2CSLAVE_COMM_TASK_ID, &rcvMsg );
        
        //QLY
      //  D0_printf("%s-> Received message id: %d\r\n", rcvMsg->msgId);
		switch (rcvMsg->msgId) {
		case MSG_ACC_DATA:
			SendSensorData(AP_PSENSOR_ACCELEROMETER_UNCALIBRATED,
					&rcvMsg->msg.msgAccelData);
			break;
		case MSG_MAG_DATA:
			SendSensorData(SENSOR_MAGNETIC_FIELD_UNCALIBRATED,
					&rcvMsg->msg.msgMagData);
			break;
		case MSG_GYRO_DATA:
			SendSensorData(SENSOR_GYROSCOPE_UNCALIBRATED,
					&rcvMsg->msg.msgGyroData);
			break;
		case MSG_CAL_ACC_DATA:
			SendSensorData(SENSOR_ACCELEROMETER,
					&rcvMsg->msg.msgAccelData);
			break;
		case MSG_CAL_MAG_DATA:
			SendSensorData(SENSOR_MAGNETIC_FIELD,
					&rcvMsg->msg.msgMagData);
			break;
		case MSG_CAL_GYRO_DATA:
			SendSensorData(SENSOR_GYROSCOPE,
					&rcvMsg->msg.msgGyroData);
			break;
		case MSG_ORIENTATION_DATA:
			SendSensorData(SENSOR_ORIENTATION,
					&rcvMsg->msg.msgOrientationData);
			break;
		case MSG_QUATERNION_DATA:
			SendSensorData(SENSOR_ROTATION_VECTOR,
					&rcvMsg->msg.msgQuaternionData);
			break;
		case MSG_GEO_QUATERNION_DATA:
			SendSensorData(SENSOR_GEOMAGNETIC_ROTATION_VECTOR,
					&rcvMsg->msg.msgQuaternionData);
			break;
		case MSG_GAME_QUATERNION_DATA:
			SendSensorData(SENSOR_GAME_ROTATION_VECTOR,
					&rcvMsg->msg.msgQuaternionData);
			break;
		
		case MSG_STEP_COUNT_DATA:
			SendSensorData(SENSOR_STEP_COUNTER,
					&rcvMsg->msg.msgStepCountData);
			break;
		case MSG_CD_SEGMENT_DATA:
			break;
		case MSG_LINEAR_ACCELERATION_DATA:
			SendSensorData(SENSOR_LINEAR_ACCELERATION,
					&rcvMsg->msg.msgLinearAccelerationData);
			break;
		case MSG_GRAVITY_DATA:
			SendSensorData(SENSOR_GRAVITY,
					&rcvMsg->msg.msgGravityData);
			break;
		case MSG_STEP_DETECT_DATA:
			SendSensorBoolData(SENSOR_STEP_DETECTOR,
					&rcvMsg->msg.msgStepDetData);
			break;
		case MSG_SIG_MOTION_DATA:
			SendSensorBoolData(SENSOR_SIGNIFICANT_MOTION,
					&rcvMsg->msg.msgSigMotionData);
			D0_printf("SigMotion\n");
			break;
		case MSG_PRESS_DATA:
			SendSensorData(SENSOR_PRESSURE,
					&rcvMsg->msg.msgPressData);
			break;
		default:
			D1_printf("I2C:!!!UNHANDLED MESSAGE:%d!!!\r\n", rcvMsg->msgId);
			break;
        }
        ASFDeleteMessage(I2CSLAVE_COMM_TASK_ID, &rcvMsg );
    }
}
#endif //ANDROID_COMM_TASK

/*-------------------------------------------------------------------------------------------------*\
 |    E N D   O F   F I L E
\*-------------------------------------------------------------------------------------------------*/
