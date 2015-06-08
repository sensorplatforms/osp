/****************************************************************************************************
 * @file  SensorPackets.c
 *
 * Packet Handling routines
 *
 * < Detailed description of subsystem >
 *
 * @seealso < Links to relevant files, references, etc. >
 *
 * @info
 * $Author: rverma $
 * $DateTime: 2014/11/13 17:24:40 $
 * $Revision: #8 $
 * $Id: //AudEngr/Hardware/DeltaPlus/Software/BoskoApp_D300_SensorHub/overlay/SensorHub/HostInterface/SensorPackets.c#8 $
 *
 * Copyright 2012 Audience, Incorporated. All rights reserved.
 *
 * This software is the confidential and proprietary information of
 * Audience Inc. ("Confidential Information"). You shall not disclose 
 * such Confidential Information and shall use it only in accordance 
 * with the Terms of Sale of Audience products and the terms of any 
 * license agreement you entered into with Audience for such products.
 * 
 * AUDIENCE SOURCE CODE STRICTLY "AS IS" WITHOUT ANY WARRANTY  
 * WHATSOEVER, AND AUDIENCE EXPRESSLY DISCLAIMS ALL WARRANTIES, 
 * EXPRESS, IMPLIED OR STATUTORY WITH REGARD THERETO, INCLUDING THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR 
 * PURPOSE, TITLE OR NON-NFRINGEMENT OF THIRD PARTY RIGHTS. AUDIENCE 
 * SHALL NOT BE LIABLE FOR ANY DAMAGES SUFFERED BY YOU AS A RESULT OF 
 * USING, MODIFYING OR DISTRIBUTING THIS SOFTWARE OR ITS DERIVATIVES. 
 *
 ***************************************************************************************************/
/*-------------------------------------------------------------------------------------------------*\
 |    I N C L U D E   F I L E S
\*-------------------------------------------------------------------------------------------------*/
#include "SensorPackets.h"
#include "MQErrorCodes.h"
#include "osp-types.h"
/*-------------------------------------------------------------------------------------------------*\
 |    E X T E R N A L   V A R I A B L E S   &   F U N C T I O N S
\*-------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------*\
 |    P R I V A T E   C O N S T A N T S   &   M A C R O S
\*-------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------*\
 |    P R I V A T E   T Y P E   D E F I N I T I O N S
\*-------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------*\
 |    S T A T I C   V A R I A B L E S   D E F I N I T I O N S
\*-------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------*\
 |    F O R W A R D   F U N C T I O N   D E C L A R A T I O N S
\*-------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------*\
 |    P U B L I C   V A R I A B L E S   D E F I N I T I O N S
\*-------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------*\
 |    P R I V A T E     F U N C T I O N S
\*-------------------------------------------------------------------------------------------------*/
/* if defined (MQ_UNIT_TEST) && defined (TEST_SENSOR_DATA_PKT)  */
#if 1
/****************************************************************************************************
 * @fn      ParseSensorDataPkt
 *          Top level parser for Sensor Data Packets.
 *
 * @param   [OUT]pOut - Sensor structure that will return the parsed values
 * @param   [IN]pPacket - Packet buffer containing the packet to parse
 * @param   [IN]pktSize - Size of the packet buffer provided
 *
 * @return  MQ_SUCCESS or Error code enum corresponding to the error encountered
 *
 ***************************************************************************************************/
static int16_t ParseSensorDataPkt( SensorPacketTypes_t *pOut, uint8_t *pPacket, uint16_t pktSize )
{
    HostIFPackets_t *pHif = (HostIFPackets_t*)pPacket;
    int16_t errCode = -MQ_UNSUPPORTED_FEATURE;
    uint8_t sensType;
    uint8_t sensSubType, dSize, dFormat, timeFormat, tSize, isPrivateType, hasMetaData;
    uint8_t i;
    int16_t lengthParsed;

    /* Sanity... */
    if ((pOut == NULL) || (pPacket == NULL))
        return (-MQ_BAD_BUFFER);

    /* Get sensor type. */
    sensType = M_SensorType(pHif->SensPktRaw.Q.SensorIdByte);
    sensSubType = M_ParseSensorSubType(pHif->SensPktRaw.Q.AttributeByte);
    isPrivateType = pHif->SensPktRaw.Q.ControlByte & SENSOR_ANDROID_TYPE_MASK;
    hasMetaData = M_ParseSensorMetaData (pHif->SensPktRaw.Q.SensorIdByte);
    dSize = pHif->SensPktRaw.Q.AttributeByte & DATA_SIZE_MASK;
    dFormat = pHif->SensPktRaw.Q.ControlByte & DATA_FORMAT_MASK;
    timeFormat = pHif->SensPktRaw.Q.ControlByte & TIME_FORMAT_MASK;
    tSize = pHif->SensPktRaw.Q.AttributeByte & TIME_STAMP_SIZE_MASK;

    /* Check Sensor enumeration type Android or Private */  
    if (!isPrivateType)
    {
        /*Sensor Enumeration type is Android*/
        switch ((ASensorType_t)sensType)
        {
        case A_SENSOR_ACCELEROMETER:
        case A_SENSOR_MAGNETIC_FIELD:
        case A_SENSOR_GYROSCOPE:
            if ((dSize == DATA_SIZE_32_BIT) && (dFormat == DATA_FORMAT_FIXPOINT) &&
                (timeFormat == TIME_FORMAT_FIXPOINT) && (tSize == TIME_STAMP_64_BIT))
            {
                /* Extract sensor data from packet */
                pOut->SType = (ASensorType_t)sensType;
                pOut->SubType = SENSOR_SUBTYPE_UNUSED;
                pOut->P.CalFixP.Axis[0] = BYTES_TO_LONG(pHif->CalPktFixP.Data[0], pHif->CalPktFixP.Data[1],
                    pHif->CalPktFixP.Data[2], pHif->CalPktFixP.Data[3]);
                pOut->P.CalFixP.Axis[1] = BYTES_TO_LONG(pHif->CalPktFixP.Data[4], pHif->CalPktFixP.Data[5],
                    pHif->CalPktFixP.Data[6], pHif->CalPktFixP.Data[7]);
                pOut->P.CalFixP.Axis[2] = BYTES_TO_LONG(pHif->CalPktFixP.Data[8], pHif->CalPktFixP.Data[9],
                    pHif->CalPktFixP.Data[10], pHif->CalPktFixP.Data[11]);

                /* Extract fixed point time stamp */
                for (i = 0; i < sizeof(uint64_t); i++)
                {
                    /* Copy LSB to MSB data - remember that HIF packets are Big-Endian formatted */
                    pOut->P.CalFixP.TimeStamp.TS8[i] = pHif->CalPktFixP.TimeStamp[sizeof(uint64_t)-i-1];
                }
                errCode = MQ_SUCCESS;
                lengthParsed = CALIBRATED_FIXP_DATA_PKT_SZ;
            }
            break;

        case A_SENSOR_ROTATION_VECTOR:
            if ((dSize == DATA_SIZE_32_BIT) && (dFormat == DATA_FORMAT_FIXPOINT) &&
                (timeFormat == TIME_FORMAT_FIXPOINT) && (tSize == TIME_STAMP_64_BIT))
            {
                /* Extract Quaternion data from packet */
                pOut->SType = (ASensorType_t)sensType;
                pOut->SubType = SENSOR_SUBTYPE_UNUSED;
                pOut->P.QuatFixP.Quat[0] = BYTES_TO_LONG(pHif->QuatPktFixP.Data[0], pHif->QuatPktFixP.Data[1],
                    pHif->QuatPktFixP.Data[2], pHif->QuatPktFixP.Data[3]);
                pOut->P.QuatFixP.Quat[1] = BYTES_TO_LONG(pHif->QuatPktFixP.Data[4], pHif->QuatPktFixP.Data[5],
                    pHif->QuatPktFixP.Data[6], pHif->QuatPktFixP.Data[7]);
                pOut->P.QuatFixP.Quat[2] = BYTES_TO_LONG(pHif->QuatPktFixP.Data[8], pHif->QuatPktFixP.Data[9],
                    pHif->QuatPktFixP.Data[10], pHif->QuatPktFixP.Data[11]);
                pOut->P.QuatFixP.Quat[3] = BYTES_TO_LONG(pHif->QuatPktFixP.Data[12], pHif->QuatPktFixP.Data[13],
                    pHif->QuatPktFixP.Data[14], pHif->QuatPktFixP.Data[15]);

                /* Extract fixed point time stamp */
                for (i = 0; i < sizeof(uint64_t); i++)
                {
                    /* Copy LSB to MSB data - remember that HIF packets are Big-Endian formatted */
                    pOut->P.QuatFixP.TimeStamp.TS8[i] = pHif->QuatPktFixP.TimeStamp[sizeof(uint64_t)-i-1];
                }
                errCode = MQ_SUCCESS;
                lengthParsed = QUATERNION_FIXP_DATA_PKT_SZ;
            }
            break;

        case A_SENSOR_MAGNETIC_FIELD_UNCALIBRATED:
        case A_SENSOR_GYROSCOPE_UNCALIBRATED:
            if ((dSize == DATA_SIZE_32_BIT) && (dFormat == DATA_FORMAT_FIXPOINT) &&
                (timeFormat == TIME_FORMAT_FIXPOINT) && (tSize == TIME_STAMP_64_BIT))
            {
                /* Extract Quaternion data from packet */
                pOut->SType = (ASensorType_t)sensType;
                pOut->SubType = SENSOR_SUBTYPE_UNUSED;
                pOut->P.UncalFixP.Axis[0] = BYTES_TO_LONG(pHif->UncalPktFixP.Data[0], pHif->UncalPktFixP.Data[1],
                    pHif->UncalPktFixP.Data[2], pHif->UncalPktFixP.Data[3]);
                pOut->P.UncalFixP.Axis[1] = BYTES_TO_LONG(pHif->UncalPktFixP.Data[4], pHif->UncalPktFixP.Data[5],
                    pHif->UncalPktFixP.Data[6], pHif->UncalPktFixP.Data[7]);
                pOut->P.UncalFixP.Axis[2] = BYTES_TO_LONG(pHif->UncalPktFixP.Data[8], pHif->UncalPktFixP.Data[9],
                    pHif->UncalPktFixP.Data[10], pHif->UncalPktFixP.Data[11]);

                /* Check if META_DATA is set to 0x01 then read offset */
                if (hasMetaData)
                {
                    pOut->P.UncalFixP.Offset[0] = BYTES_TO_LONG(pHif->UncalPktFixP.Offset[0], pHif->UncalPktFixP.Offset[1],
                        pHif->UncalPktFixP.Offset[2], pHif->UncalPktFixP.Offset[3]);
                    pOut->P.UncalFixP.Offset[1] = BYTES_TO_LONG(pHif->UncalPktFixP.Offset[4], pHif->UncalPktFixP.Offset[5],
                        pHif->UncalPktFixP.Offset[6], pHif->UncalPktFixP.Offset[7]);
                    pOut->P.UncalFixP.Offset[2] = BYTES_TO_LONG(pHif->UncalPktFixP.Offset[8], pHif->UncalPktFixP.Offset[9],
                        pHif->UncalPktFixP.Offset[10], pHif->UncalPktFixP.Offset[11]);

                    lengthParsed = UNCALIB_FIXP_DATA_OFFSET_PKT_SZ;
                }
                else
                {
                    lengthParsed = UNCALIB_FIXP_DATA_PKT_SZ;
                }
                /* Extract fixed point time stamp */
                for (i = 0; i < sizeof(uint64_t); i++)
                {
                    /* Copy LSB to MSB data - remember that HIF packets are Big-Endian formatted */
                    pOut->P.UncalFixP.TimeStamp.TS8[i] = pHif->UncalPktFixP.TimeStamp[sizeof(uint64_t)-i-1];
                }
                errCode = MQ_SUCCESS;
            }
            break;

        case A_SENSOR_GAME_ROTATION_VECTOR:
            break;

        case A_SENSOR_SIGNIFICANT_MOTION:
            break;

        case A_SENSOR_STEP_DETECTOR:
            break;

        case A_SENSOR_STEP_COUNTER:
            break;

        case A_SENSOR_GEOMAGNETIC_ROTATION_VECTOR:
            break;

        default:
            break;
        }

    }
    else
    {
        /*Sensor Enumeration type is Private*/
        switch ((PSensorType_t)sensType)
        {
        case PSENSOR_ACCELEROMETER_RAW:
        case PSENSOR_MAGNETIC_FIELD_RAW:
        case PSENSOR_GYROSCOPE_RAW:
            if ((sensSubType == SENSOR_SUBTYPE_UNUSED) && (dSize == DATA_SIZE_16_BIT) &&
                (dFormat == DATA_FORMAT_RAW) && (timeFormat == TIME_FORMAT_RAW))
            {
                /* Extract Raw sensor data from packet */
                pOut->SType = (ASensorType_t)M_PSensorToAndroidBase(sensType);
                pOut->SubType = SENSOR_SUBTYPE_UNUSED;
                pOut->P.RawSensor.Axis[0] = BYTES_TO_SHORT(pHif->SensPktRaw.DataRaw[0], pHif->SensPktRaw.DataRaw[1]);
                pOut->P.RawSensor.Axis[1] = BYTES_TO_SHORT(pHif->SensPktRaw.DataRaw[2], pHif->SensPktRaw.DataRaw[3]);
                pOut->P.RawSensor.Axis[2] = BYTES_TO_SHORT(pHif->SensPktRaw.DataRaw[4], pHif->SensPktRaw.DataRaw[5]);
                
#if 0
                if (g_logging & 0x10)
                {
                    D1_printf("HIF: %d, %02X, %02X, %04X\r\n", sizeof(HostIFPackets_t),
                        pHif->SensPktRaw.DataRaw[0], pHif->SensPktRaw.DataRaw[1], pOut->P.RawSensor.Axis[0]);
                }
#endif
                /* Extract time stamp */
                pOut->P.RawSensor.TStamp.TS64 = 0; //helps clear higher 32-bit
                pOut->P.RawSensor.TStamp.TS8[3] = pHif->SensPktRaw.TimeStamp[0]; //MSB
                pOut->P.RawSensor.TStamp.TS8[2] = pHif->SensPktRaw.TimeStamp[1];
                pOut->P.RawSensor.TStamp.TS8[1] = pHif->SensPktRaw.TimeStamp[2];
                pOut->P.RawSensor.TStamp.TS8[0] = pHif->SensPktRaw.TimeStamp[3]; //LSB
                //TODO: 64-bit time stamp extension??
                errCode = MQ_SUCCESS;
                lengthParsed = SENSOR_RAW_DATA_PKT_SZ;
            }
            break;

        case PSENSOR_ACCELEROMETER_UNCALIBRATED:

            if ((dSize == DATA_SIZE_32_BIT) && (dFormat == DATA_FORMAT_FIXPOINT) &&
                (timeFormat == TIME_FORMAT_FIXPOINT) && (tSize == TIME_STAMP_64_BIT))
            {
                /* Extract uncalibrated sensor data from packet */
                pOut->SType = (ASensorType_t)M_PSensorToAndroidBase(sensType);
                pOut->SubType = SENSOR_SUBTYPE_UNUSED;
                pOut->P.UncalFixP.Axis[0] = BYTES_TO_LONG(pHif->UncalPktFixP.Data[0], pHif->UncalPktFixP.Data[1],
                    pHif->UncalPktFixP.Data[2], pHif->UncalPktFixP.Data[3]);
                pOut->P.UncalFixP.Axis[1] = BYTES_TO_LONG(pHif->UncalPktFixP.Data[4], pHif->UncalPktFixP.Data[5],
                    pHif->UncalPktFixP.Data[6], pHif->UncalPktFixP.Data[7]);
                pOut->P.UncalFixP.Axis[2] = BYTES_TO_LONG(pHif->UncalPktFixP.Data[8], pHif->UncalPktFixP.Data[9],
                    pHif->UncalPktFixP.Data[10], pHif->UncalPktFixP.Data[11]);

                /* Check if META_DATA is set to 0x01 then read offset */
                if (hasMetaData)
                {
                    pOut->P.UncalFixP.Offset[0] = BYTES_TO_LONG(pHif->UncalPktFixP.Offset[0], pHif->UncalPktFixP.Offset[1],
                        pHif->UncalPktFixP.Offset[2], pHif->UncalPktFixP.Offset[3]);
                    pOut->P.UncalFixP.Offset[1] = BYTES_TO_LONG(pHif->UncalPktFixP.Offset[4], pHif->UncalPktFixP.Offset[5],
                        pHif->UncalPktFixP.Offset[6], pHif->UncalPktFixP.Offset[7]);
                    pOut->P.UncalFixP.Offset[2] = BYTES_TO_LONG(pHif->UncalPktFixP.Offset[8], pHif->UncalPktFixP.Offset[9],
                        pHif->UncalPktFixP.Offset[10], pHif->UncalPktFixP.Offset[11]);

                    lengthParsed = UNCALIB_FIXP_DATA_OFFSET_PKT_SZ;
                }
                else
                {
                    lengthParsed = UNCALIB_FIXP_DATA_PKT_SZ;
                }
                /* Extract fixed point time stamp */
                for (i = 0; i < sizeof(uint64_t); i++)
                {
                    /* Copy LSB to MSB data - remember that HIF packets are Big-Endian formatted */
                    pOut->P.UncalFixP.TimeStamp.TS8[i] = pHif->UncalPktFixP.TimeStamp[sizeof(uint64_t)-i-1];
                }
                errCode = MQ_SUCCESS;
            }
            break;

        case SYSTEM_REAL_TIME_CLOCK:
            break;

        default:
            break;
        }
    }
    if (errCode == MQ_SUCCESS)
        return lengthParsed;
    else
        return errCode;
}
#endif

/*-------------------------------------------------------------------------------------------------*\
 |    P U B L I C     F U N C T I O N S
\*-------------------------------------------------------------------------------------------------*/

/****************************************************************************************************
 * @fn      FormatSensorDataPktRaw
 *          Using the sensor sample data, this function creates the Sensor Data Packet for sending
 *          to host in the buffer provided.
 *
 * @param   [OUT]pDest - Destination buffer supplied by the caller. This buffer size must be at least
 *                  SENSOR_DATA_PKT_RAW_SZ in length
 * @param   [IN]pSensData - Data structure carrying the raw sensor sample (typically from driver)
 * @param   [IN]sensorInstance - 0 base instance identifier if multiple sensor of same type is used
 * @param   [IN]metaData - Meta data for the sensor type used (not applicable for all sensor types)
 * @param   [IN]sType - Sensor Type for the sensor data presented
 * @param   [IN]subType - Sub type for the sensor type used (not applicable for all sensor types)
 *
 * @return  Size of the formatted packet or -Error code enum corresponding to the error encountered
 *
 ***************************************************************************************************/
int16_t FormatSensorDataPktRaw( uint8_t *pDest, const TriAxisRawData_t *pSensData, uint8_t metaData,
    ASensorType_t sType, uint8_t subType )
{
    uint8_t i = 0;
    int8_t k;
    HifSensorDataRaw_t *pOut = (HifSensorDataRaw_t *)pDest;

    /* Sanity checks... */
    if (pOut == NULL)
    {
        return (-MQ_BAD_BUFFER);
    }

    /* Check Sensor enumeration type Android or User defined */ 
    if (!(M_GetSensorType(sType)))
    {
        pOut->Q.ControlByte = SENSOR_TYPE_ANDROID;

    }
    else
    {
        pOut->Q.ControlByte = SENSOR_TYPE_PRIVATE;

    }
    /* Setup Control Byte */
    pOut->Q.ControlByte |= PKID_SENSOR_DATA | DATA_FORMAT_RAW | TIME_FORMAT_RAW;

    /* Setup Sensor ID Byte */
    pOut->Q.SensorIdByte = M_SensorMetaData(metaData) | M_SensorType(sType);

    /* Setup Attribute Byte */
    pOut->Q.AttributeByte = M_SensorSubType(subType) | DATA_SIZE_16_BIT | TIME_STAMP_32_BIT;

    /* Time Stamp: We pass only the lower 32-bit in this packet. Note that byte ordering is Big Endian */
    for (k = sizeof(uint32_t)-1; k >= 0; k--)
    {
        pOut->TimeStamp[i++] = pSensData->TStamp.TS8[k]; //MSB to LSB copy
    }

    /* Data bytes (Big Endian order) */
    i = 0;
    for (k = 0; k < 3; k++)
    {
        pOut->DataRaw[i++] = BYTE1(pSensData->Axis[k]);  //MSB of 16-bit sensor data
        pOut->DataRaw[i++] = BYTE0(pSensData->Axis[k]);  //LSB of 16-bit sensor data
    }

    /* No Checksum... we are done! */
    return SENSOR_RAW_DATA_PKT_SZ;
}


/****************************************************************************************************
 * @fn      FormatQuaternionPktFixP
 *          Using the given quaternion data, this function creates the HIF Data Packet for sending
 *          to host in the buffer provided.
 *
 * @param   [OUT]pDest - Destination buffer supplied by the caller. This buffer size must be at least
 *                  QUATERNION_PKT_FIXP_SZ in length
 * @param   [IN]pQuatData - Data structure carrying the quaternion result from fusion
 * @param   [IN]sType - Android sensor type for the quaternion data
 *
 * @return  Size of the formatted packet or -Error code enum corresponding to the error encountered
 *
 ***************************************************************************************************/
int16_t FormatQuaternionPktFixP( uint8_t *pDest, const QuaternionFixP_t *pQuatData, ASensorType_t sType )
{
    uint8_t i = 0;
    int8_t k;
    HifQuaternionFixPoint_t *pOut = (HifQuaternionFixPoint_t *)pDest;

    /* Sanity checks... */
    if (pOut == NULL)
    {
        return (-MQ_BAD_BUFFER);
    }
    /* Check Sensor enumeration type Android or User defined */ 
    if (!(M_GetSensorType(sType)))
    {
        pOut->Q.ControlByte = SENSOR_TYPE_ANDROID;

    }
    else
    {
        pOut->Q.ControlByte = SENSOR_TYPE_PRIVATE;

    }

    /* Setup Control Byte */
    pOut->Q.ControlByte |= PKID_SENSOR_DATA | DATA_FORMAT_FIXPOINT | TIME_FORMAT_FIXPOINT;

    /* Setup Sensor ID Byte */
    pOut->Q.SensorIdByte =  M_SensorType(sType);

    /* Setup Attribute Byte */
    pOut->Q.AttributeByte = DATA_SIZE_32_BIT | TIME_STAMP_64_BIT;

    /* time stamp - note that byte ordering is Big Endian */
    for (k = sizeof(uint64_t)-1; k >= 0; k--)
    {
        pOut->TimeStamp[i++] = pQuatData->TimeStamp.TS8[k]; //MSB to LSB copy
    }
    i = 0;

    /* Data bytes (Big Endian order) */
    for (k = 0; k < 4; k++)
    {
        pOut->Data[i++] = BYTE3(pQuatData->Quat[k]);  //MSB of 32-bit data
        pOut->Data[i++] = BYTE2(pQuatData->Quat[k]);
        pOut->Data[i++] = BYTE1(pQuatData->Quat[k]);
        pOut->Data[i++] = BYTE0(pQuatData->Quat[k]);  //LSB of 32-bit data
    }

    /* No Checksum... we are done! */
    return QUATERNION_FIXP_DATA_PKT_SZ;
}


/****************************************************************************************************
 * @fn      FormatUncalibratedPktFixP
 *          Using the given uncalibrated data, this function creates the Sensor Data Packet for sending
 *          to host in the buffer provided.
 *
 * @param   [OUT]pDest - Destination buffer supplied by the caller. This buffer size must be at least
 *                  UNCALIB_FIXP_DATA_PKT_SZ or UNCALIB_FIXP_DATA_OFFSET_PKT_SZ in length
 * @param   [IN]pUncalData - Data structure carrying the uncalibrated result from Algorithm
 * @param   [IN]metaData - Meta data is used to indicate if the packet contains offset data
 * @param   [IN]sType - Sensor type for the uncalibrated data
 *
 * @return  Size of the formatted packet or -Error code enum corresponding to the error encountered
 *
 ***************************************************************************************************/
int16_t FormatUncalibratedPktFixP( uint8_t *pDest, const UncalibratedFixP_t *pSensData,
    uint8_t metaData, ASensorType_t sType )
{
    uint8_t i = 0;
    int8_t k;
    HifUncalibratedFixPoint_t *pOut = (HifUncalibratedFixPoint_t *)pDest;

    /* Sanity checks... */
    if (pOut == NULL)
    {
        return (-MQ_BAD_BUFFER);
    }
    /* Check Sensor enumeration type Android or User defined */ 
    if (!(M_GetSensorType(sType)))
    {
        pOut->Q.ControlByte = SENSOR_TYPE_ANDROID;

    }
    else
    {
        pOut->Q.ControlByte = SENSOR_TYPE_PRIVATE;

    }

    /* Setup Control Byte */
    pOut->Q.ControlByte |= PKID_SENSOR_DATA | DATA_FORMAT_FIXPOINT | TIME_FORMAT_FIXPOINT;

    /* Setup Sensor ID Byte */
    pOut->Q.SensorIdByte = M_SensorMetaData(metaData) | M_SensorType(sType);

    /* Setup Attribute Byte */
    pOut->Q.AttributeByte = DATA_SIZE_32_BIT | TIME_STAMP_64_BIT;

    /* time stamp - note that byte ordering is Big Endian */
    for (k = sizeof(uint64_t)-1; k >= 0; k--)
    {
        pOut->TimeStamp[i++] = pSensData->TimeStamp.TS8[k]; //MSB to LSB copy
    }
    i = 0;
    
    /* Data bytes (Big Endian order) */
    for (k = 0; k < 3; k++)
    {
        pOut->Data[i++] = BYTE3(pSensData->Axis[k]);  //MSB of 32-bit data
        pOut->Data[i++] = BYTE2(pSensData->Axis[k]);
        pOut->Data[i++] = BYTE1(pSensData->Axis[k]);
        pOut->Data[i++] = BYTE0(pSensData->Axis[k]);  //LSB of 32-bit data
    }

    /* Check if metadata is available  if it is available then add it to HIF packet */
    if (metaData == META_DATA_OFFSET_CHANGE)
    {
        /* Data bytes (Big Endian order) */
        for (k = 0; k < 3; k++)
        {
            pOut->Data[i++] = BYTE3(pSensData->Offset[k]);  //MSB of 32-bit data
            pOut->Data[i++] = BYTE2(pSensData->Offset[k]);
            pOut->Data[i++] = BYTE1(pSensData->Offset[k]);
            pOut->Data[i++] = BYTE0(pSensData->Offset[k]);  //LSB of 32-bit data
        }
        return UNCALIB_FIXP_DATA_OFFSET_PKT_SZ;
    } 

    /* No Checksum... we are done! */
    return UNCALIB_FIXP_DATA_PKT_SZ;
}


/****************************************************************************************************
 * @fn      FormatCalibratedPktFixP
 *          Using the given Calibrated data, this function creates the Sensor Data Packet for sending
 *          to host in the buffer provided.
 *
 * @param   [OUT]pDest - Destination buffer supplied by the caller. This buffer size must be at least
 *                  CALIBRATED_PKT_FIXP_SZ in length
 * @param   [IN]pCalData - Data structure carrying the uncalibrated result from Algorithm
 * @param   [IN]sType - Sensor type for the calibrated data
 *
 * @return  Size of the formatted packet or -Error code enum corresponding to the error encountered
 *
 ***************************************************************************************************/
int16_t FormatCalibratedPktFixP( uint8_t *pDest, const CalibratedFixP_t *pSensData, ASensorType_t sType )
{
    uint8_t i = 0;
    int8_t k;
    HifCalibratedFixPoint_t *pOut = (HifCalibratedFixPoint_t *)pDest;

    /* Sanity checks... */
    if (pOut == NULL)
    {
        return (-MQ_BAD_BUFFER);
    }

    /* Check Sensor enumeration type Android or User defined */ 
    if (!(M_GetSensorType(sType)))
    {
        pOut->Q.ControlByte = SENSOR_TYPE_ANDROID;
    }
    else
    {
        pOut->Q.ControlByte = SENSOR_TYPE_PRIVATE;
    }

    /* Setup Control Byte */
    pOut->Q.ControlByte |= PKID_SENSOR_DATA | DATA_FORMAT_FIXPOINT | TIME_FORMAT_FIXPOINT;

    /* Setup Sensor ID Byte */
    pOut->Q.SensorIdByte = M_SensorType(sType);

    /* Setup Attribute Byte */
    pOut->Q.AttributeByte = DATA_SIZE_32_BIT | TIME_STAMP_64_BIT;

    /* time stamp - note that byte ordering is Big Endian */
    for (k = sizeof(uint64_t)-1; k >= 0; k--)
    {
        pOut->TimeStamp[i++] = pSensData->TimeStamp.TS8[k]; //MSB to LSB copy
    }
    i = 0;
    
    /* Data bytes (Big Endian order) */
    for (k = 0; k < 3; k++)
    {
        pOut->Data[i++] = BYTE3(pSensData->Axis[k]);  //MSB of 32-bit data
        pOut->Data[i++] = BYTE2(pSensData->Axis[k]);
        pOut->Data[i++] = BYTE1(pSensData->Axis[k]);
        pOut->Data[i++] = BYTE0(pSensData->Axis[k]);  //LSB of 32-bit data
    }

    /* No Checksum... we are done! */
    return CALIBRATED_FIXP_DATA_PKT_SZ;
}


/****************************************************************************************************
 * @fn      FormatSensorEnableReq
 *          This function creates the Sensor Control Request Packet for enabling/disabling a sensor
 *          type in the buffer provided.  This packet is sent to hub from host.
 *
 * @param   [OUT]pDest - Destination buffer supplied by the caller. This buffer size must be at least
 *                  4 bytes in length
 * @param   [IN]enable - Boolean type specifying if sensor type must be enabled or disabled.
 * @param   [IN]sType - Sensor type for the calibrated data
 * @param   [IN]subType - Sub type for the sensor type used (not applicable for all sensor types)
 * @param   [IN]seqNum - A unique request number that might be used to distinguish between responses
 *                  Sequence number of 0 implies no response is needed.
 *
 * @return  Size of the formatted packet or -Error code enum corresponding to the error encountered
 *
 ***************************************************************************************************/
int16_t FormatSensorEnableReq( uint8_t *pDest, osp_bool_t enable, ASensorType_t sType, uint8_t subType,
    uint8_t seqNum )
{
    HifSensorEnable_t *pOut = (HifSensorEnable_t*)pDest;
    uint8_t temp = (enable == TRUE)? PARAM_DATA_SZ_BOOL_TRUE : PARAM_DATA_SZ_BOOL_FALSE;

    /* Sanity checks... */
    if (pOut == NULL)
    {
        return (-MQ_BAD_BUFFER);
    }

    /* Check Sensor enumeration type Android or User defined */ 
    if (!(M_GetSensorType(sType)))
    {
        pOut->Q.ControlByte = SENSOR_TYPE_ANDROID;
    }
    else
    {
        pOut->Q.ControlByte = SENSOR_TYPE_PRIVATE;
    }

    /* Setup Control Byte */
    pOut->Q.ControlByte |= PKID_CONTROL_REQ;

    /* Setup Sensor ID Byte */
    pOut->Q.SensorIdByte = M_SensorType(sType);

    /* Setup Attribute Byte 1 */
    pOut->Q.AttributeByte = M_SensorSubType(subType) | M_SequenceNum(seqNum);

    /* Attribute Byte 2 */
    pOut->AttrByte2 = M_SetParamId(SENSOR_PARAM_ENABLE) | temp;

    /* Return the length of the packet */
    return SENSOR_ENABLE_REQ_PKT_SZ;
}


/****************************************************************************************************
 * @fn      ParseControlRequestPkt
 *          Top level parser for Sensor Control Request Packets.
 *
 * @param   [OUT]pOut - Sensor structure that will return the parsed values
 * @param   [IN]pPacket - Packet buffer containing the packet to parse
 * @param   [IN]pktSize - Size of the packet buffer provided
 *
 * @return  MQ_SUCCESS or Error code enum corresponding to the error encountered
 *
 ***************************************************************************************************/
static int16_t ParseControlRequestPkt( SensorPacketTypes_t *pOut, uint8_t *pPacket, uint16_t pktSize )
{
    HostIFPackets_t *pHif = (HostIFPackets_t*)pPacket;
    int16_t errCode = -MQ_UNSUPPORTED_FEATURE;
    uint8_t sensType;
    osp_bool_t isPrivateType;
    uint8_t seqNum;
    int16_t lengthParsed;

    /* Sanity... */
    if ((pOut == NULL) || (pPacket == NULL))
        return (-MQ_BAD_BUFFER);

    /* Get sensor type. */
    sensType = M_SensorType(pHif->Enable.Q.SensorIdByte);
    isPrivateType = (pHif->Enable.Q.ControlByte & SENSOR_ANDROID_TYPE_MASK) ? TRUE : FALSE;
    seqNum = M_SequenceNum(pHif->Enable.Q.AttributeByte);

    return MQ_SUCCESS;
}


/****************************************************************************************************
 * @fn      ParseHostIntefacePkt
 *          Top level parser for Host interface Packets. Depending on packet type & data sizes, the
 *          individual parsers are called.
 *
 * @param   [OUT]pOut - Sensor structure that will return the parsed values
 * @param   [IN]pPacket - Packet buffer containing the packet to parse
 * @param   [IN]pktSize - Size of the packet buffer provided
 *
 * @return  MQ_SUCCESS or Error code enum corresponding to the error encountered
 *
 ***************************************************************************************************/
int16_t ParseHostIntefacePkt( SensorPacketTypes_t *pOut, uint8_t *pPacket, uint16_t pktSize )
{
    uint8_t pktID;
    int16_t errCode = -MQ_UNSUPPORTED_FEATURE;

    /* Sanity... */
    if ((pOut == NULL) || (pPacket == NULL))
        return (-MQ_BAD_BUFFER);

    /* CRC Check */
    //TODO - we are not using CRC check for now!

    /* ========== Packet handling ========== */
    /* 1. Identify Packet type */
    pktID = pPacket[0] & PKID_MASK_VER0;

    switch (pktID)
    {
    case PKID_SENSOR_DATA:
        /* Invoke packet handler for Sensor Data packet */
        /* Note: This handler should be on host side. Its provided here for testing */
#if defined (MQ_UNIT_TEST) && defined (TEST_SENSOR_DATA_PKT) 
        errCode = ParseSensorDataPkt( pOut, pPacket, pktSize );
#endif
        break;

    case PKID_CONTROL_REQ:
        /* Invoke packet handler for Control Request packet */
        break;

    case PKID_CONTROL_RESP:
        /* To be implemented on the host side driver */
        break;

    case PKID_TEST_DATA:
        break;

    default:
        //Invalid or newer version packet
        errCode = -MQ_INVALID_PACKETID;
        break;
    }
    return errCode;
}


/*-------------------------------------------------------------------------------------------------*\
 |    E N D   O F   F I L E
\*-------------------------------------------------------------------------------------------------*/
