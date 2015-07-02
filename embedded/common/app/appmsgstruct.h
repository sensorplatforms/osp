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
#if !defined (APPMSGSTRUCT_H)
#define   APPMSGSTRUCT_H

/*-------------------------------------------------------------------------------------------------*\
 |    I N C L U D E   F I L E S
\*-------------------------------------------------------------------------------------------------*/
#include <stdint.h>
/*-------------------------------------------------------------------------------------------------*\
 |    C O N S T A N T S   &   M A C R O S
\*-------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------*\
 |    T Y P E   D E F I N I T I O N S
\*-------------------------------------------------------------------------------------------------*/
#pragma pack(push)  /* push current alignment to stack */
#pragma pack(4)

typedef struct MsgSensorDataTag
{
    uint32_t    timeStamp;
    int32_t     X;	/* Pitch */
    int32_t     Y;	/* Roll */
    int32_t     Z;	/* Yaw */
    int32_t	    W;
    int32_t     HeadingError;
    int32_t     TiltError;
} MsgSensorData;

typedef struct MsgSensorBoolTag 
{
	uint32_t timeStamp;
	uint8_t  active;
} MsgSensorBoolData;


typedef MsgSensorData MsgAccelData;
typedef MsgSensorData MsgMagData;
typedef MsgSensorData MsgGyroData;
typedef MsgSensorData MsgQuaternionData;
typedef MsgSensorData MsgStepData;
typedef MsgSensorData MsgOrientationData;
typedef MsgSensorData MsgGenericTriAxisData;
typedef MsgSensorData MsgPressData;
typedef MsgSensorBoolData MsgSigMotionData;
typedef MsgSensorBoolData MsgStepDetData;

typedef struct MsgInclinationDataTag
{
    uint32_t    timeStamp;
    int32_t     yaw;
    int32_t     pitch;
    int32_t     roll;   
}  MsgInclinationData;

typedef struct MsgSensorDataRdyTag
{
    uint32_t     timeStamp;
    uint8_t      sensorId;
} MsgSensorDataRdy;

typedef struct MsgCDSegmentDataTag
{
    uint64_t endTime;
    uint32_t duration;
    uint8_t  type;
} MsgCDSegmentData;


typedef struct MsgSensorControlDataTag
{
    uint32_t command;
    int32_t  data;
    uint8_t  sensorType;
} MsgSensorControlData;


/*-------------------------------------------------------------------------------------------------*\
 |    E X T E R N A L   V A R I A B L E S   &   F U N C T I O N S
\*-------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------*\
 |    P U B L I C   V A R I A B L E S   D E F I N I T I O N S
\*-------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------*\
 |    P U B L I C   F U N C T I O N   D E C L A R A T I O N S
\*-------------------------------------------------------------------------------------------------*/

#pragma pack(pop)   /* restore original alignment from stack */

#endif /* APPMSGSTRUCT_H */
/*-------------------------------------------------------------------------------------------------*\
 |    E N D   O F   F I L E
\*-------------------------------------------------------------------------------------------------*/
