/****************************************************************************************************
 * @file  MQErrorCodes.h
 *
 * Enumerated error codes for MotionQ sources
 *
 * < Detailed description of subsystem >
 *
 * @seealso < Links to relevant files, references, etc. >
 *
 * @info
 * $Author: ppatel $
 * $DateTime: 2014/10/20 18:19:46 $
 * $Revision: #6 $
 * $Id: //AudEngr/Hardware/DeltaPlus/Software/BoskoApp_D300_SensorHub/overlay/SensorHub/MQErrorCodes.h#6 $
 *
 * Copyright 2014 Audience, Incorporated. All rights reserved.
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
#if !defined (MQERRORCODES_H)
#define   MQERRORCODES_H

/*-------------------------------------------------------------------------------------------------*\
 |    I N C L U D E   F I L E S
\*-------------------------------------------------------------------------------------------------*/
#include <stdint.h> /* To help resolve 'sint' vs 'int' differences */

/*-------------------------------------------------------------------------------------------------*\
 |    C O N S T A N T S   &   M A C R O S
\*-------------------------------------------------------------------------------------------------*/
#ifndef NULL
# define NULL                           ((void *)0)
#endif

#ifndef ASSERT
#define ASSERT                          FM_ASSERT
#endif


/*-------------------------------------------------------------------------------------------------*\
 |    T Y P E   D E F I N I T I O N S
\*-------------------------------------------------------------------------------------------------*/
typedef enum _MQErr {
    MQ_SUCCESS                  = 0,
    MQ_BAD_BUFFER,                      /* Bad buffer pointer (NULL)            */
    MQ_INVALID_PACKETID,                /* Host interface packet ID invalid     */
    MQ_UNSUPPORTED_FEATURE,             /* Feature currently unsupported        */
    MQ_QUEUE_EMPTY,
    MQ_QUEUE_FULL,
    MQ_QUEUE_HIGH_THRESHOLD,
    MQ_QUEUE_LOW_THRESHOLD,
    MQ_INVALID_PARAMETER,               /* API called with invalid argument/parameter   */
    MQ_NOT_IMPLEMENTED,                 /* API or function not implemented (mostly for stubs) */
    MQ_PARAM_CROSS_LINKAGE,             /* Independent parameter setting not possible */
    MQ_UNSUPPORTED_DATA_RATE,           /* Data rate specified is not supported */
    MQ_UNSUPPORTED_RANGE,               /* Range specified is not supported */
    MQ_SAMPLE_NOT_READY,                /* Sample is not ready because of decimation*/
    MQ_ALGORITHM_API_ERR,               /* Error returned by Motion library */
    MQ_UNAVAILABLE_INPUT_SENSOR,
    MQ_UNSUPPORTED_ALGORITHM,
    MQ_SENSOR_POWER_DOWN,               /* Sensor is in Power Down mode or it is disable*/
} MQErr_t;


/*-------------------------------------------------------------------------------------------------*\
 |    E X T E R N A L   V A R I A B L E S   &   F U N C T I O N S
\*-------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------*\
 |    P U B L I C   V A R I A B L E S   D E F I N I T I O N S
\*-------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------*\
 |    P U B L I C   F U N C T I O N   D E C L A R A T I O N S
\*-------------------------------------------------------------------------------------------------*/


#endif /* MQERRORCODES_H */
/*-------------------------------------------------------------------------------------------------*\
 |    E N D   O F   F I L E
\*-------------------------------------------------------------------------------------------------*/
