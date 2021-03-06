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
/* 
 * Handle serial debugging inputs.
 */
/*-------------------------------------------------------------------------------------------------*\
 |    I N C L U D E   F I L E S
\*-------------------------------------------------------------------------------------------------*/
#include "common.h"
#include "hw_setup.h"
#include <string.h>
#include "debugprint.h"


/*-------------------------------------------------------------------------------------------------*\
 |    E X T E R N A L   V A R I A B L E S   &   F U N C T I O N S
\*-------------------------------------------------------------------------------------------------*/
extern PortInfo gDbgUartPort;
extern void CmdParse_User( int8_t *pBuffer, uint16_t size );

/*-------------------------------------------------------------------------------------------------*\
 |    P U B L I C   V A R I A B L E S   D E F I N I T I O N S
\*-------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------*\
 |    P R I V A T E   C O N S T A N T S   &   M A C R O S
\*-------------------------------------------------------------------------------------------------*/
/* Defines for command handler */
#define COMMAND_LINE_SIZE                       32

/*-------------------------------------------------------------------------------------------------*\
 |    P R I V A T E   T Y P E   D E F I N I T I O N S
\*-------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------*\
 |    S T A T I C   V A R I A B L E S   D E F I N I T I O N S
\*-------------------------------------------------------------------------------------------------*/
static int8_t inputBuffer[COMMAND_LINE_SIZE];

/*-------------------------------------------------------------------------------------------------*\
 |    F O R W A R D   F U N C T I O N   D E C L A R A T I O N S
\*-------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------*\
 |    P R I V A T E     F U N C T I O N S
\*-------------------------------------------------------------------------------------------------*/

/****************************************************************************************************
 * @fn      SerialRead
 *          This function reads 'length' bytes and copies the data into memory pointed to by pBuff.
 *          The function returns the number of bytes actually read in pBytesRead. If an error has
 *          occurred, the function returns immediately; otherwise the
 *          function does not return until the specified number of bytes has been received and copied
 *          into the buffer. If not enough received data bytes are available in the driver's internal
 *          storage area, the calling task is suspended (blocked) until the required number of bytes
 *          is available.
 *
 * @param   byte received byte
 *
 * @return  APP_OK Data received properly
 *
 ***************************************************************************************************/
static uint8_t SerialRead( PortInfo *pPort, int8_t *pBuff, uint16_t length, uint16_t *pBytesRead )
{
    uint32_t readIdx;
    uint16_t bytesRead = 0;
    uint8_t  retVal = APP_OK;
    osEvent evtFlags;
    evtFlags.value.v=0;

    if ((pBuff == NULL) || (length == 0) || (length > RX_BUFFER_SIZE))
    {
        return APP_ERR;
    }

    while (bytesRead < length)
    {
        /* If the write index is 1 in front of read, the buffer is now empty. */
        if (pPort->rxWriteIdx ==
            ((pPort->rxReadIdx + 1) % RX_BUFFER_SIZE))
        {
            /* Wait here for ISR event */
            osThreadId myId = osThreadGetId();
            while(1){
                evtFlags = osSignalWait(UART_CMD_RECEIVE,200);
                if (evtFlags.status == osEventTimeout){
                    evtFlags = osSignalWait(UART_CRLF_RECEIVE,200);
                }
                if (evtFlags.status == osEventSignal){
                    break;
                }
            }
            osSignalClear(myId,UART_CMD_RECEIVE);
            osSignalClear(myId,UART_CRLF_RECEIVE);
        }
        else
        {
            /* Data is available: get the read index, retrieve the data byte, and adjust the global
               index AFTER. */
            readIdx = pPort->rxReadIdx + 1;
            if (readIdx >= RX_BUFFER_SIZE)
            {
                readIdx = 0;
            }
            pBuff[bytesRead++] = pPort->rxBuffer[readIdx];
            pPort->rxReadIdx = readIdx;
            if ( evtFlags.value.signals & UART_CRLF_RECEIVE )
            {
                break;
            }
        }
    }

    if (pBytesRead)
    {
        *pBytesRead = bytesRead;
    }

    return retVal;
}


/*-------------------------------------------------------------------------------------------------*\
 |    P U B L I C     F U N C T I O N S
\*-------------------------------------------------------------------------------------------------*/

/****************************************************************************************************
 * @fn      CmdHandlerTask
 *          This task is responsible for command parsing and setting configuration values read from
 *          serial port.
 *
 * @param   none
 *
 * @return  none
 *
 ***************************************************************************************************/
ASF_TASK void CmdHandlerTask( ASF_TASK_ARG )
{
    uint8_t retVal;
    uint16_t bytesRead;

    D2_printf("\r\n** SERIAL COMMAND HANDLER READY **\r\n");

    while(1)
    {
#if 0
        retVal = SerialRead( &gDbgUartPort, inputBuffer, COMMAND_LINE_SIZE-1, &bytesRead );
        if (retVal == APP_OK)
        {
            CmdParse_User( inputBuffer, bytesRead ); //Implemented in application code
        }
#endif
    }
}


/*-------------------------------------------------------------------------------------------------*\
 |    E N D   O F   F I L E
\*-------------------------------------------------------------------------------------------------*/
