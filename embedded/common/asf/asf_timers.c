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
/*-------------------------------------------------------------------------------------------------*\
 |    I N C L U D E   F I L E S
\*-------------------------------------------------------------------------------------------------*/
#include "common.h"
#include "asf_taskstruct.h"
/*-------------------------------------------------------------------------------------------------*\
 |    E X T E R N A L   V A R I A B L E S   &   F U N C T I O N S
\*-------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------*\
 |    P U B L I C   V A R I A B L E S   D E F I N I T I O N S
\*-------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------*\
 |    P R I V A T E   C O N S T A N T S   &   M A C R O S
\*-------------------------------------------------------------------------------------------------*/
#ifndef RAM_START
# define RAM_START           NVIC_VectTab_RAM
#endif
extern const AsfTaskInitDef C_gAsfTaskInitTable[NUMBER_OF_TASKS];

#ifndef OS_TIMERCNT 
    #define OS_TIMERCNT (8)    // Make sure this value matches file rtx_conf_cm.c
#endif 
/*-------------------------------------------------------------------------------------------------*\
 |    P R I V A T E   T Y P E   D E F I N I T I O N S
\*-------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------*\
 |    S T A T I C   V A R I A B L E S   D E F I N I T I O N S
\*-------------------------------------------------------------------------------------------------*/
/* Maintain the list of registered timer currently running */
static AsfTimer* _asfTimerList[OS_TIMERCNT];
/* Indicator if this timer module is initialized once */
static osp_bool_t _asfTimerInitialized = FALSE;     

/*-------------------------------------------------------------------------------------------------*\
 |    F O R W A R D   F U N C T I O N   D E C L A R A T I O N S
\*-------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------*\
 |    P R I V A T E     F U N C T I O N S
\*-------------------------------------------------------------------------------------------------*/


/****************************************************************************************************
 * @fn      AsfTimerInit
 *          Perform one time initialization 
 *
 * @param   none
 *
 * @return  none
 *
 ***************************************************************************************************/
static void AsfTimerInit(void)
{
    if ( _asfTimerInitialized == FALSE ) { 
        uint16_t i; 
        for ( i = 0; i < OS_TIMERCNT; i++ ) {
            _asfTimerList[i] = NULL;
        }
        _asfTimerInitialized = TRUE;
    }    
}


static uint16_t AsfTimerAddTimerToList(AsfTimer *pTimer)
{
    uint16_t i;
    
    for ( i = 0; i < OS_TIMERCNT; i++ ) {
        if ( _asfTimerList[i] == NULL ) {
            _asfTimerList[i] = pTimer;
            return i;
        }
    }

    /* Trigger assert. Use too many timers. */
    ASF_assert(FALSE);
}

static AsfTimer*  AsfTimerGetTimerFromList( uint16_t info) 
{     
    if ( info < OS_TIMERCNT ) 
        return _asfTimerList[info];
    
    return (AsfTimer*)NULL;
}


static void AsfTimerRemoveTimerFromList(uint16_t info)
{
    if ( info < OS_TIMERCNT ) {
        if( _asfTimerList[info] != NULL ) _asfTimerList[info] = NULL;     
    }
}

/****************************************************************************************************
 * @fn      SendTimerExpiry
 *          Sends the timer expiry message to the owner of the timer
 *
 * @param   pTimer  Pointer to the timer control block
 *
 * @return  none
 *
 ***************************************************************************************************/
static void SendTimerExpiry ( AsfTimer *pTimer )
{
    MessageBuffer *pSendMsg = NULLP;

    ASF_assert( ASFCreateMessage( MSG_TIMER_EXPIRY, sizeof(MsgTimerExpiry), &pSendMsg ) == ASF_OK );
    pSendMsg->msg.msgTimerExpiry.timerId   = (osTimerId)pTimer->timerId;
    pSendMsg->msg.msgTimerExpiry.userValue = pTimer->userValue;
    __enable_irq();
    ASF_assert( ASFSendMessage( pTimer->owner, pSendMsg ) == ASF_OK );
    __disable_irq();
}

/*-------------------------------------------------------------------------------------------------*\
 |    P U B L I C     F U N C T I O N S
\*-------------------------------------------------------------------------------------------------*/

/****************************************************************************************************
 * @fn      ASFTimerStarted
 *          Checks if the timer has already been started
 *
 * @param   pTimer  Pointer to timer control block containing the attributes of the timer to be
 *                  created.
 *
 * @return  true - Timer already started; false otherwise
 *
 * @see     ASFTimerStart()
 ***************************************************************************************************/
osp_bool_t ASFTimerStarted ( AsfTimer *pTimer )
{
    return (pTimer->sysUse == TIMER_SYS_ID? true : false);
}


/****************************************************************************************************
 * @fn      ASFTimerStart
 *          Creates a timer with given reference and tick value assigned to the owner.
 *
 * @param   owner  Task ID of the task that will receive the expiry message
 * @param   ref  Unique reference number for the timer
 * @param   tick  Tick count in OS ticks
 * @param   pTimer  Pointer to timer type
 *
 * @return  none
 *
 * @see     ASFTimerKill()
***************************************************************************************************/
void _ASFTimerStart( TaskId owner, uint16_t ref, uint16_t tick, AsfTimer *pTimer, char *_file, int _line  )
{
    uint16_t index = AsfTimerAddTimerToList(pTimer);  // Add this timer to the managed list 

    pTimer->owner = owner;
    pTimer->ticks = tick;
    pTimer->userValue = ref;
    if(pTimer->timerId == NULL)
    {
        pTimer->timerId = osTimerCreate(C_gAsfTaskInitTable[owner].timerDef,osTimerOnce,(void *)index);
    }
    if (pTimer->timerId) 
    {
        pTimer->sysUse = TIMER_SYS_ID;
        osTimerStart(pTimer->timerId, pTimer->ticks);
    }

}


/****************************************************************************************************
 * @fn      ASFTimerExpiry
 *          Handles the timer expiry by sending message to the task that created the timer
 *
 * @param   info  pseudo pointer to timer control block of the timer that expired.
 *
 * @return  none
 *
 * @see     ASFKillTimer()
 ***************************************************************************************************/
void _ASFTimerExpiry ( uint32_t info, char *_file, int _line )
{
    AsfTimer *pTimer;
    int wasMasked = __disable_irq();
    pTimer = AsfTimerGetTimerFromList(info);
    AsfTimerRemoveTimerFromList(info);

    //Look for our magic number to be sure we got the right pointer
    ASF_assert_var( pTimer->sysUse == TIMER_SYS_ID,  pTimer->ticks, pTimer->userValue, pTimer->owner);
    /* Reset timer before starting to process it.
     *  This is to prevent a race condition, where the processing task restarts a timer before we reset here.
     * Timer thread runs on a lower priority then, the processing task
     */
    pTimer->sysUse = (uint32_t)-1; //Timer no longer in use
    SendTimerExpiry( pTimer );
    if (!wasMasked) __enable_irq();
}


/****************************************************************************************************
 * @fn      ASFKillTimer
 *          Kills the timer that was created earlier
 *
 * @param   pTimer  Pointer to timer control block containing the attributes of the timer to be
 *                  created.
 *
 * @return  none
 *
 * @see     ASFTimerStart()
 ***************************************************************************************************/
void _ASFKillTimer ( AsfTimer *pTimer, char *_file, int _line )
{
    osStatus    os_ret = osErrorOS;
    ASF_assert( pTimer != NULLP );
    os_ret = osTimerDelete(pTimer->timerId);
    ASF_assert( os_ret == osOK );
    pTimer->sysUse = (uint32_t)-1; //Timer no longer in use
    AsfTimerRemoveTimerFromList(pTimer->info);
}
/****************************************************************************************************
 * @fn      ASFTimerCallback
 *          Timer callback registered with CMSIS for timer expiry notification
 *
 * @param   argument  Param provided to CMSIS for callback, this is the index used to retrieve the task(owner) info.
 *
 * @return  none
 *
 * @see     ASFTimerExpiry()
 ***************************************************************************************************/
void ASFTimerCallback(void const *argument)
{
    ASFTimerExpiry((uint32_t)argument);
}

/*-------------------------------------------------------------------------------------------------*\
 |    E N D   O F   F I L E
\*-------------------------------------------------------------------------------------------------*/
