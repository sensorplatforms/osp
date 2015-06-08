/****************************************************************************************************
 * @file  Queue.c
 *
 * Implements utility functions for FIFO queues that can be used in the application
 *
 * < Detailed description of subsystem >
 *
 * @seealso < Links to relevant files, references, etc. >
 *
 * @info
 * $Author: rverma $
 * $DateTime: 2014/10/30 17:45:42 $
 * $Revision: #4 $
 * $Id: //AudEngr/Hardware/DeltaPlus/Software/BoskoApp_D300_SensorHub/overlay/SensorHub/Queue.c#4 $
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
/*-------------------------------------------------------------------------------------------------*\
 |    I N C L U D E   F I L E S
\*-------------------------------------------------------------------------------------------------*/
#include "Queue.h"
#include "MQErrorCodes.h"
#include "BlockMemory.h"
#include <string.h> //for memset


/*-------------------------------------------------------------------------------------------------*\
 |    E X T E R N A L   V A R I A B L E S   &   F U N C T I O N S
\*-------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------*\
 |    P R I V A T E   C O N S T A N T S   &   M A C R O S
\*-------------------------------------------------------------------------------------------------*/
#define NUM_APPLICATION_QUEUES          1

/*-------------------------------------------------------------------------------------------------*\
 |    P R I V A T E   T Y P E   D E F I N I T I O N S
\*-------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------*\
 |    S T A T I C   V A R I A B L E S   D E F I N I T I O N S
\*-------------------------------------------------------------------------------------------------*/
static uint8_t QPoolInitialized = 0;

/*-------------------------------------------------------------------------------------------------*\
 |    F O R W A R D   F U N C T I O N   D E C L A R A T I O N S
\*-------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------*\
 |    P U B L I C   V A R I A B L E S   D E F I N I T I O N S
\*-------------------------------------------------------------------------------------------------*/
DECLARE_BLOCK_POOL( QObjectPool, sizeof(Queue_t), NUM_APPLICATION_QUEUES );

/*-------------------------------------------------------------------------------------------------*\
 |    P R I V A T E     F U N C T I O N S
\*-------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------*\
 |    P U B L I C     F U N C T I O N S
\*-------------------------------------------------------------------------------------------------*/

/****************************************************************************************************
 * @fn      QueueCreate
 *          Creates a queue object with the given parameters. No memory is allocated for the given
 *          capacity. It only serves as a control point. Buffers must be allocated by application
 *
 * @param   [IN]capacity - Max capacity of the queue
 * @param   [IN]lowThreshold - Queue size low threshold. Used to trigger low threshold callback
 * @param   [IN]highThreshold - Queue size high threshold. Used to trigger high threshold callback
 *
 * @return  Pointer to the queue object created. Null otherwise.
 *
 ***************************************************************************************************/
Queue_t *QueueCreate( uint32_t capacity, uint32_t lowThreshold, uint32_t highThreshold )
{
    Queue_t *pQ;

    /* Initialize the memory pool for the queue objects */
    if (!QPoolInitialized)
    {
        InitBlockPool( QObjectPool, sizeof(QObjectPool), sizeof(Queue_t) );
        QPoolInitialized = 1;
    }
    
    /* Allocate memory for queue object */
    pQ = AllocBlock(QObjectPool);
    if (pQ == NULL)
    {
        return NULL;
    }
    else
    {
        /* It is important to clear the queue structure */
        memset( pQ, 0, sizeof(Queue_t));
    }

    /* Initialize queue */
    pQ->Capacity = capacity;
    pQ->HighThres = highThreshold;
    pQ->LowThres = lowThreshold;

    /* Return reference to the queue */
    return pQ;
}


/****************************************************************************************************
 * @fn      EnQueue
 *          Called by application to enqueue the given buffer in the queue (FIFO order). Note that
            buffers must be allocated & filled by application before queuing.
 *
 * @param   [IN]pMyQ - Pointer to a queue previously created
 * @param   [IN]pBuf - Pointer to user buffer that needs to be queued. This must remain valid until
 *                     dequeued!
 *
 * @return  MQ_SUCCESS or -Error code enum corresponding to the error encountered
 *
 ***************************************************************************************************/
int16_t EnQueue( Queue_t *pMyQ, Buffer_t *pBuf )
{
    SETUP_CRITICAL_SECTION();

    ENTER_CRITICAL_SECTION();
    /* Check if queue is already full */
    if (pMyQ->Size == pMyQ->Capacity)
    {
        EXIT_CRITICAL_SECTION();
        return (-MQ_QUEUE_FULL);
    }

    /* Enqueue the buffer */
    if (pMyQ->Size == 0)
    {
        pMyQ->pHead = pBuf; //Add to head
    }
    else
    {
        pMyQ->pTail->Header.pNext = (uint8_t*)pBuf;
    }
    /* Update links */
    pMyQ->pTail = pBuf;
    pBuf->Header.pNext = NULL;
    pMyQ->Size++;

    /* Check for high threshold */
    if ((pMyQ->HighThres < pMyQ->Capacity) && (pMyQ->Size == pMyQ->HighThres))
    {
        EXIT_CRITICAL_SECTION();

        /* Invoke high threshold callback if registered */
        if (pMyQ->pfCB[QUEUE_HIGH_THRESHOLD_CB] != NULL)
        {
            pMyQ->pfCB[QUEUE_HIGH_THRESHOLD_CB]( pMyQ->pCbArg[QUEUE_HIGH_THRESHOLD_CB] );
        }
        return (-MQ_QUEUE_HIGH_THRESHOLD);
    }

    /* Check if queue is full to its capacity */
    if (pMyQ->Size == pMyQ->Capacity)
    {
        EXIT_CRITICAL_SECTION();
        /* Invoke callback if one was registered */
        if (pMyQ->pfCB[QUEUE_FULL_CB] != NULL)
        {
            pMyQ->pfCB[QUEUE_FULL_CB]( pMyQ->pCbArg[QUEUE_FULL_CB] );
        }
        return MQ_SUCCESS;
    }
    EXIT_CRITICAL_SECTION();

    return MQ_SUCCESS;
}


/****************************************************************************************************
 * @fn      DeQueue
 *          Called by application to dequeue the oldest buffer in the queue (FIFO order)
 *
 * @param   [IN]pMyQ - Pointer to a queue previously created
 * @param   [OUT]pBuf - Pointer that returns the dequeued buffer
 *
 * @return  MQ_SUCCESS or -Error code enum corresponding to the error encountered
 *
 ***************************************************************************************************/
int16_t DeQueue( Queue_t *pMyQ, Buffer_t **pBuf )
{
    Buffer_t *pTemp;
    SETUP_CRITICAL_SECTION();

    ENTER_CRITICAL_SECTION();
    if (pMyQ->Size == 0)
    {
        EXIT_CRITICAL_SECTION();

        *pBuf = NULL;
        return (-MQ_QUEUE_EMPTY);
    }
    else
    {
        pTemp = pMyQ->pHead;
        pMyQ->pHead = (Buffer_t*)pTemp->Header.pNext;
        *pBuf = pTemp; //set return pointer

        /* update tail if this was the last buffer dequeued */
        if (pMyQ->pHead == NULL)
        {
            pMyQ->pTail = NULL;
        }

        pMyQ->Size--;
        /* Invoke relevant callbacks... */
        if (pMyQ->Size == 0)
        {
            EXIT_CRITICAL_SECTION();

            /* Invoke high threshold callback if registered */
            if (pMyQ->pfCB[QUEUE_EMPTY_CB] != NULL)
            {
                pMyQ->pfCB[QUEUE_EMPTY_CB]( pMyQ->pCbArg[QUEUE_EMPTY_CB] );
            }
            return MQ_SUCCESS;
        }

        if ((pMyQ->LowThres > 0) && (pMyQ->Size == pMyQ->LowThres))
        {
            EXIT_CRITICAL_SECTION();

            /* Invoke high threshold callback if registered */
            if (pMyQ->pfCB[QUEUE_LOW_THRESHOLD_CB] != NULL)
            {
                pMyQ->pfCB[QUEUE_LOW_THRESHOLD_CB]( pMyQ->pCbArg[QUEUE_LOW_THRESHOLD_CB] );
            }
            return (-MQ_QUEUE_LOW_THRESHOLD);
        }
    }

    EXIT_CRITICAL_SECTION();
    return MQ_SUCCESS;
}


/****************************************************************************************************
 * @fn      QueueRegisterCallBack
 *          Allows user to register callback for queue related events (low/high threshold, empty/full)
 *          Callbacks can be registered one at a time.
 *
 * @param   [IN]pMyQ - Pointer to a queue previously created
 * @param   [IN]cbid - Identifier for the callback being registered
 * @param   [IN]pFunc - Callback function pointer
 * @param   [IN]pUser - User provided argument that is passed to the callback when invoked.
 *
 * @return  MQ_SUCCESS if callback was registered; -MQ_INVALID_PARAMETER otherwise
 *
 ***************************************************************************************************/
int16_t QueueRegisterCallBack( Queue_t *pMyQ, Q_CBId_t cbid, fpQueueEvtCallback_t pFunc, void *pUser )
{
    if ((pFunc != NULL) && (cbid < NUM_CB_IDS))
    {
        pMyQ->pfCB[cbid] = pFunc;
        pMyQ->pCbArg[cbid] = pUser;
        return MQ_SUCCESS;
    }
    return (-MQ_INVALID_PARAMETER);
}


/*-------------------------------------------------------------------------------------------------*\
 |    E N D   O F   F I L E
\*-------------------------------------------------------------------------------------------------*/
