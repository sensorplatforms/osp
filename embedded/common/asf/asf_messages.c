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

//#undef USE_ALLOC
#define USE_ALLOC 1
/*-------------------------------------------------------------------------------------------------*\
 |    E X T E R N A L   V A R I A B L E S   &   F U N C T I O N S
\*-------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------*\
 |    P U B L I C   V A R I A B L E S   D E F I N I T I O N S
\*-------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------*\
 |    P R I V A T E   C O N S T A N T S   &   M A C R O S
\*-------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------*\
 |    P R I V A T E   T Y P E   D E F I N I T I O N S
\*-------------------------------------------------------------------------------------------------*/
uint32_t debugMessageBlockSize = 0;
/*-------------------------------------------------------------------------------------------------*\
 |    S T A T I C   V A R I A B L E S   D E F I N I T I O N S
\*-------------------------------------------------------------------------------------------------*/
/**
 * The _declare_box macro declares an array of bytes that can be used as a memory pool for fixed
 * block allocation.
 */

#define TRACK_MSG_POOL	0
#if USE_ALLOC
osPoolDef(cmpool, MAX_SYSTEM_MESSAGES, MessageBlock); 		   /* Define memory pool			  */
osPoolId cmpool;
#else
struct _MsgPool {
	volatile int32_t	status;
#if TRACK_MSG_POOL
	char *_file;
	int _line;
#endif
	uint8_t msg[MESSAGE_BLOCK_SIZE];
} MsgPool[MAX_SYSTEM_MESSAGES];

static void MsgPool_init(struct _MsgPool *p)
{
	int i;

#if 1  //QLY: initialize pool memory to debug memory overwrite issue
        memset(MsgPool, 0xff, sizeof(MsgPool));
        debugMessageBlockSize = sizeof( struct _MsgPool);
#endif 

	for (i = 0; i < MAX_SYSTEM_MESSAGES; i++) {
		p[i].status = 0;
	}
}

static void * MsgPool_get(struct _MsgPool *p, char *_file, int _line)
{
	int i;
    __disable_irq();
	for (i = 0; i < MAX_SYSTEM_MESSAGES; i++) {
		p[i].status += (i+1);
		if (p[i].status != (i+1)) {
			p[i].status -= (i+1);
		} else {
            __enable_irq();
#if TRACK_MSG_POOL
			p[i]._file = _file;
			p[i]._line = _line;
#endif
			return p[i].msg;
		}
	}
    __enable_irq();
	return NULL;
}
static void MsgPool_put(struct _MsgPool *p, void *m)
{
	int i;
	for (i = 0; i < MAX_SYSTEM_MESSAGES; i++) {
		if  (p[i].msg == m) {
			if (p[i].status == 0) {
				//D0_printf("Double free!\n");
				return;
			}
			p[i].status = 0;
#if TRACK_MSG_POOL
			p[i]._file = "";
			p[i]._line = 1;
#endif
			return;
		}
	}
        ASF_assert(0); /* Something is wrong! */
}
#endif
/*-------------------------------------------------------------------------------------------------*\
 |    F O R W A R D   F U N C T I O N   D E C L A R A T I O N S
\*-------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------*\
 |    P R I V A T E     F U N C T I O N S
\*-------------------------------------------------------------------------------------------------*/

/****************************************************************************************************
 * @fn      ASFDeleteMessage
 *          This function releases the buffer memory associated with the message contents. The caller
 *          must call this function after a message's contents have been read to free the memory
 *          associated with the message.
 *
 * @param   pMbuf Message buffer pointer containing the message to be deleted.
 *
 * @return  none
 *
 * @see     ASFCreateMessage(), ASFSendMessage(), ASFReceiveMessage()
 ***************************************************************************************************/
void _ASFDeleteMessage ( TaskId rcvTask, MessageBuffer **pMbuf, char *_file, int _line )
{
    MessageBlock *pBlock;

    /** This is where we release the memory allocated when the message was created */
    if (*pMbuf != NULLP)
    {
        /* Get the block pointer */
        M_GetMsgBlockFromBuffer (pBlock, *pMbuf);
#if USE_ALLOC
        ASF_assert( osPoolFree( cmpool, pBlock ) == osOK);
#else
        MsgPool_put(MsgPool, pBlock);
#endif
    }

    *pMbuf = NULLP;
}



/*-------------------------------------------------------------------------------------------------*\
 |    P U B L I C     F U N C T I O N S
\*-------------------------------------------------------------------------------------------------*/

/****************************************************************************************************
 * @fn      ASFMessagingInit
 *          Initializes the messaging scheme in the system
 *
 ***************************************************************************************************/
void ASFMessagingInit( void )
{
#if USE_ALLOC
    cmpool = osPoolCreate(osPool(cmpool));
#else
	MsgPool_init(MsgPool);
#endif
}


/****************************************************************************************************
 * @fn      ASFCreateMessage
 *          This function is called to create (allocate) the space for the message type specified.
 *          This is the first step before the message parameters are filled in and the message is
 *          sent via the ASFSendMessage() call.
 *
 * @param   msgId   Message ID of the message to be created.
 * @param   msgSize size (in bytes) of the message corresponding to the ID. This is typically the
 *                  sizeof structure corresponding to the message.
 *
 * @return  pMbuf   This pointer returns the allocated buffer space for the message type specified.
 *
 * @see     ASFSendMessage(), ASFReceiveMessage()
 ***************************************************************************************************/
AsfResult_t _ASFCreateMessage( MessageId msgId, uint16_t msgSize, MessageBuffer **pMbuf, char *_file, int _line )
{
    MessageBlock   *pBlock;

    /* At this time it is assumed that the memory pool for message allocation has been
       created and initialized */
    ASF_assert_var( *pMbuf == NULLP, msgId, 0, 0 );

#if USE_ALLOC
    pBlock = osPoolAlloc(cmpool);
#else
	pBlock = MsgPool_get(MsgPool, _file, _line);
#endif
    if (pBlock == NULLP) return ASF_ERR_MSG_BUFF;

    pBlock->header.length = msgSize;
    pBlock->rec.msgId     = msgId;

    /* Set the user message buffer now */
    *pMbuf = (MessageBuffer *)&pBlock->rec;
    return ASF_OK;
}

uint32_t	wtf_msg_cnt = 0;
/****************************************************************************************************
 * @fn      ASFSendMessage
 *          This function is called after a message has been created and the message parameters
 *          filled in. It sends the message to the destination task queue without blocking.
 *
 * @param   destTask  TaskId of the destination task which will receive the message.
 * @param   pMbuf Message buffer pointer containing the message to send.
 * @param   cntxt Specify if the message is being sent from ISR or Thread context
 *
 * @return  none
 *
 * @see     ASFCreateMessage(), ASFReceiveMessage()
 ***************************************************************************************************/
AsfResult_t _ASFSendMessage ( TaskId destTask, MessageBuffer *pMbuf, char *_file, int _line )
{
    MessageBlock *pBlock;
    osStatus os_ret = osErrorOS;

    /* Check for the usual - null pointers etc. */
    ASF_assert_var( pMbuf != NULLP, pMbuf->msgId, 0, 0 );

    /* Get the block pointer */
    M_GetMsgBlockFromBuffer (pBlock, pMbuf);

    pBlock->header.destTask = destTask;
    os_ret = osMailPut(asfTaskHandleTable[destTask].posMailQId,pMbuf);
    if(osOK != os_ret)
    {
#if USE_ALLOC
        ASF_assert( osPoolFree( cmpool, pBlock ) == 0 );
#else
		MsgPool_put(MsgPool, pBlock);
#endif
        return ASF_ERR_Q_FULL;
	}
    return ASF_OK;
}


/****************************************************************************************************
 * @fn      ASFReceiveMessage
 *          This function blocks until a message is received on the queue of the calling task.
 *
 * @param   rcvTask  TaskId of the receiving task which will receive the message. This id must be
 *                   task's own id.
 *
 * @param   pMbuf Message buffer pointer that will return the message received. Note that the
 *                pointer to the memory allocated for the message contents is passed in the
 *                message buffer structure passed in.
 *
 * @return  none
 *
 * @see     ASFCreateMessage(), ASFSendMessage(), ASFDeleteMessage(), ASFReceiveMessagePoll()
 ***************************************************************************************************/
void _ASFReceiveMessage ( TaskId rcvTask, MessageBuffer **pMbuf, char *_file, int _line )
{
    osEvent  evt;

    /* Delete old/previous message to release its buffer */
    _ASFDeleteMessage( rcvTask, pMbuf, _file, _line );

    evt = osMailGet(asfTaskHandleTable[rcvTask].posMailQId,osWaitForever);
    if (evt.status == osEventMail)
    {
        *pMbuf = evt.value.p;
    }
}

/****************************************************************************************************
 * @fn      ASFReceiveMessagePoll
 *          This function tries to receive a message on the queue of the calling task without blocking.
 *
 * @param   rcvTask  TaskId of the receiving task which will receive the message. This id must be
 *                   task's own id.
 *
 * @param   pMbuf Message buffer pointer that will return the message received. Note that the
 *                pointer to the memory allocated for the message contents is passed in the
 *                message buffer structure passed in.
 *
 * @return  true if message was received.
 *
 * @see     ASFCreateMessage(), ASFSendMessage(), ASFDeleteMessage(), ASFReceiveMessage()
 ***************************************************************************************************/
osp_bool_t _ASFReceiveMessagePoll ( TaskId rcvTask, MessageBuffer **pMbuf, char *_file, int _line )
{
    osEvent  evt;

    /* Delete old message to release its buffer */
    _ASFDeleteMessage( rcvTask, pMbuf, _file, _line );

    /* Wait for receive */
    evt = osMailGet( asfTaskHandleTable[rcvTask].posMailQId, 0 );
    if (evt.status == osEventTimeout)
    {
        return false;
    }
    if (evt.status == osEventMail)
    {
        *pMbuf = evt.value.p;
    }
    return true;
}


/*-------------------------------------------------------------------------------------------------*\
 |    E N D   O F   F I L E
\*-------------------------------------------------------------------------------------------------*/
