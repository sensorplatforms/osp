/****************************************************************************************************
 * @file  BlockMemory.c
 *
 * Memory management using fixed size block pools. Good replacement for malloc where memory size is
 * known in advance
 *
 * < Detailed description of subsystem >
 *
 * @seealso < Links to relevant files, references, etc. >
 *
 * @info
 * $Author: rverma $
 * $DateTime: 2014/09/15 17:18:32 $
 * $Revision: #2 $
 * $Id: //AudEngr/Hardware/DeltaPlus/Software/BoskoApp_D300_SensorHub/overlay/SensorHub/BlockMemory.c#2 $
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
#include "BlockMemory.h"
#include "MQErrorCodes.h"


/*-------------------------------------------------------------------------------------------------*\
 |    E X T E R N A L   V A R I A B L E S   &   F U N C T I O N S
\*-------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------*\
 |    P R I V A T E   C O N S T A N T S   &   M A C R O S
\*-------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------*\
 |    P R I V A T E   T Y P E   D E F I N I T I O N S
\*-------------------------------------------------------------------------------------------------*/
typedef struct _BlkMem {
    void *pFree;                    /* Pointer to first free memory block       */
    void *pEnd;                     /* Pointer to memory block end              */
    uint32_t  BlkSz;                /* Memory block size                        */
    uint32_t  TotalCnt;             /* Total memory blocks in the pool          */
    uint32_t  UsedCnt;              /* Total allocated block count              */
} BlkMem_t;

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

/*-------------------------------------------------------------------------------------------------*\
 |    P U B L I C     F U N C T I O N S
\*-------------------------------------------------------------------------------------------------*/

/****************************************************************************************************
 * @fn      InitBlockPool
 *          Initialize the memory pool with the given block sizes. Memory pool can be a static array
 *          or one time allocation from a bigger pool/heap. Pool pointer must be 32-bit aligned.
 *          Memory pool must be initialized using this call after a DECLARE_BLOCK_POOL
 *
 * @param   [IN]pPool - Pointer (32-bit aligned) to the memory pool that needs to be initialized
 * @param   [IN]poolSize - Total size of the memory pool in bytes that will be divided into blocks
 * @param   [IN]blkSize - Individual memory block sizes that will be allocated from the pool
 *
 * @return  MQ_SUCCESS if initialization successful, -MQ_INVALID_PARAMETER otherwise
 *
 ***************************************************************************************************/
int16_t InitBlockPool( void *pPool, uint32_t poolSize, uint32_t blkSize )
{
    /* Initialize memory block system */
    BlkMem_t *pBlkPool = (BlkMem_t*)pPool;
    void *pEnd;
    void *pBlk;
    void *pNext;

    /* Adjust block size to 32-bit boundary */
    blkSize = (blkSize + 3) & ~3;
    if (blkSize == 0)
    {
        return (-MQ_INVALID_PARAMETER);
    }

    if ((blkSize + sizeof(BlkMem_t)) > poolSize)
    {
        return (-MQ_INVALID_PARAMETER);
    }

    /* Initialize the block pool structure. */
    pBlk = ((uint8_t*)pPool) + sizeof(BlkMem_t);
    pBlkPool->pFree = pBlk;
    pEnd = ((uint8_t*)pPool) + poolSize;
    pBlkPool->pEnd = pEnd;
    pBlkPool->BlkSz = blkSize;
    pBlkPool->TotalCnt = 0;
    pBlkPool->UsedCnt = 0;

    /* Link all free blocks using offsets. */
    pEnd = ((uint8_t*)pEnd) - blkSize;
    while (1)
    {
        pNext = ((uint8_t*)pBlk) + blkSize;
        pBlkPool->TotalCnt++;
        if (pNext > pEnd)
            break;
        *((void **)pBlk) = pNext;
        pBlk = pNext;
    }

    /* End marker */
    *((void **)pBlk) = 0;
    return MQ_SUCCESS;
}


/****************************************************************************************************
 * @fn      AllocBlock
 *          Allocate a memory block from the given memory pool
 *
 * @param   [IN]pPool - Pointer to the memory pool from which fixed size memory block is requested
 *
 * @return  Pointer to the allocated memory block
 *
 ***************************************************************************************************/
void *AllocBlock( void *pPool )
{
    /* Allocate a memory block and return start address. */
    void **free;
    SETUP_CRITICAL_SECTION();

    ENTER_CRITICAL_SECTION();
    free = ((BlkMem_t*)pPool)->pFree;
    if (free)
    {
        ((BlkMem_t*)pPool)->pFree = *free;
        ((BlkMem_t*)pPool)->UsedCnt++;
    }
    EXIT_CRITICAL_SECTION();
    return (free);
}


/****************************************************************************************************
 * @fn      FreeBlock
 *          Return to the memory pool a block of memory that was previously allocated from it.
 *
 * @param   [IN]pPool - Pointer to the memory pool to which the previously allocated memory block is
 *              being returned
 * @param   [IN]pBlock - Pointer to the memory block that is being returned to the pool
 *
 * @return  MQ_SUCCESS if successful, -MQ_INVALID_PARAMETER otherwise
 *
 ***************************************************************************************************/
int16_t FreeBlock( void *pPool, void *pBlock )
{
    SETUP_CRITICAL_SECTION();
    /* Check if the block belongs to pool before trying to free it */
    if ((pBlock < pPool) || (pBlock >= ((BlkMem_t*)pPool)->pEnd))
    {
        return (-MQ_INVALID_PARAMETER);
    }

    ENTER_CRITICAL_SECTION();
    *((void **)pBlock) = ((BlkMem_t*)pPool)->pFree;
    ((BlkMem_t*)pPool)->pFree = pBlock;
    ((BlkMem_t*)pPool)->UsedCnt--;
    EXIT_CRITICAL_SECTION();
    return MQ_SUCCESS;
}


/****************************************************************************************************
 * @fn      GetPoolStats
 *          Returns the total block count and used blocks count from the given pool
 *
 * @param   [IN]pPool - Pointer to the memory pool that has been initialized
 * @param   [OUT]pTotalCount - return the total number of blocks in the given pool
 * @param   [OUT]pUsedCount - return the current used block count
 *
 * @return  none
 *
 ***************************************************************************************************/
void GetPoolStats( void *pPool, uint32_t *pTotalCount, uint32_t *pUsedCount )
{
    SETUP_CRITICAL_SECTION();
    ENTER_CRITICAL_SECTION();
    if (pTotalCount)
    {
        *pTotalCount = ((BlkMem_t*)pPool)->TotalCnt;
    }
    if (pUsedCount)
    {
        *pUsedCount = ((BlkMem_t*)pPool)->UsedCnt;
    }
    EXIT_CRITICAL_SECTION();
}


/*-------------------------------------------------------------------------------------------------*\
 |    E N D   O F   F I L E
\*-------------------------------------------------------------------------------------------------*/
