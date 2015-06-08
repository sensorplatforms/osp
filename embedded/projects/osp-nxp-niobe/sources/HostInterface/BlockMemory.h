/****************************************************************************************************
 * @file  BlockMemory.h
 *
 * Utility APIs & macros to declare and use block memory pool
 *
 * < Detailed description of subsystem >
 *
 * @seealso < Links to relevant files, references, etc. >
 *
 * @info
 * $Author: rverma $
 * $DateTime: 2014/10/30 17:45:42 $
 * $Revision: #3 $
 * $Id: //AudEngr/Hardware/DeltaPlus/Software/BoskoApp_D300_SensorHub/overlay/SensorHub/BlockMemory.h#3 $
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
#if !defined (BLOCK_MEMORY_H)
#define   BLOCK_MEMORY_H

/*-------------------------------------------------------------------------------------------------*\
 |    I N C L U D E   F I L E S
\*-------------------------------------------------------------------------------------------------*/
#include <stdint.h>
#if 0
#include "xtos.h"
#endif
/*-------------------------------------------------------------------------------------------------*\
 |    C O N S T A N T S   &   M A C R O S
\*-------------------------------------------------------------------------------------------------*/
//Define these for thread-safe/interrupt-safe usage
#include "hw_setup_niobe.h"

#if 0
#define SETUP_CRITICAL_SECTION()        unsigned register int __intState
#define ENTER_CRITICAL_SECTION()        __intState = xtos_ints_disable()
#define EXIT_CRITICAL_SECTION()         xtos_ints_enable(__intState)
#else
#define SETUP_CRITICAL_SECTION()        unsigned register int __intState
#define ENTER_CRITICAL_SECTION()        do {__intState = GetContext(); if (__intState == CTX_THREAD) __disable_irq(); } while (0)
#define EXIT_CRITICAL_SECTION()         do {if (__intState == CTX_THREAD) __enable_irq(); } while (0)
#endif

/* The DECLARE_BLOCK_POOL macro declares an array of bytes that can be used as a memory pool for fixed
 * block allocation.
 * @param [IN]pool - User defined unique C-Style name of the pool
 * @param [IN]size - Size in bytes of each memory block that will be allocated from the pool
 * @param [IN]cnt - Number of blocks the pool should contain
 */
#define DECLARE_BLOCK_POOL(pool,size,cnt)     uint32_t pool[(((size)+3)/4)*(cnt) + 5]
/* Note: +5 is to account for size of local block structure */


/*-------------------------------------------------------------------------------------------------*\
 |    T Y P E   D E F I N I T I O N S
\*-------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------*\
 |    E X T E R N A L   V A R I A B L E S   &   F U N C T I O N S
\*-------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------*\
 |    P U B L I C   V A R I A B L E S   D E F I N I T I O N S
\*-------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------*\
 |    P U B L I C   F U N C T I O N   D E C L A R A T I O N S
\*-------------------------------------------------------------------------------------------------*/
int16_t InitBlockPool( void *pPool, uint32_t poolSizeBytes, uint32_t blkSize );
void *AllocBlock( void *pPool );
int16_t FreeBlock( void *pPool, void *pBlock );
void GetPoolStats( void *pPool, uint32_t *pTotalCount, uint32_t *pUsedCount );


#endif /* BLOCK_MEMORY_H */
/*-------------------------------------------------------------------------------------------------*\
 |    E N D   O F   F I L E
\*-------------------------------------------------------------------------------------------------*/
