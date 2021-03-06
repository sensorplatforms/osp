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
#if !defined (COMMON_H)
#define   COMMON_H

/*-------------------------------------------------------------------------------------------------*\
 |    I N C L U D E   F I L E S
\*-------------------------------------------------------------------------------------------------*/
#include <stdio.h>
#include "hw_setup.h"
#include "asf_msgstruct.h"
//#include "main.h"
#include "osp-types.h"
#include "asf_types.h"
#include "osp-version.h"

/*-------------------------------------------------------------------------------------------------*\
 |    C O N S T A N T S   &   M A C R O S
\*-------------------------------------------------------------------------------------------------*/
#ifdef ANDROID_DEMO
# define ANDROID_COMM_TASK              I2CSLAVE_COMM_TASK_ID
#endif

#define TIMER_SYS_ID                            0xC0DEFEEDUL

/* Critical Section Locks */
#define OS_SETUP_CRITICAL()                     int wasMasked
#define OS_ENTER_CRITICAL()                     wasMasked = __disable_irq()
#define OS_LEAVE_CRITICAL()                     if (!wasMasked) __enable_irq()

    //#define RESET_ON_ASSERT
    
#ifdef RESET_ON_ASSERT
# define SysRESET()         NVIC_SystemReset()
#else
# define SysRESET()         while(1)
#endif


#ifdef DEBUG_BUILD
#define printf( format, ... ) do { char localbuf[128]; snprintf(localbuf, 127, format, ## __VA_ARGS__); putLineUART(localbuf); } while(0)

/* Note: The USART enable in the asserts is required because in SensorAcq task we disable
   it during sampling and in some rare cases there can be an assert while the UART was
   in the disabled state (e.g. ISR doing CreateMessage and asserting) */
# define ERR_LOG_MSG_SZ                         150
# define ASF_assert( condition )                                                           \
    if (!(condition)) {                                                                     \
        extern char _errBuff[];                                                            \
        __disable_irq();                                                                   \
        AssertIndication();                                                                \
        FlushUart();                                                                       \
        snprintf(_errBuff, ERR_LOG_MSG_SZ, "ASSERT: %s(%d) - [%s]", __MODULE__,            \
            __LINE__, #condition);                                                         \
        printf("%s\r\n", _errBuff);                                                        \
        SysRESET();                                                                        \
    }

# define ASF_assert_var( condition, var1, var2, var3 )                                     \
    if (!(condition)) {                                                                     \
        extern char _errBuff[];                                                            \
        __disable_irq();                                                                   \
        AssertIndication();                                                                \
        FlushUart();                                                                       \
        snprintf(_errBuff, ERR_LOG_MSG_SZ, "ASSERT: %s(%d) - [%s], 0x%X, 0x%X, 0x%X",      \
         __MODULE__, __LINE__, #condition, (uint32_t)var1, (uint32_t)var2, (uint32_t)var3);\
        printf("%s\r\n", _errBuff);                                                        \
        SysRESET();                                                                        \
    }

# define ASF_assert_fatal( condition )                                                     \
    if (!(condition)) {                                                                    \
        extern char _errBuff[];                                                            \
        __disable_irq();                                                                   \
        AssertIndication();                                                                \
        FlushUart();                                                                       \
        snprintf(_errBuff, ERR_LOG_MSG_SZ, "ASSERT_FATAL: %s(%d) - [%s]", __MODULE__,      \
            __LINE__, #condition);                                                         \
        printf("%s\r\n", _errBuff);                                                        \
        SysRESET();                                                                        \
    }

# define ASF_assert_msg( condition, message )                                              \
    if (!(condition)) {                                                                    \
        extern char _errBuff[];                                                            \
        __disable_irq();                                                                   \
        AssertIndication();                                                                \
        FlushUart();                                                                       \
        snprintf(_errBuff, ERR_LOG_MSG_SZ, "ASSERT: %s(%d) - [%s], MSG:%.100s",            \
            __MODULE__, __LINE__, #condition, message);                                    \
        printf("%s\r\n", _errBuff);                                                        \
        SysRESET();                                                                        \
    }

#else
# define ASF_assert( condition )                         ((void)0)
# define ASF_assert_var( condition, var1, var2, var3 )   ((void)0)
# define ASF_assert_fatal( condition )                   ((void)0)
# define ASF_assert_msg( condition, message )            ((void)0)
#endif //DEBUG_BUILD

/* Command Parser / UART receive event */
#define UART_CMD_RECEIVE            0x0020
#define UART_CRLF_RECEIVE           0x0040 ///< Receive CR or LF character
#define EVT_WAIT_FOREVER            0xFFFF

#define ASFKillTimer( ptim )         \
    _ASFKillTimer( ptim, __MODULE__, __LINE__ )
#define ASFTimerExpiry( info )       \
    _ASFTimerExpiry( info, __MODULE__, __LINE__ )
#define ASFTimerStart( owner, ref, tick, pTimer )  \
    _ASFTimerStart( owner, ref, tick, pTimer, __MODULE__, __LINE__ )

/* Defines for MTCK Commands & handlers */
#define CR                          0x0D
#define LF                          0x0A

/*-------------------------------------------------------------------------------------------------*\
 |    T Y P E   D E F I N I T I O N S
\*-------------------------------------------------------------------------------------------------*/
typedef enum AppResultCodesTag
{
    APP_OK      = 0,
    APP_ERR     = 1
} AppResult;

typedef struct AsfTimerTag
{
    osTimerId       timerId;   /**< Id of the timer - internal use    */
    TaskId          owner;     /**< Owner task that created the timer */
    uint16_t        ticks;     /**< Timeout value in system ticks     */
    uint16_t        userValue; /**< User defined value                */
    uint16_t        info;      /**< use by AsfTimer module            */
    uint32_t        sysUse;    /**< For use by the system             */    
} AsfTimer;

#define NULL_TIMER {(TimerId)0, (TaskId)0, 0, 0, 0}

typedef void (*fpDmaEnables_t)(void);
typedef osp_bool_t (*fpInputValidate_t)(uint8_t);

/* Buffers used by DMA */
typedef unsigned long Address;
typedef struct PktBufferTag
{
    Address  *pNext;
    uint32_t  bufLen;
    Address   bufStart;
} PktBuff_t;

/* Buffer Macros to manage linked list */
#define M_GetBuffStart(x)   (void *)(&(((PktBuff_t*)(x))->bufStart))
#define M_GetBuffLen(x)     (((PktBuff_t*)(x))->bufLen)
#define M_SetBuffLen(x,l)   (((PktBuff_t*)(x))->bufLen) = l
#define M_NextBlock(x)      (((PktBuff_t*)(x))->pNext)
/* Get the address of the block having the buffer address */
#define M_GetBuffBlock(p)   (void *)((uint8_t*)(p) - offsetof(PktBuff_t, bufStart))


/*-------------------------------------------------------------------------------------------------*\
 |    E X T E R N A L   V A R I A B L E S   &   F U N C T I O N S
\*-------------------------------------------------------------------------------------------------*/
int _dprintf( uint8_t dbgLvl, const char *fmt, ...);
extern uint32_t 	g_logging;

/*-------------------------------------------------------------------------------------------------*\
 |    P U B L I C   V A R I A B L E S   D E F I N I T I O N S
\*-------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------*\
 |    P U B L I C   F U N C T I O N   D E C L A R A T I O N S
\*-------------------------------------------------------------------------------------------------*/
/* Debug print scheme for different levels:
 *   0 - This level must only be used for released code
 *   1 - Regular debug messages
 *   2 - Extra debug messages
**/
//New macros
void putLineUART(const char *);
void put_u32(const uint32_t);

#define D0_printf( format, ... ) do { char localbuf[128]; snprintf(localbuf, 127, format, ## __VA_ARGS__); putLineUART(localbuf); } while(0)       
#define D1_printf( format, ... ) do { char localbuf[128]; snprintf(localbuf, 127, format, ## __VA_ARGS__); putLineUART(localbuf); } while(0)
#define D2_printf( format, ... ) do { char localbuf[128]; snprintf(localbuf, 127, format, ## __VA_ARGS__); putLineUART(localbuf); } while(0)
#define Print_LIPS(d,x...) D0_printf("{!" d "!}\r\n",x)

void _ASFTimerStart( TaskId owner, uint16_t ref, uint16_t tick, AsfTimer *pTimer, char *_file, int _line  );
osp_bool_t ASFTimerStarted ( AsfTimer *pTimer );
void _ASFKillTimer( AsfTimer *pTimer, char *_file, int _line );
void _ASFTimerExpiry ( uint32_t info, char *_file, int _line );
void ASFTimerCallback(void const *argument);
void AsfInitialiseTasks ( void );

/* User instrumentation hooks */
void InstrManagerUserInit( void );
osp_bool_t InstrManagerUserHandler( MessageBuffer *pMsg );

/* Uart Support Functions */
#ifdef UART_DMA_ENABLE
void *RemoveFromList( PortInfo *pPort );
void AddToList( PortInfo *pPort, void *pPBuff, uint16_t length );
#endif

void SendDataReadyIndication(uint8_t sensorId, uint32_t timeStamp);

uint32_t GetCurrentTime(void);

#endif /* COMMON_H */
/*-------------------------------------------------------------------------------------------------*\
 |    E N D   O F   F I L E
\*-------------------------------------------------------------------------------------------------*/
