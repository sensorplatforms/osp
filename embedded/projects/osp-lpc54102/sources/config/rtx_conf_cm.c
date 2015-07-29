/*----------------------------------------------------------------------------
 *      RL-ARM - RTX
 *----------------------------------------------------------------------------
 *      Name:    RTX_CONFIG.C
 *      Purpose: Configuration of RTX Kernel for Cortex-M
 *      Rev.:    V4.70
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include "rtl.h"
#include "board.h"
#include "common.h"

/*----------------------------------------------------------------------------
 *      RTX User configuration part BEGIN
 *---------------------------------------------------------------------------*/

//-------- <<< Use Configuration Wizard in Context Menu >>> -----------------
//
// <h>Thread Configuration
// =======================
//
//   <o>Number of concurrent running threads <0-250>
//   <i> Defines max. number of threads that will run at the same time.
//   <i> Default: 6
#ifndef OS_TASKCNT
 #define OS_TASKCNT     NUMBER_OF_TASKS
 /* This should be NUMBER_OF_TASKS but code size surprisingly increases if OS_TASKCNT is reduced.*/
#endif

//   <o>Task stack size [bytes] <20-4096:8><#/4>
//   <i> Set the stack size for tasks which is assigned by the system.
//   <i> Default: 200
#ifndef OS_STKSIZE
 #define OS_STKSIZE     0x500
#endif

//   <o>Main Thread stack size [bytes] <64-4096:8><#/4>
//   <i> Defines stack size for main thread.
//   <i> Default: 200
#ifndef OS_MAINSTKSIZE
#define OS_MAINSTKSIZE 0x200
#endif

//   <o>Number of tasks with user-provided stack <0-250>
//   <i> Define the number of tasks that will use a bigger stack.
//   <i> The memory space for the stack is provided by the user.
//   <i> Default: 0
#ifndef OS_PRIVCNT
 #define OS_PRIVCNT     0
#endif

//   <o>Total stack size [bytes] for threads with user-provided stack size <0-4096:8><#/4>
//   <i> Defines the combined stack size for threads with user-provided stack size.
//   <i> Default: 0
#ifndef OS_PRIVSTKSIZE
#define OS_PRIVSTKSIZE OS_PRIVCNT*0x600
#endif

// <q>Check for the stack overflow
// ===============================
// <i> Include the stack checking code for a stack overflow.
// <i> Note that additional code reduces the Kernel performance.
#ifndef OS_STKCHECK
 #define OS_STKCHECK    1
#endif

//   <q>Stack usage watermark
//   <i> Initialize thread stack with watermark pattern for analyzing stack usage (current/maximum) in System and Thread Viewer.
//   <i> Enabling this option increases significantly the execution time of osThreadCreate.
#ifndef OS_STKINIT
#define OS_STKINIT      0
#endif
 
// <q>Run in privileged mode
// =========================
// <i> Run all Tasks in privileged mode.
// <i> Default: Unprivileged
#ifndef OS_RUNPRIV
 #define OS_RUNPRIV     1
#endif

//   <o>Timer Thread stack size [bytes] <64-4096:8><#/4>
//   <i> Defines stack size for Timer thread.
//   <i> Default: 200
#ifndef OS_TIMERSTKSZ
#define OS_TIMERSTKSZ 0x100
#endif

#if (OS_TIMERS != 0)
#define OS_TASK_CNT (OS_TASKCNT + 1)
#define OS_PRIV_CNT (OS_PRIVCNT + 2)
#define OS_STACK_SZ (4*(OS_PRIVSTKSIZE+OS_MAINSTKSIZE+OS_TIMERSTKSZ))
#else
#define OS_TASK_CNT  OS_TASKCNT
#define OS_PRIV_CNT (OS_PRIVCNT + 1)
#define OS_STACK_SZ (4*(OS_PRIVSTKSIZE+OS_MAINSTKSIZE))
#endif

const U16 C_gOsStkSize = OS_STKSIZE*4;

// </h>
// <h>Tick Timer Configuration
// =============================
//   <o>Hardware timer <0=> Core SysTick <1=> Peripheral Timer
//   <i> Define the on-chip timer used as a time-base for RTX.
//   <i> Default: Core SysTick
#ifndef OS_TIMER
 #define OS_TIMER       0
#endif

//   <o>Timer clock value [Hz] <1-1000000000>
//   <i> Set the timer clock value for selected timer.
//   <i> Default: 6000000  (6MHz)
#ifndef OS_CLOCK
 #define OS_CLOCK       SYSTEM_CLOCK_FREQ
#endif

//   <o>Timer tick value [us] <1-1000000>
//   <i> Set the timer tick value for selected timer.
//   <i> Default: 10000  (10ms)
#ifndef OS_TICK
 #define OS_TICK        USEC_PER_TICK
#endif

// </h>

// <h>System Configuration
// =======================
// <e>Round-Robin Task switching
// =============================
// <i> Enable Round-Robin Task switching.
#ifndef OS_ROBIN
 #define OS_ROBIN       0
#endif

//   <o>Round-Robin Timeout [ticks] <1-1000>
//   <i> Define how long a task will execute before a task switch.
//   <i> Default: 5
#ifndef OS_ROBINTOUT
 #define OS_ROBINTOUT   5
#endif

// </e>

//   <o>Number of user timers <0-250>
//   <i> Define max. number of user timers that will run at the same time.
//   <i> Default: 0  (User timers disabled)
#ifndef OS_TIMERCNT
 #define OS_TIMERCNT    8
#endif

/* Memory pool for user specified stack allocation (+main, +timer) */
uint64_t       os_stack_mem[2+OS_PRIV_CNT+(OS_STACK_SZ/8)];
uint32_t const os_stack_sz = sizeof(os_stack_mem);

/* User Timers Resources */
#if (OS_TIMERS != 0)
extern void osTimerThread (void const *argument);
osThreadDef(osTimerThread, osPriorityNormal-2, 1, 4*OS_TIMERSTKSZ);
osThreadId osThreadId_osTimerThread;
osMessageQDef(osTimerMessageQ, OS_TIMERCBQS, void *);
osMessageQId osMessageQId_osTimerMessageQ;

#else
osThreadDef_t os_thread_def_osTimerThread = { NULL };
osThreadId osThreadId_osTimerThread;
osMessageQDef(osTimerMessageQ, 0, void *);
osMessageQId osMessageQId_osTimerMessageQ;
#endif

//   <o>ISR FIFO Queue size<4=>   4 entries  <8=>   8 entries
//                         <12=> 12 entries  <16=> 16 entries
//                         <24=> 24 entries  <32=> 32 entries
//                         <48=> 48 entries  <64=> 64 entries
//                         <96=> 96 entries
//   <i> ISR functions store requests to this buffer,
//   <i> when they are called from the iterrupt handler.
//   <i> Default: 16 entries
#ifndef OS_FIFOSZ
 #define OS_FIFOSZ      16
#endif

// </h>

//------------- <<< end of configuration section >>> -----------------------

// Standard library system mutexes
// ===============================
//  Define max. number system mutexes that are used to protect 
//  the arm standard runtime library. For microlib they are not used.
#ifndef OS_MUTEXCNT
 #define OS_MUTEXCNT    8
#endif

/*----------------------------------------------------------------------------
 *      RTX User configuration part END
 *---------------------------------------------------------------------------*/

#define OS_TRV          ((U32)(((double)OS_CLOCK*(double)OS_TICK)/1E6)-1)

/*----------------------------------------------------------------------------
 *      Global Functions
 *---------------------------------------------------------------------------*/

/*--------------------------- os_idle_demon ---------------------------------*/

__task void os_idle_demon (void) {
	/* The idle demon is a system task, running when no other task is ready */
	/* to run. The 'os_xxx' function calls are not allowed from this task.  */
	for (;;) {
		__WFI();
	}
}

/*--------------------------- os_tick_init ----------------------------------*/

#if (OS_TIMER != 0)
int os_tick_init (void) {
  /* Initialize hardware timer as system tick timer. */
  /* ... */
  return (-1);  /* Return IRQ number of timer (0..239) */
}
#endif

/*--------------------------- os_tick_irqack --------------------------------*/

#if (OS_TIMER != 0)
void os_tick_irqack (void) {
  /* Acknowledge timer interrupt. */
  /* ... */
}
#endif

/*--------------------------- os_tmr_call -----------------------------------*/

void os_tmr_call (U16 info) {
  /* This function is called when the user timer has expired. Parameter  */
  /* 'info' holds the value, defined when the timer was created.         */
  ASFTimerExpiry( info );

}

/*--------------------------- os_error --------------------------------------*/

void os_error (U32 err_code) {
  /* This function is called when a runtime error is detected. Parameter */
  /* 'err_code' holds the runtime error code (defined in RTL.H).         */

  /* HERE: include optional code to be executed on runtime error. */
   ASF_assert_var( false, err_code, 0, 0 );
   //for (;;);
}


/*----------------------------------------------------------------------------
 *      RTX Configuration Functions
 *---------------------------------------------------------------------------*/

#include "rtx_lib.c"

/*----------------------------------------------------------------------------
 *      RTX Startup
 *---------------------------------------------------------------------------*/

/* Main Thread definition */
extern int main (void);
osThreadDef_t os_thread_def_main = {(os_pthread)main, osPriorityNormal, 1, 4*OS_MAINSTKSIZE };


#if defined (__CC_ARM)

#ifdef __MICROLIB
void _main_init (void) __attribute__((section(".ARM.Collect$$$$000000FF")));
void _main_init (void) {
  osKernelInitialize();
  osThreadCreate(&os_thread_def_main, NULL);
  osKernelStart();
  for (;;);
}
#else
__asm void __rt_entry (void) {

  IMPORT  __user_setup_stackheap
  IMPORT  __rt_lib_init
  IMPORT  os_thread_def_main
  IMPORT  osKernelInitialize
  IMPORT  osKernelStart
  IMPORT  osThreadCreate
  IMPORT  exit

  BL      __user_setup_stackheap
  MOV     R1,R2
  BL      __rt_lib_init
  BL      osKernelInitialize
  LDR     R0,=os_thread_def_main
  MOVS    R1,#0
  BL      osThreadCreate
  BL      osKernelStart
  BL      exit

  ALIGN
}
#endif

#elif defined (__GNUC__)

#ifdef __CS3__

/* CS3 start_c routine.
 *
 * Copyright (c) 2006, 2007 CodeSourcery Inc
 *
 * The authors hereby grant permission to use, copy, modify, distribute,
 * and license this software and its documentation for any purpose, provided
 * that existing copyright notices are retained in all copies and that this
 * notice is included verbatim in any distributions. No written agreement,
 * license, or royalty fee is required for any of the authorized uses.
 * Modifications to this software may be copyrighted by their authors
 * and need not follow the licensing terms described here, provided that
 * the new terms are clearly indicated on the first page of each file where
 * they apply.
 */

#include "cs3.h"

extern void __libc_init_array (void);

__attribute ((noreturn)) void __cs3_start_c (void){
  unsigned regions = __cs3_region_num;
  const struct __cs3_region *rptr = __cs3_regions;

  /* Initialize memory */
  for (regions = __cs3_region_num, rptr = __cs3_regions; regions--; rptr++) {
    long long *src = (long long *)rptr->init;
    long long *dst = (long long *)rptr->data;
    unsigned limit = rptr->init_size;
    unsigned count;

    if (src != dst)
      for (count = 0; count != limit; count += sizeof (long long))
        *dst++ = *src++;
    else 
      dst = (long long *)((char *)dst + limit);
    limit = rptr->zero_size;
    for (count = 0; count != limit; count += sizeof (long long))
      *dst++ = 0;
  }

  /* Run initializers.  */
  __libc_init_array ();

  osKernelInitialize();
  osThreadCreate(&os_thread_def_main, NULL);
  osKernelStart();
  for (;;);
}

#else

__attribute__((naked)) void software_init_hook (void) {
  __asm (
    ".syntax unified\n"
    ".thumb\n"
    "movs r0,#0\n"
    "movs r1,#0\n"
    "mov  r4,r0\n"
    "mov  r5,r1\n"
    "ldr  r0,= __libc_fini_array\n"
    "bl   atexit\n"
    "bl   __libc_init_array\n"
    "mov  r0,r4\n"
    "mov  r1,r5\n"
    "bl   osKernelInitialize\n"
    "ldr  r0,=os_thread_def_main\n"
    "movs r1,#0\n"
    "bl   osThreadCreate\n"
    "bl   osKernelStart\n"
    "bl   exit\n"
  );
}

#endif

#elif defined (__ICCARM__)

extern int  __low_level_init(void);
extern void __iar_data_init3(void);
extern void exit(int arg);

__noreturn __stackless void __cmain(void) {
  int a;
  
  if (__low_level_init() != 0) {
    __iar_data_init3();
  }
  osKernelInitialize();
  osThreadCreate(&os_thread_def_main, NULL);
  a = osKernelStart();
  exit(a);
}

#endif

/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/
