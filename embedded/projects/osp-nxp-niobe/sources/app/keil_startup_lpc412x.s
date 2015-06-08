;/*****************************************************************************
; * @file:    startup_LPC412x.s
; * @purpose: CMSIS Cortex-M4/M0+ Core Device Startup File
; *           for the NXP LPC412x Device Series
; * @version: V1.0
; * @date:    16. Aug. 2012
; *------- <<< Use Configuration Wizard in Context Menu >>> ------------------
; *
; * Copyright (C) 2012 ARM Limited. All rights reserved.
; * ARM Limited (ARM) is supplying this software for use with Cortex-M0+
; * processor based microcontrollers.  This file can be freely distributed
; * within development tools that are supporting such ARM based processors.
; *
; * THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
; * OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
; * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
; * ARM SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
; * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
; *
; *****************************************************************************/


; <h> Stack Configuration
;   <o> Stack Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>
                EXPORT  gStackSize
gStackSize      EQU     0x00000400

                EXPORT  gStackMem
                AREA    STACK, NOINIT, READWRITE, ALIGN=3
gStackMem       SPACE   gStackSize
__initial_sp


; <h> Heap Configuration
;   <o>  Heap Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

Heap_Size       EQU     0x00000000

                AREA    HEAP, NOINIT, READWRITE, ALIGN=3
__heap_base
Heap_Mem        SPACE   Heap_Size
__heap_limit


                PRESERVE8
                THUMB


; Vector Table Mapped to Address 0 at Reset

                AREA    RESET, DATA, READONLY
                EXPORT  __Vectors

__Vectors       DCD     __initial_sp              ; Top of Stack
                DCD     Reset_Handler             ; Reset Handler
                DCD     NMI_Handler               ; NMI Handler
                DCD     HardFault_Handler         ; Hard Fault Handler
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     SVC_Handler               ; SVCall Handler
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     PendSV_Handler            ; PendSV Handler
                DCD     SysTick_Handler           ; SysTick Handler

                ; Available on M0/M4
                ; External Interrupts
                DCD     WDT_IRQHandler              ; Watchdog
                DCD     BOD_IRQHandler              ; Brown Out Detect
                DCD     FLASH_IRQHandler            ; Non-Volatile Memory Controller
                DCD     DMA_IRQHandler              ; DMA Controller
                DCD     GINT0_IRQHandler            ; GPIO Group0 Interrupt
                DCD     PIN_INT0_IRQHandler          ; PIO INT0
                DCD     PIN_INT1_IRQHandler          ; PIO INT1
                DCD     PIN_INT2_IRQHandler          ; PIO INT2
                DCD     PIN_INT3_IRQHandler          ; PIO INT3
                DCD     UTICK_IRQHandler            ; UTICK timer
                DCD     MRT_IRQHandler              ; Multi-Rate Timer
                DCD     CTIMER0_IRQHandler          ; CTIMER0
                DCD     CTIMER1_IRQHandler          ; CTIMER1
                DCD     CTIMER2_IRQHandler          ; CTIMER2
                DCD     CTIMER3_IRQHandler          ; CTIMER3
                DCD     CTIMER4_IRQHandler          ; CTIMER4
                DCD     SCT_IRQHandler              ; Smart Counter Timer
                DCD     UART0_IRQHandler            ; UART0
                DCD     UART1_IRQHandler            ; UART1
                DCD     UART2_IRQHandler            ; UART2
                DCD     UART3_IRQHandler            ; UART3
                DCD     I2C0_IRQHandler             ; I2C0 controller
                DCD     I2C1_IRQHandler             ; I2C1 controller
                DCD     I2C2_IRQHandler             ; I2C2 controller
                DCD     SPI0_IRQHandler             ; SPI0 controller
                DCD     SPI1_IRQHandler             ; SPI1 controller
                DCD     ADC_SEQA_IRQHandler         ; ADC SEQA
                DCD     ADC_SEQB_IRQHandler         ; ADC SEQB
                DCD     ADC_THCMP_OVR_IRQHandler    ; ADC THCMP and OVERRUN ORed
                DCD     RTC_IRQHandler              ; RTC Timer
                DCD     EZH_IRQHandler              ; EZH
                DCD     MBOX_IRQHandler             ; Mailbox

                IF      :DEF: CORE_M4
                ; For M4 only
                DCD     GINT1_IRQHandler            ; GPIO Group1 Interrupt
                DCD     PIN_INT4_IRQHandler          ; PIO INT4
                DCD     PIN_INT5_IRQHandler          ; PIO INT5
                DCD     PIN_INT6_IRQHandler          ; PIO INT6
                DCD     PIN_INT7_IRQHandler          ; PIO INT7
                DCD     SPI2_IRQHandler             ; SPI2 controller
                DCD     SPI3_IRQHandler             ; SPI3 controller
                DCD     0                           ; Reserved
                DCD     OSTIMER_IRQHandler          ; OS Timer
                DCD     PVT0_A_IRQHandler           ; PVT0 Amber
                DCD     PVT0_R_IRQHandler           ; PVT0 Red
                DCD     PVT1_A_IRQHandler           ; PVT1 Amber
                DCD     PVT1_R_IRQHandler           ; PVT1 Red
                ENDIF

;//   <h> Code Read Protection level (CRP)
;//     <o>    CRP_Level:
;//                     <0xFFFFFFFF=> Disabled
;//                     <0x4E697370=> NO_ISP
;//                     <0x12345678=> CRP1
;//                     <0x87654321=> CRP2
;//                     <0x43218765=> CRP3 (Are you sure?)
;//   </h>
CRP_Level        EQU     0xFFFFFFFF

                IF      :LNOT::DEF:NO_CRP
                AREA    |.ARM.__at_0x02FC|, CODE, READONLY
CRP_Key         DCD     0xFFFFFFFF
                ENDIF


                AREA    |.text|, CODE, READONLY


; Reset Handler

Reset_Handler   PROC
                EXPORT  Reset_Handler             [WEAK]
                IMPORT  SystemInit
                IMPORT  __main

;--------------------------------------------------------------------------------
;    Support for Dual Core, we check if we are M0 or M4
;--------------------------------------------------------------------------------
cpu_ctrl         EQU    0x40000300
coproc_boot     EQU    0x40000304
coproc_stack    EQU    0x40000308

enable_FPU
               ; Enable Cortex-M4 FPU Need this for initialisation of ARM fpu lib
            LDR.W R0, =0xE000ED88
            LDR R1, [R0]
            ORR R1, R1, #(0xF << 20)
            STR R1, [R0]
dual_boot
;            ; Find out if M0 or M4 is booting (both can come through here)
                        MOVS     r5,#0x1
                        LDR      r0, =0xE000ED00
                        LDR      r1, [r0]                        ; READ CPUID register
                        LDR      r2,=0x410cc601                 ; CM0 r0p1 identifier
                        EORS     r1,r1,r2                       ; XOR to see if we are C0
                        LDR      r3, =cpu_ctrl                    ; get address of CPU_CTRL
                        LDR      r1,[r3]                        ; read cpu_ctrl reg into r1
                        BEQ     cm0_boot
cm4_boot
                        LDR    r0,=coproc_boot                  ; coproc boot address
                        LDR    r0,[r0]                         ; get address to branch to
                        MOVS    r0,r0                        ; Check if 0
                        BEQ    check_master_m4                 ; if zero in boot reg, we just branch to __main (assuming we are main proc)
                        BX      r0                              ; otherwise, we branch to boot address
gotomain
                        LDR     r0,=openStart
                        BX    r0

cm0_boot

                        LDR    r0,=coproc_boot                 ; coproc boot address
                        LDR    r0,[r0]                         ; get address to branch to
                        MOVS r0,r0                            ; Check if 0
                        BEQ    check_master_m0                 ; if zero in boot reg, we just branch to __main (assuming we are main proc)

                        LDR  r1,=coproc_stack               ; pickup co procesor stackpointer (from syscon CPSTACK)
                        LDR  r1,[r1]
                        MOV  sp,r1

                        BX      r0                          ; goto boot address

check_master_m0
                        ANDS     r1,r1,r5                       ; bit test bit0
                        BEQ      gotomain                       ; if we get 0, that means we are masters
                        B     goto_sleep_pending_reset        ; Otherwise, there is no startup vector for slave, so we go to sleep

check_master_m4
                        ANDS     r1,r1,r5                       ; bit test bit0
                        BNE      gotomain                       ; if we get 1, that means we are masters
                        B     goto_sleep_pending_reset        ; Otherwise, there is no startup vector for slave, so we go to sleep

goto_sleep_pending_reset
                        MOV     SP,r5                           ; load 0x1 into SP so that any stacking (eg on NMI) will not cause us to wakeup
                                                                ; and write to uninitialised STack area (instead it will LOCK us up before we cause damage)
                                                                ; this code should only be reached if debugger bypassed ROM or we changed master without giving
                                                                ; correct start address, the only way out of this is through a debugger change of SP and PC
sleepo
             WFI                                     ; go to sleep
             b sleepo

openStart
                LDR     R0, =SystemInit
                BLX     R0
                LDR     R0, =__main
                BX      R0
                ENDP


; Dummy Exception Handlers (infinite loops which can be modified)
; now, under COMMON lpc8xx_nmi.c and lpc8xx_nmi.h, a real NMI handler is created if NMI is enabled
; for particular peripheral.
NMI_Handler     PROC
                EXPORT  NMI_Handler               [WEAK]
                B       .
                ENDP
HardFault_Handler\
                PROC
                EXPORT  HardFault_Handler         [WEAK]
				tst lr, #4
				ite eq 
				mrseq r0, msp
				mrsne r0, psp
				ldr r1, [r0, #24]
                B       .
                ENDP
SVC_Handler     PROC
                EXPORT  SVC_Handler               [WEAK]
                B       .
                ENDP
PendSV_Handler  PROC
                EXPORT  PendSV_Handler            [WEAK]
                B       .
                ENDP
SysTick_Handler PROC
                EXPORT  SysTick_Handler           [WEAK]
                B       .
                ENDP

Default_Handler PROC
                EXPORT  WDT_IRQHandler            [WEAK]
                EXPORT  BOD_IRQHandler            [WEAK]
                EXPORT  FLASH_IRQHandler          [WEAK]
                EXPORT  DMA_IRQHandler            [WEAK]
                EXPORT  GINT0_IRQHandler          [WEAK]
                EXPORT  PIN_INT0_IRQHandler        [WEAK]
                EXPORT  PIN_INT1_IRQHandler        [WEAK]
                EXPORT  PIN_INT2_IRQHandler        [WEAK]
                EXPORT  PIN_INT3_IRQHandler        [WEAK]
                EXPORT  UTICK_IRQHandler          [WEAK]
                EXPORT  MRT_IRQHandler            [WEAK]
                EXPORT  CTIMER0_IRQHandler        [WEAK]
                EXPORT  CTIMER1_IRQHandler        [WEAK]
                EXPORT  CTIMER2_IRQHandler        [WEAK]
                EXPORT  CTIMER3_IRQHandler        [WEAK]
                EXPORT  CTIMER4_IRQHandler        [WEAK]
                EXPORT  SCT_IRQHandler            [WEAK]
                EXPORT  UART0_IRQHandler          [WEAK]
                EXPORT  UART1_IRQHandler          [WEAK]
                EXPORT  UART2_IRQHandler          [WEAK]
                EXPORT  UART3_IRQHandler          [WEAK]
                EXPORT  I2C0_IRQHandler           [WEAK]
                EXPORT  I2C1_IRQHandler           [WEAK]
                EXPORT  I2C2_IRQHandler           [WEAK]
                EXPORT  SPI0_IRQHandler           [WEAK]
                EXPORT  SPI1_IRQHandler           [WEAK]
                EXPORT  ADC_SEQA_IRQHandler       [WEAK]
                EXPORT  ADC_SEQB_IRQHandler       [WEAK]
                EXPORT  ADC_THCMP_OVR_IRQHandler  [WEAK]
                EXPORT  RTC_IRQHandler            [WEAK]
                EXPORT  EZH_IRQHandler            [WEAK]
                EXPORT  MBOX_IRQHandler           [WEAK]

                IF      :DEF: CORE_M4
                ; For M4 only
                EXPORT  GINT1_IRQHandler          [WEAK]
                EXPORT  PIN_INT4_IRQHandler        [WEAK]
                EXPORT  PIN_INT5_IRQHandler        [WEAK]
                EXPORT    PIN_INT6_IRQHandler        [WEAK]
                EXPORT    PIN_INT7_IRQHandler        [WEAK]
                EXPORT  SPI2_IRQHandler           [WEAK]
                EXPORT  SPI3_IRQHandler           [WEAK]
                EXPORT  Reserved_IRCHandler       [WEAK]
                EXPORT  OSTIMER_IRQHandler        [WEAK]
                EXPORT  PVT0_A_IRQHandler         [WEAK]
                EXPORT  PVT0_R_IRQHandler         [WEAK]
                EXPORT  PVT1_A_IRQHandler         [WEAK]
                EXPORT  PVT1_R_IRQHandler         [WEAK]
                ENDIF

WDT_IRQHandler
BOD_IRQHandler
FLASH_IRQHandler
DMA_IRQHandler
GINT0_IRQHandler
PIN_INT0_IRQHandler
PIN_INT1_IRQHandler
PIN_INT2_IRQHandler
PIN_INT3_IRQHandler
UTICK_IRQHandler
MRT_IRQHandler
CTIMER0_IRQHandler
CTIMER1_IRQHandler
CTIMER2_IRQHandler
CTIMER3_IRQHandler
CTIMER4_IRQHandler
SCT_IRQHandler
UART0_IRQHandler
UART1_IRQHandler
UART2_IRQHandler
UART3_IRQHandler
I2C0_IRQHandler
I2C1_IRQHandler
I2C2_IRQHandler
SPI0_IRQHandler
SPI1_IRQHandler
ADC_SEQA_IRQHandler
ADC_SEQB_IRQHandler
ADC_THCMP_OVR_IRQHandler
RTC_IRQHandler
EZH_IRQHandler
MBOX_IRQHandler

                IF      :DEF: CORE_M4
; For M4 only
GINT1_IRQHandler
PIN_INT4_IRQHandler
PIN_INT5_IRQHandler
PIN_INT6_IRQHandler
PIN_INT7_IRQHandler
SPI2_IRQHandler
SPI3_IRQHandler
Reserved_IRCHandler
OSTIMER_IRQHandler
PVT0_A_IRQHandler
PVT0_R_IRQHandler
PVT1_A_IRQHandler
PVT1_R_IRQHandler
                ENDIF

                B       .

                ENDP


                ALIGN


; User Initial Stack & Heap

                IF      :DEF:__MICROLIB

                EXPORT  __initial_sp
                EXPORT  __heap_base
                EXPORT  __heap_limit

                ELSE

                IMPORT  __use_two_region_memory
                EXPORT  __user_initial_stackheap

                IMPORT NewHeap
                IMPORT TotalStkNeeded
__user_initial_stackheap

                LDR     R0, =  NewHeap
                LDR     R4, =  TotalStkNeeded
                LDR     R1, =(gStackMem + gStackSize)
                ADD     R2, R0, R4
                LDR     R3, = gStackMem
                BX      LR

                ALIGN

                ENDIF


                END
