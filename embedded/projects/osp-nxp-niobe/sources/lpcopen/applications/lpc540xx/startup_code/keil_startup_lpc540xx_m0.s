;/*
; * @brief LPC540xx M0 core startup code for Keil
; *
; * @note
; * Copyright(C) NXP Semiconductors, 2014
; * All rights reserved.
; *
; * @par
; * Software that is described herein is for illustrative purposes only
; * which provides customers with programming information regarding the
; * LPC products.  This software is supplied "AS IS" without any warranties of
; * any kind, and NXP Semiconductors and its licensor disclaim any and
; * all warranties, express or implied, including all implied warranties of
; * merchantability, fitness for a particular purpose and non-infringement of
; * intellectual property rights.  NXP Semiconductors assumes no responsibility
; * or liability for the use of the software, conveys no license or rights under any
; * patent, copyright, mask work right, or any other intellectual property rights in
; * or to any products. NXP Semiconductors reserves the right to make changes
; * in the software without notification. NXP Semiconductors also makes no
; * representation or warranty that such application will be suitable for the
; * specified use without further testing or modification.
; *
; * @par
; * Permission to use, copy, modify, and distribute this software and its
; * documentation is hereby granted, under NXP Semiconductors' and its
; * licensor's relevant copyrights in the software, without fee, provided that it
; * is used in conjunction with NXP Semiconductors microcontrollers.  This
; * copyright, permission, and disclaimer notice must appear in all copies of
; * this code.
; */

; <h> Stack Configuration
;   <o> Stack Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

Stack_Size      EQU     0x00000200

                AREA    STACK, NOINIT, READWRITE, ALIGN=3
Stack_Mem       SPACE   Stack_Size
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

                ; External Interrupts
                DCD     WDT_IRQHandler              ; Watchdog
                DCD     BOD_IRQHandler              ; Brown Out Detect
                DCD     FLASH_IRQHandler            ; Non-Volatile Memory Controller
                DCD     DMA_IRQHandler              ; DMA Controller
                DCD     GINT0_IRQHandler            ; GPIO Group0 Interrupt
                DCD     PIN_INT0_IRQHandler         ; PIO INT0
                DCD     PIN_INT1_IRQHandler         ; PIO INT1
                DCD     PIN_INT2_IRQHandler         ; PIO INT2
                DCD     PIN_INT3_IRQHandler         ; PIO INT3
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
                DCD     ADC0A_IRQHandler            ; ADC0 A sequence (A/D Converter) interrupt
                DCD     ADC0B_IRQHandler            ; ADC0 B sequence (A/D Converter) interrupt
                DCD     ADC_THCMP_OVR_IRQHandler    ; ADC THCMP and OVERRUN ORed
                DCD     RTC_IRQHandler              ; RTC Timer
                DCD     EZH_IRQHandler              ; EZH
                DCD     MBOX_IRQHandler             ; Mailbox 

;//   <h> Code Read Protection level (CRP)
;//     <o>    CRP_Level:
;//                     <0xFFFFFFFF=> Disabled
;//                     <0x4E697370=> NO_ISP
;//                     <0x12345678=> CRP1
;//                     <0x87654321=> CRP2
;//                     <0x43218765=> CRP3 (Are you sure?)
;//   </h>
CRP_Level		EQU		0xFFFFFFFF

				IF		:LNOT::DEF:NO_CRP
				AREA	|.ARM.__at_0x02FC|, CODE, READONLY
CRP_Key			DCD		0xFFFFFFFF
				ENDIF

				AREA	|.text|, CODE, READONLY

cpu_id	 		EQU		0xE000ED00
cpu_ctrl 		EQU		0x40000300
coproc_boot 	EQU		0x40000304
coproc_stack	EQU		0x40000308

rel_vals
				DCD		cpu_id, cpu_ctrl, coproc_boot, coproc_stack
				DCW		0xFFF, 0xC24

; Reset Handler - shared for both cores
Reset_Handler   PROC
                EXPORT	Reset_Handler				[WEAK]
                EXPORT	SystemInit					[WEAK]
                IMPORT	__main

				IF		:LNOT::DEF:SLAVEBOOT
				; Both the M0+ and M4 core come via this shared startup code,
				; but the M0+ and M4 core have different vector tables.
				; Determine if the core executing this code is the master or
				; the slave and handle each core state individually.
shared_boot_entry
				LDR		r6, =rel_vals
				MOVS	r4, #0							; Flag for slave core (0)
				MOVS	r5, #1

				; Determine which core (M0+ or M4) this code is running on
				; r2 = (((*cpu_id) >> 4) & 0xFFF); (M4 core == 0xC24)
get_current_core_id
				LDR		r0, [r6, #0]
				LDR		r1, [r0]						; r1 = CPU ID status
				LSRS	r1, r1, #4						; Right justify 12 CPU ID bits
				LDRH	r2, [r6, #16]					; Mask for CPU ID bits
				ANDS	r2, r1, r2						; r2 = ARM COrtex CPU ID
				LDRH	r3, [r6, #18]					; Mask for CPU ID bits
				CMP		r3, r2							; Core ID matches M4 identifier
				BNE		get_master_status
				MOV		r4, r5							; Set flag for master core (1)

				; Determine if M4 core is the master or slave
				; r3 = ((*cpu_ctrl) & 1); (0 == m0+, 1 == M4)
get_master_status
				LDR		r0, [r6, #4]
				LDR		r3, [r0]			            ; r3 = SYSCON co-processor CPU control status
				ANDS	r3, r3, r5						; r3 = (Bit 0: 1 = M4 is master, 0 = M4 is slave)

				; Select boot based on selected master core and core ID
select_boot
				EORS 	r3, r3, r4						; r4 = (Bit 0: 0 = master, 1 = slave)
				BNE		slave_boot
				B		normal_boot

				; Slave boot
slave_boot
				LDR		r0, [r6, #8]
				LDR		r2, [r0]						; r1 = SYSCON co-processor boot address
				CMP		r2, #0							; Slave boot address = 0 (not set up)?
				BEQ		cpu_sleep
				LDR		r0, [r6, #12]
				LDR		r1, [r0]						; r5 = SYSCON co-processor stack address
				MOV		sp, r1							; Update slave CPU stack pointer
				; Be sure to update VTOR for the slave MCU to point to the
				; slave vector table in boot memory
				BX		r2								; Jump to slave boot address

				; Slave isn't yet setup for system boot from the master
				; so sleep until the master sets it up and then reboots it
cpu_sleep
				MOV		sp, r5							; Will force exception if something happens
cpu_sleep_wfi
				WFI										; Sleep forever until master reboots
				B		cpu_sleep_wfi
				ENDIF

				; Normal boot for master/slave
normal_boot
				LDR		r0, =SystemInit
				BLX		r0
				LDR		r0, =__main
				BX		r0
				ENDP

; For cores with SystemInit() or __main(), the code will sleep the MCU
SystemInit		PROC
                EXPORT  SystemInit					[WEAK]
				BX		lr
				ENDP

; Dummy Exception Handlers (infinite loops which can be modified)                
NMI_Handler     PROC
                EXPORT  NMI_Handler               [WEAK]
                B       .
                ENDP
HardFault_Handler\
                PROC
                EXPORT  HardFault_Handler         [WEAK]
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
                EXPORT  PIN_INT0_IRQHandler       [WEAK]
                EXPORT  PIN_INT1_IRQHandler       [WEAK]
                EXPORT  PIN_INT2_IRQHandler       [WEAK]
                EXPORT  PIN_INT3_IRQHandler       [WEAK]				
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
                EXPORT  ADC0A_IRQHandler          [WEAK]
                EXPORT  ADC0B_IRQHandler          [WEAK]
                EXPORT  ADC_THCMP_OVR_IRQHandler  [WEAK]
                EXPORT  RTC_IRQHandler            [WEAK]
                EXPORT  EZH_IRQHandler            [WEAK]
                EXPORT  MBOX_IRQHandler           [WEAK]
				
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
ADC0A_IRQHandler
ADC0B_IRQHandler
ADC_THCMP_OVR_IRQHandler
RTC_IRQHandler
EZH_IRQHandler
MBOX_IRQHandler

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
__user_initial_stackheap

                LDR     R0, =  Heap_Mem
                LDR     R1, =(Stack_Mem + Stack_Size)
                LDR     R2, = (Heap_Mem +  Heap_Size)
                LDR     R3, = Stack_Mem
                BX      LR

                ALIGN

                ENDIF


                END
