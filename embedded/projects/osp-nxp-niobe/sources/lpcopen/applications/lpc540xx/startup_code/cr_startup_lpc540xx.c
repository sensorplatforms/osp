//*****************************************************************************
// LPC54xxx Microcontroller Startup code for use with LPCXpresso IDE
//
// Version : 140121
//*****************************************************************************
//
// Copyright(C) NXP Semiconductors, 2014
// All rights reserved.
//
// Software that is described herein is for illustrative purposes only
// which provides customers with programming information regarding the
// LPC products.  This software is supplied "AS IS" without any warranties of
// any kind, and NXP Semiconductors and its licensor disclaim any and
// all warranties, express or implied, including all implied warranties of
// merchantability, fitness for a particular purpose and non-infringement of
// intellectual property rights.  NXP Semiconductors assumes no responsibility
// or liability for the use of the software, conveys no license or rights under any
// patent, copyright, mask work right, or any other intellectual property rights in
// or to any products. NXP Semiconductors reserves the right to make changes
// in the software without notification. NXP Semiconductors also makes no
// representation or warranty that such application will be suitable for the
// specified use without further testing or modification.
//
// Permission to use, copy, modify, and distribute this software and its
// documentation is hereby granted, under NXP Semiconductors' and its
// licensor's relevant copyrights in the software, without fee, provided that it
// is used in conjunction with NXP Semiconductors microcontrollers.  This
// copyright, permission, and disclaimer notice must appear in all copies of
// this code.
//*****************************************************************************

#if defined (__cplusplus)
#ifdef __REDLIB__
#error Redlib does not support C++
#else
//*****************************************************************************
//
// The entry point for the C++ library startup
//
//*****************************************************************************
extern "C" {
    extern void __libc_init_array(void);
}
#endif
#endif

#define WEAK __attribute__ ((weak))
#define ALIAS(f) __attribute__ ((weak, alias (#f)))

//*****************************************************************************
#if defined (__cplusplus)
extern "C" {
#endif

//*****************************************************************************
#if defined (__USE_CMSIS) || defined (__USE_LPCOPEN)
// Declaration of external SystemInit function
extern void SystemInit(void);
#endif

//*****************************************************************************
//
// Forward declaration of the default handlers. These are aliased.
// When the application defines a handler (with the same name), this will
// automatically take precedence over these weak definitions
//
//*****************************************************************************
void ResetISR(void);
WEAK void NMI_Handler(void);
WEAK void HardFault_Handler(void);
WEAK void MemManage_Handler(void);
WEAK void BusFault_Handler(void);
WEAK void UsageFault_Handler(void);
WEAK void SVC_Handler(void);
WEAK void DebugMon_Handler(void);
WEAK void PendSV_Handler(void);
WEAK void SysTick_Handler(void);
WEAK void IntDefaultHandler(void);

//*****************************************************************************
//
// Forward declaration of the specific IRQ handlers. These are aliased
// to the IntDefaultHandler, which is a 'forever' loop. When the application
// defines a handler (with the same name), this will automatically take
// precedence over these weak definitions
//
//*****************************************************************************
void WDT_IRQHandler(void) ALIAS(IntDefaultHandler);
void BOD_IRQHandler(void) ALIAS(IntDefaultHandler);
void FLASH_IRQHandler(void) ALIAS(IntDefaultHandler);
void DMA_IRQHandler(void) ALIAS(IntDefaultHandler);
void GINT0_IRQHandler(void) ALIAS(IntDefaultHandler);
void PIN_INT0_IRQHandler(void) ALIAS(IntDefaultHandler);
void PIN_INT1_IRQHandler(void) ALIAS(IntDefaultHandler);
void PIN_INT2_IRQHandler(void) ALIAS(IntDefaultHandler);
void PIN_INT3_IRQHandler(void) ALIAS(IntDefaultHandler);
void UTICK_IRQHandler(void) ALIAS(IntDefaultHandler);
void MRT_IRQHandler(void) ALIAS(IntDefaultHandler);
void CTIMER0_IRQHandler(void) ALIAS(IntDefaultHandler);
void CTIMER1_IRQHandler(void) ALIAS(IntDefaultHandler);
void CTIMER2_IRQHandler(void) ALIAS(IntDefaultHandler);
void CTIMER3_IRQHandler(void) ALIAS(IntDefaultHandler);
void CTIMER4_IRQHandler(void) ALIAS(IntDefaultHandler);
void SCT_IRQHandler(void) ALIAS(IntDefaultHandler);
void UART0_IRQHandler(void) ALIAS(IntDefaultHandler);
void UART1_IRQHandler(void) ALIAS(IntDefaultHandler);
void UART2_IRQHandler(void) ALIAS(IntDefaultHandler);
void UART3_IRQHandler(void) ALIAS(IntDefaultHandler);
void I2C0_IRQHandler(void) ALIAS(IntDefaultHandler);
void I2C1_IRQHandler(void) ALIAS(IntDefaultHandler);
void I2C2_IRQHandler(void) ALIAS(IntDefaultHandler);
void SPI0_IRQHandler(void) ALIAS(IntDefaultHandler);
void SPI1_IRQHandler(void) ALIAS(IntDefaultHandler);
void ADC0A_IRQHandler(void) ALIAS(IntDefaultHandler);
void ADC0B_IRQHandler(void) ALIAS(IntDefaultHandler);
void ADC_THCMP_OVR_IRQHandler(void) ALIAS(IntDefaultHandler);
void RTC_IRQHandler(void) ALIAS(IntDefaultHandler);
void EZH_IRQHandler(void) ALIAS(IntDefaultHandler);
void MBOX_IRQHandler(void) ALIAS(IntDefaultHandler);
// For M4 only
void GINT1_IRQHandler(void) ALIAS(IntDefaultHandler);
void PIN_INT4_IRQHandler(void) ALIAS(IntDefaultHandler);
void PIN_INT5_IRQHandler(void) ALIAS(IntDefaultHandler);
void PIN_INT6_IRQHandler(void) ALIAS(IntDefaultHandler);
void PIN_INT7_IRQHandler(void) ALIAS(IntDefaultHandler);
void SPI2_IRQHandler(void) ALIAS(IntDefaultHandler);
void SPI3_IRQHandler(void) ALIAS(IntDefaultHandler);
void OSTIMER_IRQHandler(void) ALIAS(IntDefaultHandler);
void PVT0_A_IRQHandler(void) ALIAS(IntDefaultHandler);
void PVT0_R_IRQHandler(void) ALIAS(IntDefaultHandler);
void PVT1_A_IRQHandler(void) ALIAS(IntDefaultHandler);
void PVT1_R_IRQHandler(void) ALIAS(IntDefaultHandler);

//*****************************************************************************
//
// The entry point for the application.
// __main() is the entry point for Redlib based applications
// main() is the entry point for Newlib based applications
//
//*****************************************************************************
#if defined (__REDLIB__)
extern void __main(void);
#endif
extern int main(void);
//*****************************************************************************
//
// External declaration for the pointer to the stack top from the Linker Script
//
//*****************************************************************************
extern void _vStackTop(void);

//*****************************************************************************
#if defined (__cplusplus)
} // extern "C"
#endif
//*****************************************************************************
//
// The vector table.
// This relies on the linker script to place at correct location in memory.
//
//*****************************************************************************
extern void (* const g_pfnVectors[])(void);
__attribute__ ((section(".isr_vector")))
void (* const g_pfnVectors[])(void) = {
    // Core Level - CM3
    &_vStackTop,                       // The initial stack pointer
    ResetISR,                          // The reset handler
    NMI_Handler,                       // The NMI handler
    HardFault_Handler,                 // The hard fault handler
    MemManage_Handler,                 // The MPU fault handler
    BusFault_Handler,                  // The bus fault handler
    UsageFault_Handler,                // The usage fault handler
    0,                                 // Reserved
    0,                                 // Reserved
    0,                                 // Reserved
    0,                                 // Reserved
    SVC_Handler,                       // SVCall handler
    DebugMon_Handler,                  // Debug monitor handler
    0,                                 // Reserved
    PendSV_Handler,                    // The PendSV handler
    SysTick_Handler,                   // The SysTick handler

    // External Interrupts - Available on M0/M4
    WDT_IRQHandler,                    // Watchdog
    BOD_IRQHandler,                    // Brown Out Detect
    FLASH_IRQHandler,                  // Non-Volatile Memory Controller
    DMA_IRQHandler,                    // DMA Controller
    GINT0_IRQHandler,                  // GPIO Group0 Interrupt
    PIN_INT0_IRQHandler,               // PIO INT0
    PIN_INT1_IRQHandler,                // PIO INT1
    PIN_INT2_IRQHandler,                // PIO INT2
    PIN_INT3_IRQHandler,                // PIO INT3
    UTICK_IRQHandler,                  // UTICK timer
    MRT_IRQHandler,                    // Multi-Rate Timer
    CTIMER0_IRQHandler,                // CTIMER0
    CTIMER1_IRQHandler,                // CTIMER1
    CTIMER2_IRQHandler,                // CTIMER2
    CTIMER3_IRQHandler,                // CTIMER3
    CTIMER4_IRQHandler,                // CTIMER4
    SCT_IRQHandler,                    // Smart Counter Timer
    UART0_IRQHandler,                  // UART0
    UART1_IRQHandler,                  // UART1
    UART2_IRQHandler,                  // UART2
    UART3_IRQHandler,                  // UART3
    I2C0_IRQHandler,                   // I2C0 controller
    I2C1_IRQHandler,                   // I2C1 controller
    I2C2_IRQHandler,                   // I2C2 controller
    SPI0_IRQHandler,                   // SPI0 controller
    SPI1_IRQHandler,                   // SPI1 controller
    ADC0A_IRQHandler,                  // ADC SEQA
    ADC0B_IRQHandler   ,               // ADC SEQB
    ADC_THCMP_OVR_IRQHandler,          // ADC THCMP and OVERRUN ORed
    RTC_IRQHandler,                    // RTC Timer
    EZH_IRQHandler,                    // EZH
    MBOX_IRQHandler,                   // Mailbox

    // External Interrupts - For M4 only
    GINT1_IRQHandler,                  // GPIO Group1 Interrupt
    PIN_INT4_IRQHandler,               // PIO INT4
    PIN_INT5_IRQHandler,               // PIO INT5
    PIN_INT6_IRQHandler,               // PIO INT6
    PIN_INT7_IRQHandler,               // PIO INT7
    SPI2_IRQHandler,                   // SPI2 controller
    SPI3_IRQHandler,                   // SPI3 controller
    0,                                 // Reserved
    OSTIMER_IRQHandler,                // OS Timer
    PVT0_A_IRQHandler,                 // PVT0 Amber
    PVT0_R_IRQHandler,                 // PVT0 Red
    PVT1_A_IRQHandler,                 // PVT1 Amber
    PVT1_R_IRQHandler,                 // PVT1 Red

}; /* End of g_pfnVectors */

//*****************************************************************************
// Functions to carry out the initialization of RW and BSS data sections. These
// are written as separate functions rather than being inlined within the
// ResetISR() function in order to cope with MCUs with multiple banks of
// memory.
//*****************************************************************************
__attribute__ ((section(".after_vectors")))
void data_init(unsigned int romstart, unsigned int start, unsigned int len) {
    unsigned int *pulDest = (unsigned int*) start;
    unsigned int *pulSrc = (unsigned int*) romstart;
    unsigned int loop;
    for (loop = 0; loop < len; loop = loop + 4)
        *pulDest++ = *pulSrc++;
}

__attribute__ ((section(".after_vectors")))
void bss_init(unsigned int start, unsigned int len) {
    unsigned int *pulDest = (unsigned int*) start;
    unsigned int loop;
    for (loop = 0; loop < len; loop = loop + 4)
        *pulDest++ = 0;
}

//*****************************************************************************
// The following symbols are constructs generated by the linker, indicating
// the location of various points in the "Global Section Table". This table is
// created by the linker via the Code Red managed linker script mechanism. It
// contains the load address, execution address and length of each RW data
// section and the execution and length of each BSS (zero initialized) section.
//*****************************************************************************
extern unsigned int __data_section_table;
extern unsigned int __data_section_table_end;
extern unsigned int __bss_section_table;
extern unsigned int __bss_section_table_end;

//*****************************************************************************
// Reset entry point for your code.
// Sets up a simple runtime environment and initializes the C/C++
// library.
//*****************************************************************************
__attribute__ ((section(".after_vectors")))
void
ResetISR(void) {
    //
    // Copy the data sections from flash to SRAM.
    //
    unsigned int LoadAddr, ExeAddr, SectionLen;
    unsigned int *SectionTableAddr;

    // Load base address of Global Section Table
    SectionTableAddr = &__data_section_table;

    // Copy the data sections from flash to SRAM.
    while (SectionTableAddr < &__data_section_table_end) {
        LoadAddr = *SectionTableAddr++;
        ExeAddr = *SectionTableAddr++;
        SectionLen = *SectionTableAddr++;
        data_init(LoadAddr, ExeAddr, SectionLen);
    }
    // At this point, SectionTableAddr = &__bss_section_table;
    // Zero fill the bss segment
    while (SectionTableAddr < &__bss_section_table_end) {
        ExeAddr = *SectionTableAddr++;
        SectionLen = *SectionTableAddr++;
        bss_init(ExeAddr, SectionLen);
    }
    
#if !defined (__USE_LPCOPEN)
// LPCOpen init code deals with FP and VTOR initialisation
#if defined (__VFP_FP__) && !defined (__SOFTFP__)
    /*
     * Code to enable the Cortex-M4 FPU only included
     * if appropriate build options have been selected.
     * Code taken from Section 7.1, Cortex-M4 TRM (DDI0439C)
     */
    // CPACR is located at address 0xE000ED88
    asm("LDR.W R0, =0xE000ED88");
    // Read CPACR
    asm("LDR R1, [R0]");
    // Set bits 20-23 to enable CP10 and CP11 coprocessors
    asm(" ORR R1, R1, #(0xF << 20)");
    // Write back the modified value to the CPACR
    asm("STR R1, [R0]");
#endif // (__VFP_FP__) && !(__SOFTFP__)
    // ******************************
    // Check to see if we are running the code from a non-zero
    // address (eg RAM, external flash), in which case we need
    // to modify the VTOR register to tell the CPU that the
    // vector table is located at a non-0x0 address.

    // Note that we do not use the CMSIS register access mechanism,
    // as there is no guarantee that the project has been configured
    // to use CMSIS.
    unsigned int * pSCB_VTOR = (unsigned int *) 0xE000ED08;
    if ((unsigned int *) g_pfnVectors != (unsigned int *) 0x00000000) {
        // CMSIS : SCB->VTOR = <address of vector table>
        *pSCB_VTOR = (unsigned int) g_pfnVectors;
    }
#endif
#if defined (__USE_CMSIS) || defined (__USE_LPCOPEN)
    SystemInit();
#endif

#if defined (__cplusplus)
    //
    // Call C++ library initialisation
    //
    __libc_init_array();
#endif

#if defined (__REDLIB__)
    // Call the Redlib library, which in turn calls main()
    __main() ;
#else
    main();
#endif

    //
    // main() shouldn't return, but if it does, we'll just enter an infinite loop
    //
    while (1) {
        ;
    }
}

//*****************************************************************************
// Default exception handlers. Override the ones here by defining your own
// handler routines in your application code.
//*****************************************************************************
__attribute__ ((section(".after_vectors")))
void NMI_Handler(void)
{    while(1) {}
}

__attribute__ ((section(".after_vectors")))
void HardFault_Handler(void)
{    while(1) {}
}

__attribute__ ((section(".after_vectors")))
void SVC_Handler(void)
{    while(1) {}
}

__attribute__ ((section(".after_vectors")))
void PendSV_Handler(void)
{    while(1) {}
}

__attribute__ ((section(".after_vectors")))
void SysTick_Handler(void)
{    while(1) {}
}

//*****************************************************************************
//
// Processor ends up here if an unexpected interrupt occurs or a specific
// handler is not present in the application code.
//
//*****************************************************************************
__attribute__ ((section(".after_vectors")))
void IntDefaultHandler(void)
{    while(1) {}
}


