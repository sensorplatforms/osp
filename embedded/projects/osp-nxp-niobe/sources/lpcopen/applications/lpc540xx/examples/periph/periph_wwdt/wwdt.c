/*
 * @brief Windowed Watchdog Timer (WWDT) example
 *
 * @note
 * Copyright(C) NXP Semiconductors, 2014
 * All rights reserved.
 *
 * @par
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * LPC products.  This software is supplied "AS IS" without any warranties of
 * any kind, and NXP Semiconductors and its licensor disclaim any and
 * all warranties, express or implied, including all implied warranties of
 * merchantability, fitness for a particular purpose and non-infringement of
 * intellectual property rights.  NXP Semiconductors assumes no responsibility
 * or liability for the use of the software, conveys no license or rights under any
 * patent, copyright, mask work right, or any other intellectual property rights in
 * or to any products. NXP Semiconductors reserves the right to make changes
 * in the software without notification. NXP Semiconductors also makes no
 * representation or warranty that such application will be suitable for the
 * specified use without further testing or modification.
 *
 * @par
 * Permission to use, copy, modify, and distribute this software and its
 * documentation is hereby granted, under NXP Semiconductors' and its
 * licensor's relevant copyrights in the software, without fee, provided that it
 * is used in conjunction with NXP Semiconductors microcontrollers.  This
 * copyright, permission, and disclaimer notice must appear in all copies of
 * this code.
 */

#include "board.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

static volatile uint8_t wdt_update_count;

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/*****************************************************************************
 * Public functions
 ****************************************************************************/
/**
 * @brief	watchdog timer Interrupt Handler
 * @return	Nothing
 * @note	Handles watchdog timer warning and timeout events
 */
void WDT_IRQHandler(void)
{
	uint32_t wdtStatus = Chip_WWDT_GetStatus(LPC_WWDT);

	Board_LED_Toggle(0);

	/* The chip will reset before this happens */
	if (wdtStatus & WWDT_WDMOD_WDTOF) {
		/* A watchdog feed didn't occur prior to window timeout */
		Chip_WWDT_UnsetOption(LPC_WWDT, WWDT_WDMOD_WDEN);	/* Stop WDT */
		Chip_WWDT_ClearStatusFlag(LPC_WWDT, WWDT_WDMOD_WDTOF);
		Chip_WWDT_Start(LPC_WWDT);	/* Needs restart */
	}

	/* Handle warning interrupt */
	if (wdtStatus & WWDT_WDMOD_WDINT) {
		/* A watchdog feed didn't occur prior to warning timeout */
		Chip_WWDT_ClearStatusFlag(LPC_WWDT, WWDT_WDMOD_WDINT);
		if (wdt_update_count < 5) {
			/* Feed only for the first 5 warnings then allow for a WDT reset to occur */
			Chip_WWDT_Feed(LPC_WWDT);
			wdt_update_count++;
		}
	}
}

/**
 * @brief	Application Main entry point
 * @return	Nothing (This will not return)
 */
int main(void)
{
	uint32_t wdtFreq;

	SystemCoreClockUpdate();
	Board_Init();

	/* Enable the power to the WDT Oscillator */
	Chip_SYSCTL_PowerUp(PDRUNCFG_PD_WDT_OSC);

	/* The WDT divides the input frequency into it by 4 */
	wdtFreq = Chip_Clock_GetWDTOSCRate() / 4;

	/* Check if reset is due to Watchdog */
	if (Chip_WWDT_GetStatus(LPC_WWDT) & WWDT_WDMOD_WDTOF) {
		DEBUGOUT("Watchdog reset occurred\r\n");
	}

	/* Initialize WWDT (also enables WWDT clock) */
	Chip_WWDT_Init(LPC_WWDT);

	/* Set watchdog feed time constant to approximately 2s
	   Set watchdog warning time to 512 ticks after feed time constant
	   Set watchdog window time to 1s */
	Chip_WWDT_SetTimeOut(LPC_WWDT, wdtFreq * 2);
	Chip_WWDT_SetWarning(LPC_WWDT, 512);
	Chip_WWDT_SetWindow(LPC_WWDT, wdtFreq * 1);

	/* Configure WWDT to reset on timeout */
	Chip_WWDT_SetOption(LPC_WWDT, WWDT_WDMOD_WDRESET);

	/* Clear watchdog warning and timeout interrupts */
	Chip_WWDT_ClearStatusFlag(LPC_WWDT, WWDT_WDMOD_WDTOF | WWDT_WDMOD_WDINT);

	/* Allow WDT to wake from deep sleep. */
	Chip_SYSCTL_EnableWakeup(STARTERP0_WWDT);

	/* Clear and enable watchdog interrupt */
	NVIC_ClearPendingIRQ(WWDT_IRQn);
	NVIC_EnableIRQ(WWDT_IRQn);

	/* Start watchdog */
	Chip_WWDT_Start(LPC_WWDT);

	while (1) {
		// FIXME - Use ROM API's to put chip to power down modes
		__WFI();
	}
	return 0;
}
