/*
 * @brief LED blinking example using timers
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
#include <stdio.h>

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

#define TICKRATE_HZ1 (1)/* 1 ticks per second */
#define TICKRATE_HZ2 (2)/* 2 ticks per second */
#define PRESCALE_HZ2 (0xFFFF)	/* 16-bit prescale count */

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
 * @brief	Handle interrupt from Timer 0
 * @return	Nothing
 */
void CTIMER0_IRQHandler(void)
{
	if (Chip_TIMER_MatchPending(LPC_CTIMER0, 1)) {
		Chip_TIMER_ClearMatch(LPC_CTIMER0, 1);
		Board_LED_Toggle(0);
	}
}

/**
 * @brief	Handle interrupt from Timer 1
 * @return	Nothing
 */
void CTIMER1_IRQHandler(void)
{
	if (Chip_TIMER_MatchPending(LPC_CTIMER1, 1)) {
		Chip_TIMER_ClearMatch(LPC_CTIMER1, 1);
		Board_LED_Toggle(1);
	}
}

/**
 * @brief	main routine for timer example
 * @return	Function should not exit.
 */
int main(void)
{
	SystemCoreClockUpdate();
	Board_Init();
	Board_LED_Set(0, false);
	Board_LED_Set(1, false);

	/* Initialize Timer 0 and Timer 1 */
	Chip_TIMER_Init(LPC_CTIMER0);
	Chip_TIMER_Init(LPC_CTIMER1);

	/* Setup prescale value on Timer 0 to PCLK */
	Chip_TIMER_PrescaleSet(LPC_CTIMER0, 0);
	/* Setup prescale value on Timer 1 for lower resolution */
	Chip_TIMER_PrescaleSet(LPC_CTIMER1, PRESCALE_HZ2);

	/* Reset timers */
	Chip_TIMER_Reset(LPC_CTIMER0);
	Chip_TIMER_Reset(LPC_CTIMER1);

	/* Enable both timers to generate interrupts when time matches */
	Chip_TIMER_MatchEnableInt(LPC_CTIMER0, 1);
	Chip_TIMER_MatchEnableInt(LPC_CTIMER1, 1);

	/* Setup Timer 0 for a match every 1s */
	Chip_TIMER_SetMatch(LPC_CTIMER0, 1, (SystemCoreClock / TICKRATE_HZ1));

	/* Setup Timer 1 for a match twice in a second */
	Chip_TIMER_SetMatch(LPC_CTIMER1, 1, (SystemCoreClock / ((PRESCALE_HZ2 + 1) * TICKRATE_HZ2)) );

	/* Setup both timers to restart when match occurs */
	Chip_TIMER_ResetOnMatchEnable(LPC_CTIMER0, 1);
	Chip_TIMER_ResetOnMatchEnable(LPC_CTIMER1, 1);

	/* Start both timers */
	Chip_TIMER_Enable(LPC_CTIMER0);
	Chip_TIMER_Enable(LPC_CTIMER1);

	/* Clear both timers of any pending interrupts */
	NVIC_ClearPendingIRQ(CTIMER0_IRQn);
	NVIC_ClearPendingIRQ(CTIMER1_IRQn);

	/* Enable both timer interrupts */
	NVIC_EnableIRQ(CTIMER0_IRQn);
	NVIC_EnableIRQ(CTIMER1_IRQn);

	/* Wait for timers to generate interrupts (LEDs toggle in interrupt handlers) */
	while (1) {
		__WFI();
	}

	return 0;
}
