/*
 * @brief OS Timer example
 *
 * @note
 * Copyright(C) NXP Semiconductors, 2013
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

#define TICKRATE_HZ (SystemCoreClock/10 - 1)	/* 10 ticks per second */

/* SystemTick Counter */
static volatile uint32_t ostimerTick;

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
 * @brief	Handle interrupt from SysTick timer
 * @return	Nothing
 */
void OSTIMER_IRQHandler(void)
{
	Chip_OSTIMER_ClearInt(LPC_OSTIMER);
	Board_LED_Toggle(0);
	ostimerTick++;
}

/**
 * @brief	main routine for OS timer example
 * @return	Function should not exit.
 */
int main(void)
{
	SystemCoreClockUpdate();
	Board_Init();

	Board_LED_Set(0, false);

	Chip_OSTIMER_Init();

	Chip_OSTIMER_Disable(LPC_OSTIMER, 1, 0);
	/* mask low and high are zero. */
	Chip_OSTIMER_SetMask(LPC_OSTIMER, 0, 0);
	/* COMP low is zero. */
	Chip_OSTIMER_SetCOMP(LPC_OSTIMER, TICKRATE_HZ, 0);
	Chip_OSTIMER_ClearInt(LPC_OSTIMER);

  NVIC_DisableIRQ(OSTIMER_IRQn);
  NVIC_ClearPendingIRQ(OSTIMER_IRQn);
	NVIC_EnableIRQ(OSTIMER_IRQn);

	/* Disable debugger, enabled clear on start. */
	Chip_OSTIMER_Enable(LPC_OSTIMER, 1, 0);

	while (1) {
		__WFI();
	}
	return 0;
}
