/*
 * @brief Frequency Measurement example
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
volatile uint32_t freqMeasure = 0;
volatile uint32_t Frequency = 0;

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
 * @brief	main routine for blinky example
 * @return	Function should not exit.
 */
int main(void)
{
	SystemCoreClockUpdate();
	Board_Init();

	/* Enable INMUX and IOCON clock */
  Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_MUX);
  Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_IOCON);

	/* Configure PIN0.21 as CLKOUT with pull-up, monitor the MAINCLK on scope */
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 21, IOCON_MODE_PULLUP | IOCON_FUNC1 | IOCON_DIGITAL_EN | IOCON_INPFILT_OFF);
	Chip_Clock_SetCLKOUTSource(SYSCTL_CLKOUTSRC_MAINSYSCLK, 1);

  Chip_INMUX_SetFreqMeasRefClock(FREQ_MEAS_MAIN_CLK);
  Chip_INMUX_SetFreqMeasTargClock(FREQMSR_IRC);

  Chip_SYSCTL_SetFreqMeasure( SYSCTL_FREMEA_ENABLE );
	/* Self cleared once the measurement is done. */
  while ( (freqMeasure = Chip_SYSCTL_GetFreqMeasure()) & SYSCTL_FREMEA_ENABLE );
  freqMeasure &= 0x3FFF;

  Frequency = (freqMeasure -1) * (SystemCoreClock/16384);  /* in MHz, Divider of 2^14 */
	DEBUGOUT("Measured frequency of IRC = %dHz\r\n", Frequency);
	while (1) {
		__WFI();
	}
	return 0;
}
