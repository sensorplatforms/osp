/*
 * @brief Brown-out detector example
 *
 * @note
 * Copyright(C) NXP Semiconductors, 2012
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

#if defined (BOARD_NXP_LPCXPRESSO_54000)
#define GPIO_GOSLEEP_PORT							0
#define GPIO_GOSLEEP_PIN							4

#define GPIO_BOD_RESET_LED_PORT				0
#define GPIO_BOD_RESET_LED_PIN				5

#define GPIO_POR_RESET_LED_PORT				0
#define GPIO_POR_RESET_LED_PIN				6

#define GPIO_BOD_INTERRUPT_LED_PORT		0
#define GPIO_BOD_INTERRUPT_LED_PIN		7

#define DEEPSLEEP_BOD_WAKEUP					0
#endif

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

void BOD_IRQHandler(void)
{
	if ( Chip_POWER_GetBODControl() & POWER_BOD_INT ) {
		if ( Chip_SYSCTL_GetSystemRSTStatus() & SYSCTL_RST_BOD ) {
			NVIC_DisableIRQ(BOD_IRQn);
			return;
		}

		Chip_POWER_SetBODControl(POWER_BOD_INT);
		Chip_GPIO_SetPinState(LPC_GPIO, GPIO_BOD_RESET_LED_PORT, GPIO_BOD_RESET_LED_PIN, 0);
		Chip_GPIO_SetPinState(LPC_GPIO, GPIO_POR_RESET_LED_PORT, GPIO_POR_RESET_LED_PIN, 0);
		Chip_GPIO_SetPinState(LPC_GPIO, GPIO_BOD_INTERRUPT_LED_PORT, GPIO_BOD_INTERRUPT_LED_PIN, 0);
	}
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/**
 * @brief	main routine for bown-out example
 * @return	Function should not exit.
 */
int main(void)
{
#if DEEPSLEEP_BOD_WAKEUP
	uint32_t saved_clksrc;
#endif
	uint32_t i;

	/* Generic Initialization */
	SystemCoreClockUpdate();
	Board_Init();

	/* Configure GPIO pin as input pin */
#if DEEPSLEEP_BOD_WAKEUP
	Chip_GPIO_SetPinDIRInput(LPC_GPIO, 0, 4);
#endif
	/* Configure GPIO pin as out pin */
	Chip_GPIO_SetPinState(LPC_GPIO, GPIO_BOD_RESET_LED_PORT, GPIO_BOD_RESET_LED_PIN, 0);
	Chip_GPIO_SetPinState(LPC_GPIO, GPIO_POR_RESET_LED_PORT, GPIO_POR_RESET_LED_PIN, 0);
	Chip_GPIO_SetPinState(LPC_GPIO, GPIO_BOD_INTERRUPT_LED_PORT, GPIO_BOD_INTERRUPT_LED_PIN, 1);
	Chip_GPIO_SetPinDIROutput(LPC_GPIO, GPIO_BOD_RESET_LED_PORT, GPIO_BOD_RESET_LED_PIN);
	Chip_GPIO_SetPinDIROutput(LPC_GPIO, GPIO_POR_RESET_LED_PORT, GPIO_POR_RESET_LED_PIN);
	Chip_GPIO_SetPinDIROutput(LPC_GPIO, GPIO_BOD_INTERRUPT_LED_PORT, GPIO_BOD_INTERRUPT_LED_PIN);

	/* Power up BOD related. At POR, BOD is NOT powered up according to the design team. */
	if ( Is_Chip_SYSCTL_PowerUp(PDRUNCFG_PD_BOD_INTR) )
		Chip_SYSCTL_PowerUp(PDRUNCFG_PD_BOD_INTR);
	if ( Is_Chip_SYSCTL_PowerUp(PDRUNCFG_PD_BOD_RESET) )
		Chip_SYSCTL_PowerUp(PDRUNCFG_PD_BOD_RESET);

	/* Set brown-out interrupt level with reset  */
	Chip_POWER_SetBODLevels(POWER_BODRSTLVL_3, POWER_BODINTVAL_LVL3);
	Chip_POWER_SetBODControl(POWER_BOD_INT);
	Chip_POWER_SetBODControl(POWER_BOD_RST);
	Chip_POWER_EnableBODInterrupt();
	Chip_POWER_EnableBODReset();

	/* Enable BOD interrupt */
	NVIC_DisableIRQ(BOD_IRQn);
	NVIC_ClearPendingIRQ(BOD_IRQn);
	NVIC_EnableIRQ(BOD_IRQn);

	/* If the board was reset due to a BOD event, the reset can be
	   detected here. If the board was completely powered off, the BOD
	   reset event won't be active. */
	if ( Chip_SYSCTL_GetSystemRSTStatus() & SYSCTL_RST_BOD ) {
		Chip_SYSCTL_ClearSystemRSTStatus(SYSCTL_RST_BOD);
		/* Wait forever */
		while (1) {
			if ( Chip_GPIO_ReadPortBit(LPC_GPIO, GPIO_BOD_INTERRUPT_LED_PORT, GPIO_BOD_INTERRUPT_LED_PIN) == 0x0 ) {
				Chip_GPIO_SetPinState(LPC_GPIO, GPIO_BOD_INTERRUPT_LED_PORT, GPIO_BOD_INTERRUPT_LED_PIN, 1);
			}
			for (i = 0; i < 0x100000; i++);
			Chip_GPIO_SetPinState(LPC_GPIO, GPIO_BOD_RESET_LED_PORT, GPIO_BOD_RESET_LED_PIN, 1);
			for (i = 0; i < 0x100000; i++);
			Chip_GPIO_SetPinState(LPC_GPIO, GPIO_BOD_RESET_LED_PORT, GPIO_BOD_RESET_LED_PIN, 0);
		}
	} else {
		Chip_SYSCTL_ClearSystemRSTStatus(SYSCTL_RST_POR | SYSCTL_RST_EXTRST);
#if DEEPSLEEP_BOD_WAKEUP
			Chip_SYSCTL_EnableWakeup(STARTERP0_BOD);
			while (  Chip_GPIO_ReadPortBit(LPC_GPIO, GPIO_GOSLEEP_PORT, GPIO_GOSLEEP_PIN) == 0x1 );
#if 0
		if ( (saved_clksrc = Chip_Clock_GetMainClockSource()) == SYSCTL_MAINCLKSRC_PLLOUT ) {
			Chip_Clock_SetMainClockSource(SYSCTL_MAINCLKSRC_IRC);
			Chip_SYSCTL_PowerDown(PDRUNCFG_PD_SYS_PLL0);
		}
		LPC_PWRD_API->power_mode_configure( PMU_DEEP_SLEEP, (PDRUNCFG_PD_BOD_RESET | PDRUNCFG_PD_BOD_INTR), 1 );
		if ( saved_clksrc == SYSCTL_MAINCLKSRC_PLLOUT ) {
			Board_SystemInit();
		}
#endif

#if 1
		if ( (saved_clksrc = Chip_Clock_GetMainClockSource()) == SYSCTL_MAINCLKSRC_PLLOUT ) {
			Chip_Clock_SetMainClockSource(SYSCTL_MAINCLKSRC_IRC);
			Chip_SYSCTL_PowerDown(PDRUNCFG_PD_SYS_PLL0);
		}
		LPC_PWRD_API->power_mode_configure( PMU_POWERDOWN, (PDRUNCFG_PD_BOD_RESET | PDRUNCFG_PD_BOD_INTR), 1 );
		if ( saved_clksrc == SYSCTL_MAINCLKSRC_PLLOUT ) {
			Board_SystemInit();
		}
#endif
#endif
		/* Wait forever */
		while (1) {
			if ( Chip_GPIO_ReadPortBit(LPC_GPIO, GPIO_BOD_INTERRUPT_LED_PORT, GPIO_BOD_INTERRUPT_LED_PIN) == 0x0 ) {
				Chip_GPIO_SetPinState(LPC_GPIO, GPIO_BOD_INTERRUPT_LED_PORT, GPIO_BOD_INTERRUPT_LED_PIN, 1);
			}
			for (i = 0; i < 0x100000; i++);
			Chip_GPIO_SetPinState(LPC_GPIO, GPIO_POR_RESET_LED_PORT, GPIO_POR_RESET_LED_PIN, 1);
			for (i = 0; i < 0x100000; i++);
			Chip_GPIO_SetPinState(LPC_GPIO, GPIO_POR_RESET_LED_PORT, GPIO_POR_RESET_LED_PIN, 0);
		}
	}
	return 0;
}
