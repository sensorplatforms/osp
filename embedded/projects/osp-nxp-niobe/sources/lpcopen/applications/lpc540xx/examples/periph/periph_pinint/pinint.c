/*
 * @brief Pin Interrupt example
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

#if defined(BOARD_NXP_LPCXPRESSO_54000)
/* GPIO pin for PININT interrupt */
#define GPIO_PININT_PIN     24	/* GPIO pin number mapped to PININT */
#define GPIO_PININT_PORT    0	/* GPIO port number mapped to PININT */
#define GPIO_PININT_INDEX   PININTSELECT0	/* PININT index used for GPIO mapping */
#define PININT_IRQ_HANDLER  PININT0_IRQHandler	/* GPIO interrupt IRQ function name */
#define PININT_NVIC_NAME    PININT0_IRQn	/* GPIO interrupt NVIC interrupt name */

#else
#error "PININT not configured for this example"
#endif /* defined (BOARD_NXP_LPCXPRESSO_54000) */

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
 * @brief	Handle interrupt from GPIO pin or GPIO pin mapped to PININT
 * @return	Nothing
 */
void PININT_IRQ_HANDLER(void)
{
	Chip_PININT_ClearIntStatus(LPC_PININT, PININTCH(GPIO_PININT_INDEX));
	Board_LED_Toggle(0);
}

/**
 * @brief	Main program body
 * @return	Does not return
 */
int main(void)
{
	/* Generic Initialization */
	SystemCoreClockUpdate();

	/* Board_Init calls Chip_GPIO_Init and enables GPIO clock if needed,
	   Chip_GPIO_Init is not called again */
	Board_Init();
	Board_LED_Set(0, false);

	Chip_PININT_Init(LPC_PININT);

	/* Configure GPIO pin as input */
	Chip_GPIO_SetPinDIRInput(LPC_GPIO, GPIO_PININT_PORT, GPIO_PININT_PIN);

	/* Configure pin as GPIO */
	Chip_IOCON_PinMuxSet(LPC_IOCON, GPIO_PININT_PORT, GPIO_PININT_PIN,
						 (IOCON_FUNC0 | IOCON_DIGITAL_EN  | IOCON_GPIO_MODE));

	/* Configure pin interrupt selection for the GPIO pin in Input Mux Block */
	Chip_INMUX_PININT_Config(LPC_INMUX, GPIO_PININT_INDEX, GPIO_PININT_PORT, GPIO_PININT_PIN);

	/* Configure channel interrupt as edge sensitive and falling edge interrupt */
	Chip_PININT_ClearIntStatus(LPC_PININT, PININTCH(GPIO_PININT_INDEX));
	Chip_PININT_SetPinModeEdge(LPC_PININT, PININTCH(GPIO_PININT_INDEX));
	Chip_PININT_EnableIntLow(LPC_PININT, PININTCH(GPIO_PININT_INDEX));

	/* Enable interrupt in the NVIC */
	NVIC_ClearPendingIRQ(PININT_NVIC_NAME);
	NVIC_EnableIRQ(PININT_NVIC_NAME);

	/* Go to sleep mode - LED will toggle on each wakeup event */
	while (1) {
		/* Go to sleep state - will wake up automatically on interrupt */
		__WFI();
	}

	return 0;
}
