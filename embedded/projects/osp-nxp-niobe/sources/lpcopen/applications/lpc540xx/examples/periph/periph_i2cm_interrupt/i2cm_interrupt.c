/*
 * @brief I2CM bus master example using interrupt mode
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

/* I2CM transfer record */
static I2CM_XFER_T  i2cmXferRec;
/* I2C clock is set to 1.8MHz */
#define I2C_CLK_DIVIDER     (40)
/* 400KHz I2C bit-rate */
#define I2C_BITRATE         (400000)

#if defined(BOARD_NXP_LPCXPRESSO_54000)
/** 7-bit I2C addresses of 6-axis sensor on sensor board */
#define I2C_ADDR_7BIT       (0x10)
#define LPC_I2C_PORT         LPC_I2C0
#define LPC_I2C_INTHAND      I2C0_IRQHandler
#define LPC_IRQNUM           I2C0_IRQn
#endif

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/* Initializes pin muxing for I2C interface - note that SystemInit() may
   already setup your pin muxing at system startup */
static void Init_I2C_PinMux(void)
{
#if defined(BOARD_NXP_LPCXPRESSO_54000)
	/* Connect the I2C1_SDA and I2C1_SCL signals to port pins */
#if (I2C_BITRATE > 400000)
	/* Enable Fast Mode Plus for I2C pins */
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 23, (IOCON_FUNC1 | IOCON_FASTI2C_EN | IOCON_DIGITAL_EN));
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 24, (IOCON_FUNC1 | IOCON_FASTI2C_EN | IOCON_DIGITAL_EN));

#else
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 23, (IOCON_FUNC1 | IOCON_DIGITAL_EN));
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 24, (IOCON_FUNC1 | IOCON_DIGITAL_EN));
#endif

#else
	/* Configure your own I2C pin muxing here if needed */
#warning "No I2C pin muxing defined"
#endif
}

/* Setup I2C */
static void setupI2CMaster(void)
{
	/* Enable I2C clock and reset I2C peripheral */
	Chip_I2C_Init(LPC_I2C_PORT);

	/* Setup clock rate for I2C */
	Chip_I2C_SetClockDiv(LPC_I2C_PORT, I2C_CLK_DIVIDER);

	/* Setup I2CM transfer rate */
	Chip_I2CM_SetBusSpeed(LPC_I2C_PORT, I2C_BITRATE);

	/* Enable I2C master interface */
	Chip_I2CM_Enable(LPC_I2C_PORT);
}

/* Function to wait for I2CM transfer completion */
static void WaitForI2cXferComplete(I2CM_XFER_T *xferRecPtr)
{
	/* Test for still transferring data */
	while (xferRecPtr->status == I2CM_STATUS_BUSY) {
		/* Sleep until next interrupt */
		__WFI();
	}
}

/* Function to setup and execute I2C transfer request */
static void SetupXferRecAndExecute(uint8_t devAddr,
								   uint8_t *txBuffPtr,
								   uint16_t txSize,
								   uint8_t *rxBuffPtr,
								   uint16_t rxSize)
{
	/* Setup I2C transfer record */
	i2cmXferRec.slaveAddr = devAddr;
	i2cmXferRec.status = 0;
	i2cmXferRec.txSz = txSize;
	i2cmXferRec.rxSz = rxSize;
	i2cmXferRec.txBuff = txBuffPtr;
	i2cmXferRec.rxBuff = rxBuffPtr;

	Chip_I2CM_Xfer(LPC_I2C_PORT, &i2cmXferRec);
	/* Enable Master Interrupts */
	Chip_I2C_EnableInt(LPC_I2C_PORT, I2C_INTENSET_MSTPENDING | I2C_INTENSET_MSTRARBLOSS | I2C_INTENSET_MSTSTSTPERR);
	/* Wait for transfer completion */
	WaitForI2cXferComplete(&i2cmXferRec);
	/* Disable all Interrupts */
	Chip_I2C_DisableInt(LPC_I2C_PORT, I2C_INTENSET_MSTPENDING | I2C_INTENSET_MSTRARBLOSS | I2C_INTENSET_MSTSTSTPERR);

	if (i2cmXferRec.status != I2CM_STATUS_OK) {
		DEBUGOUT("\r\nI2C error: %d\r\n", i2cmXferRec.status);
	}
}

/* Master I2CM receive in interrupt mode */
#if defined(BOARD_NXP_LPCXPRESSO_54000)
/* Function to read X/Y/Z accel counts */
static void ReadCCData(void)
{
	uint8_t rx[16], tx[4];

	/* Get 6 values starting at register 2 */
	tx[0] = 2;
	SetupXferRecAndExecute(I2C_ADDR_7BIT, tx, 1, rx, 8);
	if (i2cmXferRec.status == I2CM_STATUS_OK) {
		/* Register 2/3 - X accel counts
		   Register 4/5 - Y accel counts
		   Register 6/7 - Z accel counts */
		DEBUGOUT("X: %04x, Y: %04x, Z: %04x",
				 (((int16_t) rx[1] << 4) | ((int16_t) rx[0] >> 4)),
				 (((int16_t) rx[3] << 4) | ((int16_t) rx[2] >> 4)),
				 (((int16_t) rx[5] << 4) | ((int16_t) rx[4] >> 4))
				 );
	}

	/* Get 1 value starting at register 8 */
	tx[0] = 8;
	SetupXferRecAndExecute(I2C_ADDR_7BIT, tx, 1, rx, 2);
	if (i2cmXferRec.status == I2CM_STATUS_OK) {
		/* Register 8   - temperature */
		DEBUGOUT(", T: %02x\r",
				 ((int16_t) rx[0])
				 );
	}
}

#endif

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/**
 * @brief	Handle I2C1 interrupt by calling I2CM interrupt transfer handler
 * @return	Nothing
 */
void LPC_I2C_INTHAND(void)
{
	uint32_t state = Chip_I2C_GetPendingInt(LPC_I2C_PORT);

	/* Error handling */
	if (state & (I2C_INTENSET_MSTRARBLOSS | I2C_INTENSET_MSTSTSTPERR)) {
		Chip_I2CM_ClearStatus(LPC_I2C_PORT, I2C_STAT_MSTRARBLOSS | I2C_STAT_MSTSTSTPERR);
	}

	/* Call I2CM ISR function with the I2C device and transfer rec */
	if (state & I2C_INTENSET_MSTPENDING) {
		Chip_I2CM_XferHandler(LPC_I2C_PORT, &i2cmXferRec);
	}
}

/**
 * @brief	Main routine for I2C example
 * @return	Function should not exit
 */
int main(void)
{
	uint8_t rx[4], tx[4];

	/* Generic Initialization */
	SystemCoreClockUpdate();
	Board_Init();

	/* Clear activity LED */
	Board_LED_Set(0, false);

	/* Setup I2C pin muxing */
	Init_I2C_PinMux();

	/* Setup I2C and master */
	setupI2CMaster();

	/* Enable the interrupt for the I2C */
	NVIC_EnableIRQ(LPC_IRQNUM);

	/* Read chip ID of Niobe sensor board's e-compass */
	tx[0] = 0;
	SetupXferRecAndExecute(I2C_ADDR_7BIT, tx, 1, rx, 1);
	DEBUGOUT("E-Compass Chip ID = 0x%x\r\n", (int16_t) rx[0]);

	/* Read data as fast as possible in loop */
	while (1) {

#if defined(BOARD_NXP_LPCXPRESSO_54000)
		/* Read data from Coulomb Counter */
		ReadCCData();
#endif

		/* Toggle LED to show activity. */
		Board_LED_Toggle(0);
	}

	/* Code never reaches here. Only used to satisfy standard main() */
	return 0;
}
