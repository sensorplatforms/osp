/*
 * @brief SPI bus master example using the ROM API in interrupt mode
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

/* SPI master handle and memory for ROM API */
static SPI_HANDLE_T *spiHandleMaster;

/* Use a buffer size larger than the expected return value of
   spi_get_mem_size() for the static SPI handle type */
static uint32_t spiMasterHandleMEM[0x20];

static uint16_t xferArray[] = {0x1111, 0x2222, 0x3333, 0x4444};
static uint16_t rx_buff[sizeof(xferArray) / sizeof(uint16_t)];

/* Interrupt error code (used as semaphore) */
static volatile int intErrCode;

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/* Initializes pin muxing for SPI interface - note that SystemInit() may
   already setup your pin muxing at system startup */
static void Init_SPI_PinMux(void)
{
#if (defined(BOARD_NXP_LPCXPRESSO_54000))
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 14, (IOCON_FUNC1 | IOCON_MODE_PULLUP | IOCON_DIGITAL_EN));	/* SSEL */
	Chip_IOCON_PinMuxSet(LPC_IOCON, 1, 3,  (IOCON_FUNC5 | IOCON_MODE_PULLUP | IOCON_DIGITAL_EN));	/* SCK */
	Chip_IOCON_PinMuxSet(LPC_IOCON, 1, 4,  (IOCON_FUNC5 | IOCON_MODE_PULLUP | IOCON_DIGITAL_EN));	/* MISO */
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 12, (IOCON_FUNC1 | IOCON_MODE_PULLUP | IOCON_DIGITAL_EN));	/* MOSI */

#else
	/* Configure your own SPI pin muxing here if needed */
#warning "No SPI pin muxing defined"
#endif
}

/* Turn on LED to indicate an error */
static void errorSPI(void)
{
	Board_LED_Set(0, true);
	while (1) {}
}

/* Setup SPI handle and parameters */
static void setupSpiMaster()
{
	SPI_CONFIG_T spiConfigRec;

	/* Enable SPI clock and reset SPI peripheral - the boot ROM does not
	   do this */
#if 0
	Chip_SPI_IF_Init(LPC_SPI0); // FIXME - API need to be standardized to other platforms
#else
	// FIXME - temp workaround
	Chip_Clock_EnableAsyncPeriphClock(ASYNC_SYSCTL_CLOCK_SPI0);
	Chip_SYSCTL_AsyncPeriphReset(ASYNC_RESET_SPI0);
#endif
	
	/* Perform a sanity check on the storage allocation */
	if (LPC_SPID_API->spi_get_mem_size() > sizeof(spiMasterHandleMEM)) {
		/* Example only: this should never happen and probably isn't needed for
		   most SPI code. */
		errorSPI();
	}

	/* Setup the SPI0 handle */
	spiHandleMaster = LPC_SPID_API->spi_setup(LPC_SPI0_BASE, (uint8_t *) &spiMasterHandleMEM);
	if (spiHandleMaster == NULL) {
		errorSPI();
	}
	/* Setup SPI0 configuration record */
	spiConfigRec.delay = 0x2222;
	/* SysClock divided is set to maximum */
	spiConfigRec.divider = 0xFFFF;
	/* Loopback mode, master mode and SPI block enabled */
	spiConfigRec.config = 0x85;
	spiConfigRec.error_en = 0;

	/* Init SPI0 */
	LPC_SPID_API->spi_init(spiHandleMaster, &spiConfigRec);
}

/* SPI interrupt callback, called on completion of SPI operation when in
   interrupt mode. Called in interrupt context. */
static void cbSpiComplete(uint32_t err_code, uint32_t n)
{
	uint32_t i;
	if ((err_code == LPC_OK) && (n == (sizeof(xferArray) / sizeof(xferArray[0])))) {
		/* Verify if received data is same as transmit */
		for (i = 0; i < n; i++) {
			if (rx_buff[i] != xferArray[i]) {
				errorSPI();
			}
		}
		intErrCode = (int) err_code;
	}
	else {
		/* Signal Error */
		errorSPI();
	}
}

/* Master SPI transmit in interrupt mode */
static void WriteSpiMssg(uint16_t *xferPtr, uint32_t xferSize)
{
	SPI_PARAM_T paramRec;

	/* Init variable used as semaphore */
	intErrCode = -1;

	/* Setup transfer record */
	paramRec.tx_buffer = xferPtr;	/* SPI TX buffer */
	paramRec.size = xferSize;		/* total number of SPI transfers */
	paramRec.rx_buffer = rx_buff;	/* SPI RX buffer */
	paramRec.fsize_sel = 0x0F0E0000;/* Set Tx Control for 16 bit transfer, SSEL0 asserted */
	paramRec.eof_flag = 1;	/* End of Frame enabled */
	paramRec.tx_rx_flag = 2;		/* transmit and receive */
	paramRec.driver_mode = 1;		/* interrupt mode */
	paramRec.dma_cfg = NULL;		/* DMA configuration */
	paramRec.cb = (SPI_CALLBK_T) cbSpiComplete;	/* SPI completion callback */
	paramRec.dma_cb = NULL;			/* DMA completion callback */

	/* Transfer message as SPI master via interrupt */
	if (LPC_SPID_API->spi_master_transfer(spiHandleMaster, &paramRec) != LPC_OK) {
		/* Signal SPI error */
		errorSPI();
	}

	/* Sleep until transfer is complete, but allow IRQ to wake system
	   to handle SPI IRQ */
	while (intErrCode == -1) {
		__WFI();
	}
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/**
 * @brief	Handle SPI0 interrupt by calling SPI ROM handler
 * @return	Nothing
 */
void SPI0_IRQHandler(void)
{
	/* Call SPI ISR function in ROM with the SPI handle */
	LPC_SPID_API->spi_isr(spiHandleMaster);
}

/**
 * @brief	Main routine for SPI example
 * @return	Function should not exit
 */
int main(void)
{
	/* Generic Initialization */
	SystemCoreClockUpdate();
	Board_Init();

	/* Clear activity LED */
	Board_LED_Set(0, false);

	/* Setup SPI pin muxing */
	Init_SPI_PinMux();

	/* Allocate SPI handle, setup rate, and initialize clocking */
	setupSpiMaster();

	/* Enable SPI0 interrupt */
	NVIC_EnableIRQ(SPI0_IRQn);

	/* Loop forever */
	while (1) {
		/* Write simple message over SPI */
		WriteSpiMssg(xferArray, sizeof(xferArray) / sizeof(xferArray[0]));

		/* Toggle LED to show activity. */
		Board_LED_Toggle(0);
	}

	/* Code never reaches here. Only used to satisfy standard main() */
	return 0;
}
