/*
 * @brief I2CM driver used by Sensor acquisition task
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
#include <string.h>
#include "sensorhub.h"
#include "sensacq_i2c.h"
#include "common.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/* I2CM transfer record */
static I2CM_XFER_T  i2cmXferRec;
static uint8_t g_i2cmTxBuf[4];

#if defined(I2CM_DMA_BASED)
/* DMA descriptors must be aligned to 16 bytes */
#if defined(__CC_ARM)
__align(16) static DMA_CHDESC_T dmaI2CMDesc;
#endif /* defined (__CC_ARM) */

/* IAR support */
#if defined(__ICCARM__)
#pragma data_alignment=16
static DMA_CHDESC_T dmaI2CMDesc;
#endif /* defined (__ICCARM__) */

#if defined( __GNUC__ )
static DMA_CHDESC_T dmaI2CMDesc __attribute__ ((aligned(16)));
#endif /* defined (__GNUC__) */
#endif /* defined(I2CM_DMA_BASED) */

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/
extern uint8_t timeStampExtender;

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/* Setup I2C */
static void setupI2CMaster(void)
{
	/* Enable I2C clock and reset I2C peripheral */
	Chip_I2C_Init(I2C_SENSOR_BUS);

	/* Setup clock rate for I2C */
	Chip_I2C_SetClockDiv(I2C_SENSOR_BUS, I2C_MASTER_CLOCK_DIV);

	/* Setup I2CM transfer rate */
	Chip_I2CM_SetBusSpeed(I2C_SENSOR_BUS, I2C_MCLOCK_SPEED);

	/* Enable I2C master interface */
	Chip_I2CM_Enable(I2C_SENSOR_BUS);

#if defined(I2CM_DMA_BASED)
	/* Setup I2C0 slave channel for the following configuration:
	   - Peripheral DMA request
	   - Single transfer
	   - High channel priority */
	Chip_DMA_EnableChannel(LPC_DMA, I2C_SENSOR_BUS_DMAID);
	Chip_DMA_EnableIntChannel(LPC_DMA, I2C_SENSOR_BUS_DMAID);
	Chip_DMA_SetupChannelConfig(LPC_DMA, I2C_SENSOR_BUS_DMAID,
		(DMA_CFG_PERIPHREQEN | DMA_CFG_TRIGBURST_SNGL | DMA_CFG_CHPRIORITY(0)));
#endif
}

#ifdef I2CM_IRQ_BASED
/* Function to wait for I2CM transfer completion */
static void WaitForI2cXferComplete(I2CM_XFER_T *xferRecPtr)
{
	/* Are we still transferring data ? */
	while (xferRecPtr->status == I2CM_STATUS_BUSY) {
		/* Sleep until next interrupt */
		__WFI();
	}
}
#endif

#if defined(I2CM_DMA_BASED)
/* Setup and start a DMA transfer */
static setupI2CDMAXfer(void *buff, uint8_t bytes, bool tx)
{
	/* Enable DMA for I2C controller */
	I2C_SENSOR_BUS->MSTCTL = I2C_MSTCTL_MSTDMA;

	/* Master to slave */
	if (tx) {
		dmaI2CMDesc.source = DMA_ADDR(buff) + bytes - 1;
		dmaI2CMDesc.dest = DMA_ADDR(&I2C_SENSOR_BUS->MSTDAT);
		dmaI2CMDesc.next = DMA_ADDR(0);
		dmaI2CMDesc.xfercfg = DMA_XFERCFG_CFGVALID | DMA_XFERCFG_SETINTA |
						 DMA_XFERCFG_SWTRIG | DMA_XFERCFG_WIDTH_8 | DMA_XFERCFG_SRCINC_1 |
						 DMA_XFERCFG_DSTINC_0 | DMA_XFERCFG_XFERCOUNT(bytes);
	}
	else {
		dmaI2CMDesc.source = DMA_ADDR(&I2C_SENSOR_BUS->MSTDAT);
		dmaI2CMDesc.dest = DMA_ADDR(buff) + bytes - 1;
		dmaI2CMDesc.next = DMA_ADDR(0);
		dmaI2CMDesc.xfercfg = DMA_XFERCFG_CFGVALID | DMA_XFERCFG_SETINTA |
						 DMA_XFERCFG_SWTRIG | DMA_XFERCFG_WIDTH_8 | DMA_XFERCFG_SRCINC_0 |
						 DMA_XFERCFG_DSTINC_1 | DMA_XFERCFG_XFERCOUNT(bytes);
	}

	/* Setup transfer descriptor and validate it */
	Chip_DMA_SetupTranChannel(LPC_DMA, I2C_SENSOR_BUS_DMAID, &dmaI2CMDesc);

	/* Setup data transfer */
	Chip_DMA_SetupChannelTransfer(LPC_DMA, I2C_SENSOR_BUS_DMAID, dmaI2CMDesc.xfercfg);

	Chip_DMA_SetValidChannel(LPC_DMA, I2C_SENSOR_BUS_DMAID);
}
#endif

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/* DMA error handler and notification for I2C */
void dev_i2cm_dma_callback(bool error)
{
	/* ##KW## What do we do here for errors? */
	if (error) {
		/* Nothing to do here. I2C will terminate the trasnfer. */
		;
	}

	I2C_SENSOR_BUS->MSTCTL = 0;
}

/**
 * @brief	Handle I2C1 interrupt by calling I2CM interrupt transfer handler
 * @return	Nothing
 */
void I2C_SENSOR_BUS_IRQHandler(void)
{
#if defined(I2CM_DMA_BASED)
	uint32_t cState;
#endif
	uint32_t state = Chip_I2C_GetPendingInt(I2C_SENSOR_BUS);

	/* Error handling */
	if (state & (I2C_INTENSET_MSTRARBLOSS | I2C_INTENSET_MSTSTSTPERR)) {
		Chip_I2CM_ClearStatus(I2C_SENSOR_BUS, I2C_STAT_MSTRARBLOSS | I2C_STAT_MSTSTSTPERR);
	}

	if (state & I2C_INTENSET_MSTPENDING) {
#if defined(I2CM_DMA_BASED)
		/* DMA may still be active, but the transfer might of completed due to
		  a NAK or error. Handle the error here before queueing up the DMA
		  transfer. */
		cState = Chip_I2CM_GetMasterState(I2C_SENSOR_BUS);
		if (cState == I2C_STAT_MSTCODE_RXREADY) {
			if (i2cmXferRec.rxSz > I2C_SENSOR_BUS_DMABYTELIM) {
				/* Master receive */
				setupI2CDMAXfer((void *) i2cmXferRec.rxBuff, i2cmXferRec.rxSz, false);
				i2cmXferRec.rxSz = 0;
			} else {
				Chip_I2CM_XferHandler(I2C_SENSOR_BUS, &i2cmXferRec);
			}
		}
		else if (cState == I2C_STAT_MSTCODE_TXREADY) {
			if (i2cmXferRec.txSz > I2C_SENSOR_BUS_DMABYTELIM) {
				/* Master transmit */
				setupI2CDMAXfer((void *) i2cmXferRec.txBuff, i2cmXferRec.txSz, true);
				i2cmXferRec.txSz = 0;
			} else {
				/* Transistion to read state */
				Chip_I2CM_XferHandler(I2C_SENSOR_BUS, &i2cmXferRec);
			}
		} else {
			/* Not in a transfer state, so handle using the normal I2C handler */
			Chip_I2CM_XferHandler(I2C_SENSOR_BUS, &i2cmXferRec);
		}
#else
		Chip_I2CM_XferHandler(I2C_SENSOR_BUS, &i2cmXferRec);
#endif
	}

	/* If transfer is no longer in progress, disable interrupts */
	if (i2cmXferRec.status != I2CM_STATUS_BUSY) {
		Chip_I2C_DisableInt(I2C_SENSOR_BUS, I2C_INTENSET_MSTPENDING |
			I2C_INTENSET_MSTRARBLOSS | I2C_INTENSET_MSTSTSTPERR);
	}
}

/**
 * @brief	Initialize I2C bus connected sensors
 * @return	True if initalized
 */
osp_bool_t dev_i2c_init(void)
{
	static bool init = false;

	if (init == false) {
		/* Setup I2C and master */
		setupI2CMaster();

#ifdef I2CM_IRQ_BASED
		NVIC_SetPriority(I2C_SENSOR_BUS_IRQn, SENSOR_I2C_PRIORITY);
		/* Enable the interrupt for the I2C */
		NVIC_EnableIRQ(I2C_SENSOR_BUS_IRQn);
#endif		
		init = true;
	}
	return init;
}

/**
 * @brief	adaptation of the Bosch API functions to the
 *		BOARD specific function
 * @return	Nothing
 */
void dev_i2c_delay(unsigned int msec)
{
	/* Will WFI for the entire sleep duration. If an interrupt
	   occurs that wakes the device, the sleep handler will
	   automatically re-enter WFI until the duration has expired. */
	os_dly_wait(MSEC_TO_TICS(msec)); /* Allow startup time for sensors */
}

/**
 * @brief	adaptation of the Bosch API functions to the BOARD specific function
 * @return	Nothing
 */
char dev_i2c_write(unsigned char dev_addr, unsigned char reg_addr,
		unsigned char *reg_data, unsigned char cnt)
{
	/* Setup I2C transfer record */
	g_i2cmTxBuf[0] = reg_addr;
	if (cnt == 1) {
		g_i2cmTxBuf[1] = reg_data[0];
	} else {
		memcpy(&g_i2cmTxBuf[1], reg_data, cnt);
	}

	memset(&i2cmXferRec, 0, sizeof(I2CM_XFER_T));

	i2cmXferRec.slaveAddr = dev_addr;
	i2cmXferRec.status = 0;
	i2cmXferRec.txSz = cnt + 1;
	i2cmXferRec.rxSz = 0;
	i2cmXferRec.txBuff = &g_i2cmTxBuf[0];
	i2cmXferRec.rxBuff = 0;

#ifdef I2CM_IRQ_BASED
	Chip_I2CM_Xfer(I2C_SENSOR_BUS, &i2cmXferRec);
	/* Enable Master Interrupts */
	Chip_I2C_EnableInt(I2C_SENSOR_BUS, I2C_INTENSET_MSTPENDING |
		I2C_INTENSET_MSTRARBLOSS | I2C_INTENSET_MSTSTSTPERR);
	/* Wait for transfer completion */
	WaitForI2cXferComplete(&i2cmXferRec);
#else
	Chip_I2CM_XferBlocking(I2C_SENSOR_BUS, &i2cmXferRec);
#endif	

	return LPC_OK;
}

/**
 * @brief	adaptation of the Bosch API functions to the 
 *              BOARD specific function
 * @return	Nothing
 */
char dev_i2c_read(unsigned char dev_addr, unsigned char reg_addr,
		unsigned char *reg_data, unsigned char cnt)
{
	memset(&i2cmXferRec, 0, sizeof(I2CM_XFER_T));
	/* Setup I2C transfer record */
	i2cmXferRec.slaveAddr = dev_addr;
	i2cmXferRec.status = 0;
	i2cmXferRec.txSz = 1;
	i2cmXferRec.rxSz = cnt;
	i2cmXferRec.txBuff = &reg_addr;
	i2cmXferRec.rxBuff = reg_data;

#ifdef I2CM_IRQ_BASED
	Chip_I2CM_Xfer(I2C_SENSOR_BUS, &i2cmXferRec);
	/* Enable Master Interrupts */
	Chip_I2C_EnableInt(I2C_SENSOR_BUS, I2C_INTENSET_MSTPENDING |
			I2C_INTENSET_MSTRARBLOSS | I2C_INTENSET_MSTSTSTPERR);
	/* Wait for transfer completion */
	WaitForI2cXferComplete(&i2cmXferRec);
#else
	Chip_I2CM_XferBlocking(I2C_SENSOR_BUS, &i2cmXferRec);
#endif	

	return LPC_OK;
}
