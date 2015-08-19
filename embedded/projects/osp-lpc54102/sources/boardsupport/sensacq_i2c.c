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
#include "i2c_api.h"


/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/


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
i2c_t	i2c_master;
static uint8_t g_i2cmTxBuf[CFG_MAX_SA_TX_BUFFER];

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
    /* Call I2CM transfer handler function */
    ROM_I2CM_TransferHandler(i2c_master.px_master_handle);
}

/**
 * @brief	Initialize I2C bus connected sensors
 * @return	True if initalized
 */
osp_bool_t dev_i2c_init(void)
{
    i2c_master.i2c = I2C_LPC_0;
#ifdef I2CM_IRQ_BASED
    i2c_master.ui_irq_priority = SENSOR_I2C_PRIORITY;
#endif /* I2CM_IRQ_BASED */

    i2c_init(&i2c_master,0,0);
    return true;
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
	osDelay(msec); /* Allow startup time for sensors */
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
	}
    i2c_write(&i2c_master,dev_addr,&g_i2cmTxBuf[0],cnt,0);
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
    i2c_master.tx_buff = &reg_addr;
    i2c_read(&i2c_master,dev_addr,reg_data,cnt,0);
    return LPC_OK;
}
