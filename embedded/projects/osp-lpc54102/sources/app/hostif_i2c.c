/*
 * @brief Implements I2C slave driver for host interface module
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
#include <stdint.h>
#include "sensorhub.h"
#include "hostif.h"
#include "osp-types.h"
#include "common.h"
/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/
#define RX_LENGTH		16
#define I2C_SLAVE_XFER_DONE	0x10
#define I2C_OVERREACH_VAL	0xCC// This value is sent for access beyond register area

typedef struct __HOSTIF_Ctrl_t {
	int	magic;   
	uint16_t rxCount;			/* Bytes so far received  */
	uint16_t rxLength;			/* Expected Rx buffer length */
	uint16_t txCountSet;
	uint16_t txCount;			/* Bytes so far transmitted */
	uint16_t txLength;			/* Total transfer length of Tx buffer */
	uint8_t *txBuff;			/* Tx buffer pointer */
	uint8_t *oldtxBuff;
	uint8_t rxBuff[RX_LENGTH];	/* Rx buffer */
	uint8_t *txBuff_next;
	uint16_t txLength_next;
} Hostif_Ctrl_t;

static Hostif_Ctrl_t g_hostif;

#define I2C_MEM_SZ    64 /* Size of memory for I2C Slave ROM driver */

/* Handle to I2C */
static ROM_I2CS_HANDLE_T hI2C;

/* I2C transfer structure; directly manipulated by ROM driver */
static ROM_I2CS_XFER_T i2cXfer;

/* tx state handling variable */
static int32_t i2cs_tx_done = 0;

/* Rx state handling variable */
static int32_t i2cs_rx_done = 0;

#if defined(I2CS_DMA_BASED)
/* DMA descriptors must be aligned to 16 bytes */
#if defined(__CC_ARM)
__align(16) static DMA_CHDESC_T dmaI2CSDesc;
#endif /* defined (__CC_ARM) */

/* IAR support */
#if defined(__ICCARM__)
#pragma data_alignment=16
static DMA_CHDESC_T dmaI2CSDesc;
#endif /* defined (__ICCARM__) */

#if defined( __GNUC__ )
static DMA_CHDESC_T dmaI2CSDesc __attribute__ ((aligned(16)));
#endif /* defined (__GNUC__) */
#endif

static void Hostif_TxNext(int magic);

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/* I2C start event callback. */
static void i2cs_start(ROM_I2CS_HANDLE_T hI2C, uint16_t addr)
{
    // nothing requires to do for I2C Start.  
   // D0_printf("I2CS address 0x%x\r\n", addr);
}

/* I2C transmit callback. 
 * This function if registered will be called when I2C master is requesting 
 * data from I2C slave. This function should setup the I2CS transmit buffer 
 * pointer in the xfer descriptor. The ROM driver will perform the multbytes
 * transfer if the master I2C requested more than one byte. 
 */
static ROM_I2CS_TRANCTRL_T i2cs_tx(ROM_I2CS_HANDLE_T hI2C, ROM_I2CS_XFER_T *pXfer)
{ 
    // Must have this statement or else the multi-bytes transmit sequence will be incorrect. 
	i2cXfer.txBuff = (uint8_t *)i2cXfer.txBuff + i2cXfer.txSz;
    // There is no need to update the txSz parameter in the xfer descriptor. 
	i2cs_tx_done = 1;
	return ROM_I2CS_CONTINUE;
}

/* I2C Receive callback. 
 * This callback happens when there is data to receive. Receive data will be 
 * placed in the i2cXfer.rxBuff after the return statement. 
 */ 
static ROM_I2CS_TRANCTRL_T i2cs_rx(ROM_I2CS_HANDLE_T hI2C, ROM_I2CS_XFER_T *pXfer)
{
	i2cXfer.rxBuff = (uint8_t *)i2cXfer.rxBuff + i2cXfer.rxSz;
	i2cXfer.rxSz++;

	//D0_printf("RxCB rxBuff = 0x%p,  rxSz = %d\r\n", i2cXfer.rxBuff, i2cXfer.rxSz);
	i2cs_rx_done = 1;    
	return ROM_I2CS_CONTINUE;
}


/* I2C Done event callback [Will be called after STOP bit is received] */
static void i2cs_done(ROM_I2CS_HANDLE_T hI2C, ROM_I2CS_XFER_T *pXfer)
{
	i2cXfer.rxBuff = g_hostif.rxBuff;
	//i2cXfer.txSz = 0;      // Do not reset the txSz in this callback.
    i2cXfer.rxSz = 0;    
	ROM_I2CS_Transfer(hI2C, &i2cXfer);    // Wait for another master request
}

 


/*****************************************************************************
 * Public functions
 ****************************************************************************/

/* I2CS Interrupt handler. 
 * This function is called whenever there is an I2C interrupt service request 
 * on the hostif I2C bus. 
 */
void I2C_HOSTIF_IRQHandler(void)
{
    i2cs_rx_done = 0;
    i2cs_tx_done = 0;

    // This transfer handler will call one of the registered callback to
    // to service the I2C event. 
	ROM_I2CS_TransferHandler(hI2C);  
	
    if ( i2cs_rx_done ) {
        uint8_t ret;
       // D0_printf("   bytesReceived = %d, rxCount = %d, txCount = %d\r\n", i2cXfer.bytesRecv, rxCount, txCount);
        ret = process_command( i2cXfer.rxBuff, i2cXfer.rxSz);
        if ( g_hostif.rxBuff == i2cXfer.rxBuff ) {
            i2cXfer.rxSz  = ret;
        }
    }

    if ( i2cs_tx_done ) {  
        // Assume each transmission will transmit all the requested data so no need to keep track how many been transmitted.       
        /* check if there is any additional data to send */
        Hostif_TxNext(0x0d000000);        // Not sure what the magic number mean? 	                     
    }
}


/* This function get calls directly when processing receiving host command */
void Hostif_StartTx(uint8_t *pBuf, uint16_t size, int magic)
{
    int32_t i;

	if (magic != -1) 
		g_hostif.magic = magic;
	if (magic == -1)
		g_hostif.magic &= 0xffff0000;
	g_hostif.txBuff = pBuf;
	if (size != 0)
		g_hostif.txCountSet = size;
	if (pBuf != NULL) 
		g_hostif.oldtxBuff = pBuf;    // keep a copy of the requested transmit buffer pointer 
	g_hostif.txCount = 0;             // number of bytes transmitted 
	g_hostif.txLength = size;         // request bytes to be transmitted 
	g_hostif.txLength_next = 0;
	g_hostif.txBuff_next = 0;

    /* Update the I2C transfer descriptor */
    i2cXfer.txBuff = pBuf;
    i2cXfer.txSz   = size -1;         //Note: This field needs to be set one less then than tx buffer size or else I2C will not generate tx callback.  
}


/* This function get calls after a transmit has complete. This
 * function will setup another transmit if there are data pending 
 * to be sent
 */ 
void Hostif_TxNext(int magic)
{
	g_hostif.magic |= magic;
	g_hostif.txCountSet = g_hostif.txLength_next;
	g_hostif.txCount = 0;

	if (g_hostif.txBuff_next) {
		g_hostif.txBuff = g_hostif.txBuff_next;
		g_hostif.txLength = g_hostif.txLength_next;
		g_hostif.txBuff_next = NULL;
		g_hostif.txLength_next = 0;
	} else {
		g_hostif.txBuff = NULL;
		g_hostif.txLength = 0;   
	}

    // update the I2C transfer descriptor
    i2cXfer.txBuff = g_hostif.txBuff;
    i2cXfer.txSz   = g_hostif.txLength > 0 ? g_hostif.txLength -1 : 0;

//    D0_printf("txNext: i2cXfer.txBuff = 0x%p, txSz = %d\r\n", i2cXfer.txBuff, i2cXfer.txSz);

}

void CHostif_StartTxChained(uint8_t *pBuf, uint16_t size, uint8_t *pBuf_next, uint16_t size_next, int magic)
{
	if (magic != -1) 
		g_hostif.magic = magic;
	if (magic == -1)
		g_hostif.magic &= 0xffff0000;
	g_hostif.txBuff = pBuf;
	if (size != 0)
		g_hostif.txCountSet = size;
	if (pBuf != NULL) 
		g_hostif.oldtxBuff = pBuf;
	g_hostif.txCount = 0;
	g_hostif.txLength = size;
	g_hostif.txBuff_next = pBuf_next;
	g_hostif.txLength_next = size_next;

    // Update xfer descriptor 
    i2cXfer.txBuff = pBuf;
    i2cXfer.txSz  = size-1;
   
    // Note: printf in IRQ tends to crash the system so use only in debugging code
#if 0
    if ( i2cXfer.txBuff != NULL ) 
        D0_printf("StartTxChained:  pBuf = 0x%p, txSz = %d, pBuf_next = 0x%p, txSz_next = %d \r\n   Data: ", pBuf, i2cXfer.txSz, pBuf_next, size_next);
#endif 
}

/** Initialize the Sensor Hub host/AP interface */
void Hostif_Init(void)
{     
    ROM_I2CS_INIT_T i2csInit;
    ROM_I2CS_SLAVE_T slaveSetup;
    static uint32_t devMem[I2C_MEM_SZ];
    uint32_t memSize, optimalDev;
    static osp_bool_t i2cXferDone; 

   /* reset host IF control data structure */
    memset(&g_hostif, 0, sizeof(Hostif_Ctrl_t));

    /* Setup I2C pin mux */
    Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 27, (IOCON_FUNC1 | IOCON_DIGITAL_EN));  /* i2c2 */
    Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 28, (IOCON_FUNC1 | IOCON_DIGITAL_EN));  /* I2C2 */

    Chip_Clock_EnablePeriphClock(I2C_HOSTIF_CLK);
    Chip_SYSCON_PeriphReset(I2C_HOSTIF_RST);

    memSize = ROM_I2CS_GetMemSize();
    if ( memSize > sizeof(devMem) ) {
        /* Increase I2CS_MEM_SZ if this happens*/
        D0_printf("ERROR: Insufficient I2C device memory buffer.\r\n");
        while(1);
    }
    
    i2csInit.pUserData = NULL;
    i2csInit.base      = (uint32_t) I2C_HOSTIF;    
    hI2C = ROM_I2CS_Init(devMem, &i2csInit);     // Initializing I2C
   
    ROM_I2CS_RegisterCallback(hI2C, ROM_I2CS_START_CB, (void*) i2cs_start);  // slave start 
    ROM_I2CS_RegisterCallback(hI2C, ROM_I2CS_XFERRECV_CB, (void*) i2cs_rx);
    ROM_I2CS_RegisterCallback(hI2C, ROM_I2CS_XFERSEND_CB, (void*) i2cs_tx); // From master
    ROM_I2CS_RegisterCallback(hI2C, ROM_I2CS_DONE_CB, (void*) i2cs_done);     // slave xfer complete

    /* Setup slave address to respond to */
	slaveSetup.slaveAddr = I2C_HOSTIF_ADDR;     /* host address is 7 bit */
	slaveSetup.SlaveIndex = 0;
	slaveSetup.EnableSlave = 1;
	ROM_I2CS_SetupSlave(hI2C, &slaveSetup);

    /* Setup the clock rate for I2C. */
    // Replace i2c clock rate 400khz with #define
    optimalDev = Chip_Clock_GetAsyncSyscon_ClockRate()/4000000;  /* 250ns */

    I2C_HOSTIF->CLKDIV = optimalDev;

    D0_printf("I2C Slave address: 0x%x, CLOCK_DIV: %d\r\n", I2C_HOSTIF_ADDR, optimalDev);


	/* Setup I2C receiving buffer */
	i2cXfer.rxBuff = g_hostif.rxBuff;    
	i2cXfer.rxSz = 0;     
    
    // Queue an I2C transfer pending master request
	ROM_I2CS_Transfer(hI2C, &i2cXfer);
    
    /* init host interrupt pin */
    Chip_GPIO_SetPinDIROutput(LPC_GPIO, HOSTIF_IRQ_PORT, HOSTIF_IRQ_PIN);
    
    /* de-assert interrupt line to high to indicate Host/AP that 
     * there is no data to receive
     */
    Chip_GPIO_SetPinState(LPC_GPIO, HOSTIF_IRQ_PORT, HOSTIF_IRQ_PIN, 1);
    
    /* Enable the interrupt for the I2C */
	NVIC_SetPriority(I2C_HOSTIF_IRQn, HOSTIF_IRQ_PRIORITY);
	NVIC_EnableIRQ(I2C_HOSTIF_IRQn);

	/* enable I2C hostif to wake-up the sensor hub */
	Chip_SYSCON_EnableWakeup(I2C_HOSTIF_WAKE);
    
    D0_printf("%s initialization done\r\n", __FUNCTION__);
}
