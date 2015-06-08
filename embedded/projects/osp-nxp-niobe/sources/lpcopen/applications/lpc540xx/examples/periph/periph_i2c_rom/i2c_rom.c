/*
 * @brief I2C bus master example using the ROM API
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

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/* Enable the following definition to build this example for interrupt mode */
#define INTERRUPTMODE

/* I2C master handle and memory for ROM API */
static I2C_HANDLE_T *i2cHandleMaster;

/* Use a buffer size larger than the expected return value of
   i2c_get_mem_size() for the static I2C handle type */
static uint32_t i2cMasterHandleMEM[0x20];

/* 1Mbps I2C bit-rate */
#define I2C_BITRATE             (100000)

/** 7-bit and 10-bit I2C addresses */
#define I2C_ADDR_7BIT     (0x48)
#define I2C_ADDR_10BIT    (0x2CA)

#if defined(INTERRUPTMODE)
static volatile int intErrCode;
#endif

/* SysTick rate in Hz */
#define TICKRATE_HZ (10)

/* Current state for LED control via I2C cases */
static volatile int state;

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
	/* Connect the I2C_SDA and I2C_SCL signals to port pins(P0.23, P0.24) */
	//FIXME - The IOCON_FASTI2C_EN should not be set
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 23, IOCON_FASTI2C_EN | IOCON_MODE_PULLUP | IOCON_FUNC1);
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 24, IOCON_FASTI2C_EN | IOCON_MODE_PULLUP | IOCON_FUNC1);

#if (I2C_BITRATE > 400000)
	/* Enable Fast Mode Plus for I2C pins */
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 23, IOCON_OPENDRAIN_EN | IOCON_FASTI2C_EN | IOCON_MODE_PULLUP | IOCON_FUNC1);
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 24, IOCON_OPENDRAIN_EN | IOCON_FASTI2C_EN | IOCON_MODE_PULLUP | IOCON_FUNC1);
#endif
#else
	/* Configure your own I2C pin muxing here if needed */
#warning No I2C pin muxing defined
#endif
}

/* Turn on LED to indicate an error */
static void errorI2C(void)
{
	Board_LED_Set(0, true);
	while (1) {}
}

/* Setup I2C handle and parameters */
static void setupI2CMaster()
{
	/* Enable I2C clock and reset I2C peripheral - the boot ROM does not
	   do this */
	Chip_I2C_Init(LPC_I2C0);

	/* Perform a sanity check on the storage allocation */
	if (LPC_I2CD_API->i2c_get_mem_size() > sizeof(i2cMasterHandleMEM)) {
		/* Example only: this should never happen and probably isn't needed for
		   most I2C code. */
		errorI2C();
	}

	/* Setup the I2C handle */
	i2cHandleMaster = LPC_I2CD_API->i2c_setup(LPC_I2C0_BASE, i2cMasterHandleMEM);
	if (i2cHandleMaster == NULL) {
		errorI2C();
	}

	/* Set I2C bitrate */
	if (LPC_I2CD_API->i2c_set_bitrate(i2cHandleMaster, Chip_Clock_GetSystemClockRate(),
									  I2C_BITRATE) != LPC_OK) {
		errorI2C();
	}
}

#if defined(INTERRUPTMODE)
/* I2C interrupt callback, called on completion of I2C operation when in
   interrupt mode. Called in interrupt context. */
static void cbI2CComplete(uint32_t err_code, uint32_t n)
{
	intErrCode = (int) err_code;
}

#endif

/* Master transmit in polling or interrupt mode */
static void sendI2CMaster(uint16_t AddressI2C, bool ledStateOut, bool address10Bit)
{
	uint8_t SendData[10];
	I2C_PARAM_T param;
	I2C_RESULT_T result;
	ErrorCode_t error_code;
	int index = 0;

	/* Setup I2C send for address/send, send desired LED state, then stop */
	if (address10Bit) {
		/* 10-bit addressing - 4 MSBs of slave address in first byte of
		   transmit buffer */
		SendData[index++] = (uint8_t) (((AddressI2C >> 7) & 0x06) | 0xF0);
		SendData[index++] = (uint8_t) (AddressI2C & 0x0FF);
	}
	else {
		/* 7-bit address */
		SendData[index++] = (uint8_t) AddressI2C;
	}
	SendData[index++] = (uint8_t) ledStateOut;

	/* Setup I2C paameters for number of bytes with stop - appears as follows on bus:
	   Start - address7 or address10upper - ack
	   (10 bits addressing only) address10lower - ack
	   value 1 - ack
	   value 2 - ack - stop */
	param.num_bytes_send    = index;
	param.buffer_ptr_send   = &SendData[0];
	param.num_bytes_rec     = 0;
	param.stop_flag         = 1;
#if defined(INTERRUPTMODE)
	param.func_pt           = cbI2CComplete;
#endif

	/* Set timeout (much) greater than the transfer length */
	LPC_I2CD_API->i2c_set_timeout(i2cHandleMaster, 100000);

	/* Do master write transfer */
#if defined(INTERRUPTMODE)
	intErrCode = -1;

	/* Function is non-blocking, returned error should be LPC_OK, but isn't checked here */
	error_code = LPC_I2CD_API->i2c_master_transmit_intr(i2cHandleMaster, &param, &result);

	/* Sleep until transfer is complete, but allow IRQ to wake system
	   to handle I2C IRQ */
	while (intErrCode == -1) {
		__WFI();
	}

	/* Cast saved error code from callback */
	error_code = (ErrorCode_t) intErrCode;

#else
	error_code = LPC_I2CD_API->i2c_master_transmit_poll(i2cHandleMaster, &param, &result);
#endif

	/* Completed without erors? */
	if (error_code != LPC_OK) {
		/* Likely cause is NAK */
		DEBUGOUT("i2c_master_transmit error code : %x\r\b", error_code);
		errorI2C();
	}

	/* Note results are only valid when there are no errors */
}

/* Master receive in polling or interrupt mode */
static void readI2CMaster(uint16_t AddressI2C, bool *ledStateIn, bool address10Bit)
{
	uint8_t recvData[10];
	I2C_PARAM_T param;
	I2C_RESULT_T result;
	ErrorCode_t error_code;
	int index = 0;

	/* Setup I2C receive for address/read, read desired LED state, then stop */
	if (address10Bit) {
		/* 10-bit addressing - 4 MSBs of slave address in first byte of
		   transmit buffer */
		recvData[index++] = (uint8_t) (((AddressI2C >> 7) & 0x06) | 0xF0);
		recvData[index++] = (uint8_t) (AddressI2C & 0x0FF);
	}
	else {
		/* 7-bit address */
		recvData[index++] = (uint8_t) AddressI2C;
	}

	/* Setup I2C paameters for number of bytes with stop - appears as follows on bus:
	   Start - address7 or address10upper - ack
	   (10 bits addressing only) address10lower - ack
	   value 1 (read) - ack
	   value 2 read) - ack - stop */
	param.num_bytes_send    = 0;
	param.num_bytes_rec     = 1;
	param.buffer_ptr_rec    = &recvData[0];
	param.stop_flag         = 1;
#if defined(INTERRUPTMODE)
	param.func_pt           = cbI2CComplete;
#endif

	/* Set timeout (much) greater than the transfer length */
	LPC_I2CD_API->i2c_set_timeout(i2cHandleMaster, 100000);

	/* Do master read transfer */
#if defined(INTERRUPTMODE)
	intErrCode = -1;

	/* Function is non-blocking, returned error should be LPC_OK, but isn't checked here */
	error_code = LPC_I2CD_API->i2c_master_receive_intr(i2cHandleMaster, &param, &result);

	/* Sleep until transfer is complete, but allow IRQ to wake system
	   to handle I2C IRQ */
	while (intErrCode == -1) {
		__WFI();
	}

	/* Cast saved error code from callback */
	error_code = (ErrorCode_t) intErrCode;

#else
	error_code = LPC_I2CD_API->i2c_master_receive_poll(i2cHandleMaster, &param, &result);
#endif

	/* Completed without erors? */
	if (error_code != LPC_OK) {
		/* Likely cause is NAK */
		DEBUGOUT("i2c_master_receive error code : %x\r\b", error_code);
		errorI2C();
	}

	/* Note results are only valid when there are no errors */
	*ledStateIn = (bool) recvData[1];
}

#if 0	/* Function not used in this example, provided for reference */
/* Master transmit/receive in polling or interrupt mode */
static void SendReadI2CMaster(uint16_t AddressI2C, bool ledStateOut, bool *ledStateIn, bool address10Bit)
{
	uint8_t recvData[10], sendData[10];
	I2C_PARAM_T param;
	I2C_RESULT_T result;
	ErrorCode_t error_code;
	int sindex = 0, rindex = 0;

	/* Setup I2C send for address/send, send desired LED state, then stop */
	/* Setup I2C receive for address/read, read desired LED state, then stop */
	if (address10Bit) {
		/* 10-bit addressing - 4 MSBs of slave address in first byte of
		   transmit buffer */
		sendData[sindex] = (uint8_t) (((AddressI2C >> 7) & 0x06) | 0xF0);
		recvData[rindex++] = sendData[sindex++];
		sendData[sindex] = (uint8_t) (AddressI2C & 0x0FF);
		recvData[rindex++] = sendData[sindex++];
	}
	else {
		/* 7-bit address */
		sendData[sindex] = (uint8_t) AddressI2C  | 0x01;
		recvData[rindex++] = sendData[sindex++];
	}
	sendData[sindex++] = (uint8_t) ledStateOut;

	/* Setup parameters for transfer */
	param.num_bytes_send    = sindex;
	param.num_bytes_rec     = rindex;
	param.buffer_ptr_send   = &sendData[0];
	param.buffer_ptr_rec    = &recvData[0];
	param.stop_flag         = 1;
#if defined(INTERRUPTMODE)
	param.func_pt           = cbI2CComplete;
#endif

	/* Set timeout (much) greater than the transfer length */
	LPC_I2CD_API->i2c_set_timeout(i2cHandleMaster, 100000);

	/* Do master read transfer */
#if defined(INTERRUPTMODE)
	intErrCode = -1;

	/* Function is non-blocking, returned error should be LPC_OK, but isn't checked here */
	error_code = LPC_I2CD_API->i2c_master_receive_intr(i2cHandleMaster, &param, &result);

	/* Sleep until transfer is complete, but allow IRQ to wake system
	   to handle I2C IRQ */
	while (intErrCode == -1) {
		__WFI();
	}

	/* Cast saved error code from callback */
	error_code = (ErrorCode_t) intErrCode;

#else
	error_code = LPC_I2CD_API->i2c_master_tx_rx_poll(i2cHandleMaster, &param, &result);
#endif

	/* Completed without erors? */
	if (error_code != LPC_OK) {
		/* Likely cause is NAK */
		DEBUGOUT("i2c_master_tx_rx error code : %x\r\b", error_code);
		errorI2C();
	}

	/* Note results are only valid when there are no errors */
	*ledStateIn = (bool) recvData[1];
}

#endif

/*****************************************************************************
 * Public functions
 ****************************************************************************/

#if defined(INTERRUPTMODE)
/**
 * @brief	I2C interrupt handler
 * @return	Nothing
 */
void I2C0_IRQHandler(void)
{
	/* Call I2C ISR function in ROM with the I2C handle */
	LPC_I2CD_API->i2c_isr_handler(i2cHandleMaster);
}

#endif

/**
 * @brief	Handle interrupt from SysTick timer
 * @return	Nothing
 */
void SysTick_Handler(void)
{
	static int ticks = 0;

	ticks++;
	if (ticks > TICKRATE_HZ) {
		ticks = 0;
		state = 1 - state;
	}
}

/**
 * @brief	Main routine for I2C example
 * @return	Function should not exit
 */
int main(void)
{
	bool ledState = false;
	int lastState = -1;

	/* Generic Initialization */
	SystemCoreClockUpdate();
	Board_Init();

	Board_LED_Set(0, false);

	/* Setup I2C pin muxing */
	Init_I2C_PinMux();

	/* Allocate I2C handle, setup I2C rate, and initialize I2C
	   clocking */
	setupI2CMaster();

#if defined(INTERRUPTMODE)
	/* Enable the interrupt for the I2C */
	NVIC_EnableIRQ(I2C0_IRQn);
#endif

	/* Enable SysTick Timer */
	SysTick_Config(SystemCoreClock / TICKRATE_HZ);

	/* Toggle LED on other board via I2C */
	while (1) {
		/* Sleep until a state change occurs in SysTick */
		while (lastState == state) {
			__WFI();
		}

		/* Handle states */
		switch (state) {
		case 0:
			/* Set LED state on other board */
			sendI2CMaster(I2C_ADDR_7BIT, ledState, false);
			break;

		case 1:
		default:
			/* Get LED state on other board */
			readI2CMaster(I2C_ADDR_7BIT, &ledState, false);
			break;
		}

		lastState = state;

		/* Match this board's LED to other boards state */
		Board_LED_Set(0, ledState);
	}
	return 0;
}
