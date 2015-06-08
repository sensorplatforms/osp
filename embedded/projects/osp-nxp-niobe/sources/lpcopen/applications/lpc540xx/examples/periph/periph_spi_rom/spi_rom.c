/*
 * @brief SPI example using the ROM API
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
 * Public types/enumerations/variables
 ****************************************************************************/

#define MASTER_MODE			1

#define RAMBLOCK_H  		20
uint32_t  start_of_ram_block0[ RAMBLOCK_H ] ;
uint32_t  start_of_ram_block1[ RAMBLOCK_H ] ;

SPI_HANDLE_T*  spi_handle_0 ;
SPI_HANDLE_T*  spi_handle_1 ;
SPI_HANDLE_T*  spi_handle;

SPI_PARAM_T   param;

#define BUFFER_SIZE   100
#define MASTER_FRAME_SIZE		(8-1)
#define SLAVE_FRAME_SIZE		(8-1)

uint16_t tx_buffer[BUFFER_SIZE];
uint16_t rx_buffer[BUFFER_SIZE];

volatile uint8_t send_tag;
volatile uint8_t receive_tag;
ErrorCode_t error_code;

/*****************************************************************************
 * Private functions
 *****************************************************************************/
int init_spi( void)
{
	volatile int size_in_bytes;

	/* Enable SPI clock in ASYNC SYSCON */
	Chip_Clock_EnableAsyncPeriphClock(ASYNC_SYSCTL_CLOCK_SPI0);
	Chip_Clock_EnableAsyncPeriphClock(ASYNC_SYSCTL_CLOCK_SPI1);
	Chip_SYSCTL_AsyncPeriphReset(ASYNC_RESET_SPI0);
	Chip_SYSCTL_AsyncPeriphReset(ASYNC_RESET_SPI1);

  size_in_bytes =  LPC_SPID_API->spi_get_mem_size();
  if (	 RAMBLOCK_H	 < (size_in_bytes /4 ) ) {
    return 1;
  }

  /* Setup Spi0 driver */
	spi_handle_0 = LPC_SPID_API->spi_setup(LPC_SPI0_BASE, (uint8_t *)start_of_ram_block0);
  /* Setup Spi1 driver */
	spi_handle_1 = LPC_SPID_API->spi_setup(LPC_SPI1_BASE, (uint8_t *)start_of_ram_block1);
  return 0;

} /* init_spi */

/*****************************************************************************
 * Public functions
 ****************************************************************************/

void SPI0_IRQHandler(void){
  LPC_SPID_API->spi_isr(spi_handle);
}

void SPI1_IRQHandler(void){
  LPC_SPID_API->spi_isr(spi_handle);
}

void  receive_callback(ErrorCode_t err_code, uint32_t n ) {
	error_code = (ErrorCode_t)err_code;
  if (err_code != LPC_OK)
    while(1);
	receive_tag = 1;
}

void  send_callback(uint32_t err_code, uint32_t n ) {
	error_code = (ErrorCode_t)err_code;
  if (err_code != LPC_OK)
    while(1);
	send_tag = 1;
}

/**
 * @brief	Main routine for I2C example
 * @return	Function should not exit
 */
int main(void)
{
  uint32_t i;
  SPI_CONFIG_T spi_set;

	/* Generic Initialization */
	SystemCoreClockUpdate();
	Board_Init();

	Board_LED_Set(0, false);

  if (init_spi())
		return 1;

	spi_set.delay = SPI_DLY_PRE_DELAY(0x0)|SPI_DLY_POST_DELAY(0x0)|SPI_DLY_FRAME_DELAY(0x0)|SPI_DLY_TRANSFER_DELAY(0x0);
	spi_set.divider = 0xFFFF;
#if MASTER_MODE
	spi_set.config = SPI_CFG_MASTER_EN;
#else
	spi_set.config = SPI_CFG_SLAVE_EN;
#endif
	spi_set.error_en = SPI_STAT_RXOV | SPI_STAT_TXUR;

	LPC_SPID_API->spi_init(spi_handle_0, &spi_set);
	LPC_SPID_API->spi_init(spi_handle_1, &spi_set);

#if defined(BOARD_NXP_LPCXPRESSO_54000)
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 14, IOCON_MODE_PULLUP | IOCON_FUNC1 | IOCON_DIGITAL_EN | IOCON_INPFILT_OFF); /* SSEL */
	Chip_IOCON_PinMuxSet(LPC_IOCON, 1, 3, IOCON_MODE_PULLUP | IOCON_FUNC5 | IOCON_DIGITAL_EN | IOCON_INPFILT_OFF); /* SCK */
	Chip_IOCON_PinMuxSet(LPC_IOCON, 1, 4, IOCON_MODE_PULLUP | IOCON_FUNC5 | IOCON_DIGITAL_EN | IOCON_INPFILT_OFF); /* MISO */
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 12, IOCON_MODE_PULLUP | IOCON_FUNC1 | IOCON_DIGITAL_EN | IOCON_INPFILT_OFF); /* MOSI */
#else
	/* Configure your own SPI pin muxing here if needed */
#warning No SPI pin muxing defined
#endif
  spi_handle = spi_handle_0;

  for ( i = 0; i < BUFFER_SIZE; i++ ) {
		tx_buffer[i] = 0x30+i;
		rx_buffer[i] = 0x00;
	}

#if 0
  //test polling mode
  param.driver_mode = 0; //polling mode
  param.tx_rx_flag  = 0x00;	// 0x00 TX only, 0x01, RX only, 0x02 TX AND RX
  param.fsize_sel = SPI_TXCTL_DEASSERTNUM_SSEL(SLAVE0) | SPI_TXCTL_FLEN(MASTER_FRAME_SIZE);  
	param.tx_buffer = (uint16_t *)tx_buffer;
	param.rx_buffer = (uint16_t *)rx_buffer;
	param.eof_flag  = 1;
	param.size = 10; //Max of buffer
	LPC_SPID_API->spi_master_transfer(spi_handle, &param);
#endif

#if 0
  //test polling mode
  param.driver_mode = 0; //polling mode
  param.tx_rx_flag  = 0x01;	// 0x00 TX only, 0x01, RX only, 0x02 TX AND RX
  param.fsize_sel = SPI_TXCTL_SSELN(SLAVE0) | SPI_TXCTL_FLEN(MASTER_FRAME_SIZE);  
	param.tx_buffer = (uint16_t *)tx_buffer;
	param.rx_buffer = (uint16_t *)rx_buffer;
	param.eof_flag  = 1;
	param.size = 10; //Max of buffer
	LPC_SPID_API->spi_master_transfer(spi_handle, &param);
#endif

#if 0
  //test polling mode
  param.driver_mode = 0; //polling mode
  param.tx_rx_flag  = 0x02;	// 0x00 TX only, 0x01, RX only, 0x02 TX AND RX
  param.fsize_sel = SPI_TXCTL_SSELN(SLAVE0) | SPI_TXCTL_FLEN(MASTER_FRAME_SIZE);  
	param.tx_buffer = (uint16_t *)tx_buffer;
	param.rx_buffer = (uint16_t *)rx_buffer;
	param.eof_flag  = 1;
	param.size = 10; //Max of buffer
	LPC_SPID_API->spi_master_transfer(spi_handle, &param);
#endif

#if 0
  param.driver_mode = 0; //polling mode
  param.tx_rx_flag  = 0x00;	// 0x00 TX only, 0x01, RX only
  param.fsize_sel = SPI_TXCTL_FLEN(SLAVE_FRAME_SIZE);  
	param.tx_buffer = (uint16_t *)tx_buffer;
	param.size = 10; //Max of buffer
  LPC_SPID_API->spi_slave_transfer(spi_handle, &param);
#endif

#if 0
  param.driver_mode = 0; //polling mode
  param.tx_rx_flag  = 0x01;	// 0x00 TX only, 0x01, RX only
  param.fsize_sel = SPI_TXCTL_FLEN(SLAVE_FRAME_SIZE);  
	param.rx_buffer = (uint16_t *)rx_buffer;
	param.size = 10; //Max of buffer
  LPC_SPID_API->spi_slave_transfer(spi_handle, &param);
#endif

#if 0
  param.driver_mode = 0; //polling mode
  param.tx_rx_flag  = 0x02;	// 0x00 TX only, 0x01 RX only, 0x2 TX and RX
  param.fsize_sel = SPI_TXCTL_FLEN(SLAVE_FRAME_SIZE); 
	param.tx_buffer = (uint16_t *)tx_buffer;	
	param.rx_buffer = (uint16_t *)rx_buffer;
	param.size = 10; //Max of buffer
  LPC_SPID_API->spi_slave_transfer(spi_handle, &param);
#endif

  // test INT mode
  NVIC_DisableIRQ(SPI0_IRQn);
  NVIC_ClearPendingIRQ(SPI0_IRQn);
  NVIC_EnableIRQ(SPI0_IRQn);

  // test INT mode
  NVIC_DisableIRQ(SPI1_IRQn);
  NVIC_ClearPendingIRQ(SPI1_IRQn);
  NVIC_EnableIRQ(SPI1_IRQn);

#if 0
  param.driver_mode = 0x01; //interrupt mode.
  param.tx_rx_flag  = 0x00;	// 0x00 TX only, 0x01, RX only, 0x02 TX AND RX
  param.fsize_sel = SPI_TXCTL_SSELN(SLAVE0) | SPI_TXCTL_FLEN(MASTER_FRAME_SIZE);  
	param.tx_buffer = (uint16_t *)tx_buffer;
	param.rx_buffer = (uint16_t *)rx_buffer;
	param.eof_flag  = 1;		/* 0 or 1 only. if 0, the value of DLY_FRAMEDELAY doesn't apply. */
	param.size = 10; //Max of buffer
  param.cb = send_callback;
	send_tag = 0;
	LPC_SPID_API->spi_master_transfer(spi_handle, &param);
	while(!send_tag);
#endif

#if 0
  param.driver_mode = 0x01; //interrupt mode.
  param.tx_rx_flag  = 0x01;	// 0x00 TX only, 0x01, RX only, 0x02 TX AND RX
  param.fsize_sel = SPI_TXCTL_SSELN(SLAVE0) | SPI_TXCTL_FLEN(MASTER_FRAME_SIZE);  
	param.tx_buffer = (uint16_t *)tx_buffer;
	param.rx_buffer = (uint16_t *)rx_buffer;
	param.eof_flag  = 0;		/* 0 or 1 only. if 0, the value of DLY_FRAMEDELAY doesn't apply. */
	param.size = 10; //Max of buffer
  param.cb = receive_callback;
	receive_tag = 0;
	LPC_SPID_API->spi_master_transfer(spi_handle, &param);
	while(!receive_tag);
#endif

#if 1
  param.driver_mode = 0x01; //interrupt mode.
  param.tx_rx_flag  = 0x02;	// 0x00 TX only, 0x01, RX only, 0x02 TX AND RX
  param.fsize_sel = SPI_TXCTL_DEASSERTNUM_SSEL(SLAVE0) | SPI_TXCTL_FLEN(MASTER_FRAME_SIZE);  
	param.tx_buffer = (uint16_t *)tx_buffer;
	param.rx_buffer = (uint16_t *)rx_buffer;
	param.eof_flag  = 0;		/* 0 or 1 only. if 0, the value of DLY_FRAMEDELAY doesn't apply. */
	param.size = 10; //Max of buffer
  param.cb = receive_callback;
	receive_tag = 0;
	LPC_SPID_API->spi_master_transfer(spi_handle, &param);
	while(!receive_tag);
#endif

// Check and make sure MASTER_MODE is defined 0

#if 0
	/* Slave TX mode */
  param.driver_mode = 0x01; //interrupt mode.
  param.tx_rx_flag  = 0x00;	// 0x00 TX only, 0x01, RX only
  param.fsize_sel = SPI_TXCTL_FLEN(SLAVE_FRAME_SIZE);  
	param.tx_buffer = (uint16_t *)tx_buffer;
	param.size = 10; //Max of buffer
  param.cb = send_callback;
	send_tag = 0;
  LPC_SPID_API->spi_slave_transfer(spi_handle, &param);
	while(!send_tag);
#endif

#if 0
	/* Slave RX mode */
  param.driver_mode = 0x01; //interrupt mode.
  param.tx_rx_flag  = 0x01;	// 0x00 TX only, 0x01, RX only
  param.fsize_sel = SPI_TXCTL_FLEN(SLAVE_FRAME_SIZE); 
	param.rx_buffer = (uint16_t *)rx_buffer;
	param.size = 10; //Max of buffer
  param.cb = receive_callback;
	receive_tag = 0;
  LPC_SPID_API->spi_slave_transfer(spi_handle, &param);
	while(!receive_tag);
#endif

#if 0
	/* Slave TA and RX mode */
  param.driver_mode = 0x01; //interrupt mode.
  param.tx_rx_flag  = 0x02;	// 0x00 TX only, 0x01 RX only, 0x2 TX and RX
  param.fsize_sel = SPI_TXCTL_FLEN(SLAVE_FRAME_SIZE);
	param.tx_buffer = (uint16_t *)tx_buffer;
	param.rx_buffer = (uint16_t *)rx_buffer;
	param.size = 10; //Max of buffer
  param.cb = receive_callback;
	receive_tag = 0;
  LPC_SPID_API->spi_slave_transfer(spi_handle, &param);
	while(!receive_tag);
#endif

  while (1) {
		__WFI();
  }
  return 0;

}//main



