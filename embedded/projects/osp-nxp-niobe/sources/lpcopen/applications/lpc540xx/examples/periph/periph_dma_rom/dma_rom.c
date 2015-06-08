/*
 * @brief DMA example using the ROM API
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
#define DMARAMBLOCK_H		200
#define BUFFER_SIZE   	100

uint32_t src_buffer[BUFFER_SIZE];
uint32_t dst_buffer[BUFFER_SIZE];

volatile int size_in_bytes;
ErrorCode_t error_code;
uint32_t int_count = 0;
volatile uint32_t done_tag;

uint32_t  start_of_ram_block0[ DMARAMBLOCK_H ] ;
DMA_HANDLE_T*  dma_handle;
DMA_CHANNEL_T chn;
DMA_TASK_T tsk;

/*****************************************************************************
 * Private functions
 *****************************************************************************/
int init_dma_clk_setup( void)
{
  uint32_t *align_location;

	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_DMA);
	Chip_SYSCTL_PeriphReset(RESET_DMA);

  size_in_bytes =  LPC_DMAD_API->dma_get_mem_size();
  if (	 DMARAMBLOCK_H	 < (size_in_bytes /4 ) ) {
    return 1;
  }

  /* Setup DMA driver */
	align_location = (uint32_t *)0x02008000;
	dma_handle = LPC_DMAD_API->dma_setup(LPC_DMA_BASE, (uint8_t *)align_location);
  return 0;

} /* init_dma */

/*****************************************************************************
 * Public functions
 ****************************************************************************/
void DMA_IRQHandler(void){
  LPC_DMAD_API->dma_isr(dma_handle);
	int_count++;
}

/* dma callback is invoked when DMA transfer is completed. */
void dma_callback( uint32_t err_code, uint32_t n )
{
  uint32_t i;

  if (err_code != LPC_OK)
    while(1);
	for ( i = 0; i < BUFFER_SIZE; i++ ) {
    if ( src_buffer[i] != dst_buffer[i] )
			while ( 1 );
  }
  done_tag = 1;
}


/**
 * @brief	Main routine for I2C example
 * @return	Function should not exit
 */
int main(void)
{
	uint32_t i;

	/* Setup SystemCoreClock and any needed board code */
	SystemCoreClockUpdate();
	Board_Init();

  if (init_dma_clk_setup())
		return 1;

  for ( i = 0; i < BUFFER_SIZE; i++ ) {
		src_buffer[i] = i;
		dst_buffer[i] = 0x00;
	}

  //test DMA mode
  NVIC_DisableIRQ(DMA_IRQn);
  NVIC_ClearPendingIRQ(DMA_IRQn);
  NVIC_EnableIRQ(DMA_IRQn);

  // Test M2M 8-bit transfer
  done_tag = 0;

  // set channel
  chn.event = DMA_SW_REQ;
  chn.hd_trigger = 0;
  chn.priority = 0;
  chn.callback_func_pt = dma_callback;

	// set task
  tsk.ch_num = 0;
  tsk.config = DMA_XFERCFG_SWTRIG | DMA_XFERCFG_SETINTA;
  tsk.data_type = DMA_8_BIT | DMA_SRC_INC_1 | DMA_DST_INC_1;
  tsk.data_length = BUFFER_SIZE * 4;     /* Multiple of 4 because the buffer is 32-bit width */
  tsk.src = (uint32_t)src_buffer + (tsk.data_length) * (0x1<<DMA_8_BIT);
  tsk.dst = (uint32_t)dst_buffer + (tsk.data_length) * (0x1<<DMA_8_BIT);
  tsk.task_addr = (uint32_t)NULL; //for task head, no memory is needed here.
  error_code = LPC_DMAD_API->dma_init(dma_handle, &chn, &tsk);
	while ( done_tag == 0 );

  // Test M2M 32-bit transfer
  done_tag = 0;
  /* Clear destination buffer */
  for ( i = 0; i < BUFFER_SIZE; i++ ) {
		dst_buffer[i] = 0x00;
	}

  // set channel
  chn.event = DMA_SW_REQ;
  chn.hd_trigger = 0;
  chn.priority = 0;
  chn.callback_func_pt = dma_callback;

	// set task
  tsk.ch_num = 0;
  tsk.config = DMA_SW_ON | DMA_INT_A;
  tsk.data_type = DMA_32_BIT | DMA_SRC_INC_1 | DMA_DST_INC_1;
  tsk.data_length = BUFFER_SIZE;
  tsk.src = (uint32_t)src_buffer + (tsk.data_length) * (0x1<<DMA_32_BIT);
  tsk.dst = (uint32_t)dst_buffer + (tsk.data_length) * (0x1<<DMA_32_BIT);
  tsk.task_addr = (uint32_t) NULL; //for task head, no memory is needed here.
  error_code = LPC_DMAD_API->dma_init(dma_handle, &chn, &tsk);
	while ( done_tag == 0 );

  while (1) {
	  __WFI();
  }
	return 0;
}//main




