/*
 * @brief ADC example using the ROM API
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

#define ADC_LINK_LIST							0
#define ADC_PING_PONG							0
#define ADC_SEQ_HW_TRIG_ENA       0

#if ADC_LINK_LIST
/* These two addresses are for link list test only. */
#define RELOAD_LINK_LIST_ADDR0	0x02008400
#define RELOAD_LINK_LIST_ADDR1	0x02008410
#endif

/* Use one of the unused DMA requests for ADC. */
#define DMAREQ_ADC0							RESERVED_SPARE_DMA

#define ADC_NUM										12

#define ADC_THR0_LOW_VALUE    0xFF
#define ADC_THR0_HIGH_VALUE	  0x300
#define ADC_THR1_LOW_VALUE    0xBF
#define ADC_THR1_HIGH_VALUE	  0x340

#define RAMBLOCK_H  		60
#define DMARAMBLOCK_H  	100
uint32_t  start_of_ram_block0[ RAMBLOCK_H ];
uint32_t  start_of_ram_block1[ RAMBLOCK_H ];
uint32_t  start_of_ram_block2[ DMARAMBLOCK_H ];

ADC_HANDLE_T*  adc_handle;
DMA_HANDLE_T*  dma_handle;

ADC_PARAM_T        param;
ADC_CONFIG_T       adc_cfg;
ADC_DMA_CFG_T      dma_cfg;

#define BUFFER_SIZE   100

uint32_t adc_buffer[BUFFER_SIZE];


volatile uint8_t completiona_tag;
volatile uint8_t completionb_tag;
volatile uint8_t seqa_burst_flag = 0;
volatile uint8_t seqb_burst_flag = 0;
ErrorCode_t error_code;

/*****************************************************************************
 * Private functions
 ****************************************************************************/
int init_adc_clk_setup( void)
{
	volatile int size_in_bytes;

	Chip_ADC_Init(LPC_ADC, 0);

	Chip_Clock_SetADCASYNCSource(SYSCTL_ADCASYNCCLKSRC_MAINCLK);
	Chip_Clock_SetADCASYNCDivider(0x1);

  size_in_bytes =  LPC_ADCD_API->adc_get_mem_size() ;
  if (	 RAMBLOCK_H	 < (size_in_bytes /4 ) ) {
    return 1;
  }

  /* Setup ADC0 driver */
	adc_handle = LPC_ADCD_API->adc_setup(LPC_ADC_BASE, (uint8_t *)start_of_ram_block0);
  return 0;

} /* init_adc_clk_setup */

int init_dma_clk_setup( void)
{
	volatile int size_in_bytes;
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

} /* init_dma_clk_setup */

/*****************************************************************************
 * Public functions
 ****************************************************************************/

void DMA_IRQHandler(void){
  LPC_DMAD_API->dma_isr(dma_handle);
}

/* dma adc callback is to setup ADC DMA for ADC SEQA conversion */
ErrorCode_t adc_seqa_dma_setup_callback ( ADC_HANDLE_T handle, ADC_DMA_CFG_T *dma_cfg )
{
    DMA_CHANNEL_T chn;
    DMA_TASK_T tsk;
		ErrorCode_t error_code;

	  LPC_ADC_T *adc = ((LPC_ADC_T *)((ADC_DRIVER_TypeDef *)handle)->base_addr);
	  ADC_DRIVER_TypeDef *driver = (ADC_DRIVER_TypeDef *)handle;

    chn.event = DMA_HD_REQ; //enable H/W trigger
		if ( adc->SEQA_CTRL & ADC_SEQ_CTRL_MODE_EOS )
      chn.hd_trigger = 0;
		else
      chn.hd_trigger = 0x1<<6;		/* Set trigger burst */
    chn.priority = 0;
    tsk.config = DMA_CLEAR_TRIGGER; // Trigger is cleared when descriptors are exhausted.
    tsk.config |= DMA_INT_A; //enable DMA interrupt for this channel

    tsk.data_length = driver->seqa_channel_num - 1;

		if (dma_cfg->dma_done_callback_pt != NULL){
			chn.callback_func_pt = dma_cfg->dma_done_callback_pt; //set callback function
		}

		tsk.ch_num = dma_cfg->dma_adc_num;
		/* Beware that, in this example, if end of sequence is used, DMA setup is the simplist case that
		assuming all ADC channels are selected for one sequence, the source ADC addresses are continuous.
		If some of the ADC channels are selected and some are not for one sequence, there are address gaps
		among ADC channels. A DMA linked descriptors will be needed to take care of this case. */
		if ( adc->SEQA_CTRL & ADC_SEQ_CTRL_MODE_EOS ) {
      tsk.data_type = DMA_32_BIT | DMA_SRC_INC_1 | DMA_DST_INC_1;			/* P2M */
      tsk.src = (uint32_t)&adc->DAT[tsk.data_length];
    }
		else {
      tsk.data_type = DMA_32_BIT | DMA_DST_INC_1;			/* P2M */
      tsk.src = (uint32_t)&adc->SEQA_GDAT;
    }
    tsk.dst = (uint32_t)driver->seqa_buffer + (tsk.data_length)*4;
    tsk.task_addr = (uint32_t) NULL; // for task head, no momery is needed.

    error_code = LPC_DMAD_API->dma_init((DMA_HANDLE_T*)dma_cfg->dma_handle, &chn, &tsk);
    if ( error_code )
    {
      return( error_code ); //init adc dma channel
    }
#if ADC_LINK_LIST
    /* If link list is used for ADC test. Two entries are created and two more conversion will
		be executed if ping pong mode flag is NOT set. If ping pong mode is enabled, the ADC
		conversion will continue as long as there is hardware trigger. */
    tsk.task_addr = (uint32_t)RELOAD_LINK_LIST_ADDR0;
    error_code = LPC_DMAD_API->dma_link((DMA_HANDLE_T*)dma_cfg->dma_handle, &tsk, 1);
    if ( error_code )
    {
      return( error_code ); //init adc dma channel
    }
#if ADC_PING_PONG
		tsk.config |= DMA_PING_PONG;
#endif
    tsk.task_addr = (uint32_t)RELOAD_LINK_LIST_ADDR1;
    error_code = LPC_DMAD_API->dma_link((DMA_HANDLE_T*)dma_cfg->dma_handle, &tsk, 1);
#endif
    return( error_code ); //init adc dma channel
}

/* dma adc callback is to setup ADC DMA for ADC SEQB conversion */
ErrorCode_t adc_seqb_dma_setup_callback ( ADC_HANDLE_T handle, ADC_DMA_CFG_T *dma_cfg )
{
    DMA_CHANNEL_T chn;
    DMA_TASK_T tsk;
		ErrorCode_t error_code;

	  LPC_ADC_T *adc = ((LPC_ADC_T *)((ADC_DRIVER_TypeDef *)handle)->base_addr);
	  ADC_DRIVER_TypeDef *driver = (ADC_DRIVER_TypeDef *)handle;

    chn.event = DMA_HD_REQ; //enable H/W trigger
		if ( adc->SEQB_CTRL & ADC_SEQ_CTRL_MODE_EOS )
      chn.hd_trigger = 0;
		else
      chn.hd_trigger = 0x1<<6;		/* Set trigger burst */
    chn.priority = 0;
    tsk.config = DMA_CLEAR_TRIGGER; // Trigger is cleared when descriptors are exhausted.
    tsk.config |= DMA_INT_A; //enable DMA interrupt for this channel

    tsk.data_length = driver->seqb_channel_num - 1;

		if (dma_cfg->dma_done_callback_pt != NULL){
			chn.callback_func_pt = dma_cfg->dma_done_callback_pt; //set callback function
		}

		tsk.ch_num = dma_cfg->dma_adc_num;
		/* Beware that, in this example, if end of sequence is used, DMA setup is the simplist case that
		assuming all ADC channels are selected for one sequence, the source ADC addresses are continuous.
		If some of the ADC channels are selected and some are not for one sequence, there are address gaps
		among ADC channels. A DMA linked descriptors will be needed to take care of this case. */
		if ( adc->SEQB_CTRL & ADC_SEQ_CTRL_MODE_EOS ) {
      tsk.data_type = DMA_32_BIT | DMA_SRC_INC_1 | DMA_DST_INC_1;			/* P2M */
      tsk.src = (uint32_t)&adc->DAT[tsk.data_length];
    }
		else {
      tsk.data_type = DMA_32_BIT | DMA_DST_INC_1;			/* P2M */
      tsk.src = (uint32_t)&adc->SEQB_GDAT;
    }
    tsk.dst = (uint32_t)driver->seqb_buffer + (tsk.data_length)*4;
    tsk.task_addr = (uint32_t) NULL; // for task head, no momery is needed.

    error_code = LPC_DMAD_API->dma_init((DMA_HANDLE_T*)dma_cfg->dma_handle, &chn, &tsk);
    return( error_code ); //init adc dma channel
}

/* When DMA is fimished, one of the receive or send callback should happen */
void  dma_seqa_complete_callback( uint32_t err_code, uint32_t n ) {
  if (err_code != LPC_OK)
    while(1);
	/* Once the DMA is completed, disable SEQx */
//	adcr->SEQA_CTRL &= ~( ADC_SEQ_ENA );	            /* stop ADC SEQx now. */
	completiona_tag = 1;
}

/* When DMA is fimished, one of the receive or send callback should happen */
void  dma_seqb_complete_callback( uint32_t err_code, uint32_t n ) {
  if (err_code != LPC_OK)
    while(1);
	/* Once the DMA is completed, disable SEQx */
//	adcr->SEQB_CTRL &= ~( ADC_SEQ_ENA );	            /* stop ADC SEQx now. */
	completionb_tag = 1;
}


/**
 * @brief	Main routine for I2C example
 * @return	Function should not exit
 */
int main(void)
{
	uint32_t i;
  ADC_CONFIG_T adc_set;

	/* Generic Initialization */
	SystemCoreClockUpdate();
	Board_Init();

	Board_LED_Set(0, false);

	/* Enable INMUX clock */
  Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_MUX);

  if (init_adc_clk_setup())
		return 1;

  if (init_dma_clk_setup())
		return 1;

  NVIC_DisableIRQ(ADC_THCMP_OVR_IRQn);
  NVIC_DisableIRQ(ADC_SEQA_IRQn);
  NVIC_DisableIRQ(ADC_SEQB_IRQn);

	adc_set.system_clock = SystemCoreClock;
	LPC_ADCD_API->adc_calibration(adc_handle, &adc_set);

	adc_set.adc_clock = ADC_MAX_SAMPLE_RATE;
	/* 12-bit, calibration will be applied during conversion(no-bypass), sync mode. */
	adc_set.adc_ctrl = ADC_CR_RESOL(ADC_RESOL_12BIT);
	adc_set.thr0_low = ADC_THR0_LOW_VALUE;
	adc_set.thr0_high = ADC_THR0_HIGH_VALUE;
	adc_set.thr1_low = ADC_THR1_LOW_VALUE;
	adc_set.thr1_high = ADC_THR1_HIGH_VALUE;
	adc_set.error_en = ADC_INTEN_OVRRUN_ENABLE;
#if 0
  /* Be aware that if ADC is running at very high speed while all interrpts are enabled, it may cause
	overrun error due to the interrupt latency. */
	adc_set.thcmp_en = ADC_INTEN_CMP_ENABLE(1, 0) | ADC_INTEN_CMP_ENABLE(1, 1) | ADC_INTEN_CMP_ENABLE(1, 2) | ADC_INTEN_CMP_ENABLE(1, 3)
		| ADC_INTEN_CMP_ENABLE(1, 4) | ADC_INTEN_CMP_ENABLE(1, 5) | ADC_INTEN_CMP_ENABLE(1, 6) | ADC_INTEN_CMP_ENABLE(1, 7)
		| ADC_INTEN_CMP_ENABLE(1, 8) | ADC_INTEN_CMP_ENABLE(1, 9) | ADC_INTEN_CMP_ENABLE(1, 10) | ADC_INTEN_CMP_ENABLE(1, 11);
#endif

	LPC_ADCD_API->adc_init(adc_handle, &adc_set);

  /* All pins to inactive, neither pull-up nor pull-down. */
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 29, IOCON_MODE_INACT | IOCON_FUNC1 | IOCON_ANALOG_EN);
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 30, IOCON_MODE_INACT | IOCON_FUNC1 | IOCON_ANALOG_EN);
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 31, IOCON_MODE_INACT | IOCON_FUNC1 | IOCON_ANALOG_EN);
	Chip_IOCON_PinMuxSet(LPC_IOCON, 1, 0, IOCON_MODE_INACT | IOCON_FUNC1 | IOCON_ANALOG_EN);
	Chip_IOCON_PinMuxSet(LPC_IOCON, 1, 1, IOCON_MODE_INACT | IOCON_FUNC1 | IOCON_ANALOG_EN);
	Chip_IOCON_PinMuxSet(LPC_IOCON, 1, 2, IOCON_MODE_INACT | IOCON_FUNC1 | IOCON_ANALOG_EN);
	Chip_IOCON_PinMuxSet(LPC_IOCON, 1, 3, IOCON_MODE_INACT | IOCON_FUNC1 | IOCON_ANALOG_EN);
	Chip_IOCON_PinMuxSet(LPC_IOCON, 1, 4, IOCON_MODE_INACT | IOCON_FUNC1 | IOCON_ANALOG_EN);
	Chip_IOCON_PinMuxSet(LPC_IOCON, 1, 5, IOCON_MODE_INACT | IOCON_FUNC1 | IOCON_ANALOG_EN);
	Chip_IOCON_PinMuxSet(LPC_IOCON, 1, 6, IOCON_MODE_INACT | IOCON_FUNC1 | IOCON_ANALOG_EN);
	Chip_IOCON_PinMuxSet(LPC_IOCON, 1, 7, IOCON_MODE_INACT | IOCON_FUNC1 | IOCON_ANALOG_EN);
	Chip_IOCON_PinMuxSet(LPC_IOCON, 1, 8, IOCON_MODE_INACT | IOCON_FUNC1 | IOCON_ANALOG_EN);
	

  for ( i = 0; i < BUFFER_SIZE; i++ ) {
		adc_buffer[i] = 0x00;
	}

  //test DMA mode
  NVIC_DisableIRQ(DMA_IRQn);
  NVIC_ClearPendingIRQ(DMA_IRQn);
  NVIC_EnableIRQ(DMA_IRQn);

#if 1
  //test ADC DMA using SEQA, in interrupt mode only

  /* This setting will be passed to SEQx_CTRL register, make sure bit 31 (SEQx_ENA) and bit 26 (START)
	shouldn't be set. It's controlled inside the ROM API. */
	adc_cfg.seqa_ctrl = ADC_SEQ_CTRL_CHANSEL_BITPOS(0xFFF) | ADC_SEQ_CTRL_MODE_EOS;
	adc_cfg.channel_num = ADC_NUM;

	param.driver_mode = 0x02; //DMA mode.
	param.seqa_hwtrig = 0x0;
	param.adc_cfg = (ADC_CONFIG_T *)&adc_cfg;
	param.buffer = (uint32_t *)adc_buffer;

	dma_cfg.dma_adc_num = RESERVED_SPARE_DMA;
	dma_cfg.dma_pinmux_num = INMUX_ADC0_SEQA_DMA;
	Chip_INMUX_Config_ITRIG_DMA(LPC_INMUX, RESERVED_SPARE_DMA, INMUX_ADC0_SEQA_DMA );

	dma_cfg.dma_done_callback_pt = dma_seqa_complete_callback;
	dma_cfg.dma_handle  = (uint32_t)dma_handle;
	param.dma_cfg       = &dma_cfg;
  param.dma_setup_func_pt = adc_seqa_dma_setup_callback;

	param.comp_flags = 0;
	completiona_tag = 0;
	LPC_ADCD_API->adc_seqa_read(adc_handle, &param);
	while(!completiona_tag);
#endif

#if 0
  //test ADC DMA using SEQA, end of conversion, in interrupt mode only

  /* This setting will be passed to SEQx_CTRL register, make sure bit 31 (SEQx_ENA) and bit 26 (START)
	shouldn't be set. It's controlled inside the ROM API. */
	adc_cfg.seqa_ctrl = ADC_SEQ_CTRL_CHANSEL_BITPOS(0xFFF);
	adc_cfg.channel_num = ADC_NUM;

	param.driver_mode = 0x02; //DMA mode.
	param.seqa_hwtrig = 0x0;
	param.adc_cfg = (ADC_CONFIG_T *)&adc_cfg;
	param.buffer = (uint32_t *)adc_buffer;

	dma_cfg.dma_adc_num = DMAREQ_ADC0;
	dma_cfg.dma_pinmux_num = INMUX_ADC0_SEQA_DMA;
	Chip_INMUX_Config_ITRIG_DMA(LPC_INMUX, DMAREQ_ADC0, INMUX_ADC0_SEQA_DMA );

	dma_cfg.dma_done_callback_pt = dma_seqa_complete_callback;
	dma_cfg.dma_handle  = (uint32_t)dma_handle;
	param.dma_cfg       = &dma_cfg;
  param.dma_setup_func_pt = adc_seqa_dma_setup_callback;

	param.comp_flags = 0;
	completiona_tag = 0;
	LPC_ADCD_API->adc_seqa_read(adc_handle, &param);
	while(!completiona_tag);
#endif

#if 0
  //test ADC DMA using SEQB, in interrupt mode only

  /* This setting will be passed to SEQx_CTRL register, make sure bit 31 (SEQx_ENA) and bit 26 (START)
	shouldn't be set. It's controlled inside the ROM API. */
	adc_cfg.seqb_ctrl = ADC_SEQ_CTRL_CHANSEL_BITPOS(0xFFF) | ADC_SEQ_CTRL_MODE_EOS;
	adc_cfg.channel_num = ADC_NUM;

	param.driver_mode = 0x02; //DMA mode.
	param.seqa_hwtrig = 0x0;
	param.adc_cfg = (ADC_CONFIG_T *)&adc_cfg;
	param.buffer = (uint32_t *)adc_buffer;

	dma_cfg.dma_adc_num = DMAREQ_ADC0;
	dma_cfg.dma_pinmux_num = INMUX_ADC0_SEQB_DMA;
	Chip_INMUX_Config_ITRIG_DMA(LPC_INMUX, DMAREQ_ADC0, INMUX_ADC0_SEQB_DMA );

	dma_cfg.dma_done_callback_pt = dma_seqb_complete_callback;
	dma_cfg.dma_handle  = (uint32_t)dma_handle;
	param.dma_cfg       = &dma_cfg;
  param.dma_setup_func_pt = adc_seqb_dma_setup_callback;

	param.comp_flags = 0;
	completionb_tag = 0;
	LPC_ADCD_API->adc_seqb_read(adc_handle, &param);
	while(!completionb_tag);
#endif

#if 0
  //test ADC DMA using SEQB, end of conversion, in interrupt mode only

  /* This setting will be passed to SEQx_CTRL register, make sure bit 31 (SEQx_ENA) and bit 26 (START)
	shouldn't be set. It's controlled inside the ROM API. */
	adc_cfg.seqb_ctrl = ADC_SEQ_CTRL_CHANSEL_BITPOS(0xFFF);
	adc_cfg.channel_num = ADC_NUM;

	param.driver_mode = 0x02; //DMA mode.
	param.seqb_hwtrig = 0x0;
	param.adc_cfg = (ADC_CONFIG_T *)&adc_cfg;
	param.buffer = (uint32_t *)adc_buffer;

	dma_cfg.dma_adc_num = DMAREQ_ADC0;
	dma_cfg.dma_pinmux_num = INMUX_ADC0_SEQB_DMA;
	Chip_INMUX_Config_ITRIG_DMA(LPC_INMUX, DMAREQ_ADC0, INMUX_ADC0_SEQB_DMA );

	dma_cfg.dma_done_callback_pt = dma_seqb_complete_callback;
	dma_cfg.dma_handle  = (uint32_t)dma_handle;
	param.dma_cfg       = &dma_cfg;
  param.dma_setup_func_pt = adc_seqb_dma_setup_callback;

	param.comp_flags = 0;
	completionb_tag = 0;
	LPC_ADCD_API->adc_seqb_read(adc_handle, &param);
	while(!completionb_tag);
#endif

#if 0
  //test ADC DMA using hardware trigger, in interrupt mode only

  /* This setting will be passed to SEQx_CTRL register, make sure bit 31 (SEQx_ENA) and bit 26 (START)
	shouldn't be set. It's controlled inside the ROM API. */
	adc_cfg.seqa_ctrl = ADC_SEQ_CTRL_CHANSEL_BITPOS(0xFFF) | ADC_SEQ_CTRL_TRIGGER(0x0) | ADC_SEQ_CTRL_MODE_EOS;
	adc_cfg.channel_num = ADC_NUM;

	param.driver_mode = 0x02; //DMA mode.
	param.seqa_hwtrig = 0x1;
	param.adc_cfg = (ADC_CONFIG_T *)&adc_cfg;
	param.buffer = (uint32_t *)adc_buffer;

	dma_cfg.dma_adc_num = DMAREQ_ADC0;
	dma_cfg.dma_pinmux_num = INMUX_ADC0_SEQA_DMA;
	Chip_INMUX_Config_ITRIG_DMA(LPC_INMUX, DMAREQ_ADC0, INMUX_ADC0_SEQA_DMA );

	dma_cfg.dma_done_callback_pt = dma_seqa_complete_callback;
	dma_cfg.dma_handle  = (uint32_t)dma_handle;
	param.dma_cfg       = &dma_cfg;
  param.dma_setup_func_pt = adc_seqa_dma_setup_callback;

	param.comp_flags = 0;
	completiona_tag = 0;
	LPC_ADCD_API->adc_seqa_read(adc_handle, &param);
	while(!completiona_tag);
#endif

#if 0
  //test ADC DMA using hardware trigger, end of conversion, in interrupt mode only

  /* This setting will be passed to SEQx_CTRL register, make sure bit 31 (SEQx_ENA) and bit 26 (START)
	shouldn't be set. It's controlled inside the ROM API. */
	adc_cfg.seqa_ctrl = ADC_SEQ_CTRL_CHANSEL_BITPOS(0xFFF) | ADC_SEQ_CTRL_TRIGGER(0x0);
	adc_cfg.channel_num = ADC_NUM;

	param.driver_mode = 0x02; //DMA mode.
	param.seqa_hwtrig = 0x1;
	param.adc_cfg = (ADC_CONFIG_T *)&adc_cfg;
	param.buffer = (uint32_t *)adc_buffer;

	dma_cfg.dma_adc_num = DMAREQ_ADC0;
	dma_cfg.dma_pinmux_num = INMUX_ADC0_SEQA_DMA;
	Chip_INMUX_Config_ITRIG_DMA(LPC_INMUX, DMAREQ_ADC0, INMUX_ADC0_SEQA_DMA );

	dma_cfg.dma_done_callback_pt = dma_seqa_complete_callback;
	dma_cfg.dma_handle  = (uint32_t)dma_handle;
	param.dma_cfg       = &dma_cfg;
  param.dma_setup_func_pt = adc_seqa_dma_setup_callback;

	param.comp_flags = 0;
	completiona_tag = 0;
	LPC_ADCD_API->adc_seqa_read(adc_handle, &param);
	while(!completiona_tag);
#endif

  while (1) {
	  __WFI();
  }
	return 0;
}
