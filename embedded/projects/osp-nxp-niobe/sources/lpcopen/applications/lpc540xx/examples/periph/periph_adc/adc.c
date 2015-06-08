/*
 * @brief LPC412x ADC example
 *
 * @note
 * Copyright(C) NXP Semiconductors, 2013
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
#include <stdio.h>

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

#define TICKRATE_HZ (100)	/* 100 ticks per second */

#if defined(BOARD_NXP_LPCXPRESSO_54000)
#define BOARD_ADC_CH 12

#else
#warning "Using ADC channel 12 for this example, please select for your baoad"
#define BOARD_ADC_CH 12
#endif

volatile uint32_t channel_completion = 0;
volatile uint32_t seq_burst_flag = 0;

volatile uint32_t SEQAComplete, SEQBComplete, thresholdCrossed, overrun_error;
volatile uint32_t ADC_Data_Buffer[BOARD_ADC_CH];

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
 * @brief	Handle interrupt from ADC sequencer A
 * @return	Nothing
 */
void ADC0A_IRQHandler(void)
{
	uint32_t i, pending;
	uint32_t gdata_value, data_value, seq_ctrl_value, ch_offset;

	/* Get pending interrupts */
	pending = Chip_ADC_GetFlags(LPC_ADC);

	/* Sequence A completion interrupt */
	if (pending & ADC_FLAGS_SEQA_INT_MASK) {
		seq_ctrl_value = Chip_ADC_GetSequencerCtrl(LPC_ADC, ADC_SEQA_IDX);
		if ( seq_ctrl_value & ADC_SEQ_CTRL_MODE_EOS ) {
			/* End of sequence, get raw sample data for channels 0-11 */
			for (i = 0; i < BOARD_ADC_CH; i++) {
				if ( seq_ctrl_value & (0x1<<i) ) {
					if ( (data_value = Chip_ADC_GetDataReg(LPC_ADC, i)) & ADC_DR_DATAVALID ) {
						ADC_Data_Buffer[i] = ADC_DR_RESULT(data_value);
						channel_completion |= ( 0x1 << i );
					}
				}
			}
		}
		else {
			/* End of conversion, get raw sample data for channels 0-11 */
			gdata_value = Chip_ADC_GetGlobalDataReg(LPC_ADC, ADC_SEQA_IDX);
			if ( gdata_value & ADC_SEQ_GDAT_DATAVALID ) {
				ch_offset = (gdata_value & ADC_SEQ_GDAT_CHAN_MASK) >> ADC_SEQ_GDAT_CHAN_BITPOS;
				ADC_Data_Buffer[ch_offset] = ADC_DR_RESULT(gdata_value);
				channel_completion |= (0x1<<ch_offset);
			}
			if ( channel_completion != (seq_ctrl_value & 0xFFF) ) {
				/* Not all channels are completed. */
				if ( (seq_ctrl_value & ADC_SEQ_CTRL_SINGLESTEP) && !(seq_ctrl_value & ADC_SEQ_CTRL_BURST) ) {
					/* If SINGLE_STEP is set and BURST is not, this sequence needs to be restarted. */
					Chip_ADC_StartSequencer(LPC_ADC, ADC_SEQA_IDX);
				}
			}
		}

		/* IT'S VERY IMPORTANT THAT: If the BURST mode is enabled, sequence is completed,
		but BURST is still running, turn off the BURST bit first, when the next SEQx completed, the BURST
		finally stops, the completion flag can be set, then SEQx can be disabled. */
		if ( channel_completion == (seq_ctrl_value & 0xFFF) ) {
			if ( (seq_ctrl_value & ADC_SEQ_CTRL_BURST) && !seq_burst_flag ) {
				/* If the BURST is set and all conversion is done, reset the flag, it's important for
				End of Conversion mode. Otherwise, we don't know when the last channel has finished or not. */
				seq_burst_flag = 1;
				Chip_ADC_ClearSequencerBits(LPC_ADC, ADC_SEQA_IDX, ADC_SEQ_CTRL_BURST);	/* turn off BURST bit. */
			}
			else if ( !(seq_ctrl_value & ADC_SEQ_CTRL_BURST) && seq_burst_flag ) {
				/* This is in burst mode, interrupt has been serviced once, BURST bit has been cleared, the second time,
				ADC sequence can be stopped now. */
				Chip_ADC_DisableSequencer(LPC_ADC, ADC_SEQA_IDX);	              /* stop ADC SEQA now. */
				Chip_ADC_DisableInt(LPC_ADC, (ADC_INTEN_SEQA_ENABLE | ADC_INTEN_OVRRUN_ENABLE));
				/* If all channel flags have been set, a true completion of a sequence. */
				SEQAComplete = true;
			}
			else {
				/* This is in the non-BURST mode, when all channels are finished, stop ADC sequence. */
				Chip_ADC_DisableSequencer(LPC_ADC, ADC_SEQA_IDX);	              /* stop ADC SEQA now. */
				Chip_ADC_DisableInt(LPC_ADC, (ADC_INTEN_SEQA_ENABLE | ADC_INTEN_OVRRUN_ENABLE));
				/* If all channel flags have been set, a true completion of a sequence. */
				SEQAComplete = true;
			}
		}
	}
	/* Clear any pending interrupts */
	Chip_ADC_ClearFlags(LPC_ADC, pending);
}

/**
 * @brief	Handle interrupt from ADC sequencer B
 * @return	Nothing
 */
void ADC0B_IRQHandler(void)
{
	uint32_t i, pending;
	uint32_t gdata_value, data_value, seq_ctrl_value, ch_offset;

	/* Get pending interrupts */
	pending = Chip_ADC_GetFlags(LPC_ADC);

	/* Sequence A completion interrupt */
	if (pending & ADC_FLAGS_SEQB_INT_MASK) {
		seq_ctrl_value = Chip_ADC_GetSequencerCtrl(LPC_ADC, ADC_SEQB_IDX);
		if ( seq_ctrl_value & ADC_SEQ_CTRL_MODE_EOS ) {
			/* End of sequence, get raw sample data for channels 0-11 */
			for (i = 0; i < BOARD_ADC_CH; i++) {
				if ( seq_ctrl_value & (0x1<<i) ) {
					if ( (data_value = Chip_ADC_GetDataReg(LPC_ADC, i)) & ADC_DR_DATAVALID ) {
						ADC_Data_Buffer[i] = ADC_DR_RESULT(data_value);
						channel_completion |= ( 0x1 << i );
					}
				}
			}
		}
		else {
			/* End of conversion, get raw sample data for channels 0-11 */
			gdata_value = Chip_ADC_GetGlobalDataReg(LPC_ADC, ADC_SEQB_IDX);
			if ( gdata_value & ADC_SEQ_GDAT_DATAVALID ) {
				ch_offset = (gdata_value & ADC_SEQ_GDAT_CHAN_MASK) >> ADC_SEQ_GDAT_CHAN_BITPOS;
				ADC_Data_Buffer[ch_offset] = ADC_DR_RESULT(gdata_value);
				channel_completion |= (0x1<<ch_offset);
			}
			if ( channel_completion != (seq_ctrl_value & 0xFFF) ) {
				/* Not all channels are completed. */
				if ( (seq_ctrl_value & ADC_SEQ_CTRL_SINGLESTEP) && !(seq_ctrl_value & ADC_SEQ_CTRL_BURST) ) {
					/* If SINGLE_STEP is set and BURST is not, this sequence needs to be restarted. */
					Chip_ADC_StartSequencer(LPC_ADC, ADC_SEQB_IDX);
				}
			}
		}

		/* IT'S VERY IMPORTANT THAT: If the BURST mode is enabled, sequence is completed,
		but BURST is still running, turn off the BURST bit first, when the next SEQx completed, the BURST
		finally stops, the completion flag can be set, then SEQx can be disabled. */
		if ( channel_completion == (seq_ctrl_value & 0xFFF) ) {
			if ( (seq_ctrl_value & ADC_SEQ_CTRL_BURST) && !seq_burst_flag ) {
				/* If the BURST is set and all conversion is done, reset the flag, it's important for
				End of Conversion mode. Otherwise, we don't know when the last channel has finished or not. */
				seq_burst_flag = 1;
				Chip_ADC_ClearSequencerBits(LPC_ADC, ADC_SEQB_IDX, ADC_SEQ_CTRL_BURST);	/* turn off BURST bit. */
			}
			else if ( !(seq_ctrl_value & ADC_SEQ_CTRL_BURST) && seq_burst_flag ) {
				/* This is in burst mode, interrupt has been serviced once, BURST bit has been cleared, the second time,
				ADC sequence can be stopped now. */
				Chip_ADC_DisableSequencer(LPC_ADC, ADC_SEQB_IDX);	              /* stop ADC SEQA now. */
				Chip_ADC_DisableInt(LPC_ADC, (ADC_INTEN_SEQB_ENABLE | ADC_INTEN_OVRRUN_ENABLE));
				/* If all channel flags have been set, a true completion of a sequence. */
				SEQBComplete = true;
			}
			else {
				/* This is in the non-BURST mode, when all channels are finished, stop ADC sequence. */
				Chip_ADC_DisableSequencer(LPC_ADC, ADC_SEQB_IDX);	              /* stop ADC SEQA now. */
				Chip_ADC_DisableInt(LPC_ADC, (ADC_INTEN_SEQB_ENABLE | ADC_INTEN_OVRRUN_ENABLE));
				/* If all channel flags have been set, a true completion of a sequence. */
				SEQBComplete = true;
			}
		}
	}
	/* Clear any pending interrupts */
	Chip_ADC_ClearFlags(LPC_ADC, pending);
}

void ADC_THCMP_OVR_IRQHandler(void)
{
	uint32_t i, pending;

	/* Get pending interrupts */
	pending = Chip_ADC_GetFlags(LPC_ADC);

	for (i = 0; i < BOARD_ADC_CH; i++) {
		/* Threshold crossing interrupt on ADC input channel */
		if (pending & ADC_FLAGS_THCMP_MASK(i)) {
			thresholdCrossed |= ADC_FLAGS_THCMP_MASK(i);
		}
		if (pending & ADC_FLAGS_OVRRUN_MASK(i)) {
			overrun_error |= ADC_FLAGS_OVRRUN_MASK(i);
		}
	}
	/* Clear any pending interrupts */
	Chip_ADC_ClearFlags(LPC_ADC, pending);
}

/**
 * @brief	main routine for ADC example
 * @return	Function should not exit
 */
int main(void)
{
	uint32_t i;

	SystemCoreClockUpdate();
	Board_Init();

	/* Setup ADC for 12-bit mode and normal power */
	Chip_ADC_Init(LPC_ADC, ADC_CR_RESOL(ADC_RESOL_12BIT));

	Chip_Clock_SetADCASYNCSource(SYSCTL_ADCASYNCCLKSRC_MAINCLK);
	Chip_Clock_SetADCASYNCDivider(0x1);

	/* Need to do a calibration after initialization and trim */
	Chip_ADC_StartCalibration(LPC_ADC);
	
	/* Setup for maximum ADC clock rate */
	Chip_ADC_SetClockRate(LPC_ADC, ADC_MAX_SAMPLE_RATE);

	/* Setup sequencer A for all 12 ADC channels, EOS interrupt */
#if defined(BOARD_NXP_LPCXPRESSO_54000)
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
	
#else
#warning "No ADC setup for this example"
#endif

	/* Clear all error flags */
	SEQAComplete = SEQBComplete = false;
	thresholdCrossed = overrun_error = 0;

	/* Setup threshold 0 low and high values to about 25% and 75% of max */
	Chip_ADC_SetThrLowValue(LPC_ADC, 0, ((1 * 0xFFF) / 4));
	Chip_ADC_SetThrHighValue(LPC_ADC, 0, ((3 * 0xFFF) / 4));

	/* Clear all pending interrupts */
	Chip_ADC_ClearFlags(LPC_ADC, Chip_ADC_GetFlags(LPC_ADC));

	/* Use threshold 0 for ADC channel and enable threshold interrupt mode for
	   channel as crossing */
	Chip_ADC_SelectTH0Channels(LPC_ADC, ADC_THRSEL_CHAN_SEL_THR1(BOARD_ADC_CH));
	Chip_ADC_SetThresholdInt(LPC_ADC, BOARD_ADC_CH, ADC_INTEN_THCMP_CROSSING);

	/* Enable ADC NVIC interrupt */
	NVIC_EnableIRQ(ADC_SEQA_IRQn);

	/* Regardless in BURST or non-BURST mode, once all ADC chamnels have been completed,
	the sequence and associated interrupt will be disabled. Re-enable the sequence is needed. */
	/* Enable ADC overrun and sequence A completion interrupts */
#if 1
	Chip_ADC_EnableInt(LPC_ADC, (ADC_INTEN_SEQA_ENABLE | ADC_INTEN_OVRRUN_ENABLE));
	Chip_ADC_SetupSequencer(LPC_ADC, ADC_SEQA_IDX, 0xFFF);
	Chip_ADC_EnableSequencer(LPC_ADC, ADC_SEQA_IDX);
	/* Manual start for ADC conversion sequence A using End of Conversion */
	Chip_ADC_StartSequencer(LPC_ADC, ADC_SEQA_IDX);
#endif

#if 0
	Chip_ADC_EnableInt(LPC_ADC, (ADC_INTEN_SEQA_ENABLE | ADC_INTEN_OVRRUN_ENABLE));
	Chip_ADC_SetupSequencer(LPC_ADC, ADC_SEQA_IDX, 0xFFF|ADC_SEQ_CTRL_BURST);
	Chip_ADC_EnableSequencer(LPC_ADC, ADC_SEQA_IDX);
	/* Manual start for ADC conversion sequence A using End of Conversion with BURST bit set. */
	Chip_ADC_StartBurstSequencer(LPC_ADC, ADC_SEQA_IDX);
#endif

#if 0
	Chip_ADC_EnableInt(LPC_ADC, (ADC_INTEN_SEQA_ENABLE | ADC_INTEN_OVRRUN_ENABLE));
	Chip_ADC_SetupSequencer(LPC_ADC, ADC_SEQA_IDX, 0xFFF|ADC_SEQ_CTRL_MODE_EOS);
	Chip_ADC_EnableSequencer(LPC_ADC, ADC_SEQA_IDX);
	/* Manual start for ADC conversion sequence A using End of Sequence */
	Chip_ADC_StartSequencer(LPC_ADC, ADC_SEQA_IDX);
#endif

#if 0
	Chip_ADC_EnableInt(LPC_ADC, (ADC_INTEN_SEQA_ENABLE | ADC_INTEN_OVRRUN_ENABLE));
	Chip_ADC_SetupSequencer(LPC_ADC, ADC_SEQA_IDX, 0xFFF|ADC_SEQ_CTRL_MODE_EOS|ADC_SEQ_CTRL_BURST);
	Chip_ADC_EnableSequencer(LPC_ADC, ADC_SEQA_IDX);
	/* Manual start for ADC conversion sequence A using End of Sequence with BURST bit set. */
	Chip_ADC_StartBurstSequencer(LPC_ADC, ADC_SEQA_IDX);
#endif

	if (thresholdCrossed) {
		thresholdCrossed = 0x0;
		DEBUGSTR("********ADC threshold event********\r\n");
	}

	if (overrun_error) {
		overrun_error = 0x0;
		DEBUGSTR("********ADC Overrun event********\r\n");
	}

	/* Is a conversion sequence complete? */
	if (SEQAComplete) {
		SEQAComplete = false;
	}

	/* Get raw sample data for channels 0-11 */
	for (i = 0; i < BOARD_ADC_CH; i++) {
		DEBUGOUT("Sample value = 0x%x\r\n", ADC_Data_Buffer[i]);
	}

	while (1) {
		__WFI();
	}
	return 0;
}
