/* mbed Microcontroller Library
 *******************************************************************************
 * Copyright (c) 2014, STMicroelectronics
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of STMicroelectronics nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************
 */
#include "rtc_api.h"

static uint32_t calWdtFreq, upperOfLimit;

static uint32_t volatile wdtOCount;
static uint8_t volatile timer40HiByte;

/* Maximum WDT timeout value */
#define WDTMAXTIMEOUTVAL	0x00FFFFFF

/* WDT window size */
#define WDTWINDOWSIZE			512

/* WDT timeout value used for this timer. Must be smaller than
   (WDTMAXTIMEOUTVAL + window size) */
#define WDTTIMEOUTVAL (WDTMAXTIMEOUTVAL - (WDTWINDOWSIZE + 1))
/* Enable this define to lower the kernel timer resolution by the shift
   value. Use 1 for /2, 2 for /4, 3 for /8, etc. This won't affect how the
	 kernel timer works. Use 0 to disable. Maximum is 8. */
#define KERNELTIMERSHIFT	0

/**
 * @brief	watchdog timer Interrupt Handler
 * @return	Nothing
 * @note	Handles watchdog timer feed and overflow count
 */
void WDT_IRQHandler(void)
{
	/* A watchdog feed didn't occur prior to warning timeout */
	Chip_WWDT_ClearStatusFlag(LPC_WWDT, (WWDT_WDMOD_WDINT | WWDT_WDMOD_WDTOF));

	/* Feed WDT or reset will occur */
	Chip_WWDT_Feed(LPC_WWDT);

	/* Will fire every WDT timeout */
	wdtOCount++;
	if (wdtOCount > upperOfLimit) {
		wdtOCount = 0;
		timer40HiByte++;
	}
}

/* Returns current WDT count (modified to 32-bit size) based on
   calibrated WDT rate (calWdtFreq). */
static uint32_t wdt_GetCurrent(void)
{
	uint32_t current, lastSecs;

	/* Overflow at 32-bits. Typical overflow is (0xFFFFFFFF/(500K))
	   = 8589 seconds (~2-3 hours). */

	/* Handle WDT count change during read */
	do {
		lastSecs = wdtOCount;
		/* Counts down, with 6PCLK + 6WDT clock delay */
		current = (lastSecs * (WDTTIMEOUTVAL >> KERNELTIMERSHIFT)) +
			((WDTTIMEOUTVAL >> KERNELTIMERSHIFT) - (Chip_WWDT_GetCurrentCount(LPC_WWDT) >> KERNELTIMERSHIFT));
	} while (wdtOCount != lastSecs);

	return current;
}

/* Calibrate WDT oscillator as best as possible to IRC rate. Assumes the IRC
   is already running, but doesn't need to be the main clock source. */
static void wdt_calOsc(void)
{
	Chip_Clock_EnablePeriphClock(SYSCON_CLOCK_INPUTMUX);

	/* Setup to measure the selected target using the IRC as the reference */
	Chip_INMUX_SetFreqMeasRefClock(FREQMSR_IRC);
	Chip_INMUX_SetFreqMeasTargClock(FREQMSR_WDOSC);

	/* Start a measurement cycle and wait for it to complete. */
	Chip_SYSCON_StartFreqMeas();
	while (!Chip_SYSCON_IsFreqMeasComplete()) {}

	/* Get computed frequency */
	calWdtFreq = Chip_SYSCON_GetCompFreqMeas(Chip_Clock_GetIntOscRate());

	/* Frequency into WDT as a fixed divider of 4 */
	calWdtFreq = calWdtFreq / 4;
}

static void wdt_init(void)
{
	/* Enable the power to the WDT Oscillator */
	Chip_SYSCON_PowerUp(SYSCON_PDRUNCFG_PD_WDT_OSC);

	/* Get WDT oscilaltor rate using frequency measurement */
	wdt_calOsc();

	/* Adjust kernel frequency by divider */
	calWdtFreq = calWdtFreq >> KERNELTIMERSHIFT;

	/* Determine overflow limit */
	//upperOfLimit = (0xFFFFFFFF / calWdtFreq) - 1;
	upperOfLimit = (0xFFFFFFFF / (WDTMAXTIMEOUTVAL >> KERNELTIMERSHIFT)) -1;
    
#if 0	/* update thresholds */
	g_Timer.sleep_threshold_us   = SLEEP_THRESHOLD_US;
	g_Timer.pwrDown_threshold_us = PWRDOWN_THRESHOLD_US;
	g_Timer.bgProc_threshold_us  = BACKGROUND_THRESHOLD_US;
#endif
	/* Initialize WWDT (also enables WWDT clock) */
	Chip_WWDT_Init(LPC_WWDT);

	/* Set watchdog feed time constant (timeout) */
	Chip_WWDT_SetTimeOut(LPC_WWDT, WDTTIMEOUTVAL);
	Chip_WWDT_SetWarning(LPC_WWDT, 512);

	/* Clear watchdog timeout interrupt */
	Chip_WWDT_ClearStatusFlag(LPC_WWDT, (WWDT_WDMOD_WDTOF | WWDT_WDMOD_WDINT));

	/* Allow WDT to wake from deep sleep */
	Chip_SYSCON_EnableWakeup(SYSCON_STARTER_WWDT);

	/* Clear and enable watchdog interrupt */
	NVIC_ClearPendingIRQ(WDT_IRQn);
	NVIC_EnableIRQ(WDT_IRQn);

	/* Start watchdog */
	Chip_WWDT_Start(LPC_WWDT);
}

/* PLL clock source, must be one of the following:
 * SYSCON_PLLCLKSRC_IRC, SYSCON_PLLCLKSRC_CLKIN,
 * SYSCON_PLLCLKSRC_WDTOSC, SYSCON_PLLCLKSRC_RTC. 
 */
#define PLLCLOCKSOURCE  SYSCON_PLLCLKSRC_IRC

/* Sets the best FLASH clock arte for the passed frequency */
static void setupFlashClocks(uint32_t freq)
{
	/* v17.0 ROM support only - coarse FLASH clocking timing.
	   FLASH access is setup based on voltage for v17.1 and later ROMs
	   as part of the power library. */
	if (Chip_POWER_GetROMVersion() == LPC5410X_ROMVER_0) {
		if (freq < 20000000) {
			Chip_SYSCON_SetFLASHAccess(SYSCON_FLASH_1CYCLE);
		}
		else if (freq < 48000000) {
			Chip_SYSCON_SetFLASHAccess(SYSCON_FLASH_2CYCLE);
		}
		else if (freq < 72000000) {
			Chip_SYSCON_SetFLASHAccess(SYSCON_FLASH_3CYCLE);
		}
		else {
			Chip_SYSCON_SetFLASHAccess(SYSCON_FLASH_4CYCLE);
		}
	}
}

#define PLL_OUTPUT_CLOCK_RATE (100000000)

/*
 * @fn      setupClocking
 *          Clock initialization function
 *
 * @param   void
 *
 * @return  none
 *
 * @see     rtc_init()
 */
static void setupClocking(void)
{
    PLL_CONFIG_T pllConfig;
	PLL_SETUP_T pllSetup;
	PLL_ERROR_T pllError;
	volatile int j;


	/* Set main clock source to the IRC clock  This will drive 24MHz
	   for the main clock and 24MHz for the system clock */
	Chip_Clock_SetMainClockSource(SYSCON_MAINCLKSRC_IRC);
    Chip_SYSCON_PowerUp(SYSCON_PDRUNCFG_PD_SYS_PLL);
   
	/* Make sure the PLL is off */
	Chip_SYSCON_PowerDown(SYSCON_PDRUNCFG_PD_SYS_PLL);

	/* Select the PLL input clock source */
	Chip_Clock_SetSystemPLLSource(PLLCLOCKSOURCE);

	if ((PLLCLOCKSOURCE == SYSCON_PLLCLKSRC_RTC) || (PLLCLOCKSOURCE == SYSCON_PLLCLKSRC_WDTOSC)) {
		/* When switching clock sources for the PLL, both the current and new source
		   must be enabled and requires a small sync time. */
		for (j = 0; j < 0x10000; j++) {}
	}

	/* Setup PLL configuration */
	pllConfig.desiredRate = PLL_OUTPUT_CLOCK_RATE;       // pll output clock rate in unit of Hz
	pllConfig.InputRate = 0;/* Not used unless PLL_CONFIGFLAG_USEINRATE flag is used */
#if defined(USEPLLPRECISE)
	pllConfig.flags = 0;
	pllConfig.ss_mf = SS_MF_64;
	pllConfig.ss_mr = SS_MR_K3;
	pllConfig.ss_mc = SS_MC_RECC;
	pllConfig.mfDither = false;
#else
	/* Force non-fractional mode */
	pllConfig.flags = PLL_CONFIGFLAG_FORCENOFRACT;
#endif

	pllError = Chip_Clock_SetupPLLData(&pllConfig, &pllSetup);
	if (pllError != PLL_ERROR_SUCCESS) {
		DEBUGOUT("PLL config error %d\r\n", pllError);
		while (1) {}
	}

	if ((PLLCLOCKSOURCE == SYSCON_PLLCLKSRC_RTC) || (PLLCLOCKSOURCE == SYSCON_PLLCLKSRC_WDTOSC)) {
		/* Disable PLL wait lock flag when using RTC or WDTOSC, since
		   it will never lock */
		pllSetup.flags = PLL_SETUPFLAG_POWERUP | PLL_SETUPFLAG_ADGVOLT;
	}
	else {
		/* If using WAITLOCK, powerup is implied, but we set the flag here for
		   consistency */
#if defined(USEPLLPRECISE)
		/* Don't wait for PLL lock when using SS, it might not lock */
		pllSetup.flags = PLL_SETUPFLAG_POWERUP | PLL_SETUPFLAG_ADGVOLT;
#else
		pllSetup.flags = PLL_SETUPFLAG_POWERUP | PLL_SETUPFLAG_WAITLOCK | PLL_SETUPFLAG_ADGVOLT;
#endif
	}

	pllError = Chip_Clock_SetupSystemPLLPrec(&pllSetup);
	if (pllError != PLL_ERROR_SUCCESS) {
		DEBUGOUT("PLL setup error %d\r\n", pllError);
		while (1) {}
	}

	/* Setup FLASH access speed */
	setupFlashClocks(PLL_OUTPUT_CLOCK_RATE);

#if defined(USEPLLPRECISE)
	/* Since PLL lock may not happen when using SS mode, force a small
	   delay here to let it stabilize. */
	for (j = 0; j < 0x8000; j++) {}
#else
	if ((PLLCLOCKSOURCE == SYSCON_PLLCLKSRC_RTC) || (PLLCLOCKSOURCE == SYSCON_PLLCLKSRC_WDTOSC)) {
		/* Small delay for PLL lock when using RTC or WDTOSC */
		for (j = 0; j < 0x8000; j++) {}
	}
#endif
	/* Set system main clock source to the system PLL  */
	Chip_Clock_SetMainClockSource(SYSCON_MAINCLKSRC_PLLOUT);


	/* Set system clock divider to 1. This output of system clock 
     * divider drives CPU, AHB bus, Sync APB etc... 
     */
	Chip_Clock_SetSysClockDiv(1);
    
    /* ASYSNC SYSCON needs to be on or all serial peripheral won't work.
	   Be careful if PLL is used or not, ASYNC_SYSCON source needs to be
	   selected carefully. */
	Chip_SYSCON_Enable_ASYNC_Syscon(true);
	Chip_Clock_SetAsyncSysconClockDiv(1);
	Chip_Clock_SetAsyncSysconClockSource(SYSCON_ASYNC_IRC);

#if 1   // enable this to pipe system main clock to CLKOUT pin 
	/* Map P0.21 as CLKOUT pin */
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 21,
		 (IOCON_FUNC1 | IOCON_MODE_INACT | IOCON_DIGITAL_EN));

	/* Setup CLKOUT for main system clock divided by 10 */
	Chip_Clock_SetCLKOUTSource(SYSCON_CLKOUTSRC_MAINCLK, 10);
#endif
}

void rtc_init(void) {
    setupClocking();
    wdt_init();
}

time_t rtc_read(void) {
    return (time_t)7*wdt_GetCurrent();;
}

