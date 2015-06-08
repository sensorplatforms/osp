/* Open Sensor Platform Project
 * https://github.com/sensorplatforms/open-sensor-platform
 *
 * Copyright (C) 2013 Sensor Platforms Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
/*---------------------------------------------------------------------*\
 |    I N C L U D E   F I L E S
\*---------------------------------------------------------------------*/
#include "board.h"
#include "common.h"
#include "hw_setup.h"
#include "sensorhub.h"
#include <string.h>
/*---------------------------------------------------------------------*\
 |    E X T E R N A L   V A R I A B L E S   &   F U N C T I O N S
\*---------------------------------------------------------------------*/

/*---------------------------------------------------------------------*\
 |    P U B L I C   V A R I A B L E S   D E F I N I T I O N S
\*---------------------------------------------------------------------*/
#ifdef DEBUG_BUILD
char _errBuff[ERR_LOG_MSG_SZ];
#endif
uint32_t	g_logging = 0xffffffff;

/*---------------------------------------------------------------------*\
 |    P R I V A T E   C O N S T A N T S   &   M A C R O S
\*---------------------------------------------------------------------*/
/* UART handle and memory for ROM API */
static UART_HANDLE_T uartHandle;

/* Use a buffer size larger than the expected return value of
   uart_get_mem_size() for the static UART handle type */
static uint32_t uartHandleMEM[0x10];

/*---------------------------------------------------------------------*\
 |    P R I V A T E   T Y P E   D E F I N I T I O N S
\*---------------------------------------------------------------------*/

/*---------------------------------------------------------------------*\
 |    S T A T I C   V A R I A B L E S   D E F I N I T I O N S
\*---------------------------------------------------------------------*/

/*---------------------------------------------------------------------*\
 |    F O R W A R D   F U N C T I O N   D E C L A R A T I O N S
\*---------------------------------------------------------------------*/

/*---------------------------------------------------------------------*\
 |    P R I V A T E     F U N C T I O N S
\*---------------------------------------------------------------------*/
static void errorUART(void)
{
	Board_LED_Set(1, true);
	while (1) {}
}

/* UART Pin mux function - note that SystemInit() may already setup your
   pin muxing at system startup */
static void Init_UART_PinMux(void)
{
#if defined(BOARD_NXP_LPCXPRESSO_54000)
	Chip_Clock_SetAsyncSysconClockDiv(1);	/* divided by 1 */

	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 0, IOCON_MODE_INACT | IOCON_FUNC1 | IOCON_DIGITAL_EN | IOCON_INPFILT_OFF);
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 1, IOCON_MODE_INACT | IOCON_FUNC1 | IOCON_DIGITAL_EN | IOCON_INPFILT_OFF);

#else
	/* Configure your own UART pin muxing here if needed */
#warning No UART pin muxing defined
#endif
}

/* Setup UART handle and parameters */
static void setupUART()
{
	uint32_t frg_mult;

	/* 4*115.2KBPS, 8N1, ASYNC mode, no errors, clock filled in later */
	UART_CONFIG_T cfg = {
		0,				/* U_PCLK frequency in Hz */
		115200*4,		/* Baud Rate in Hz */
		1,				/* 8N1 */
		0,				/* Asynchronous Mode */
		NO_ERR_EN	/* Enable No Errors */
	};

	/* Perform a sanity check on the storage allocation */
	if (LPC_UARTD_API->uart_get_mem_size() > sizeof(uartHandleMEM)) {
		/* Example only: this should never happen and probably isn't needed for
		   most UART code. */
		errorUART();
	}

	/* Setup the UART handle */
	uartHandle = LPC_UARTD_API->uart_setup((uint32_t) LPC_USART0, (uint8_t *) &uartHandleMEM);
	if (uartHandle == NULL) {
		errorUART();
	}

	/* Need to tell UART ROM API function the current UART peripheral clock
	     speed */
	cfg.sys_clk_in_hz = Chip_Clock_GetAsyncSysconClockRate();

	/* Initialize the UART with the configuration parameters */
	frg_mult = LPC_UARTD_API->uart_init(uartHandle, &cfg);
	if (frg_mult) {
		Chip_Clock_EnableAsyncPeriphClock(ASYNC_SYSCTL_CLOCK_FRG);
		Chip_SYSCTL_SetUSARTFRGCtrl(frg_mult, 0xFF);
	}
}

/* Send a string on the UART terminated by a NULL character using
   polling mode. */
void putLineUART(const char *send_data)
{
	UART_PARAM_T param;

	param.buffer = (uint8_t *) send_data;
	param.size = strlen(send_data);

	/* Polling mode, do not append CR/LF to sent data */
	param.transfer_mode = TX_MODE_SZERO;
	param.driver_mode = DRIVER_MODE_POLLING;

	/* Transmit the data */
	if (LPC_UARTD_API->uart_put_line(uartHandle, &param)) {
		while(0) errorUART();
	}
}

void put_u32(uint32_t v)
{
	int i;
	uint32_t m;
	uint8_t c;
	char str[9];
	m = 0xf0000000;
	for (i = 0; i < 8; i++) {
		c= ((m&v) >> (28-i*4));
		str[i] = (c > 9) ? (c+'a'):(c+'0');
	}
	str[8] = '\0';
	putLineUART(str);
}
static uint32_t volatile wdtOCount;
static uint32_t calWdtFreq, upperOfLimit;
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

/* Calibrate WDT oscillator as best as possible to IRC rate. Assumes the IRC
   is already running, but doesn't need to be the main clock source. */
static void wdt_calOsc(void)
{
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_MUX);

	/* Setup to measure the selected target using the IRC as the reference */
	Chip_INMUX_SetFreqMeasRefClock(FREQMSR_IRC);
	Chip_INMUX_SetFreqMeasTargClock(FREQMSR_WDOSC);

	/* Start a measurement cycle and wait for it to complete. */
	Chip_SYSCTL_StartFreqMeas();
	while (!Chip_SYSCTL_IsFreqMeasComplete()) {}

	/* Get computed frequency */
	calWdtFreq = Chip_SYSCTL_GetCompFreqMeas(Chip_Clock_GetIntOscRate());

	/* Frequency into WDT as a fixed divider of 4 */
	calWdtFreq = calWdtFreq / 4;
}

static wdt_init(void)
{
	/* Enable the power to the WDT Oscillator */
	Chip_SYSCTL_PowerUp(PDRUNCFG_PD_WDT_OSC);

	/* Get WDT oscilaltor rate using frequency measurement */
	wdt_calOsc();

	/* Adjust kernel frequency by divider */
	calWdtFreq = calWdtFreq >> KERNELTIMERSHIFT;

	/* Determine overflow limit */
	upperOfLimit = (0xFFFFFFFF / calWdtFreq) - 1;
	
#if 0	/* update thresholds */
	g_Timer.sleep_threshold = (SLEEP_THRESHOLD * calWdtFreq)/1000000;
	g_Timer.pwrDown_threshold = (PWRDOWN_THRESHOLD * calWdtFreq)/1000000;
	g_Timer.bgProc_threshold = (BACKGROUND_THRESHOLD * calWdtFreq)/1000000;
#endif
	/* Initialize WWDT (also enables WWDT clock) */
	Chip_WWDT_Init(LPC_WWDT);

	/* Set watchdog feed time constant (timeout) */
	Chip_WWDT_SetTimeOut(LPC_WWDT, WDTTIMEOUTVAL);
	Chip_WWDT_SetWarning(LPC_WWDT, 512);

	/* Clear watchdog timeout interrupt */
	Chip_WWDT_ClearStatusFlag(LPC_WWDT, (WWDT_WDMOD_WDTOF | WWDT_WDMOD_WDINT));

	/* Allow WDT to wake from deep sleep */
	Chip_SYSCTL_EnableWakeup(STARTERP0_WWDT);

	/* Clear and enable watchdog interrupt */
	NVIC_ClearPendingIRQ(WWDT_IRQn);
	NVIC_EnableIRQ(WWDT_IRQn);

	/* Start watchdog */
	Chip_WWDT_Start(LPC_WWDT);
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
/*---------------------------------------------------------------------*\
 |    P U B L I C     F U N C T I O N S
\*---------------------------------------------------------------------*/
uint32_t GetCurrentTime(void)
{
	return 7*wdt_GetCurrent();
}

#define	SAVE_HARDFAULT
#if defined(SAVE_HARDFAULT)
enum { r0, r1, r2, r3, r12, lr, pc, psr};
static uint32_t fault_hfsr;
static uint32_t fault_cfsr;
static uint32_t fault_pc;
static uint32_t fault_lr;
static uint32_t fault_r0;
#endif
void HardFault_Handler(uint32_t stack[])
{
#if defined(SAVE_HARDFAULT)
	fault_hfsr = SCB->HFSR;
	fault_cfsr = SCB->CFSR;
	fault_pc = stack[pc];
	fault_lr = stack[lr];
	fault_r0 = stack[r0];
#endif
	__ASM volatile("BKPT #01");
	while(1);
}
/***********************************************************************
 * @fn      main
 *          Main entry point to the application firmware
 *
 * @param   none
 *
 * @return  none
 *
 ***********************************************************************/
int main(void)
{ 
	/* Update core clock variables */
	SystemCoreClockUpdate();
	/* Initialize the pinmux and clocking per board connections */
	Board_Init();
	Board_SystemInit();
	/* Initialize GPIO pin interrupt module */
	Chip_PININT_Init(LPC_PININT);
        /* turn INMUX clock off */
	Init_UART_PinMux();
	Chip_UART_Init(LPC_USART0);
	/* Allocate UART handle, setup UART parameters, and initialize UART
	   clocking */
	setupUART();
	/* Transmit the welcome message and instructions using the
	   putline function */
	D0_printf("OSP-port compiled %s on %s\r\n", __TIME__, __DATE__);
	wdt_init();
	// Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_MUX);
	Board_LED_Set(0, false);
	Board_LED_Set(1, false);
	Board_LED_Set(2, false);
	Chip_GPIO_SetPinDIROutput(LPC_GPIO, HOSTIF_IRQ_PORT, HOSTIF_IRQ_PIN);

	{
		/* By default, VDx levels are set to low to medium to preserve power.
		   In order to run the code at higher speed using PLL, the VDx level
		   have to be raised. Be careful with the FLASH wait state and VD level
		   if you want to set the PLL in order to run at the highest frequency possible. */
		LPC_PWRD_API->set_vd_level(VD1, V1100, FINE_V_NONE);
		LPC_PWRD_API->set_vd_level(VD8, V1100, FINE_V_NONE);

		Chip_Clock_SetSystemPLLSource(SYSCTL_PLLCLKSRC_IRC);
		/* Wait State setting TBD */
		/* Setup FLASH access to 5 clocks (up to 84MHz) */
		Chip_FMC_SetFLASHAccess(FLASHTIM_72MHZ_CPU);

		/* Power down PLL to change the PLL divider ratio */
		Chip_SYSCTL_PowerDown(PDRUNCFG_PD_SYS_PLL0);

		/* First parameter is the multiplier, the second parameter is the input frequency in MHz */
		Chip_Clock_SetupSystemPLL(7, SYSCTL_IRC_FREQ);

		/* VD2_ANA needs to be powered up too. According to the design team, it's powered up at chip reset. */
		if ( Is_Chip_SYSCTL_PowerUp(PDRUNCFG_PD_VD2_ANA) ) {
			Chip_SYSCTL_PowerUp(PDRUNCFG_PD_VD2_ANA);
		}
		/* Turn on the PLL by clearing the power down bit */
		Chip_SYSCTL_PowerUp(PDRUNCFG_PD_SYS_PLL0);

		/* Wait for PLL to lock */
		while (!Chip_Clock_IsSystemPLLLocked()) {}

		/* Set main clock source to the system PLL. This will drive 84MHz
		   for the main clock and 84MHz for the system clock */
		Chip_Clock_SetMainClockSource(SYSCTL_MAINCLKSRC_PLLOUT);
	}

        /* Get the OS going - This must be the last call */
	AsfInitialiseTasks();

	/* If it got here something bad happened */
	ASF_assert_fatal(false);
}

/*------------------------------------------------------------------*\
 |    E N D   O F   F I L E
\*------------------------------------------------------------------*/
