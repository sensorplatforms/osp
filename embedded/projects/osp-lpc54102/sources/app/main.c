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
#include "romapi_uart.h"


/*---------------------------------------------------------------------*\
 |    E X T E R N A L   V A R I A B L E S   &   F U N C T I O N S
\*---------------------------------------------------------------------*/

/*---------------------------------------------------------------------*\
 |    P U B L I C   V A R I A B L E S   D E F I N I T I O N S
\*---------------------------------------------------------------------*/
#ifdef DEBUG_BUILD
char _errBuff[ERR_LOG_MSG_SZ];
#endif
uint32_t	g_logging = 0x00;

/*---------------------------------------------------------------------*\
 |    P R I V A T E   C O N S T A N T S   &   M A C R O S
\*---------------------------------------------------------------------*/
/* UART handle and memory for ROM API */
//static UART_HANDLE_T uartHandle;

/* Use a buffer size larger than the expected return value of
   uart_get_mem_size() for the static UART handle type */
//static uint32_t uartHandleMEM[0x10];

/* UART Driver context memory */
#define RAMBLOCK_H          60
static uint32_t  uartHandleMem[RAMBLOCK_H];

/* UART ROM Driver Handle */
static UART_HANDLE_T *hUART;

#define UART_BAUD_RATE     115200	/* Required UART Baud rate */
#define UART_BUAD_ERR      1/* Percentage of Error allowed in baud */

#define tmsg1(x) "Type chars to echo them back\r\n"
#define tmsg(x) tmsg1(x)
#define RX_SZ  16
#define msg   tmsg(RX_SZ)
#define rmsg  "UART received : "

volatile uint32_t tx_done, rx_done;
/*---------------------------------------------------------------------*\
 |    P R I V A T E   T Y P E   D E F I N I T I O N S
\*---------------------------------------------------------------------*/
/* PLL clock source, must be one of the following:
 * SYSCON_PLLCLKSRC_IRC, SYSCON_PLLCLKSRC_CLKIN,
 * SYSCON_PLLCLKSRC_WDTOSC, SYSCON_PLLCLKSRC_RTC. 
 */
#define PLLCLOCKSOURCE  SYSCON_PLLCLKSRC_IRC

/*---------------------------------------------------------------------*\
 |    S T A T I C   V A R I A B L E S   D E F I N I T I O N S
\*---------------------------------------------------------------------*/

/*---------------------------------------------------------------------*\
 |    F O R W A R D   F U N C T I O N   D E C L A R A T I O N S
\*---------------------------------------------------------------------*/

/*---------------------------------------------------------------------*\
 |    P R I V A T E     F U N C T I O N S
\*---------------------------------------------------------------------*/
#if 0   // comment out since it is not use 
static void errorUART(void)
{
	Board_LED_Set(1, true);
	while (1) {}
}
#endif 

/* UART Pin mux function - note that SystemInit() may already setup your
   pin muxing at system startup */
static void UART_PinMuxSetup(void)
{   
#if defined(BOARD_NXP_LPCXPRESSO_54102)
	/* Setup UART TX Pin */
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 0, (IOCON_FUNC1 | IOCON_MODE_INACT | IOCON_DIGITAL_EN));
	/* Setup UART RX Pin */
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 1, (IOCON_FUNC1 | IOCON_MODE_INACT | IOCON_DIGITAL_EN));
	Chip_SYSCON_Enable_ASYNC_Syscon(true);	/* Enable Async APB */
	Chip_Clock_SetAsyncSysconClockDiv(1);	/* Set Async clock divider to 1 */
#else
#warning "No UART PIN/CLK setup for this example"
#endif    
}

/* Initialize the UART ROM Driver */
static int uartrom_init(void)
{
	int sz;

	UART_PinMuxSetup();

	sz =  ROM_UART_GetMemSize();

	if (RAMBLOCK_H < (sz / 4)) {
		while (1) {}
	}

	hUART = ROM_UART_Init(uartHandleMem, LPC_USART0_BASE, 0);

	return 0;
}

#define ABS(x) ((int) (x) < 0 ? -(x) : (x))
/* Configure UART ROM Driver and pripheral */
static int uartrom_config(void)
{
	UART_CFG_T cfg;
	UART_BAUD_T baud;

	/* Set up baudrate parameters */
	baud.clk = Chip_Clock_GetAsyncSyscon_ClockRate();	/* Clock frequency */
	baud.baud = UART_BAUD_RATE;	/* Required baud rate */
	baud.ovr = 0;	/* Set the oversampling to the recommended rate */
	baud.mul = baud.div = 0;

	if (ROM_UART_CalBaud(&baud) != LPC_OK) {
		/* Unable to calculate the baud rate parameters */
		while (1) {}
	}

	/* Set fractional control register */
	Chip_SYSCON_SetUSARTFRGCtrl(baud.mul, 255);

	/* See if the calculated baud is < +/- UART_BUAD_ERR% of the required baud */
	if (ABS(baud.baud - UART_BAUD_RATE) > (UART_BAUD_RATE * UART_BUAD_ERR) / 100) {
		/* WARNING: Baud rate is has more than UART_BUAD_ERR percentage */
		/* Try to auto-detect the Oversampling rate by setting baud.ovr to 0 */
		while (1) {}
	}

	/* Configure the UART */
	cfg.cfg = UART_CFG_8BIT | UART_CFG_BRKRX;
	cfg.div = baud.div;	/* Use the calculated div value */
	cfg.ovr = baud.ovr;	/* Use oversampling rate from baud */
	cfg.res = UART_BIT_DLY(UART_BAUD_RATE);

	/* Configure the UART */
	ROM_UART_Configure(hUART, &cfg);
	NVIC_ClearPendingIRQ(UART0_IRQn);
	NVIC_EnableIRQ(UART0_IRQn);

	return 0;
}


/* UART ROM error handler */
static void uartrom_error(UART_HANDLE_T hUART, uint32_t err)
{
	switch (err) {
	case UART_ERROR_FRAME:
		/* No stop bit in uart frame; mismatched baud(?) or incorrect/short BREAK condition(?) */
		Board_LED_Set(0, 1);
		break;

	case UART_ERROR_PARITY:
		/* Parity error; mismatched baud(?) */
		while (1) {}

	case UART_ERROR_AUTOBAUD:
		/* Autobaud timeout error */
		while (1) {}

	case UART_ERROR_OVERRUN:
		/* Uart received character before ROM_UART_Receive() is called */
		Board_LED_Set(1, 1);
		break;

	case UART_ERROR_RXNOISE:
		/* Typically problem is with the baud rate and or Over sampling count */
		while (1) {}

	default:
		/* Control will never reach this */
		break;
	}
}

/* UART ROM event handler */
static void uartrom_event(UART_HANDLE_T hUART, uint32_t evt)
{
	switch (evt) {
	case UART_EVENT_BREAK:
		Board_LED_Set(2, 1);		/* TURN ON LED_2 when BREAK is received on RX line */
		break;

	case UART_EVENT_NOBREAK:
		Board_LED_Set(2, 0);		/* TURN OFF LED_2 when RX comes out of break */
		break;

	case UART_EVENT_TXIDLE:
		/* Can be used for flow control */
		/* This will be called when the TX shift register is done with
		   sending the last bit to uart line; event will be called only
		   after calling ROM_UART_SetCtrl(hUART, UART_TXIDLE_ON), event
		   can be turned off using ROM_UART_SetCtrl(hUART, UART_TXIDLE_OFF)
		 */
		break;

	case UART_EVENT_TXPAUSED:
		/* Event will happen after ROM_UART_SetCtrl(hUART, UART_TX_PAUSE) is
		   called. This event does not mean the the TX is idle, meaning, the
		   TX holding register might contain data (which is not loaded into
		   TX shift register anymore) and the the TX shift register is done
		   sending the data that it was sending when UART_TX_PAUSE was called.
		 */
		/* Can be used to implement flow control or safely send BREAK signal
		   without createing frame errors in the currently transmitted data
		 */
		break;

	case UART_EVENT_CTSHI:
		/* CTS line went from Low to High */
		/* Could be used for flow control or RS-485 implementations */
		break;

	case UART_EVENT_CTSLO:
		/* CTS line went from High to Low */
		/* Could be used for flow control or RS-485 implementations */
		break;

	default:
		while (1) {}	/* Control will never reach here */
	}
}

/* Call-back handler for error/event */
static void uartrom_xfer_errevt(UART_HANDLE_T hUART, UART_EVENT_T evt, void *arg)
{
	uint32_t val = (uint32_t) arg;
	if (evt == UART_EV_ERROR) {
		uartrom_error(hUART, val);
	}
	else if (evt == UART_EV_EVENT) {
		uartrom_event(hUART, val);
	}
	else {
		while (1) {}/* Control will never reach this */

	}
}

/* UART Transfer done */
static void uartrom_xfer_done(UART_HANDLE_T hUART, UART_EVENT_T evt, void *arg)
{
	UART_DATA_T *dat = (UART_DATA_T *) arg;

	switch (evt) {
	case UART_TX_DONE:
		tx_done = 1;
		break;

	case UART_RX_DONE:
		rx_done = dat->count;
		break;

	default:
		while (1) {}	/* Control will never reach here */
	}
}

/* UART transfer start */
static void uartrom_xfer_start(UART_HANDLE_T hUART, UART_EVENT_T evt, void *arg)
{
	UART_DATA_T *dat = (UART_DATA_T *) arg;

	(void) *dat;
	switch (evt) {
	case UART_TX_START:
		/* Transmit of new buffer started */
		break;

	case UART_RX_START:
		/* Receive of data started; can be used to implement timer logic */
		break;

	default:
		while (1) {}	/* Control will never reach here */
	}
}

static void uartrom_rx_prog(UART_HANDLE_T hUART, UART_EVENT_T evt, void *arg)
{
	UART_DATA_T *dat = (UART_DATA_T *) arg;

	switch (evt) {
	case UART_RX_INPROG:
		break;

	case UART_RX_NOPROG:
		/* In case of frame error restart xfer */
		if (dat->state == UART_ST_ERRFRM) {
			dat->offset = 0;
			dat->state = UART_ST_BUSY;
		}
		else {
			/* If the received frame has errors don't stop just restart */
			ROM_UART_SetCtrl(hUART, UART_RX_STOP);
		}
		break;

	default:
		break;		/* Control should never reach here */
	}
}

/* Register call-backs */
static void uartrom_regcb(void)
{
	ROM_UART_RegisterCB(hUART, UART_CB_START, uartrom_xfer_start);	/* Start of transfer */
	ROM_UART_RegisterCB(hUART, UART_CB_DONE, uartrom_xfer_done);/* End of transfer */
	ROM_UART_RegisterCB(hUART, UART_CB_ERREVT, uartrom_xfer_errevt);/* Error/Event callbacks */
	ROM_UART_RegisterCB(hUART, UART_CB_RXPROG, uartrom_rx_prog);/* Receive progress callback */
}



#if 0 
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
#endif 

/* Send a string on the UART terminated by a NULL character using
   polling mode. */
void putLineUART(const char *send_data)
{
#if 0
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
#else 
    //ROM_UART_Send(hUART, send_data, sizeof(send_data) - 1);
    Board_UARTPutSTR((char*)send_data);
#endif 
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

#if 1
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
#endif 

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



/*****************************************************************************
 * Public functions
 ****************************************************************************/

/**
 * @brief	UART interrupt handler
 */
void UART0_IRQHandler(void)
{
	ROM_UART_Handler(hUART);
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


#define PLL_OUTPUT_CLOCK_RATE (100000000)
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

    // So we know main() is running     
    D0_printf("%s started\r\n", __FUNCTION__);
        
    Board_LED_Set(0, false);
    Board_LED_Set(1, false);
    Board_LED_Set(2, false);
    
    /* Initialize GPIO pin interrupt module */
	Chip_PININT_Init(LPC_PININT);    
        
    wdt_init();
   
    Chip_GPIO_SetPinDIROutput(LPC_GPIO, HOSTIF_IRQ_PORT, HOSTIF_IRQ_PIN);
   
    // Use the same setting in the bosch example. Restore to Audience setting later when able to run on the new board 
    setupClocking();
         
    /* Get the OS going - This must be the last call */
	AsfInitialiseTasks();   
    
	/* If it got here something bad happened */
	ASF_assert_fatal(false);
}

#ifdef __MICROLIB
/***********************************************************************
 * @fn      exit
 *          Main Exit point from the application firmware
 *          This function is required if microlib is used
 *
 * @param   Error code
 *
 * @return  none
 *
 ***********************************************************************/
void exit(uint32_t error)
{
    __ASM volatile("BKPT #01");
    while(1);
}
#endif

/*------------------------------------------------------------------*\
 |    E N D   O F   F I L E
\*------------------------------------------------------------------*/
