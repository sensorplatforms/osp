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
#include "common.h"
#include "hw_setup.h"
#include "sensorhub.h"
#include <string.h>
#include "romapi_uart.h"
#include "gpio_api.h"
#include "rtc_api.h"
#include "pinmap.h"
#include "hostif.h"

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
    pin_function( ENCODE_PORT_PIN((uint8_t)Port_0, (uint8_t)Pin_0), (PINMAP_FUNC1 | PINMAP_MODE_INACT | PINMAP_DIGITAL_EN));
	/* Setup UART RX Pin */
    pin_function( ENCODE_PORT_PIN((uint8_t)Port_0, (uint8_t)Pin_1), (PINMAP_FUNC1 | PINMAP_MODE_INACT | PINMAP_DIGITAL_EN));
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

/*---------------------------------------------------------------------*\
 |    P U B L I C     F U N C T I O N S
\*---------------------------------------------------------------------*/

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
    gpio_init((gpio_t *)NULL,(PinName)NULL);
        
    rtc_init();
   
    gpio_dir(&hostifIrq,PIN_OUTPUT);

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
