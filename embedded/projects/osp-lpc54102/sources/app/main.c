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
#include "serial_api.h"
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

serial_t uart0;

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
#endif

/* This is board specific, need to move to hardware specific file. */
static const uint8_t ledBits[] = {29, 30, 31};

void Board_LED_Init(void)
{
    int i;

    /* Pin muxing setup as part of board_sysinit */
    for (i = 0; i < sizeof(ledBits); i++) 
    {
        Chip_GPIO_SetPinDIROutput(LPC_GPIO, 0, ledBits[i]);
        Chip_GPIO_SetPinState(LPC_GPIO, 0, ledBits[i], true);
    }
}

/* Send a string on the UART terminated by a NULL character using
   polling mode. */
void putLineUART(const char *send_data)
{
    unsigned int index = 0;

    for (index = 0; index < strlen(send_data); index++) 
    {
        serial_putc(&uart0, send_data[index]);
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

    /* board init does LED, UART and clock init. 
     * Moving peripheral clock init and LED init here. */
    /* INMUX and IOCON are used by many apps, enable both INMUX and IOCON clock bits here. */
    Chip_Clock_EnablePeriphClock(SYSCON_CLOCK_INPUTMUX);
    Chip_Clock_EnablePeriphClock(SYSCON_CLOCK_IOCON);
    
    /* Initialize GPIO */
    Chip_GPIO_Init(LPC_GPIO);
    
    Chip_PININT_Init(LPC_PININT);

        
    /* Initialize the LEDs. Be careful with below routine, once it's called some of the I/O will be set to output. */
    Board_LED_Init();

    uart0.baudrate = DEBUGBAUDRATE;
    uart0.databits = UART_CFG_8BIT;
    uart0.serial_num = SERIAL_LPC_0;

    serial_init(&uart0,0,0);

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
