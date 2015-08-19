/* mbed Microcontroller Library
 * Copyright (c) 2006-2013 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "device.h"
#include "serial_api.h"

/* UART ROM RX call back */
static void  serial_done(UART_HANDLE_T hUART, UART_EVENT_T ev, void *arg)
{
    if (ev == UART_RX_DONE) 
    {
        *(int *) ROM_UART_HANDLE_TOUDATA(hUART) = 1;
    }
}
void serial_init(serial_t *obj, PinName tx, PinName rx)
{
    int sz;
    UART_CFG_T cfg;
    UART_BAUD_T baud;
    static int cval;
    uint32_t ui_serial_base = 0;

    if (obj->serial_num == SERIAL_LPC_0) 
    {
        /* Enable peripheral clock to UART0 */
        Chip_Clock_EnablePeriphClock(SYSCON_CLOCK_USART0);
        ui_serial_base = (uint32_t)LPC_USART0;
    }

    /* Enable clock to Fractional divider */
    Chip_Clock_EnablePeriphClock(SYSCON_CLOCK_FRG);

    /* UART0 pins are setup in board_sysinit.c */

    /* Test memory size */
    sz =  ROM_UART_GetMemSize();
    if (sz > sizeof(obj->rom_mem)) 
    {
        /* Not enough memory, return leaving UART handle as NULL */
    }

    /* Get handle to UART driver */
    obj->uart_handle = ROM_UART_Init(obj->rom_mem, ui_serial_base , &cval);

    /* Set up baudrate parameters */
    baud.clk = Chip_Clock_GetAsyncSyscon_ClockRate();	/* Clock frequency */
    baud.baud = obj->baudrate;	/* Required baud rate */
    baud.ovr = 0;	/* Set the oversampling to the recommended rate */
    baud.mul = baud.div = 0;

    if (ROM_UART_CalBaud(&baud) != LPC_OK) 
    {
        /* Unable to calculate the baud rate parameters */
        obj->uart_handle = NULL;
        return;
    }

    /* Set fractional control register */
    Chip_SYSCON_SetUSARTFRGCtrl(baud.mul, 0xFF);

    /* Configure the UART */
    cfg.cfg = obj->databits;
    cfg.div = baud.div;	/* Use the calculated div value */
    cfg.ovr = baud.ovr;	/* Use oversampling rate from baud */
    cfg.res = UART_BIT_DLY(obj->baudrate);

    /* Configure the UART */
    ROM_UART_Configure(obj->uart_handle, &cfg);
    ROM_UART_RegisterCB(obj->uart_handle, UART_CB_DONE, serial_done);
}

void serial_free(serial_t *obj)
{ 
    return;
}
void serial_baud(serial_t *obj, int baudrate)
{ 
    return;
}
void serial_format(serial_t *obj, int data_bits, 
                   SerialParity parity, int stop_bits)
{ 
    return;
}

void serial_irq_handler(serial_t *obj, uart_irq_handler handler, uint32_t id)
{ 
    return;
}
void serial_irq_set(serial_t *obj, SerialIrq irq, uint32_t enable)
{ 
    return;
}

int  serial_getc(serial_t *obj)
{
    static uint8_t uart_rx, q;
    int *cv = ROM_UART_HANDLE_TOUDATA(obj->uart_handle);

    if (!q) 
    {	/* Queue a buffer for receive */
        q = ROM_UART_Receive(obj->uart_handle, &uart_rx, 1) == LPC_OK;
    }
    ROM_UART_Handler(obj->uart_handle);
    if (*cv) 
    {
        q = *cv = 0;
        return (int) uart_rx;
    }
    return 0;
}
void serial_putc(serial_t *obj, int c)
{
    if (obj->uart_handle != NULL) 
    {
        ROM_UART_Send(obj->uart_handle, &c, 1);
        ROM_UART_WaitTx(obj->uart_handle);
    }
}
int  serial_readable(serial_t *obj)
{
    return 0;
}
int  serial_writable(serial_t *obj)
{
    return 0;
}
void serial_clear(serial_t *obj)
{
    return;
}

void serial_break_set(serial_t *obj)
{ 
    return;
}
void serial_break_clear(serial_t *obj)
{ 
    return;
}

void serial_pinout_tx(PinName tx)
{ 
    return;
}

void serial_set_flow_control(serial_t *obj, FlowControl type, 
                             PinName rxflow, PinName txflow)
{ 
    return;
}
