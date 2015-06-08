void debug_init(void)
{
	uint32_t frg_mult;
	UART_CONFIG_T cfg = {
		0,		/* U_PCLK frequency in Hz */
		115200,		/* Baud Rate in Hz */
		1,		/* 8N1 */
		0,		/* Asynchronous Mode */
		NO_ERR_EN	/* Enable No Errors */
	};

	/* Setup pinmux */
	Chip_Clock_SetAsyncSysconClockDiv(1);   /* divided by 1 */

	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 0, IOCON_MODE_INACT | IOCON_FUNC1 | IOCON_DIGITAL_EN | IOCON_INPFILT_OFF);
        Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 1, IOCON_MODE_INACT | IOCON_FUNC1 | IOCON_DIGITAL_EN | IOCON_INPFILT_OFF);

	Chip_UART_Init(LPC_USART0);

        /* Setup the UART handle */
	uartHandle = LPC_UARTD_API->uart_setup((uint32_t) LPC_USART0, (uint8_t  ) &uartHandleMEM);
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

void put_str(char *str)
{
	UART_PARAM_T param;

	param.buffer = (uint8_t *) send_data;
	param.size = strlen(send_data);

	/* Polling mode, do not append CR/LF to sent data */
        param.transfer_mode = TX_MODE_SZERO;
        param.driver_mode = DRIVER_MODE_POLLING;

	/* Transmit the data */
        if (LPC_UARTD_API->uart_put_line(uartHandle, &param)) {
                errorUART();
        }
}
void put_char(char c)
{
	/* Transmit the data */
	if (LPC_UARTD_API->uart_put_char(uartHandle, c)) {
		errorUART();
	}
}
