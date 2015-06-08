LPC5410x UART interrupt example using the system FIFO
=================================

Example description
-------------------
This example shows how to configure the UART in interrupt mode
to handle data transfers using the system FIFO. In this configuration,
the system FIFO handles data streaming to and from the UART data
registers. This example uses the system FIFO UART transfer handler
instead of the ROM transfer, but still uses the ROM UART functions
for all other UART setup.

Set the terminal to 115.2K, 8 data bits, 1 stop bit, and no parity.
Use a terminal program to send some data to the example and it will
return the data to the terminal.

This example implements a ring buffer for the transmit and receive
UART data. Data received on the UART is simply transmitted back out
via the UART.

Note: The DEBUG UART and UART for this example are the same. If an
error occurs during UART setup, the DEBUG channel output might not
work. This example is best explored using a debugger.

Special connection requirements
-------------------------------
Use the UART bridge to connect a terminal program to the VCOM serial
port for the board.

Build procedures:
-----------------
Visit the at 'LPCOpen quickstart guides' [http://www.lpcware.com/content/project/lpcopen-platform-nxp-lpc-microcontrollers/lpcopen-v200-quickstart-guides]
to get started building LPCOpen projects.
