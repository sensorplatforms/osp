LPC5410x UART interrupt/DMA example using the ROM API
=================================

Example description
-------------------
This example shows how to configure UART in interrupt mode using DMA
to handle data transfers.

Set the terminal to 115.2K, 8 data bits, 1 stop bit, and no parity.
use a terminal program to send some data to the example and it will
return the data to the terminal.

This example will receive 16 bytes and then send the received 16
bytes back to the terminal via UART. Simultaneous send and
receive is not supported.

Special connection requirements
-------------------------------
Use the UART bridge to connect a terminal program to the VCOM serial
port for the board.

Build procedures:
-----------------
Visit the at 'LPCOpen quickstart guides' [http://www.lpcware.com/content/project/lpcopen-platform-nxp-lpc-microcontrollers/lpcopen-v200-quickstart-guides]
to get started building LPCOpen projects.


