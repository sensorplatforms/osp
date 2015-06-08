LPC5410x I2C master (blocking/polling mode) ROM API example
============================

Example description
-------------------
This example shows how to use the I2C master ROM API to setup and perform
an I2C master transfer in polling (blocking) mode. A blocking transfer will
not return to the caller until the transfer is complete.

Special connection requirements
-------------------------------
The I2C master example will always return NAK status as there are no slave
devices on the board to communicate with.

Build procedures:
-----------------
Visit the at 'LPCOpen quickstart guides' [http://www.lpcware.com/content/project/lpcopen-platform-nxp-lpc-microcontrollers/lpcopen-v200-quickstart-guides]
to get started building LPCOpen projects.
