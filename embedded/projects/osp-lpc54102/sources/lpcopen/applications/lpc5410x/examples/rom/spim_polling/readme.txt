LPC5410x SPI master (blocking/polling mode) ROM API example
============================

Example description
-------------------
This example shows how to use the SPI master ROM API to setup and perform
a SPI master transfer in polling (blocking) mode. A blocking transfer will
not return to the caller until the transfer is complete.

Special connection requirements
-------------------------------
This example uses loopback mode and has no special setup requirements.
However, the SPI master signals can be monitored on pins P0.11, P0.12,
P0.13, and P0.14.

Build procedures:
-----------------
Visit the at 'LPCOpen quickstart guides' [http://www.lpcware.com/content/project/lpcopen-platform-nxp-lpc-microcontrollers/lpcopen-v200-quickstart-guides]
to get started building LPCOpen projects.
