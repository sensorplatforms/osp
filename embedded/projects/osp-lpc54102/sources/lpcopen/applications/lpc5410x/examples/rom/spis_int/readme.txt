LPC5410x SPI slave (interrupt mode) ROM API example
============================

Example description
-------------------
This example shows how to use the SPI slave ROM API to setup and perform
a SPI slave transfer in interrupt mode.

Special connection requirements
-------------------------------
To use this example, a SPI master needs to be connected to the SPI slave
signals for SPI0 on the following signals:
SPI0 SCK	P1.3
SPI0 SSEL	P0.14
SPI0 MOSI	P0.12
SPI0 MISO	P1.4
It is recommended to use a lower SPI master clock rate when using this example,
as the SPI driver may overflow or underflow without DMA support.

Build procedures:
-----------------
Visit the at 'LPCOpen quickstart guides' [http://www.lpcware.com/content/project/lpcopen-platform-nxp-lpc-microcontrollers/lpcopen-v200-quickstart-guides]
to get started building LPCOpen projects.
