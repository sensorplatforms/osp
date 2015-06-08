LPC5410x SPI slave (interrupt mode) with DMA ROM API example
============================

Example description
-------------------
This example shows how to use the SPI slave ROM API to setup and perform
a SPI master transfer in interrupt mode. The DMA ROM API is used to setup
and use DMA for the transfer.

If running a lot of SPI data into this example at a high SPI clock rate,
it will take a long time for the SPI data to be displayed wwith the
(blocking, non-buffered) DEBUG output functions. This SPI may overflow
or underflow if this happens. Either disable DEBUG or slow down the
transfer rate.

Special connection requirements
-------------------------------
To use this example, a SPI master needs to be connected to the SPI slave
signals for SPI0 on the following signals:
SPI0 SCK	P1.3
SPI0 SSEL	P0.14
SPI0 MOSI	P0.12
SPI0 MISO	P1.4

Build procedures:
-----------------
Visit the at 'LPCOpen quickstart guides' [http://www.lpcware.com/content/project/lpcopen-platform-nxp-lpc-microcontrollers/lpcopen-v200-quickstart-guides]
to get started building LPCOpen projects.
