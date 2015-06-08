LPC5410x ADC polling example using the ROM API
=================================

Example description
-------------------
This example shows how to configure the ADC ROM driver to capture
ADC data on sequencer A and B in polling mode. Logged ADC data will
appear on the UART port.

When the program runs, it will capture all the ADC channel data using
sequencer A. Once the data is complete, press the wakeup button to
capture all the ADC channel data using sequencer B.

Special connection requirements
-------------------------------
Use the UART bridge to connect a terminal program to the VCOM serial
port for the board.

Build procedures:
-----------------
Visit the at 'LPCOpen quickstart guides' [http://www.lpcware.com/content/project/lpcopen-platform-nxp-lpc-microcontrollers/lpcopen-v200-quickstart-guides]
to get started building LPCOpen projects.
