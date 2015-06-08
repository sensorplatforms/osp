LPC5410x Brown-out detector example
=================================

Example description
-------------------
The brown-out example shows how to use the brown-out detector (BOD)
on the LPC5410x. The BOD is setup to generate a BOD interrupt when
power drops below the highest BOD detection level. The interrupt will
attempt to toggle the LED on when this happens.

To use this example, build and program it and then run it on the board.
Power off the board by removing the power (USB) cable. As the power is
declining on the board, the LED will toggle on (quickly) and then turn
off as power is lost.

Special connection requirements
-------------------------------
There are no special connection requirements for this example.

Build procedures:
-----------------
Visit the at 'LPCOpen quickstart guides' [http://www.lpcware.com/content/project/lpcopen-platform-nxp-lpc-microcontrollers/lpcopen-v200-quickstart-guides]
to get started building LPCOpen projects.
