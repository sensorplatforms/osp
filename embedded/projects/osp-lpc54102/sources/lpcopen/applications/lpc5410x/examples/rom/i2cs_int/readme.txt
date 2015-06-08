LPC5410x I2C slave (interrupt) ROM API example
============================

Example description
-------------------
This example shows how to use the I2C slave ROM API to setup the I2C
slave bus for a transfer. Once the transfer starts, interrupts are
used to handle the slave transfer. Transfer completion can be determined
by looking at the transfer status or using the I2C completion event
callback.

Special connection requirements
-------------------------------
To use this example, an I2C bus master must be connected to the I2C
slave signals.  The master should use I2C address 0x18.

Build procedures:
-----------------
Visit the at 'LPCOpen quickstart guides' [http://www.lpcware.com/content/project/lpcopen-platform-nxp-lpc-microcontrollers/lpcopen-v200-quickstart-guides]
to get started building LPCOpen projects.
