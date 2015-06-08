LPC5410x I2C bus monitor ROM API example
============================

Example description
-------------------
This example shows how to use the I2C Monitor ROM API to setup the I2C
bus for monitoring. The example sets up the I2C bus monitor and displays
each captured message on the DEBUG output.

Note that I2C clock stretching is used with this example and may alter
I2C bus timing.

Special connection requirements
-------------------------------
To use this example, connect the I2C0 SCL and SDA signals to an I2C
bus to monitor.

Build procedures:
-----------------
Visit the at 'LPCOpen quickstart guides' [http://www.lpcware.com/content/project/lpcopen-platform-nxp-lpc-microcontrollers/lpcopen-v200-quickstart-guides]
to get started building LPCOpen projects.
