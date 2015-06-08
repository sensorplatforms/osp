* Software Abstract * 

This software tests the SCT on the Niobe 

* Description *

Since this IP has been used in previous chips, the software primarily tests 
the connectivity of the SCT with other IP on the chip, e.g., the NVIC, IOCON, ADC,
etc.  

The test is mostly automated.  Connect a terminal and make other connections as described in 
* Test Setup * below.  When the program executes it will give instructions regarding other jumpers 
that need to be connected and measurements that need to be made.

Test results are displayed on the terminal. 


* Test Setup *

Using an FTDI USB-Serial cable connect a terminal (9600, n, 8, 1, no parity) as indicated below:

P0.22 (U0_RXD)   	<-------> FTDI-TX
P0.21 (U0_TXD)   	<-------> FTDI-RX
Ground 		 		<-------> FTDI-Ground


* Miscellaneous *
 