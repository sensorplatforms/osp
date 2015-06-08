LPC5410x Wakeup example
===============================================

Example description
-------------------
This example shows how to wakeup the chip from the different low power
modes. All power modes are available: sleep, deep sleep, power down, and
deep power down. Note that wakeup from power down modes requires a
system reset - the example will determine that this type of wakeup
occurred to properly handle the next wakeup event.

Sleep modes can be selected by changing the PDOWNMODE define.  For this example,
the possible modes are POWER_SLEEP, POWER_DEEP_SLEEP, POWER_POWER_DOWN, and
POWER_DEEP_POWER_DOWN.

The repetitive timer is used as the wakeup source and will wake the chip
up after 5 seconds. The WAKE button can also be used as a wakeup
source before the timer has woke up the chip. The LED will turn off when
the chip is placed into sleep mode and turn on when the chip is awake.

SPECIAL NOTES:
In sleep mode, all clocks are left enabled. The repetitive timer tick
rate is initially set up using the main clock with an IRC source;
although the example tends to mostly use PLL as the main clock.
Setting up the repetitive timer tick this was allows the tick to
use the IRC rate without the PLL during sleep. The sleep mode example
averages approximately 1.5mA.

In deep sleep mode, the main clock (and other clocks) are disabled.
Since the repetitive timer uses the main clock as it's source, it will
no longer wakeup the device from deep sleep mode, use the WAKE button for
wakeup. The deep sleep mode example averages approximately 300uA.

In power down mode, additional clocks are powered down over deep sleep
mode, use the WAKE button for wakeup. When using J6 (link) to power the chip
(and debug), the example uses approximately 220uA. When using J4 (target) to power
the chip, the example uses approximately 28uA.

In deep power down mode, All system clocks are disabled. Device wakeup
can occur by asserting the reset pin; RTC and BOD also can be used for
wakeup from this mode. The deep power down mode averages approximately 
15uA.

Power measurement notes
-----------------------
1) Power measurement was taken by removing JS6 and measuring current
across JP4.
2) Power measurements above are for the processor only. The rest of the board was
powered by the standard board power supplies.

Special connection requirements
-------------------------------
There are no special connection requirements for this example.

Build procedures:
-----------------
Visit the at 'LPCOpen quickstart guides' [http://www.lpcware.com/content/project/lpcopen-platform-nxp-lpc-microcontrollers/lpcopen-v200-quickstart-guides]
to get started building LPCOpen projects.
