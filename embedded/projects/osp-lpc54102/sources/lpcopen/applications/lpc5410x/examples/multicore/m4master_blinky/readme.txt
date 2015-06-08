Multicore blinky example (M4 core as master, M0 as slave)
=========================================================

Example description
-------------------
This multicore example sets up the M4 core as the MCU master that communicates with
the M0 core running as a slave. This project is meant to work with the m0slave_blinky
project as the slave and the m4master_blinky project as the master.

When this example works correctly and both cores are running, board LEDs 0 and 1
will toggle at about 1Hz. When one LED is on, the other LED is off.

What this example does: m4master_blinky
- Sets up the system via SystemInit() and initializes the board layer via Board_Init()
The M0 slave does not repeat this step.
- Shares a 32-bit value bewteen both cores that contains LED on/off state for board
LEDs 0 and 1 in bit position 0 for the master and bit position 1 for the slave. A
0 or 1 in these bit locations indicates the LED off(0) or on(1) state for board
LEDs 0 or 1, respectively.
- Sets up the mailbox and hardware mutex. This is only performed once by the master
core. Enables the mailbox interrupt.
- Initializes the M0 slave core boot entry address and stack pointer. The startup
code for both the M0 and M4 cores is shared with the M0 core being placed into a
safe, low power state. Once the master sets up the necessary addresses, the M0
slave core is reset and boots with the address and stack given by the master.
Note the shared M0/M4 startup code handles this part of the boot sequence.
- Grabs the hardware mutex for the shared LED state value.
- Sets up a periodic system tick event which toggles the LED0 state value at about
1Hz. This doesn't yet toggle board LED0 yet, only the state value.
- Triggers a mailbox event to the M0 slave core with the address of the shared LED
state value.
- Returns the hardware mutex for the shared LED state value.
There might be a small overlap period where both cores attempt to access the
hardware mutex. The core attempted to get the mutex will keep trying until the
mutex is available.
- At some point, the mailbox will be triggered from the slave core. When this
happens, a mailbox interrupt is generated and both of the board LED states are
updated from the shared LED state value.
- The mailbox interrupt is cleared.
- The master MCU sleeps while not performing any other tasks.

What this example does: m0slave_blinky
- Uses a special version of the startup code that bypasses SystemInit().
- Does not link agains tthe board library (M4 does board init)
- Enables the mailbox interrupt.
- At some point, the mailbox will be triggered from the master core. When this
happens, a mailbox interrupt is generated. The mailbox contains an addrsss that
points to the M4's shared LED state value.
- Grabs the hardware mutex for the shared LED state value.
- The LED1 state value is toggled in the shared LED state value. This doesn't yet
toggle board LED1 yet, only the state value.
- Returns the hardware mutex for the shared LED state value.
- The mailbox interrupt is cleared.
- Triggers a mailbox event to the M4 master core that the M0 slave side is done.
- The slave MCU sleeps while not performing any other tasks.

Build procedures:
-----------------
Visit the http://www.lpcware.com/content/project/lpcopen-platform-nxp-lpc-microcontrollers/lpcopen-v200-quickstart-guides
to get started building LPCOpen projects,
