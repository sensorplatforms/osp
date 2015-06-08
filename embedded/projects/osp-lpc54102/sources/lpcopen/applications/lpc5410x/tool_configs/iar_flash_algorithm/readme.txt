To install for IAR...

1) Locate your IAR install directory.
Example: C:\Program Files (x86)\IAR Systems\Embedded Workbench <version>
<IARINSTALL>=your install directory

2) Copy the following files to the following locations in the IAR install directory.
These files are located in the LPCOpen area in the lpcopen\applications\lpc540xx\tool_configs\iar_flash_algorithm directory
- FlashNXPLPC512KNiobe.board  ----> <IARINSTALL>\arm\config\flashloader\NXP
- FlashNXPLPC512KNiobe.flash  ---> <IARINSTALL>\arm\config\flashloader\NXP
- FlashNXPLPC540xx_RAM32K.out ---> <IARINSTALL>\arm\config\flashloader\NXP
- FlashNXPLPC540xx.mac        ---> <IARINSTALL>\arm\config\flashloader\NXP

- LPC540xx_M0.ddf             ---> <IARINSTALL>\arm\config\debugger\NXP
- LPC540xx_M4.ddf             ---> <IARINSTALL>\arm\config\debugger\NXP
