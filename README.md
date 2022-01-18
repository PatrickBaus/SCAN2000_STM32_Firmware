Keithley SCAN2000 SSR Replacement Firmware
===================

This repository contains the firmware for the Keithley SCAN2000 SSR replacement pcb found [here](https://github.com/PatrickBaus/SCAN2000).

About
-----
A compiled version can be found in the [/build](/build) folder.

Description
-------------------
The source code is based on the project by [George Christidis](https://github.com/macgeorge/SCAN2000STM32). It uses CubeMX instead and it makes use of the STM32 HAL labraries instead of the low-level libraries. It can be built using a Makefile.

To flash the SMT32G0, I use an ST-Link programmer along with [STM32CubeProgrammer](https://www.st.com/en/development-tools/stm32cubeprog.html).

To flash the firmware to the microcontroller do the following:
1. Compile the code using the Makefile. In Linux type *make*. Alternatively use the compiled firware in the [/build](/build) folder.
2. Power the MCU via the 5V pin. It should draw about 40 mA @ 5V and the *Power* led should be lit.
3. Connect the ST-link programmer via the header and the adapter board.
4. Open STM32CubeProgrammer and press connect
5. Now open load the .hex file and press download. STM32CubeProgrammer should report success and the board current should now drop to < 10 mA.
6. In the STM32CubeProgrammer window press disconnect. The board power typically increases by 1 mA.
7. Disconnect the ST-Link
8. Reset the MCU. The *Activity* led should flash twice to signal a successful boot. The board should now draw about 15 mA during idle and both LEDs should be lit.

Related Repositories
--------------------

See the following repositories for more information

Keithley SCAN2000 Hardware: https://github.com/PatrickBaus/SCAN2000

License
-------
This project is licensed under the GPL v3 license - see the [LICENSE](LICENSE) file for details
