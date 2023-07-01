[![Build firmware](https://github.com/PatrickBaus/SCAN2000_Firmware/actions/workflows/ci.yml/badge.svg)](https://github.com/PatrickBaus/SCAN2000_Firmware/actions/workflows/ci.yml)
# Keithley SCAN2000 SSR Replacement Firmware
This repository contains the firmware for the Keithley SCAN2000 SSR replacement pcb found [here](https://github.com/PatrickBaus/SCAN2000).

## Contents
- [Description](#description)
- [Installation](#installation)
- [Compiled binaries](#compiled-binaries)
- [Debugging](#debugging)
- [Related Repositories](#related-repositories)
- [Versioning](#versioning)
- [License](#license)

## Description
The source code is based on the project by [George Christidis](https://github.com/macgeorge/SCAN2000STM32). It uses CubeMX instead and it makes use of the STM32 HAL labraries instead of the low-level libraries. It can be built using a Makefile.

## Installation

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

## Compiled binaries
A compiled version can be found on the the [releases](../../releases) page.

## Debugging
If you want to read back what the MCU is doing, hook up a TTL to USB converter to the TX and GND pins. I use a [Waveshare Industrial USB TO TTL Converter](https://www.waveshare.com/usb-to-ttl.htm), but any of the cheap Ebay FT232 modules works. The parameters are 115200 baud, 8 bits, no parity, 1 stop bit.

To configure a Linux tty, type the following commands. Do make sure to change the tty to your tty. This example uses */dev/ttyUSB5*.

```bash
stty -F /dev/ttyUSB5 115200 cs8 -cstopb -parenb echo -echoe -echok -echoctl -igncr -icanon
```

## Related Repositories
See the following repositories for more information

Keithley SCAN2000 Hardware: https://github.com/PatrickBaus/SCAN2000

## Versioning
I use [SemVer](http://semver.org/) for versioning. For the versions available, see the [tags](../../tags) available for this repository.

- MAJOR versions in this context mean a breaking change to the external interface like changed commands or functions.
- MINOR versions contain changes that only affect the inner workings of the software, but otherwise the performance is unaffected.
- PATCH versions do not add, remove or change any features. They contain small changes like fixed typos.

## License
This project is licensed under the GPL v3 license - see the [LICENSE](LICENSE) file for details
