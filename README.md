# UF2 Bootloader

This repository contains a bootloader, derived from Atmel's SAM-BA,
which in addition to the USB CDC (serial) protocol, also supports
the USB MSC (mass storage).

## UF2

**UF2 (USB Flashing Format)** is a name of a file format, developed by Microsoft, that is particularly
suitable for flashing devices over MSC devices. The file consists
of 512 byte blocks, each of which is self-contained and independent
of others.

Each 512 byte block consist of (see `uf2format.h` for details):
* magic numbers at the beginning and at the end
* address where the data should be flashed
* size of data
* data (up to 476 bytes; for SAMD it's 256 bytes so it's easy to flash in one go)

Thus, it's really easy for the microcontroller to recognize a block of
a UF2 file is written and immediately write it to flash.

* **UF2 specification repo:** https://github.com/Microsoft/uf2
* [#DeskOfLadyada UF24U ! LIVE @adafruit #adafruit #programming](https://youtu.be/WxCuB6jxLs0)

## Features

* USB CDC (Serial emulation) monitor mode compatible with Arduino
  (including XYZ commands) and BOSSA flashing tool
* USB MSC interface for writing UF2 files
* reading of the contests of the flash as an UF2 file via USB MSC
* UART Serial (real serial wire) monitor mode (typically disabled due to space constraints)
* In-memory logging for debugging - use the `logs` target to extract the logs using `openocd`
* double-tap reset to stay in the bootloader mode
* automatic reset after UF2 file is written

## Board identification

Configuration files for board `foo` are in `boards/foo/board_config.h` and `board.mk`. You can
build it with `make BOARD=foo`. You can also create `Makefile.user` file with `BOARD=foo`
to change the default.

The board configuration specifies the USB vendor/product name and ID,
as well as the volume label (main thing that the operating systems show).

There is also `BOARD_ID`, which is meant to be machine-readable and specific
to a given version of board hardware. The programming environment might use
this to suggest packages to be imported (i.e., a package for a particular
external flash chip, SD card etc.).

These configuration values can be read from `INFO_UF2.TXT` file.
Presence of this file can be tested to see if the board supports `UF2` flashing,
while contest, particularly `Board-ID` field, can be used for feature detection.

The current flash contents of the board is exposed as `CURRENT.UF2` file.
This file includes the bootloader address space. The last word of bootloader
space points to the string holding the `INFO_UF2.TXT` file, so it can be parsed
by a programming environment to determine which board does the `.UF2` file comes from.

## Handover

When the user space application implements the USB MSC protocol, it's possible to
handover execution to the bootloader in the middle of MSC file transfer,
when the application detects that a UF2 block is written.

Details are being finalized.

## Bootloader update

The bootloader will never write to its own flash area directly.
However, the user code can write there.
Thus, to update the bootloader, one can ship a user-space program,
that contains the new version of the bootloader and copies it to the
appropriate place in flash.

Such a program is generated during build in files `update-bootloader*.uf2`.
If you're already running UF2 bootloader, the easiest way to update
it, is to just copy this file to the exposed MSD drive.

## Build

### Requirements

* `make` and an Unix environment
* `arm-none-eabi-gcc` in the path (the one coming with Yotta will do just fine). You can get the latest version from ARM: https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads
* `openocd` - you can use the one coming with Arduino (after your install the M0 board support)

Atmel Studio is not supported.

You will need a board with `openocd` support.

Arduino Zero (or M0 Pro) will work just fine as it has an integrated USB EDBG
port. You need to connect both USB ports to your machine to debug - one is for
flashing and getting logs, the other is for the exposed MSC interface.

### Configuration

There is a number of configuration parameters at the top of `uf2.h` file.
Adjust them to your liking.

By default, you cannot enable all the features, as the bootloader would exceed
the 8k allocated to it by Arduino etc. It will assert on startup that it's not bigger
than 8k. Also, the linker script will not allow it.

Three typical configurations are:

* HID, WebUSB, MSC, plus flash reading via FAT; UART and CDC disabled;
  logging optional; **recommended**
* USB CDC and MSC, plus flash reading via FAT; UART disabled;
  logging optional; this may have Windows driver problems
* USB CDC and MSC, no flash reading via FAT (or at least `index.htm` disabled); UART enabled;
  logging disabled; no handover; no HID;
  only this one if you need the UART support in bootloader for whatever reason

CDC and MSC together will work on Linux and Mac with no drivers.
On Windows, if you have drivers installed for the USB ID chosen,
then CDC might work and MSC will not work;
otherwise, if you have no drivers, MSC will work, and CDC will work on Windows 10 only.
Thus, it's best to set the USB ID to one for which there are no drivers.

The bootloader sits at 0x00000000, and the application starts at 0x00002000.

## Code of Conduct

This project has adopted the [Microsoft Open Source Code of Conduct](https://opensource.microsoft.com/codeofconduct/). For more information see the [Code of Conduct FAQ](https://opensource.microsoft.com/codeofconduct/faq/) or contact [opencode@microsoft.com](mailto:opencode@microsoft.com) with any additional questions or comments.

## License

See THIRD-PARTY-NOTICES.txt for the original SAM-BA bootloader license from Atmel.

The new code is licensed under MIT.
