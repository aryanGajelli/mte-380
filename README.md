# mte-380
Repo for MTE 380 design course work

## Pre-requisites
All of the following must be installed and available in your path:
- make
- arm-none-eabi-gcc
- STM32_Programmer_CLI

## Building
To build the project, run `make` in the root directory. This will build the project and generate a binary file in the `Cube-files\build` directory.

## Flashing
To flash the project, run `make flash` in the root directory. This will flash the binary file to the board using the STM32_Programmer_CLI tool.