#!/bin/sh

make clean
make
arm-none-eabi-size ukhasnet.elf
