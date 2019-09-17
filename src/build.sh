#!/bin/sh

echo "Starting to Compile"
make clean
make
arm-none-eabi-size ukhasnet.elf
echo "Done"
