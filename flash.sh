#!/bin/sh

arm-none-eabi-size src/ukhasnet.elf

arm-none-eabi-objcopy -Obinary src/ukhasnet.elf src/ukhasnet.bin
stlink-1.3.0-macosx-amd64/bin/st-flash write src/ukhasnet.bin 0x8000000


