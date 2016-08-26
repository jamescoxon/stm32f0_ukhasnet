#!/bin/sh

arm-none-eabi-objcopy -Obinary ukhasnet.elf ukhasnet.bin
st-flash write ukhasnet.bin 0x8000000


