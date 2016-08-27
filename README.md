# README

### Pre-requisites
#### Hardware
* STM32F0308-Discovery Dev Board
* RFM69HW/W
* USB/Serial for Debuging

#### Software
* arm-none-eabi-gcc (on OSX you can use homebrew and https://github.com/ARMmbed/homebrew-formulae)
* stlink

### Install
1. git clone https://github.com/jamescoxon/stm32f0_ukhasnet.git
2. cd stm32f0_ukhasnet 
3. mv src/settings-example.h src/settings.h
4. git submodule init
5. git submodule update
6. cd libopencm3/
7. make
8. cd..
9. cd src/
10. make
11. ..
12. sh flash.sh

## Board connections

| Port  | Function      | Description                       |
| ----- | ------------- | --------------------------------- |
| `PA9` | `(USART1_TX)` | TTL serial output `(38400,8,N,1)` |
| `PA4` | `(SPI_SS)`    | SPI SS (also know as CSS)         |
| `PA7` | `(SPI_MOSI)`  | SPI MOSI                          |
| `PA6` | `(SPI_MISO)`  | SPI MISO                          |
| `PA5` | `(SPI_SCK)`   | SPI SCK                           |
