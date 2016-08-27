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
3. git submodule init
4. git submodule update
5. cd libopencm3/
6. make
7. cd..
8. cd src/
9. make
10. ..
11. sh flash.sh

## Board connections

| Port  | Function      | Description                       |
| ----- | ------------- | --------------------------------- |
| `PA9` | `(USART1_TX)` | TTL serial output `(38400,8,N,1)` |
| `PA4` | `(SPI_SS)`    | SPI SS (also know as CSS)         |
| `PA7` | `(SPI_MOSI)`  | SPI MOSI                          |
| `PA6` | `(SPI_MISO)`  | SPI MISO                          |
| `PA5` | `(SPI_SCK)`   | SPI SCK                           |
