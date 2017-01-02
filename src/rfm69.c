// RFM69.c
//
// Ported to Arduino 2014 James Coxon
//
// Ported to bare metal AVR 2014 Jon Sowman
//
// Ported to STM32 Matt Brejza
//
// Copyright (C) 2014 Phil Crump
// Copyright (C) 2014 Jon Sowman <jon@jonsowman.com>
//
// Based on RF22 Copyright (C) 2011 Mike McCauley ported to mbed by Karl Zweimueller
// Based on RFM69 LowPowerLabs (https://github.com/LowPowerLab/RFM69/)
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/rcc.h>
#include <inttypes.h>
#include <string.h>
#include "rfm69.h"
#include "misc.h"
#include "RFM69Config.h"

//volatile uint8_t    _mode;

volatile uint8_t _bufLen;
uint8_t _buf[RFM69_MAX_MESSAGE_LEN];
volatile uint8_t _rxBufValid;
float    _temperatureFudge;
int16_t  _lastRssi;
int16_t _rssi_threshold;

/**
 * Assert SS on the RFM69 for communications.
 */
#define RFM_SS_ASSERT() do { GPIO_ODR(SPI_PORT) &= ~(SPI_SS); } while(0)

/**
 * Release SS on the RFM69 to abort or terminate comms
 */
#define RFM_SS_DEASSERT() do { GPIO_ODR(SPI_PORT) |= (SPI_SS); } while(0)

/** Track the current mode of the radio */
static uint8_t _mode;

/**
 * Initialise the RFM69 device.
 * @returns 0 on failure, nonzero on success
 */
bool rf69_init(void)
{
    uint8_t i;
    
    rcc_periph_clock_enable(R_RCC_SPI);
    rcc_periph_clock_enable(R_RCC_GPIO);
    gpio_mode_setup(SPI_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, R_SPI_PINS);
    gpio_set_af(SPI_PORT, R_SPI_AFn, R_SPI_PINS);
    gpio_mode_setup(SPI_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, SPI_SS);
    
    // Reset and enable the SPI periph
    spi_reset(R_SPI);
    spi_init_master(R_SPI, SPI_CR1_BAUDRATE_FPCLK_DIV_64,
                    SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
                    SPI_CR1_CPHA_CLK_TRANSITION_1,
                    SPI_CR1_CRCL_8BIT,
                    SPI_CR1_MSBFIRST);
    
    // Trigger an RXNE event when we have 8 bits (one byte) in the buffer
    spi_fifo_reception_threshold_8bit(R_SPI);
    
    // NSS must be set high for the peripheral to function
    spi_enable_software_slave_management(R_SPI);
    spi_set_nss_high(R_SPI);
    gpio_set(SPI_PORT, SPI_SS);
    
    // Enable
    spi_enable(R_SPI);
    
    RFM_SS_DEASSERT();
    
    delay_ms(100);
    
    // Set up device
    for(i = 0; CONFIG[i][0] != 255; i++)
        rf69_spiWrite(CONFIG[i][0], CONFIG[i][1]);
    
    /* Set initial mode */
    _mode = RFM69_MODE_RX;
    rf69_setMode(_mode);
    
    // Zero version number, RFM probably not connected/functioning
    if(!rf69_spiRead(RFM69_REG_10_VERSION))
        return false;
    
    return true;
}

/**
 * Read a single byte from a register in the RFM69. Transmit the (one byte)
 * address of the register to be read, then read the (one byte) response.
 * @param reg The register address to be read
 * @returns The value of the register
 */
uint8_t rf69_spiRead(const uint8_t reg)
{
    while(SPI_SR(R_SPI) & SPI_SR_BSY);
    gpio_clear(SPI_PORT, SPI_SS);
    spi_send8(R_SPI, reg);
    spi_read8(R_SPI);
    // Wait until send FIFO is empty
    while(SPI_SR(R_SPI) & SPI_SR_BSY);
    spi_send8(R_SPI, 0x0);
    while(SPI_SR(R_SPI) & SPI_SR_BSY);
    uint8_t out = spi_read8(R_SPI);
    gpio_set(SPI_PORT, SPI_SS);
    return out;
}

/**
 * Write a single byte to a register in the RFM69. Transmit the register
 * address (one byte) with the write mask RFM_SPI_WRITE_MASK on, and then the
 * value of the register to be written.
 * @param reg The address of the register to write
 * @param val The value for the address
 */
void rf69_spiWrite(const uint8_t reg, const uint8_t val)
{
    while(SPI_SR(R_SPI) & SPI_SR_BSY);
    gpio_clear(SPI_PORT, SPI_SS);
    spi_send8(R_SPI, reg | RFM69_SPI_WRITE_MASK);
    spi_read8(R_SPI);
    // Wait until send FIFO is empty
    while(SPI_SR(R_SPI) & SPI_SR_BSY);
    spi_send8(R_SPI, val);
    spi_read8(R_SPI);
    // Wait until send FIFO is empty
    while(SPI_SR(R_SPI) & SPI_SR_BSY);
    gpio_set(SPI_PORT, SPI_SS);
}

/**
 * Read a given number of bytes from the given register address into a provided
 * buffer
 * @param reg The address of the register to start from
 * @param dest A pointer into the destination buffer
 * @param len The number of bytes to read
 */
void rf69_spiBurstRead(const uint8_t reg, uint8_t* dest, uint8_t len)
{
    while(SPI_SR(R_SPI) & SPI_SR_BSY);
    gpio_clear(SPI_PORT, SPI_SS);
    spi_send8(R_SPI, reg & 0x7F);
    spi_read8(R_SPI);
    // Wait until send FIFO is empty
    while(SPI_SR(R_SPI) & SPI_SR_BSY);
    while(len--)
    {
        spi_send8(R_SPI, 0x0);
        while(SPI_SR(R_SPI) & SPI_SR_BSY);
        *dest++ = spi_read8(R_SPI);
    }
    gpio_set(SPI_PORT, SPI_SS);
}

/**
 * Write a given number of bytes into the registers in the RFM69.
 * @param reg The first byte address into which to write
 * @param src A pointer into the source data buffer
 * @param len The number of bytes to write
 */
void rf69_spiBurstWrite(uint8_t reg, const uint8_t* src, uint8_t len)
{
    while(SPI_SR(R_SPI) & SPI_SR_BSY);
    gpio_clear(SPI_PORT, SPI_SS);
    spi_send8(R_SPI, reg | RFM69_SPI_WRITE_MASK);
    spi_read8(R_SPI);
    // Wait until send FIFO is empty
    while(len--)
    {
        while(SPI_SR(R_SPI) & SPI_SR_BSY);
        spi_send8(R_SPI, *src++);
        spi_read8(R_SPI);
    }
    // Wait until send FIFO is empty
    while(SPI_SR(R_SPI) & SPI_SR_BSY);
    gpio_set(SPI_PORT, SPI_SS);
}

/**
 * Write data into the FIFO on the RFM69
 * @param src The source data comes from this buffer
 * @param len Write this number of bytes from the buffer into the FIFO
 */
void rf69_spiFifoWrite(const uint8_t* src, uint8_t len)
{
    
    while(SPI_SR(R_SPI) & SPI_SR_BSY);
    gpio_clear(SPI_PORT, SPI_SS);
    spi_send8(R_SPI, RFM69_REG_00_FIFO | RFM69_SPI_WRITE_MASK);
    spi_read8(R_SPI);
    // Wait until send FIFO is empty
    while(SPI_SR(R_SPI) & SPI_SR_BSY);
    spi_send8(R_SPI, len);
    spi_read8(R_SPI);
    while(len--)
    {
        while(SPI_SR(R_SPI) & SPI_SR_BSY);
        spi_send8(R_SPI, *src++);
        spi_read8(R_SPI);
    }
    while(SPI_SR(R_SPI) & SPI_SR_BSY);
    gpio_set(SPI_PORT, SPI_SS);
}

/**
 * Change the RFM69 operating mode to a new one.
 * @param newMode The value representing the new mode (see datasheet for
 * further information).
 */
void rf69_setMode(const uint8_t newMode)
{
    rf69_spiWrite(RFM69_REG_01_OPMODE, (rf69_spiRead(RFM69_REG_01_OPMODE) & 0xE3) | newMode);
    _mode = newMode;
}

void clearFifo(void)
{
    rf69_setMode(RFM69_MODE_STDBY);
    rf69_setMode(RFM69_MODE_RX);
}

uint8_t checkRx()
{
    // Check IRQ register for payloadready flag (indicates RXed packet waiting in FIFO)
    if(rf69_spiRead(RFM69_REG_28_IRQ_FLAGS2) & RF_IRQFLAGS2_PAYLOADREADY) {
        // Get packet length from first byte of FIFO
        _bufLen = rf69_spiRead(RFM69_REG_00_FIFO)+1;
        // Read FIFO into our Buffer
        rf69_spiBurstRead(RFM69_REG_00_FIFO, _buf, RFM69_FIFO_SIZE);
        // Read RSSI register (should be of the packet? - TEST THIS)
        _lastRssi = -(rf69_spiRead(RFM69_REG_24_RSSI_VALUE)/2);
        // Clear the radio FIFO (found in HopeRF demo code)
        clearFifo();
        return true;
    }
    
    return false;
}

void rf69_recv(char* buf, uint8_t* len)
{
    // Copy RX Buffer to byref Buffer
    memcpy(buf, _buf, _bufLen);
    *len = _bufLen;
    // Clear RX Buffer
    _bufLen = 0;
}

/**
 * Send a packet using the RFM69 radio.
 * @param data The data buffer that contains the string to transmit
 * @param len The number of bytes in the data packet (excluding preamble, sync
 * and checksum)
 * @param power The transmit power to be used in dBm
 */
void rf69_send(const uint8_t* data, uint8_t len, uint8_t power)
{
    uint8_t oldMode, paLevel;
    // power is TX Power in dBmW (valid values are 2dBmW-20dBmW)
    if(power < 2 || power > 20)
    {
        // Could be dangerous, so let's check this
        return;
    }
    
    oldMode = _mode;
    
    // Start Transmitter
    rf69_setMode(RFM69_MODE_TX);
    
    // Set up PA
    if(power <= 17)
    {
        // Set PA Level
        paLevel = power + 28;
        rf69_spiWrite(RFM69_REG_11_PA_LEVEL, RF_PALEVEL_PA0_ON | RF_PALEVEL_PA1_OFF | RF_PALEVEL_PA2_OFF | paLevel);
    } else {
        // Disable Over Current Protection
        rf69_spiWrite(RFM69_REG_13_OCP, RF_OCP_OFF);
        // Enable High Power Registers
        rf69_spiWrite(RFM69_REG_5A_TEST_PA1, 0x5D);
        rf69_spiWrite(RFM69_REG_5C_TEST_PA2, 0x7C);
        // Set PA Level
        paLevel = power + 11;
        rf69_spiWrite(RFM69_REG_11_PA_LEVEL, RF_PALEVEL_PA0_OFF | RF_PALEVEL_PA1_ON | RF_PALEVEL_PA2_ON | paLevel);
    }
    
    // Wait for PA ramp-up
    while(!(rf69_spiRead(RFM69_REG_27_IRQ_FLAGS1) & RF_IRQFLAGS1_TXREADY));
    
    // Throw Buffer into FIFO, packet transmission will start automatically
    rf69_spiFifoWrite(data, len);
    
    // Wait for packet to be sent
    while(!(rf69_spiRead(RFM69_REG_28_IRQ_FLAGS2) & RF_IRQFLAGS2_PACKETSENT));
    
    // Return Transceiver to original mode
    rf69_setMode(oldMode);
    
    // If we were in high power, switch off High Power Registers
    if(power > 17)
    {
        // Disable High Power Registers
        rf69_spiWrite(RFM69_REG_5A_TEST_PA1, 0x55);
        rf69_spiWrite(RFM69_REG_5C_TEST_PA2, 0x70);
        // Enable Over Current Protection
        rf69_spiWrite(RFM69_REG_13_OCP, RF_OCP_ON | RF_OCP_TRIM_95);
    }
}

/*void RFM69::SetLnaMode(uint8_t lnaMode) {*/
/*// RF_TESTLNA_NORMAL (default)*/
/*// RF_TESTLNA_SENSITIVE*/
/*spiWrite(RFM69_REG_58_TEST_LNA, lnaMode);*/
/*}*/

/**
 * The RFM69 has an onboard temperature sensor, read its value
 * @warning RFM69 must be in one of the active modes for temp sensor to work.
 * @returns The temperature in degrees C or 255.0 for failure
 */
int8_t rf69_readTemp(void)
{
    // Store current transceiver mode
    uint8_t oldMode, rawTemp, timeout;
    
    oldMode = _mode;
    // Set mode into Standby (required for temperature measurement)
    rf69_setMode(RFM69_MODE_STDBY);
    
    // Trigger Temperature Measurement
    rf69_spiWrite(RFM69_REG_4E_TEMP1, RF_TEMP1_MEAS_START);
    
    // Check Temperature Measurement has started
    timeout = 0;
    while(!(RF_TEMP1_MEAS_RUNNING & rf69_spiRead(RFM69_REG_4E_TEMP1)))
    {
        delay_ms(1);
        if(++timeout > 50)
            return -127.0;
        rf69_spiWrite(RFM69_REG_4E_TEMP1, RF_TEMP1_MEAS_START);
    }
    
    // Wait for Measurement to complete
    timeout = 0;
    while(RF_TEMP1_MEAS_RUNNING & rf69_spiRead(RFM69_REG_4E_TEMP1))
    {
        delay_ms(1);
        if(++timeout > 10)
            return -127.0;
    }
    
    // Read raw ADC value
    rawTemp = rf69_spiRead(RFM69_REG_4F_TEMP2);
    
    // Set transceiver back to original mode
    rf69_setMode(oldMode);
    
    // Return processed temperature value
    return 161 - (int8_t)rawTemp;
}

int16_t rf69_sampleRssi() {
    // Must only be called in RX mode
    if(_mode!=RFM69_MODE_RX) {
        // Not sure what happens otherwise, so check this
        return 0;
    }
    // Trigger RSSI Measurement
    rf69_spiWrite(RFM69_REG_23_RSSI_CONFIG, RF_RSSI_START);
    // Wait for Measurement to complete
    while(!(RF_RSSI_DONE && rf69_spiRead(RFM69_REG_23_RSSI_CONFIG))) { };
    // Read, store in _lastRssi and return RSSI Value
    _lastRssi = -(rf69_spiRead(RFM69_REG_24_RSSI_VALUE)/2);
    return _lastRssi;
}

int16_t rf69_lastRssiThreshold() {
    return -(_rssi_threshold/2);
}

int16_t rf69_lastRssi() {
    return _lastRssi;
}