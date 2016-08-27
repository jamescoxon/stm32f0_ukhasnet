/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
 * Copyright (C) 2011 Stephen Caudle <scaudle@doceme.com>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#define _GNU_SOURCE
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <stdio.h>
#include <errno.h>
#include <stddef.h>
#include <sys/types.h>
#include <string.h>

#include "settings.h"

#define RFM69_MODE_SLEEP    0x00 // 0.1uA
#define RFM69_MODE_STDBY    0x04 // 1.25mA
#define RFM69_MODE_RX       0x10 // 16mA
#define RFM69_MODE_TX       0x0c // >33mA

#define RF_PACKET2_RXRESTART        0x04
#define RFM69_REG_3D_PACKET_CONFIG2 0x3D
#define RF_IRQFLAGS1_TIMEOUT        0x04
#define RFM69_REG_27_IRQ_FLAGS1     0x27

uint8_t data_count = 96; // 'a' - 1 (as the first function will at 1 to make it 'a'
unsigned int rx_packets = 0, random_output = 0, rx_restarts = 0;
int16_t rx_rssi, floor_rssi, rssi_threshold, adc_result = 0;
char data_temp[66];
FILE *fp;

static ssize_t _iord(void *_cookie, char *_buf, size_t _n);
static ssize_t _iowr(void *_cookie, const char *_buf, size_t _n);

#include "rfm69.h"

/*
bool rf69_init(void);
uint8_t rf69_spiRead(const uint8_t reg);
void rf69_spiWrite(const uint8_t reg, const uint8_t val);
void rf69_spiBurstRead(const uint8_t reg, uint8_t* dest, uint8_t len);
void rf69_spiBurstWrite(uint8_t reg, const uint8_t* src, uint8_t len);
void rf69_spiFifoWrite(const uint8_t* src, uint8_t len);
void rf69_setMode(const uint8_t newMode);
void rf69_send(const uint8_t* data, uint8_t len, uint8_t power);
uint8_t checkRx(void);
void rf69_recv(uint8_t* buf, uint8_t* len);
void clearFifo(void);
int8_t rf69_readTemp(void);
int16_t rf69_sampleRssi(void);
int16_t rf69_lastRssiThreshold(void);
int16_t rf69_lastRssi(void);
void delay_ms(int msec_delay);
void incrementPacketCount(void);
void transmitData(uint8_t i);
void awaitData(int countdown);
 */

static ssize_t _iord(void *_cookie, char *_buf, size_t _n)
{
	/* dont support reading now */
	(void)_cookie;
	(void)_buf;
	(void)_n;
	return 0;
}

static ssize_t _iowr(void *_cookie, const char *_buf, size_t _n)
{
	uint32_t dev = (uint32_t)_cookie;

	int written = 0;
	while (_n-- > 0) {
		usart_send_blocking(dev, *_buf++);
		written++;
	};
	return written;
}


static FILE *usart_setup(uint32_t dev)
{
	/* Setup USART1 parameters. */
	usart_set_baudrate(dev, 38400);
	usart_set_databits(dev, 8);
	usart_set_parity(dev, USART_PARITY_NONE);
	usart_set_stopbits(dev, USART_CR2_STOP_1_0BIT);
	usart_set_mode(dev, USART_MODE_TX_RX);
	usart_set_flow_control(dev, USART_FLOWCONTROL_NONE);

	/* Finally enable the USART. */
	usart_enable(dev);

	cookie_io_functions_t stub = { _iord, _iowr, NULL, NULL };
	FILE *fp = fopencookie((void *)dev, "rw+", stub);
	/* Do not buffer the serial line */
	setvbuf(fp, NULL, _IONBF, 0);
	return fp;

}

static void clock_setup(void)
{
    //rcc_clock_setup_in_hsi_out_48mhz();
    
	/* Enable GPIOC clock for LED & USARTs. */
	rcc_periph_clock_enable(RCC_GPIOC);
	rcc_periph_clock_enable(RCC_GPIOA);

	/* Enable clocks for USART1. */
	rcc_periph_clock_enable(RCC_USART1);
}

static void gpio_setup(void)
{
	/* Setup GPIO pin GPIO8/9 on GPIO port C for LEDs. */
	gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO8 | GPIO9);

	/* Setup GPIO pins for USART1 transmit. */
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9);

	/* Setup USART1 TX pin as alternate function. */
	gpio_set_af(GPIOA, GPIO_AF1, GPIO9);
}

/**
 * Increments packet count which is transmitted in the beginning of each
 * packet. This function has to be called every packet which is initially
 * transmitted by this node.
 * Packet count is starting with 'a' and continues up to 'z', thereafter
 * it's continuing with 'b'. 'a' is only transmitted at the very first
 * transmission!
 */
void incrementPacketCount(void) {
    data_count++;
    // 98 = 'b' up to 122 = 'z'
    if(data_count > 122) {
        data_count = 98; //'b'
    }
}

void delay_ms(int msec_delay){
    int i, j = 0;
    while (j < msec_delay){
        for (i = 0; i < 1000; i++) {	/* Wait a bit. */
            __asm__("NOP");
        }
        j++;
    }
}

/**
 * Packet data transmission
 * @param Packet length
 */
void transmitData(uint8_t i) {
    
#ifdef GATEWAY
    fprintf(fp, "rx: %s|0\r\n", data_temp);
#endif
    
    // Transmit the data (need to include the length of the packet and power in dbmW)
    rf69_send((uint8_t*)data_temp, i, POWER_OUTPUT);
    
//#ifdef ZOMBIE_MODE
    //Ensure we are in Sleep mode to save power
//    rf69_setMode(RFM69_MODE_SLEEP);
//#else
    //Ensure we are in RX mode
    rf69_setMode(RFM69_MODE_RX);
    
    delay_ms(500);
//#endif

}

/**
 * This function is called when a packet is received by the radio. It will
 * process the packet.
 */
inline void processData(uint32_t len) {
    uint8_t i, packet_len;
    
    for(i=0; i<len; i++) {
        //finds the end of the packet
        if(data_temp[i] != ']')
            continue;
        
        //then terminates the string, ignore everything afterwards
        data_temp[i+1] = '\0';
        
        //Check validity of string
        // 1) is the first position in array a number
        //printf("%d\r\n", data_temp[0]);
        if((int)data_temp[0] <= 48 || (int)data_temp[0] > 57) {
            //printf("Error1\r\n");
            break;
        }
        
        // 2) is the second position in array a letter
        //      < 'a' or > 'z' then break
        //printf("%d\r\n", data_temp[1]);
        if((int)data_temp[1] < 97 || (int)data_temp[1] > 122){
            //printf("Error2\r\n");
            break;
        }
        
#ifdef GATEWAY
        fprintf(fp, "rx: %s|%d\r\n",data_temp, rf69_lastRssi());
#endif
        //Reduce the repeat value
        data_temp[0] = data_temp[0] - 1;
        //Now add , and end line and let string functions do the rest
        data_temp[i] = ',';
        data_temp[i+1] = '\0';
        
        if(strstr(data_temp, NODE_ID) != 0)
            break;
        
        strcat(data_temp, NODE_ID); // Add ID
        strcat(data_temp, "]"); // Add ending
        
        packet_len = strlen((char*)data_temp);
        delay_ms(random_output); // Random delay to try and avoid packet collision
        
        rx_packets++;
        
        transmitData(packet_len);
        break;
    }
}

void awaitData(int countdown) {
    
    uint8_t rx_len;
    
    //Clear buffer
    data_temp[0] = '\0';
    
    rf69_setMode(RFM69_MODE_RX);
    
    while(countdown > 0) {
        
        // Check rx buffer
        if(checkRx() == 1) {
            rf69_recv(data_temp,  &rx_len);
            data_temp[rx_len - 1] = '\0';
#ifdef DEBUG
            //rssi = RFM69_lastRssi();
            fprintf(fp, "rx: %s\r\n",data_temp);
            //printf("RSSI: %d\r\n, rssi");
#endif
            processData(rx_len);
        }
        
        countdown--;
        delay_ms
        (100);
    }
}

int main(void)
{
	int n;

	clock_setup();
	gpio_setup();
	fp = usart_setup(USART1);
    
    /* SETUP */
    fprintf(fp, "Starting Ebeko Node\n");
    
    fprintf(fp, "RFM69: %d, Temp: %d\n", rf69_init(), rf69_readTemp());
    

	/* Blink the LED (PD12) on the board with every transmitted byte. */
	while (1) {
		gpio_toggle(GPIOC, GPIO8);	/* LED on/off */
        
        incrementPacketCount();
        
        int int_temp = rf69_readTemp();
        int rssi = rf69_sampleRssi();
        
        n = sprintf(data_temp, "%d%cT%dR%d[%s]", NUM_REPEATS, data_count, int_temp, rssi, NODE_ID);
        
        fprintf(fp, "%s\n", data_temp);
        
        transmitData(n);

        //delay_ms(10000);
        awaitData(TX_GAP);

	}

	return 0;
}
