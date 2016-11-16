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
#include <stdio.h>
#include <string.h>

#include "settings.h"

uint8_t data_count = 96; // 'a' - 1 (as the first function will at 1 to make it 'a'
unsigned int rx_packets = 0, random_output = 0, rx_restarts = 0;
int16_t rx_rssi, floor_rssi, rssi_threshold;
char data_temp[66];

#include "rfm69.h"

#ifdef DEBUG
#include <libopencm3/stm32/usart.h>
void print(const char *s);

static void usart_setup(void)
{
    // Setup USART2 parameters.
    usart_set_baudrate(USART1, 38400);
    usart_set_databits(USART1, 8);
    usart_set_parity(USART1, USART_PARITY_NONE);
    usart_set_stopbits(USART1, USART_CR2_STOP_1_0BIT);
    usart_set_mode(USART1, USART_MODE_TX);
    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
    
    // Finally enable the USART.
    usart_enable(USART1);
}

//https://github.com/MrSpock/stm32f0-libopencm3-template/blob/master/main.cpp
void print(const char *s)
{
    
    // loop through entire string
    while (*s) {
        if ( *s == '\n') {
            usart_send_blocking(USART1,'\r');
            //usart_send_blocking(USART1,'\n');
        }
        usart_send_blocking(USART1,*s);
        s++;
    }
}

#endif

#ifdef ADC_1

#include <libopencm3/stm32/adc.h>
#define 	ADC_CHANNEL1   0x01
uint8_t channel_array[] = { ADC_CHANNEL1};
int16_t adc_result = 0;
uint16_t moist_sensor1();

static void adc_setup(void)
{
    rcc_periph_clock_enable(RCC_ADC);
    rcc_periph_clock_enable(RCC_GPIOA);
    
    //gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO0);
    gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO1);
    
    adc_power_off(ADC1);
    adc_set_clk_source(ADC1, ADC_CLKSOURCE_ADC);
    adc_calibrate_start(ADC1);
    adc_calibrate_wait_finish(ADC1);
    adc_set_operation_mode(ADC1, ADC_MODE_SCAN);
    adc_disable_external_trigger_regular(ADC1);
    adc_set_right_aligned(ADC1);
    adc_enable_temperature_sensor();
    adc_set_sample_time_on_all_channels(ADC1, ADC_SMPTIME_071DOT5);
    adc_set_regular_sequence(ADC1, 1, channel_array);
    adc_set_resolution(ADC1, ADC_RESOLUTION_12BIT);
    adc_disable_analog_watchdog(ADC1);
    adc_power_on(ADC1);
    
    // Wait for ADC starting up.
    int i;
    for (i = 0; i < 800000; i++) {    // Wait a bit.
        __asm__("nop");
    }
    
}

uint16_t moist_sensor1(){
    uint16_t temp = 0;
    gpio_set(GPIOA, GPIO2);
    delay_ms(1000);
    adc_start_conversion_regular(ADC1);
    while (!(adc_eoc(ADC1)));
    
    temp = adc_read_regular(ADC1);
    
    gpio_clear(GPIOA, GPIO2);
    
    return temp;
}

#endif

static void clock_setup(void)
{
    //rcc_clock_setup_in_hsi_out_48mhz();
    
	/* Enable GPIOC clock for LED & USARTs. */
	//rcc_periph_clock_enable(RCC_GPIOC);
	rcc_periph_clock_enable(RCC_GPIOA);

	/* Enable clocks for USART1. */
	//rcc_periph_clock_enable(RCC_USART1);
}

static void gpio_setup(void)
{
	/* Setup GPIO pin GPIO8/9 on GPIO port C for LEDs. */
	gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO8 | GPIO9);
    
    /* Setup GPIO pin GPIO2 on GPIO port A for Sensor PWR. */
    //gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO2);

	// Setup GPIO pins for USART1 transmit.
	gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO6);

	/* Setup USART1 TX pin as alternate function. */
	gpio_set_af(GPIOB, GPIO_AF1, GPIO6);
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
    //fprintf(fp, "rx: %s|0\r\n", data_temp);
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
        //fprintf(fp, "rx: %s|%d\r\n",data_temp, rf69_lastRssi());
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
            //fprintf(fp, "rx: %s\r\n",data_temp);
            //printf("RSSI: %d\r\n, rssi");
#endif
            processData(rx_len);
        }
        
        countdown--;
        delay_ms(100);
    }
}

int main(void)
{
    
	int n;
    uint16_t moist1 = 0;

	clock_setup();
    
	gpio_setup();
    
#ifdef DEBUG
	usart_setup();
    
    // SETUP
    print("Starting Ebeko Node\n");
#endif
    
#ifdef ADC_1
    adc_setup();
#endif
    
    rf69_init();
	
	while (1) {
        // Toggle the LED (PA9) on the board every loop.
		gpio_toggle(GPIOA, GPIO9);	// LED on/off
        
        
        incrementPacketCount();
        
        int int_temp = rf69_readTemp();
        int rssi = rf69_sampleRssi();
        
#ifdef ADC_1
        moist1 = moist_sensor1();
#endif

        n = sprintf(data_temp, "%d%cT%dR%dV%d[%s]", NUM_REPEATS, data_count, int_temp, rssi, moist1, NODE_ID);
        
        //print(data_temp);
        
        transmitData(n);

        awaitData(TX_GAP);


	}

	return 0;
}
