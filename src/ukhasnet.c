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
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/nvic.h>
#include <stdio.h>
#include <string.h>

#include "settings.h"
#include "misc.h"

uint8_t data_count = 96; // 'a' - 1 (as the first function will at 1 to make it 'a'
unsigned int random_output = 50;
char data_temp[66];

/* monotonically increasing number of milliseconds from reset
 * overflows every 49 days if you're wondering
 */
volatile uint32_t system_millis;

#include "rfm69.h"
#include "RFM69Config.h"
#include "uart.h"

char serialBuffer[128];
uint8_t serialBuffer_write = 0;

static void usart_setup(void)
{
    
    nvic_enable_irq(NVIC_USART1_IRQ);
    
    // Setup USART1 parameters.
    usart_set_baudrate(USART1, 9600);
    usart_set_databits(USART1, 8);
    usart_set_parity(USART1, USART_PARITY_NONE);
    usart_set_stopbits(USART1, USART_CR2_STOP_1_0BIT);
    usart_set_mode(USART1, USART_MODE_TX_RX);
    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
    usart_enable_rx_interrupt(USART1);
    
    // Finally enable the USART.
    usart_enable(USART1);
}

void usart1_isr(void){
    
    serialBuffer[serialBuffer_write] = usart_recv_blocking(USART1);
    
    if(serialBuffer_write < 127){
        serialBuffer_write++;
    }
    else{
        serialBuffer_write = 0;
    }
    
    //usart_send_blocking(USART1, serialBuffer[serialBuffer_write]); //Echo back
}

uint8_t usart1_available(){
    return serialBuffer_write;
}

void usart1_clear(){
    serialBuffer_write = 0;
}

char usart1_buffer(uint8_t buf_position){
    return serialBuffer[buf_position];
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
    usart_send_blocking(USART1,'\n');
}

#ifdef GPS
    #include "gps.h"
#endif

#ifdef ADC_1

#include <libopencm3/stm32/adc.h>
static uint16_t read_adc_native(uint8_t channel);

static void adc_setup(void)
{
    rcc_periph_clock_enable(RCC_ADC);
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    
    //gpio_mode_setup(GPIOB, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO0);
    gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO1);
    gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO2);
    
    adc_power_off(ADC1);
    adc_set_clk_source(ADC1, ADC_CLKSOURCE_ADC);
    adc_calibrate_start(ADC1);
    adc_calibrate_wait_finish(ADC1);
    adc_set_operation_mode(ADC1, ADC_MODE_SCAN);
    adc_disable_external_trigger_regular(ADC1);
    adc_set_right_aligned(ADC1);
    adc_enable_temperature_sensor();
    adc_set_sample_time_on_all_channels(ADC1, ADC_SMPTIME_071DOT5);
    adc_set_resolution(ADC1, ADC_RESOLUTION_12BIT);
    adc_disable_analog_watchdog(ADC1);
    adc_power_on(ADC1);
    
    // Wait for ADC starting up.
    delay_ms(800);
    
}

static uint16_t read_adc_native(uint8_t channel)
{
    uint8_t channel_array[16];
    channel_array[0] = channel;
    adc_set_regular_sequence(ADC1, 1, channel_array);
    adc_start_conversion_regular(ADC1);
    while (!adc_eoc(ADC1));
        uint16_t temp = adc_read_regular(ADC1);
    return temp;
}

#endif

static void clock_setup(void)
{
    rcc_clock_setup_in_hsi_out_48mhz();
    
	/* Enable GPIOC clock for LED & USARTs. */
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOA);

	/* Enable clocks for USART1. */
	rcc_periph_clock_enable(RCC_USART1);
    
}

/* Called when systick fires */
void sys_tick_handler(void)
{
    system_millis++;
}

/* sleep for delay milliseconds */
void delay_ms(uint32_t msec_delay)
{
    uint32_t wake = system_millis + msec_delay;
    while (wake > system_millis);
}

/*
 * Set up timer to fire every x milliseconds
 * This is a unusual usage of systick, be very careful with the 24bit range
 * of the systick counter!  You can range from 1 to 2796ms with this.
 */
static void systick_setup(int xms)
{
    /* div8 per ST, stays compatible with M3/M4 parts, well done ST */
    systick_set_clocksource(STK_CSR_CLKSOURCE_EXT);
    /* clear counter so it starts right away */
    STK_CVR = 0;
    
    systick_set_reload(rcc_ahb_frequency / 8 / 1000 * xms);
    systick_counter_enable();
    systick_interrupt_enable();
}

static void gpio_setup(void)
{
	// Setup GPIO pin GPIO8/9 on GPIO port C for LEDs.
	gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO4 | GPIO5);
    
    // Setup GPIO pin GPIO2 on GPIO port A for Sensor PWR.
    //gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO2);
    
    // Setup GPIO pins for USART1 transmit and receive.
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO6|GPIO7);
    
    // Setup USART1 TX pin as alternate function.
    gpio_set_af(GPIOB, GPIO_AF0, GPIO6);
    gpio_set_af(GPIOB, GPIO_AF0, GPIO7);
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

/**
 * Packet data transmission
 * @param Packet length
 */
void transmitData(uint8_t i) {
    
    gpio_set(GPIOB, GPIO5);	// LED on
    
#ifdef GATEWAY
    //fprintf(fp, "rx: %s|0\r\n", data_temp);
#endif
    
    // Transmit the data (need to include the length of the packet and power in dbmW)
    rf69_send((uint8_t*)data_temp, i, POWER_OUTPUT);
    
    //tx_packets++;
#ifdef POWER_SAVING
    //Ensure we are in Sleep mode to save power
    rf69_setMode(RFM69_MODE_SLEEP);
#else
    //Ensure we are in RX mode
    rf69_setMode(RFM69_MODE_RX);
    
    delay_ms(500);
    
    gpio_clear(GPIOB, GPIO5);	// LED off
#endif

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
        
#ifdef DEBUG
        //print("rx: %s|%d\r\n",data_temp, rf69_lastRssi());
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
            
            gpio_set(GPIOB, GPIO4);	// LED on
            
            rf69_recv(data_temp,  &rx_len);
            data_temp[rx_len - 1] = '\0';
            //rx_packets++;
#ifdef DEBUG
            char rx_data[66];
            int z = 0;
            z = sprintf(rx_data, "%s|%d\r\n",data_temp, rf69_lastRssi());
            
            print(rx_data);
#endif
            processData(rx_len);
            
            gpio_clear(GPIOB, GPIO4);	// LED off
        }
        
        countdown--;
        delay_ms(100);
    }
}

int main(void)
{
    
	int n;
    uint16_t volt1 = 0, volt2 = 0, raw_volt1 = 0;
    uint8_t pwr_saving_mode = 0;

	clock_setup();
    
    systick_setup(1); //ticks every 1ms
    
	gpio_setup();
    gpio_clear(GPIOB, GPIO4);	// LED off
    gpio_clear(GPIOB, GPIO5);	// LED off
    
    usart_setup();
    
#ifdef GPS
    delay_ms(5000);
    setupGPS();
#endif
    
#ifdef DEBUG
    // SETUP
    print("Starting\n");
#endif
    
#ifdef ADC_1
    adc_setup();
#endif
    print("RF Init Start\n");
    rf69_init();
    print("RF Init Done\n");
	
	while (1) {
        print("In While Loop\n");
        // Toggle the LED (PA9) on the board every loop.
		gpio_toggle(GPIOB, GPIO4);	// LED on/off
        
        incrementPacketCount();
        int int_temp, rssi;
        
        int_temp = rf69_readTemp();
        rssi = rf69_sampleRssi();
        
#ifdef ADC_1
        raw_volt1 = read_adc_native(0x01);
        volt1 = raw_volt1;
        delay_ms(100);
        volt2 = read_adc_native(0x02);
#endif

#ifdef GPS
        print(serialBuffer);
        
        int nav_outcome = gps_check_nav();
        
        if (nav_outcome != 6){
            setupGPS();
            delay_ms(5000);
        }
        
        gps_get_position();
        
        n = sprintf(data_temp, "%d%cL%ld,%ld,%ldT%dR%dV%d,%dX,%d[%s]", NUM_REPEATS, data_count, lat, lon, alt, int_temp, rssi, volt1, volt2, pwr_saving_mode, NODE_ID);
        
#else
        if (data_count == 'z'){
            n = sprintf(data_temp, "%d%cL%sT%dR%dV%d,%dX%d[%s]", NUM_REPEATS, data_count, LOCATION_STRING, int_temp, rssi, volt1, volt2, pwr_saving_mode, NODE_ID);
        }
        else{
            n = sprintf(data_temp, "%d%cT%dR%dV%d,%dX%d[%s]", NUM_REPEATS, data_count, int_temp, rssi, volt1, volt2, pwr_saving_mode, NODE_ID);
        }
#endif
        
        
        
#ifdef DEBUG
        print(data_temp);
#endif
        
        transmitData(n);

#ifdef POWER_SAVING
        if (raw_volt1 > VCC_THRES_1){
            pwr_saving_mode = 0;
            awaitData(TX_GAP);
        }
        else if (raw_volt1 > VCC_THRES_2){
            //Ideally we'll add some power saving here
            pwr_saving_mode = 1;
            rf69_setMode(RFM69_MODE_SLEEP);
            delay_ms(60000);
        }
        else{
            //Ideally we'll add some power saving here
            pwr_saving_mode = 2;
            rf69_setMode(RFM69_MODE_SLEEP);
            delay_ms(300000);
        }
#else
        pwr_saving_mode = 0;
        awaitData(TX_GAP);
#endif

	}

	return 0;
}
