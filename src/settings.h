//NODE SPECIFIC DETAILS - need to be changed
#define NUM_REPEATS			5
#define NODE_ID				"AC2"
#define LOCATION_STRING		"0.000,0.0000"
#define POWER_OUTPUT		20				// Output power in dbmW
#define TX_GAP				300			// Milliseconds between tx = tx_gap * 100, therefore 1000 = 100 seconds
#define MAX_TX_CHARS		32				// Maximum chars which can be transmitted in a single packet

//NODE TYPES
//Type of system
// MK2 - uses a slightly different pin layout to free up ACMP pins
//#define MK2

// Tx debug data and print full debug to serial
//#define DEBUG

//Print rx'd strings only
//#define GATEWAY

// Comment out if you don't want GPS (ublox binary)
#define GPS

//#define SERIAL_IN

//ADC - measure input voltage
#define ADC_1
#define ADC_1_FUDGE_1 189
#define ADC_1_FUDGE_2 97
//#define ADC_PIN 14
#define VCC_THRES 2395

// Zombie mode, no Rx, just Tx, saves power (as radio is sleeping between tx) and also flash
//#define POWER_SAVING

//TEMPERATURE SETTINGS
#define RFM_TEMP

//Details for one wire use (e.g. DS18B20)
//#define ONE_WIRE
//#define OW_PORT 0
//#define OW_PIN 10

//#define I2C

//#define PWM
//#define PWM_FREQUENCY (50000)

//#define PID
//#define RADIO_SETPOINT 10

#define LNA_SENS