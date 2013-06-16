/* Name: main.c
 * Project: sensors
 * Author: lovewilliam <lovewilliam@gmail.com>
 * Creation Date: 2012-4-28
 */

/*
This example should run on most AVRs with only little changes. No special
hardware resources except INT0 are used. You may have to change usbconfig.h for
different I/O pins for USB. Please note that USB D+ must be the INT0 pin, or
at least be connected to INT0 as well.
*/

//#define DEBUG 1

#define MAIN_DELAY_MS 5
#define READOUT_INTERVAL_MS 60000
#define READOUT_WAIT_MS DS18B20_TCONV_12BIT

#define BAUD 19200

#include <stdlib.h>

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>  /* for sei() */
#include <util/delay.h>     /* for _delay_ms() */
#include <avr/eeprom.h>

#include "onewire.h"
#include "ds18x20.h"

#include <util/delay.h>

#include <avr/pgmspace.h>   /* required by usbdrv.h */
#include "usbdrv.h"
#include "oddebug.h"        /* This is also an example for using debug macros */

#define DHT_PORT    PORTD
#define DHT_PIN     3

#define MAXTIMINGS  84

#define DHT_Temp    0
#define DHT_RH      1
#define DHT_POL     2

//#define DHT_Read_Pin    (PIN(DHT_PORT) & _BV(DHT_PIN))
#define DHT_Read_Pin    (PIND & _BV(DHT_PIN))

uint8_t DHT_Read_Data(uint8_t select);

enum MODE{TEMPERATURE,HUMIDITY,RAW} host_req_type;

///////////////////////////////
/* internal format for temperature readings */
typedef struct temp_reading {
	uint8_t subzero;
	uint8_t cel;
	uint8_t cel_frac_bits;
} temp_reading_t;
/* states of the state machine in main*/
enum read_state_t {IDLE, MEASURING, UPDATED};
#define MAXSENSORS 5
uint8_t gSensorIDs[MAXSENSORS][OW_ROMCODE_SIZE];
uint8_t nSensors; // number of attached sensors
temp_reading_t readings[MAXSENSORS];

/* USB globals */
#define DS18X20_STATUS_LED_PIN PB1
#define DS18X20_STATUS_LED_PORT PORTB
#define DS18X20_STATUS_LED_DDR DDRB

#define EEPROM_LOCATION (void *)37

//////////////////////////////


/* ------------------------------------------------------------------------- */
/* ----------------------------- USB interface ----------------------------- */
/* ------------------------------------------------------------------------- */

PROGMEM const char usbHidReportDescriptor[22] = {    /* USB report descriptor */
    0x06, 0x00, 0xff,              // USAGE_PAGE (Generic Desktop)
    0x09, 0x01,                    // USAGE (Vendor Usage 1)
    0xa1, 0x01,                    // COLLECTION (Application)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x95, 0x80,                    //   REPORT_COUNT (128)
    0x09, 0x00,                    //   USAGE (Undefined)
    0xb2, 0x02, 0x01,              //   FEATURE (Data,Var,Abs,Buf)
    0xc0                           // END_COLLECTION
};
/* Since we define only one feature report, we don't use report-IDs (which
 * would be the first byte of the report). The entire report consists of 128
 * opaque data bytes.
 */

unsigned char temperature = 0x00;
unsigned char temperaturedot = 0x00;
unsigned char humidity = 0x00;
unsigned char humiditydot = 0x00;
unsigned char checksum = 0x00;

/////////////////////////////////////////////
/** 
 * Discover the available sensors.
 */
uint8_t search_sensors(void)
{
	uint8_t i;
	uint8_t id[OW_ROMCODE_SIZE];
	uint8_t diff, nSensors;
	nSensors = 0;
	for( diff = OW_SEARCH_FIRST; 
		diff != OW_LAST_DEVICE && nSensors < MAXSENSORS ; ) {
		DS18X20_find_sensor( &diff, &id[0] );
		if( diff == OW_PRESENCE_ERR )
		{
			//logs_P( "No Sensor found\r\n" );
			break;
		}
		if( diff == OW_DATA_ERR )
		{
			//logs_P( "Bus Error\r\n" );
			break;
		}
		for (i=0;i<OW_ROMCODE_SIZE;i++)
		{
			gSensorIDs[nSensors][i]=id[i];
		}
		nSensors++;
	}
	return nSensors;
}


void async_start_read_sensors(void)
{
	if ( DS18X20_start_meas( DS18X20_POWER_PARASITE, NULL ) == DS18X20_OK)
	{
		//logs_P("started measurement.\r\n");	
	}//else Failed!
}

void async_finish_read_sensors(void)
{
	uint8_t subzero, cel, cel_frac_bits,i;
	for (i=0; i<nSensors; i++ ) {
		if ( DS18X20_read_meas( &gSensorIDs[i][0], &subzero,
			&cel, &cel_frac_bits) == DS18X20_OK ) {
			// Put the reading in the global readings array
			readings[i].subzero=subzero;
			readings[i].cel=cel;
			readings[i].cel_frac_bits=cel_frac_bits;
		}//else logs_P("CRC Error (lost connection?)");
	}
}

void sync_read_sensors(void)
{
	async_start_read_sensors();
	_delay_ms(DS18B20_TCONV_12BIT);
	async_finish_read_sensors();
}

void enable_status_led(void)
{
	DS18X20_STATUS_LED_DDR |= (1 << DS18X20_STATUS_LED_PIN);  // Output
	DS18X20_STATUS_LED_PORT &= ~( 1<<DS18X20_STATUS_LED_PIN );
}

void disable_status_led(void) 
{
	DS18X20_STATUS_LED_DDR &= ~(1 << DS18X20_STATUS_LED_PIN);  // Output
	DS18X20_STATUS_LED_PORT |= ( 1<<DS18X20_STATUS_LED_PIN );
}

/* ------------------------------------------------------------------------- */

/* usbFunctionRead() is called when the host requests a chunk of data from
 * the device. For more information see the documentation in usbdrv/usbdrv.h.
 */
uchar usbFunctionRead(uchar *data, uchar len)
{
	switch(host_req_type)
	{
		case(TEMPERATURE):
		{
			data[0] = temperature;
			break;
		}
		case(HUMIDITY):
		{
			data[0] = humidity;
			break;
		}
		case(RAW):
		{
			data[0] = humidity;
			data[1] = humiditydot;
			data[2] = temperature;
			data[3] = temperaturedot;
			data[4] = checksum;
			break;
		}
		default:
		{
			data[0] = 0;
		}
	}
	return len;
}

/* usbFunctionWrite() is called when the host sends a chunk of data to the
 * device. For more information see the documentation in usbdrv/usbdrv.h.
 */
uchar usbFunctionWrite(uchar *data, uchar len)
{
	return 1;
}

/* ------------------------------------------------------------------------- */

usbMsgLen_t usbFunctionSetup(uchar data[8])
{
	usbRequest_t *rq = (void *)data;
	static uchar    replyBuf[64];

	usbMsgPtr = replyBuf;
	if(rq->bRequest == USBTEMP_CMD_TEST){//ECHO
		replyBuf[0] = rq->wValue.bytes[0];
		replyBuf[1] = rq->wValue.bytes[1];
		return 2;
	}
	if(rq->bRequest == USBTEMP_CMD_NO_SENSORS){  // GET_NO_SENSORS -> result = 1 bytes
		replyBuf[0] = nSensors;
		return 1;
	}
	if(rq->bRequest == USBTEMP_CMD_QUERY_SENSOR) {
		// Return id of sensor, 6 bytes.  The first byte contains the type
		// of the sensor.
		uint8_t sensorId=rq->wValue.bytes[0];
		replyBuf[0] = gSensorIDs[sensorId][0];
		replyBuf[1] = gSensorIDs[sensorId][1];
		replyBuf[2] = gSensorIDs[sensorId][2];
		replyBuf[3] = gSensorIDs[sensorId][3];
		replyBuf[4] = gSensorIDs[sensorId][4];
		replyBuf[5] = gSensorIDs[sensorId][5];
		replyBuf[6] = gSensorIDs[sensorId][6];
		return 7;
	}
	if(rq->bRequest == USBTEMP_CMD_GET_TEMP) {
		//Return temperature reading of the sensor. The sensor ID is given
		// in wValue. First byte: sensorID, next three bytes: temp. reading.
		uint8_t sensorId=rq->wValue.bytes[0];
		if (sensorId > nSensors)
		sensorId=0;
		replyBuf[0] = sensorId;
		replyBuf[1] = readings[sensorId].subzero;
		replyBuf[2] = readings[sensorId].cel;
		replyBuf[3] = readings[sensorId].cel_frac_bits;
		return 4;
	}
	if((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_CLASS)
	{
		if(rq->bRequest == USBRQ_HID_GET_TEMPERATURE)//Get Temperature
		{
			host_req_type = TEMPERATURE;
			return USB_NO_MSG;
		}else if(rq->bRequest == USBRQ_HID_GET_HUMIDITY)//Get Humidity
		{
			host_req_type = HUMIDITY;
			return USB_NO_MSG;
		}else if(rq->bRequest == USBRQ_HID_GET_RAW)
		{
			host_req_type = RAW;
			return USB_NO_MSG;
		}
	}else{
		/* ignore vendor type requests, we don't use any */
	}
	return 0;
}

/*--------------------------------------------*/

uint8_t DHT_Read_Data(uint8_t select)
{
	//data[5] is 8byte table where data come from DHT are stored
	//laststate holds laststate value
	//counter is used to count microSeconds
	uint8_t data[5], laststate = 0, counter = 0, j = 0, i = 0;
    
	//Clear array
	data[0] = data[1] = data[2] = data[3] = data[4] = 0;
    
	wdt_reset();
	cli();                              //Disable interrupts
    
	//Prepare the bus
	//DDR(DHT_PORT)   |= _BV(DHT_PIN);    //Set pin Output
	DDRD |= _BV(DHT_PIN);
	DHT_PORT        |= _BV(DHT_PIN);    //Pin High
	//_delay_ms(250);                     //Wait for 250mS
	
	//Send Request Signal
	DHT_PORT        &=~_BV(DHT_PIN);    //Pin Low
	_delay_ms(20);                      //20ms Low 
	DHT_PORT        |= _BV(DHT_PIN);    //Pin High
	_delay_us(40);                      //40us High
	//Set pin Input to read Bus
	//DDR(DHT_PORT)   &=~_BV(DHT_PIN);    //Set pin Input
	DDRD &=~_BV(DHT_PIN);

	laststate=DHT_Read_Pin;             //Read Pin value
    
	//Repeat for each Transistions
	for (i=0; i<MAXTIMINGS; i++) {
		//While state is the same count microseconds
		while (laststate==DHT_Read_Pin) {
			_delay_us(1);
			counter++;
			if(counter>80)
			{
				goto not_connected;
				//NOT CONNECTED! exit!
			}
		}
		//laststate==_BV(DHT_PIN) checks if laststate was High
		//ignore the first 2 transitions which are the DHT Response
		if (laststate==_BV(DHT_PIN) && (i > 2)) {
			//Save bits in segments of bytes
			//Shift data[] value 1 position left
			//Example. 01010100 if we shift it left one time it will be
			//10101000
			data[j/8]<<=1;
			//if (counter >= 40)    //If it was high for more than 40uS
			if(counter >=30)//FIX for 12MHz Crystal
			{
				data[j/8]|=1;       //it means it is bit '1' so make a logic
			}                       //OR with the value (save it)
			j++;                    //making an OR by 1 to this value 10101000
		}                           //we will have the resault 10101001
                	                    //1 in 8-bit binary is 00000001
		//j/8 changes table record every 8 bits which means a byte has been saved
		//so change to next record. 0/8=0 1/8=0 ... 7/8=0 8/8=1 ... 15/8=1 16/8=2
		laststate=DHT_Read_Pin;     //save current state
		counter=0;                  //reset counter
	}
	sei();                          //Enable interrupts
    
	DBG1(0x10,data,5);
	//Check if data received are correct by checking the CheckSum
	if (data[0] + data[1] + data[2] + data[3] == data[4]) {
		humidity = data[0];
		humiditydot = data[1];
		temperature = data[2];
		temperaturedot = data[3];
		checksum = data[4];
		if (select==DHT_Temp) {     //Return the value has been choosen
			return(data[2]);
		}else if(select==DHT_RH){
			return(data[0]);
		}else
		{
			return 0;
		}
	}else{
	 	//Check Sum Error
		DBG1(0x13,"0",1);
		return 0;
	}
not_connected:
	sei();
	data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	humidity = 0xFF;
	temperature = 0xFF;
	return 0;
}

/* ------------------------------------------------------------------------- */

int main(void)
{
	uchar   i;
	unsigned int dht_flag = 0;
	wdt_enable(WDTO_1S);

	/* Even if you don't use the watchdog, turn it off here. On newer devices,
	* the status of the watchdog (on/off, period) is PRESERVED OVER RESET!
	*/
	/* RESET status: all port bits are inputs without pull-up.
	* That's the way we need D+ and D-. Therefore we don't need any
	* additional hardware initialization.
	*/
	
	odDebugInit();
	DBG1(0x00, 0, 0);       /* debug output: main starts */
	usbInit();
	usbDeviceDisconnect();  /* enforce re-enumeration, do this while interrupts are disabled! */
	i = 0;
	while(--i){             /* fake USB disconnect for > 250 ms */
		wdt_reset();
		_delay_ms(1);
	}
	usbDeviceConnect();
	sei();
	DBG1(0x01, 0, 0);       /* debug output: main loop starts */
	usbPoll();
/////////////////////////////////
	wdt_reset();
	enable_status_led();
	nSensors = search_sensors();
	
	if (nSensors == 0)
		disable_status_led();
	else
		enable_status_led();
	//wdt_reset();
	//sync_read_sensors();
	wdt_reset();
	DBG1(0xB0,1,nSensors);
	// main loop
	enum read_state_t state=IDLE;
	unsigned char ds18x20_flag = 0;
///////////////////////////////////
	for(;;)
	{
		//DBG1(0x02, 0, 0);   /* debug output: main loop iterates */
		wdt_reset();
		usbPoll();
		if(dht_flag%100000==0)
		{
			wdt_reset();
			DHT_Read_Data(DHT_POL);
			ds18x20_flag++;
		}
		if(nSensors>0)
		{
			if(state==IDLE && (ds18x20_flag==2))
			{
				wdt_reset();
				enable_status_led();
				async_start_read_sensors();
				state = MEASURING;
				disable_status_led();
			}
			if(state==MEASURING && (ds18x20_flag==3))
			{
				wdt_reset();
				enable_status_led();
				async_finish_read_sensors();
				state = UPDATED;
				disable_status_led();
			}
			if(state==UPDATED)
			{
				wdt_reset();
				disable_status_led();
				state=IDLE;
				ds18x20_flag = 0;
			}
		}
		dht_flag++;
	}
	return 0;
}

/* ------------------------------------------------------------------------- */
