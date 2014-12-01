/*************************************************************************
Title:    MRB-IIAB (Interlocking in a Box)
Authors:  Michael D. Petersen <railfan@drgw.net>
File:     $Id: $
License:  GNU General Public License v3

LICENSE:
    Copyright (C) 2013 Michael Petersen & Nathan Holmes

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

*************************************************************************/

#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>
#include <util/delay.h>

#include "mrbus.h"
#include "avr-i2c-master.h"

#ifdef MRBEE
// If wireless, redefine the common variables and functions
#include "mrbee.h"
#define mrbus_rx_buffer mrbee_rx_buffer
#define mrbus_tx_buffer mrbee_tx_buffer
#define mrbus_state mrbee_state
#define MRBUS_TX_PKT_READY MRBEE_TX_PKT_READY
#define MRBUS_RX_PKT_READY MRBEE_RX_PKT_READY
#define mrbux_rx_buffer mrbee_rx_buffer
#define mrbus_tx_buffer mrbee_tx_buffer
#define mrbus_state mrbee_state
#define mrbusInit mrbeeInit
#define mrbusPacketTransmit mrbeePacketTransmit
#endif

#include "mrbus.h"

uint8_t mrbus_dev_addr = 0;
uint8_t pkt_count = 0;

// Timeout = debounce applied to positive detection when that detection goes away (used to "coast" through point-style optical detectors)
// Lockout = time after interlocking clears during which the train from the opposite direction (or same train leaving) cannot get the interlocking (lets train finish leaving)
// Timelock = delay between signal going red and any other train getting a proceed signal
// Debouce = debounce applied to interlocking block detection going low.  Filters momentary non-detects to prevent premature switch from OCCUPIED to CLEAR state.
#define EE_TIMEOUT_SECONDS      0x10
// 0x10 - 0x13 = 4 sets of timeouts, one for each direction
#define EE_LOCKOUT_SECONDS      0x14
#define EE_TIMELOCK_SECONDS     0x15
#define EE_DEBOUNCE_SECONDS     0x16
#define EE_CLOCK_SOURCE_ADDRESS 0x18
#define EE_MAX_DEAD_RECKONING   0x19
#define EE_SIM_TRAIN_WINDOW     0x1A
#define EE_BLINKY_DECISECS      0x1F
#define EE_INPUT_POLARITY0      0x20
#define EE_INPUT_POLARITY1      0x21
#define EE_OUTPUT_POLARITY0     0x22
#define EE_OUTPUT_POLARITY1     0x23
#define EE_OUTPUT_POLARITY2     0x24
#define EE_OUTPUT_POLARITY3     0x25
#define EE_OUTPUT_POLARITY4     0x26
#define EE_MISC_CONFIG          0x30
#define EE_SIM_TRAINS           0x40


// interlockingStatus currently limits this to a maximum of 7
#define NUM_DIRECTIONS 4

#define OPPOSITE_DIRECTION(d) ((d)%2)?((d)-1):((d)+1)

// Assumptions:
//   "Direction" pairs are connected.  0/1, 2/3, etc
//   Even numbered directions can have turnouts (frog end).  Wired off for "main" if they don't exist.  Signals are separate signals.
//   Odd numbered directions have no turnouts (point end).  Signals are 2-headed signals using the turnout position from across the diamond.

typedef enum
{
	STATE_IDLE      = 0,
	STATE_CLEARANCE = 1,
	STATE_TIMEOUT   = 2,
	STATE_TIMER     = 3,
	STATE_OCCUPIED  = 4,
	STATE_LOCKOUT   = 5,
	STATE_CLEAR     = 6,
} InterlockState;

InterlockState state[NUM_DIRECTIONS];
InterlockState oldState[NUM_DIRECTIONS];

typedef struct
{
	uint8_t enable;
	volatile uint16_t timer;  // decisecs
	uint8_t approachTime;     // seconds
	uint8_t totalTime;        // seconds
	uint8_t sound;
} Simulator;

Simulator simulator[NUM_DIRECTIONS];

#define NUM_SIM_TRAINS 32

typedef struct
{
	// From EEPROM
	uint16_t time;  // Minutes since midnight
	uint8_t direction;
	uint8_t approachTime;  // seconds
	uint8_t totalTime;     // seconds
	uint8_t flags;         // 0: sound0, 1: sound1, ..., 7: trigger auto interlock
	// Local status
	uint8_t triggered;
} SimTrain;

SimTrain simTrain[NUM_SIM_TRAINS];
uint8_t simTrainWindow = 1;


uint8_t interchangeEnable;
uint8_t interchangeOccupancy;


uint8_t timeoutSeconds[NUM_DIRECTIONS];
volatile uint16_t timeoutTimer[NUM_DIRECTIONS];  // decisecs

uint8_t lockoutSeconds;
volatile uint16_t lockoutTimer[NUM_DIRECTIONS];  // decisecs

uint8_t timelockSeconds;
volatile uint16_t timelockTimer;  // decisecs

uint8_t interlockingDebounceSeconds;
volatile uint16_t interlockingDebounceTimer;  // decisecs

#define OCCUPANCY_MAIN   0x01
#define OCCUPANCY_SIDING 0x02

uint8_t approachOccupancy[NUM_DIRECTIONS];
uint8_t interlockingOccupancy;

#define TURNOUT_MAIN   0
#define TURNOUT_SIDING 1

uint8_t turnout[NUM_DIRECTIONS];  // Inefficient since only 1/2 the directions can have turnouts, but done this way for symmetry in the indexes
uint8_t turnoutOriginal[NUM_DIRECTIONS];

#define INTERLOCKING_LOCKED 0x80
uint8_t interlockingStatus;

// Aspect Definitions
#define ASPECT_LUNAR     0x07
#define ASPECT_FL_RED    0x06
#define ASPECT_FL_GREEN  0x05
#define ASPECT_RED       0x04
#define ASPECT_FL_YELLOW 0x03
#define ASPECT_YELLOW    0x02
#define ASPECT_GREEN     0x01
#define ASPECT_OFF       0x00

uint8_t signalHeads[2 * NUM_DIRECTIONS];


volatile uint8_t events = 0;
#define EVENT_READ_INPUTS    0x01
#define EVENT_WRITE_OUTPUTS  0x02
#define EVENT_REINIT_OUTPUTS 0x04
#define EVENT_I2C_ERROR      0x40
#define EVENT_BLINKY         0x80


// Misc variables
uint8_t debounced_inputs[2], old_debounced_inputs[2];
uint8_t clock_a[2] = {0,0}, clock_b[2] = {0,0};
uint8_t xio1Inputs[2];
uint8_t xio1Outputs[5];
uint8_t output_polarity[5];
uint8_t input_polarity[2];
uint8_t testModeEnable;
uint8_t sendPacketOnChange;

uint8_t i2cResetCounter = 0;
volatile uint8_t blinkyCounter = 0;
uint8_t blinkyCounter_decisecs = 5;


// Time variables
#define TIME_FLAGS_DISP_FAST       0x01
#define TIME_FLAGS_DISP_FAST_HOLD  0x02
#define TIME_FLAGS_DISP_REAL_AMPM  0x04
#define TIME_FLAGS_DISP_FAST_AMPM  0x08

uint16_t scaleFactor = 10;
uint8_t flags = 0;

typedef struct
{
	uint8_t seconds;
	uint8_t minutes;
	uint8_t hours;
} TimeData;

TimeData fastTime;

void incrementTime(TimeData* t, uint8_t incSeconds)
{
	uint16_t i = t->seconds + incSeconds;

	while(i >= 60)
	{
		t->minutes++;
		i -= 60;
	}
	t->seconds = (uint8_t)i;
	
	while(t->minutes >= 60)
	{
		t->hours++;
		t->minutes -= 60;
	}
	
	if (t->hours >= 24)
	{
		t->hours %= 24;
	}
}


// ******** Start 100 Hz Timer, 0.16% error version (Timer 0)
// If you can live with a slightly less accurate timer, this one only uses Timer 0, leaving Timer 1 open
// for more advanced things that actually need a 16 bit timer/counter

// Initialize a 100Hz timer for use in triggering events.
// If you need the timer resources back, you can remove this, but I find it
// rather handy in triggering things like periodic status transmissions.
// If you do remove it, be sure to yank the interrupt handler and ticks/secs as well
// and the call to this function in the main function

volatile uint8_t ticks;
volatile uint16_t decisecs=0;
uint16_t update_decisecs=10;
volatile uint16_t fastDecisecs = 0;
volatile uint8_t scaleTenthsAccum = 0;
uint8_t maxDeadReckoningTime = 50;
uint8_t deadReckoningTime = 0;
uint8_t timeSourceAddress = 0xFF;

void initialize100HzTimer(void)
{
	// Set up timer 0 for 100Hz interrupts
	TCNT0 = 0;
	OCR0A = 0xC2;
	ticks = 0;
	decisecs = 0;
	TCCR0A = _BV(WGM01);
	TCCR0B = _BV(CS02) | _BV(CS00);
	TIMSK0 |= _BV(OCIE0A);
}

ISR(TIMER0_COMPA_vect)
{
	// Read inputs every 2 ticks = 20ms
	if (ticks & 0x01)
		events |= EVENT_READ_INPUTS;

	if (++ticks >= 10)
	{
		uint8_t i;
		ticks = 0;
		decisecs++;

		// Deal with fast time
		if (deadReckoningTime)
			deadReckoningTime--;
		if (TIME_FLAGS_DISP_FAST == (flags & (TIME_FLAGS_DISP_FAST | TIME_FLAGS_DISP_FAST_HOLD)))
		{
			fastDecisecs += scaleFactor / 10;
			scaleTenthsAccum += scaleFactor % 10;
			if (scaleTenthsAccum > 10)
			{
				fastDecisecs++;
				scaleTenthsAccum -= 10;
			}		
		
		}

		// Other housekeeping
		for(i=0; i<NUM_DIRECTIONS; i++)
		{
			// Manage timers
			if(timeoutTimer[i])
				timeoutTimer[i]--;
			if(lockoutTimer[i])
				lockoutTimer[i]--;
			if(simulator[i].timer)
				simulator[i].timer--;
		}
		
		if(timelockTimer)
			timelockTimer--;
		
		if(interlockingDebounceTimer)
			interlockingDebounceTimer--;
		
		if (++blinkyCounter > blinkyCounter_decisecs)
		{
			events ^= EVENT_BLINKY;
			blinkyCounter = 0;
		}

		// Write outputs every 100ms
		events |= EVENT_WRITE_OUTPUTS;
	}
}

// End of 100Hz timer

// **** Bus Voltage Monitor

volatile uint8_t busVoltage=0;

ISR(ADC_vect)
{
	static uint16_t busVoltageAccum=0;
	static uint8_t busVoltageCount=0;

	busVoltageAccum += ADC;
	if (++busVoltageCount >= 64)
	{
		busVoltageAccum = busVoltageAccum / 64;
        //At this point, we're at (Vbus/6) / 5 * 1024
        //So multiply by 300, divide by 1024, or multiply by 150 and divide by 512
        busVoltage = ((uint32_t)busVoltageAccum * 150) / 512;
		busVoltageAccum = 0;
		busVoltageCount = 0;
	}
}


/* 0x00-0x04 - input registers */
/* 0x08-0x0C - output registers */
/* 0x10-0x14 - polarity inversion */
/* 0x18-0x1C - direction registers - 0 is output, 1 is input */

#define I2C_RESET         0
#define I2C_OUTPUT_ENABLE 1
#define I2C_IRQ           2
#define I2C_XIO1_ADDRESS 0x4E

void xioDirectionSet()
{
	uint8_t i2cBuf[8];

	// Set direction
	i2cBuf[0] = I2C_XIO1_ADDRESS;
	i2cBuf[1] = 0x80 | 0x18;  // 0x80 is auto-increment
	i2cBuf[2] = 0;
	i2cBuf[3] = 0;
	i2cBuf[4] = 0;
	i2cBuf[5] = 0xFF;
	i2cBuf[6] = 0x1F;
	i2c_transmit(i2cBuf, 7, 1);
	while(i2c_busy());

	// Set polarity
	i2cBuf[0] = I2C_XIO1_ADDRESS;
	i2cBuf[1] = 0x80 | 0x13;  // 0x80 is auto-increment
	i2cBuf[2] = input_polarity[0];
	i2cBuf[3] = input_polarity[1];
	i2c_transmit(i2cBuf, 4, 1);
	while(i2c_busy());
}

void xioInitialize()
{
	events |= EVENT_I2C_ERROR;

	PORTB &= ~(_BV(I2C_RESET) | _BV(I2C_OUTPUT_ENABLE));
	DDRB |= _BV(I2C_RESET) | _BV(I2C_OUTPUT_ENABLE);
	_delay_us(1);
	PORTB &= ~(_BV(I2C_OUTPUT_ENABLE));
	PORTB |= _BV(I2C_RESET);
	_delay_us(1);

	xioDirectionSet();
	
	if (i2c_transaction_successful())
	{
		events &= ~(EVENT_I2C_ERROR);
	}
}

void xioOutputWrite()
{
	uint8_t i2cBuf[8];
	uint8_t i;

	// Reinforce direction
	xioDirectionSet();

	while(i2c_busy());

	if (!i2c_transaction_successful())
		events |= EVENT_I2C_ERROR;

	i2cBuf[0] = I2C_XIO1_ADDRESS;
	i2cBuf[1] = 0x80 | 0x08;  // 0x80 is auto-increment
	for(i=0; i<sizeof(xio1Outputs); i++)
	{
			i2cBuf[2+i] = xio1Outputs[i] ^ output_polarity[i];
	}

	i2c_transmit(i2cBuf, 2+sizeof(xio1Outputs), 1);
}

void xioInputRead()
{
	uint8_t i2cBuf[4];
	uint8_t successful = 0;

	if (events & EVENT_I2C_ERROR)
		return;

	while(i2c_busy());

	if (!i2c_transaction_successful())
		events |= EVENT_I2C_ERROR;

	i2cBuf[0] = I2C_XIO1_ADDRESS;
	i2cBuf[1] = 0x80 | 0x03;  // 0x80 is auto-increment, 0x03 is the first register with inputs
	i2c_transmit(i2cBuf, 2, 0);
	i2cBuf[0] = I2C_XIO1_ADDRESS | 0x01;
	i2c_transmit(i2cBuf, 3, 1);
	while(i2c_busy());
	successful = i2c_receive(i2cBuf, 3);

	if (!successful)
		// In the event of a read hose-out, don't put crap in the input buffer
		events |= EVENT_I2C_ERROR;
	else
	{
		xio1Inputs[0] = i2cBuf[1];
		xio1Inputs[1] = i2cBuf[2];	
	}
}



void readEEPROM()
{
	uint8_t i;
	
	// Initialize MRBus address
	mrbus_dev_addr = eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_ADDR);
	// Bogus addresses, fix to default address
	if (0xFF == mrbus_dev_addr || 0x00 == mrbus_dev_addr)
	{
		mrbus_dev_addr = 0x03;
		eeprom_write_byte((uint8_t*)MRBUS_EE_DEVICE_ADDR, mrbus_dev_addr);
	}
	
	// Load MRBus update period
	update_decisecs = (uint16_t)eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_UPDATE_L) 
		| (((uint16_t)eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_UPDATE_H)) << 8);
	// This line assures that update_decisecs is at least 1
	update_decisecs = max(1, update_decisecs);

	// Load blinky time period
	blinkyCounter_decisecs = eeprom_read_byte((uint8_t*)EE_BLINKY_DECISECS);
	
	for(i=0; i<4; i++)
	{
		timeoutSeconds[i] = eeprom_read_byte((uint8_t*)(EE_TIMEOUT_SECONDS+i));
	}
	lockoutSeconds = eeprom_read_byte((uint8_t*)EE_LOCKOUT_SECONDS);
	timelockSeconds = eeprom_read_byte((uint8_t*)EE_TIMELOCK_SECONDS);
	interlockingDebounceSeconds = eeprom_read_byte((uint8_t*)EE_DEBOUNCE_SECONDS);
	
	// Load input polarity config
	input_polarity[0] = eeprom_read_byte((uint8_t*)EE_INPUT_POLARITY0);
	input_polarity[1] = eeprom_read_byte((uint8_t*)EE_INPUT_POLARITY1);
	output_polarity[0] = eeprom_read_byte((uint8_t*)EE_OUTPUT_POLARITY0);
	output_polarity[1] = eeprom_read_byte((uint8_t*)EE_OUTPUT_POLARITY1);
	output_polarity[2] = eeprom_read_byte((uint8_t*)EE_OUTPUT_POLARITY2);
	output_polarity[3] = eeprom_read_byte((uint8_t*)EE_OUTPUT_POLARITY3);
	output_polarity[4] = eeprom_read_byte((uint8_t*)EE_OUTPUT_POLARITY4);

	// Time configs
	timeSourceAddress = eeprom_read_byte((uint8_t*)EE_CLOCK_SOURCE_ADDRESS);
	maxDeadReckoningTime = eeprom_read_byte((uint8_t*)EE_MAX_DEAD_RECKONING);
	simTrainWindow = eeprom_read_byte((uint8_t*)EE_SIM_TRAIN_WINDOW);
}

void PktHandler(void)
{
	uint8_t pktHandlerStatus;
	uint8_t rxBuffer[MRBUS_BUFFER_SIZE];
	uint8_t txBuffer[MRBUS_BUFFER_SIZE];

	if (0 == mrbusPktQueuePop(&mrbusRxQueue, rxBuffer, sizeof(rxBuffer)))
		return;

	if((pktHandlerStatus = mrbusPktHandler(rxBuffer, txBuffer, mrbus_dev_addr)))
	{
		switch(pktHandlerStatus)
		{
			case MRBUS_HANDLER_RESET:
				cli();
				wdt_reset();
				MCUSR &= ~(_BV(WDRF));
				WDTCSR |= _BV(WDE) | _BV(WDCE);
				WDTCSR = _BV(WDE);
				while(1);  // Force a watchdog reset
				sei();
				break;
			case MRBUS_HANDLER_EEPROM:
				readEEPROM();
				txBuffer[MRBUS_PKT_SRC] = mrbus_dev_addr;  // Update src if it was changed
				mrbusPktQueuePush(&mrbusTxQueue, txBuffer, txBuffer[MRBUS_PKT_LEN]);
				break;
			case MRBUS_HANDLER_VERSION:
				txBuffer[12] = 'I';
				txBuffer[13] = 'I';
				txBuffer[14] = 'A';
				txBuffer[15] = 'B';
				mrbusPktQueuePush(&mrbusTxQueue, txBuffer, txBuffer[MRBUS_PKT_LEN]);
				break;
			case MRBUS_HANDLER_DONE:
				mrbusPktQueuePush(&mrbusTxQueue, txBuffer, txBuffer[MRBUS_PKT_LEN]);
				break;
			case MRBUS_HANDLER_CUSTOM:
				// Custom packet
				if ('T' == rxBuffer[MRBUS_PKT_TYPE] &&
					((0xFF == timeSourceAddress) || (rxBuffer[MRBUS_PKT_SRC] == timeSourceAddress)) )
				{
					// It's a time packet from our time reference source
					flags = rxBuffer[9];
					// Time source packets aren't required to have a fast section
					// Doesn't really make sense outside model railroading applications, so...
					if (rxBuffer[MRBUS_PKT_LEN] >= 14)
					{
						fastTime.hours =  rxBuffer[10];
						fastTime.minutes =  rxBuffer[11];
						fastTime.seconds = rxBuffer[12];
						scaleFactor = (((uint16_t)rxBuffer[13])<<8) + (uint16_t)rxBuffer[14];
					}

					// If we got a packet, there's no dead reckoning time anymore
					fastDecisecs = 0;
					scaleTenthsAccum = 0;
					deadReckoningTime = maxDeadReckoningTime;
				}
				else if ( ('C' == rxBuffer[MRBUS_PKT_TYPE]) && ('T' == rxBuffer[MRBUS_PKT_TYPE+1]) )
				{
					// Enable Test Mode
					debounced_inputs[0] = rxBuffer[7];
					debounced_inputs[1] = rxBuffer[8];
					testModeEnable = 1;
					ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
					{
						decisecs = update_decisecs;  // Force a status packet
					}
				}
				else if ( ('C' == rxBuffer[MRBUS_PKT_TYPE]) && ('N' == rxBuffer[MRBUS_PKT_TYPE+1]) )
				{
					// Disable Test Mode
					testModeEnable = 0;
				}
				else if ( ('C' == rxBuffer[MRBUS_PKT_TYPE]) && ('!' == rxBuffer[MRBUS_PKT_TYPE+1]) )
				{
					// Train Sim Command Packet
					// if(STATE_IDLE == state[direction]) { state[direction] = STATE_REQUEST }
					// Advance to request state only if that direction is currently idle
					/*	<direction> <simApproachTime> <simTotalTime> <sound> */
					/*	<direction> = 0-3 representing direction from which the simulated train is approaching*/
					/*	<simApproachTIme> = time in seconds between getting green and train crossing the diamond (setting the signal red).*/
					/*	<simTotalTime> = time in seconds from getting green until the diamond is cleared.  Sound plays for this entire interval.*/
					/*	simTotalTime should be > simApproachTime (enforce this)*/
					/*	<sound> = 0:no sound, 1-2: sound output to enable */
				}
				break;
		}
	}
}

void init(void)
{
	uint8_t i;
	
	// Clear watchdog (in the case of an 'X' packet reset)
	MCUSR = 0;
#ifdef ENABLE_WATCHDOG
	// If you don't want the watchdog to do system reset, remove this chunk of code
	wdt_reset();
	WDTCSR |= _BV(WDE) | _BV(WDCE);
	WDTCSR = _BV(WDE) | _BV(WDP2) | _BV(WDP1); // Set the WDT to system reset and 1s timeout
	wdt_reset();
#else
	wdt_reset();
	wdt_disable();
#endif	

	pkt_count = 0;

	readEEPROM();
	
	// Reset state machines
	for(i=0; i<NUM_DIRECTIONS; i++)
	{
		state[i] = STATE_IDLE;
		oldState[i] = STATE_IDLE;
	}
	
	// Setup ADC
	ADMUX  = 0x47;  // AVCC reference; ADC7 input
	ADCSRA = _BV(ADATE) | _BV(ADIF) | _BV(ADPS2) | _BV(ADPS1); // 128 prescaler
	ADCSRB = 0x00;
	DIDR0  = 0x00;  // No digitals were harmed in the making of this ADC

	busVoltage = 0;
	ADCSRA |= _BV(ADEN) | _BV(ADSC) | _BV(ADIE) | _BV(ADIF);

	fastDecisecs = 0;

	// Reset all occupancy to 0, turnouts to main, initialize timers
	for(i=0; i<NUM_DIRECTIONS; i++)
	{
		approachOccupancy[i] = 0;
		turnout[i] = TURNOUT_MAIN;
		timeoutTimer[i] = 0;
		lockoutTimer[i] = 0;
		simulator[i].timer = 0;
		simulator[i].enable = 0;
	}

	// Clear triggered flags in sim trains
	for(i=0; i<NUM_SIM_TRAINS; i++)
	{
		simTrain[i].triggered = 0;
	}

	interchangeEnable = 0;
	interchangeOccupancy = 0;

	interlockingOccupancy = 0;
	interlockingStatus = 0;
	
	timelockTimer = 0;
	interlockingDebounceTimer = 0;
	
	xio1Inputs[0] = 0;
	xio1Inputs[1] = 0;

	testModeEnable = 0;
	
	for(i=0; i<sizeof(debounced_inputs); i++)
		debounced_inputs[i] = old_debounced_inputs[i] = 0;
	
	sendPacketOnChange = 0;
	deadReckoningTime = 0;  // Time out until such time that we get a time packet
}

uint8_t approachBlockOccupancy(uint8_t direction)
{
	return ( approachOccupancy[direction] & ((TURNOUT_SIDING == turnout[direction]) ? OCCUPANCY_SIDING : OCCUPANCY_MAIN) );
}

uint8_t interlockingBlockOccupancy(void)
{
	return (interlockingOccupancy & OCCUPANCY_MAIN);
}

uint8_t requestInterlocking(uint8_t direction)
{
	uint16_t temp_uint16;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		temp_uint16 = timelockTimer;
	}

	if(interlockingStatus & INTERLOCKING_LOCKED)
		return 0;  // Already locked
	else if(interlockingBlockOccupancy())
		return 0;  // Interlocking occupied
	else if(temp_uint16)
		return 0;  // Timelock active after last train
	
	// Everything good.  Take it.
	interlockingStatus = INTERLOCKING_LOCKED | _BV(direction);
	return 1;
}

void startSound(uint8_t sound)
{
	xio1Outputs[4] |= (sound << 6);
}

void stopSound(uint8_t sound)
{
	xio1Outputs[4] &= ~(sound << 6);
}

void startInterchange(void)
{
	xio1Outputs[4] |= 0x20;
}

void stopInterchange(void)
{
	xio1Outputs[4] &= ~(0x20);
}

void readInputs(void)
{
	// Read occupancy into occupancy[] array and interlockingOccupancy.  Force siding occupancy = 0 on directions without a siding.
	// Read turnout positions; force directions without turnouts to main

	// debounced_inputs[0]
	//   D0: Block Detect #0 main
	//   D1: Block Detect #0 siding
	//   D2: Block Detect #1
	//   D3: Block Detect #2 main
	//   D4: Block Detect #2 siding
	//   D5: Block Detect #3
	//   D6: Block Detect Interlocking
	//   D7: unassigned

	// debounced_inputs[1]
	//   E0: Direction #0 turnout position
	//   E1: Direction #1 turnout position (not used)
	//   E2: Direction #2 turnout position
	//   E3: Direction #3 turnout position (not used)
	//   E4: Automatic Interchange Detector
	//   E5: Automatic Interchange Relay (output)
	//   E6: Sound trigger #0 (output)
	//   E7: Sound trigger #1 (output)

	uint8_t delta;
	uint8_t i;
	
	xioInputRead();
	
	xio1Inputs[1] &= 0x0F;  // Mask off Sound trigger outputs

	if(!testModeEnable)
	{
		for(i=0; i<2; i++)
		{
			// Vertical counter debounce courtesy 
			delta = xio1Inputs[i] ^ debounced_inputs[i];
			clock_a[i] ^= clock_b[i];
			clock_b[i]  = ~(clock_b[i]);
			clock_a[i] &= delta;
			clock_b[i] &= delta;
			debounced_inputs[i] ^= ~(~delta | clock_a[i] | clock_b[i]);
		}
	}

	// Get the physical turnout inputs from debounced
	for(i=0; i<NUM_DIRECTIONS; i++)
	{
		if(debounced_inputs[1] & _BV(i))
			turnout[i] = TURNOUT_SIDING;
		else
			turnout[i] = TURNOUT_MAIN;
	}
	
	// Get interlocking status
	if(debounced_inputs[1] & _BV(4))
		interchangeOccupancy = 1;
	else
		interchangeOccupancy = 0;

	// Get the physical occupancy inputs from debounced
	if(debounced_inputs[0] & _BV(0))
		approachOccupancy[0] |= OCCUPANCY_MAIN;
	else
		approachOccupancy[0] &= ~OCCUPANCY_MAIN;

	if(debounced_inputs[0] & _BV(1))
		approachOccupancy[0] |= OCCUPANCY_SIDING;
	else
		approachOccupancy[0] &= ~OCCUPANCY_SIDING;

	if(debounced_inputs[0] & _BV(2))
		approachOccupancy[1] |= OCCUPANCY_MAIN;
	else
		approachOccupancy[1] &= ~OCCUPANCY_MAIN;

	if(debounced_inputs[0] & _BV(3))
		approachOccupancy[2] |= OCCUPANCY_MAIN;
	else
		approachOccupancy[2] &= ~OCCUPANCY_MAIN;

	if(debounced_inputs[0] & _BV(4))
		approachOccupancy[2] |= OCCUPANCY_SIDING;
	else
		approachOccupancy[2] &= ~OCCUPANCY_SIDING;

	if(debounced_inputs[0] & _BV(5))
		approachOccupancy[3] |= OCCUPANCY_MAIN;
	else
		approachOccupancy[3] &= ~OCCUPANCY_MAIN;

	if(debounced_inputs[0] & _BV(6))
		interlockingOccupancy |= OCCUPANCY_MAIN;
	else
	{
		uint16_t temp_uint16;
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
		{
			temp_uint16 = interlockingDebounceTimer;
		}
		if(old_debounced_inputs[0] & _BV(6))
		{
			// Falling edge, set extra debounce timer
			ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
			{
				interlockingDebounceTimer = 10 * interlockingDebounceSeconds;
			}
		}
		else if(!temp_uint16)
		{
			// Low input and debounce has expired, clear occupancy
			interlockingOccupancy &= ~OCCUPANCY_MAIN;
		}
	}

	if( (old_debounced_inputs[0] != debounced_inputs[0]) || (old_debounced_inputs[1] != debounced_inputs[1]) )
	{
		sendPacketOnChange = 1;
	}

	old_debounced_inputs[0] = debounced_inputs[0];
	old_debounced_inputs[1] = debounced_inputs[1];
}

void InterlockingToSignals(void)
{
	uint8_t i;
	
	for(i=0; i<NUM_DIRECTIONS; i++)
	{
		switch(state[i])
		{
			case STATE_CLEARANCE:
			case STATE_TIMEOUT:
			case STATE_TIMER:
				if(i % 2)
				{
					// Odd direction = point end
					if(turnout[i-1])
					{
						// Opposite turnout set for siding
						signalHeads[2*i] = ASPECT_RED;
						signalHeads[(2*i)+1] = ASPECT_YELLOW;
					}
					else
					{
						// Opposite turnout set for main
						signalHeads[2*i] = ASPECT_GREEN;
						signalHeads[(2*i)+1] = ASPECT_RED;
					}
				}
				else
				{
					// Even direction = frog end
					if(turnout[i])
					{
						// Turnout set for siding
						signalHeads[2*i] = ASPECT_RED;
						signalHeads[(2*i)+1] = ASPECT_YELLOW;
					}
					else
					{
						// Turnout set for main
						signalHeads[2*i] = ASPECT_GREEN;
						signalHeads[(2*i)+1] = ASPECT_RED;
					}
				}
				break;
			case STATE_OCCUPIED:
				signalHeads[2*i] = ASPECT_RED;
				signalHeads[(2*i)+1] = ASPECT_RED;
				break;
			default:
				// Default to most restrictive aspect
				signalHeads[2*i] = ASPECT_RED;
				signalHeads[(2*i)+1] = ASPECT_RED;
				break;
		}
	}
}

// SignalsToOutputs is responsible for converting the signal head aspects in the
// signalHeads[] array to the physical wires on the XIO.
// Thus, it's hardware configuration dependent.

typedef struct
{
	const uint8_t signalHead;
	const uint8_t greenByte;
	const uint8_t greenMask;
	const uint8_t yellow1Byte;
	const uint8_t yellow1Mask;
	const uint8_t yellow2Byte;
	const uint8_t yellow2Mask;
	const uint8_t redByte;
	const uint8_t redMask;
	const uint8_t lunarByte;
	const uint8_t lunarMask;
} SignalPinDefinition;

// Two yellows are to allow for driving a red and green LED to produce yellow
const SignalPinDefinition sigPinDefs[8] = 
{//         green----  yellow1--  yellow2--  red------  lunar----
	{0, 0, _BV(0), 0, _BV(1), 0, _BV(1), 0, _BV(2), 0, _BV(2)},
	{1, 0, _BV(3), 0, _BV(4), 0, _BV(4), 0, _BV(5), 0, _BV(5)},
	{2, 0, _BV(6), 0, _BV(7), 0, _BV(7), 1, _BV(0), 1, _BV(0)},
	{3, 1, _BV(1), 1, _BV(2), 1, _BV(2), 1, _BV(3), 1, _BV(3)},
	{4, 1, _BV(4), 1, _BV(5), 1, _BV(5), 1, _BV(6), 1, _BV(6)},
	{5, 1, _BV(7), 2, _BV(0), 2, _BV(0), 2, _BV(1), 2, _BV(1)},
	{6, 2, _BV(2), 2, _BV(3), 2, _BV(3), 2, _BV(4), 2, _BV(4)},
	{7, 2, _BV(5), 2, _BV(6), 2, _BV(6), 2, _BV(7), 2, _BV(7)},
};

// Signal Heads (G, Y, R)
// 0: A0 - A2: #0 approach main signal  
// 1: A3 - A5: #0 approach siding signal
// 2: A6 - B0: #1 approach top signal
// 3: B1 - B3: #1 approach bottom signal
// 4: B4 - B6: #2 approach main signal
// 5: B7 - C1: #2 approach siding signal
// 6: C2 - C4: #3 approach top signal
// 7: C5 - C7: #3 approach bottom signal

void SignalsToOutputs(void)
{
	uint8_t sigDefIdx;
	for(sigDefIdx=0; sigDefIdx<sizeof(sigPinDefs)/sizeof(SignalPinDefinition); sigDefIdx++)
	{
		uint8_t redByte = sigPinDefs[sigDefIdx].redByte;
		uint8_t redMask = sigPinDefs[sigDefIdx].redMask;
		uint8_t yellow1Byte = sigPinDefs[sigDefIdx].yellow1Byte;
		uint8_t yellow1Mask = sigPinDefs[sigDefIdx].yellow1Mask;
		uint8_t yellow2Byte = sigPinDefs[sigDefIdx].yellow2Byte;
		uint8_t yellow2Mask = sigPinDefs[sigDefIdx].yellow2Mask;
		uint8_t greenByte = sigPinDefs[sigDefIdx].greenByte;
		uint8_t greenMask = sigPinDefs[sigDefIdx].greenMask;
		uint8_t lunarByte = sigPinDefs[sigDefIdx].lunarByte;
		uint8_t lunarMask = sigPinDefs[sigDefIdx].lunarMask;

		// Start by turning off all lights
		xio1Outputs[redByte] &= ~(redMask);
		xio1Outputs[yellow1Byte] &= ~(yellow1Mask);
		xio1Outputs[yellow2Byte] &= ~(yellow2Mask);
		xio1Outputs[greenByte] &= ~(greenMask);
		xio1Outputs[lunarByte] &= ~(lunarMask);

		switch(signalHeads[sigPinDefs[sigDefIdx].signalHead])
		{
			case ASPECT_OFF:
				break;
		
			case ASPECT_GREEN:
				xio1Outputs[greenByte] |= greenMask;
				break;
		
			case ASPECT_FL_GREEN:
				if (events & EVENT_BLINKY)
					xio1Outputs[greenByte] |= greenMask;
				break;

			case ASPECT_YELLOW:
				xio1Outputs[yellow1Byte] |= yellow1Mask;
				xio1Outputs[yellow2Byte] |= yellow2Mask;
				break;
		
			case ASPECT_FL_YELLOW:
				if (events & EVENT_BLINKY)
				{
					xio1Outputs[yellow1Byte] |= yellow1Mask;
					xio1Outputs[yellow2Byte] |= yellow2Mask;
				}
				break;
		
			case ASPECT_LUNAR:
				xio1Outputs[lunarByte] |= lunarMask;
				break;
		
			case ASPECT_RED:
			default:
				xio1Outputs[redByte] |= redMask;			
				break;

			case ASPECT_FL_RED:
				if (events & EVENT_BLINKY)
					xio1Outputs[redByte] |= redMask;
				break;
		}
	}
}


#define MRBUS_TX_BUFFER_DEPTH 4
#define MRBUS_RX_BUFFER_DEPTH 8
MRBusPacket mrbusTxPktBufferArray[MRBUS_TX_BUFFER_DEPTH];
MRBusPacket mrbusRxPktBufferArray[MRBUS_RX_BUFFER_DEPTH];

int main(void)
{
	uint8_t i, dir, temp_uint8;
	uint16_t temp_uint16;
	uint8_t stateChange = 0;
	uint8_t turnoutChange = 0;

	// Application initialization
	init();

	// Initialize a 100 Hz timer.  See the definition for this function - you can
	// remove it if you don't use it.
	initialize100HzTimer();

	// Initialize MRBus core
	mrbusPktQueueInitialize(&mrbusTxQueue, mrbusTxPktBufferArray, MRBUS_TX_BUFFER_DEPTH);
	mrbusPktQueueInitialize(&mrbusRxQueue, mrbusRxPktBufferArray, MRBUS_RX_BUFFER_DEPTH);
	mrbusInit();

	sei();	
	i2c_master_init();
	xioInitialize();

	while (1)
	{
		wdt_reset();

		// Handle any packets that may have come in
		// Do so before state machine so machine can act on inputs before sending status packet
		if (mrbusPktQueueDepth(&mrbusRxQueue))
			PktHandler();
		
		if (events & EVENT_I2C_ERROR)
		{
			i2cResetCounter++;
			xioInitialize();
		}

		if(events & (EVENT_READ_INPUTS))
		{
			readInputs();
			events &= ~(EVENT_READ_INPUTS);
		}			

		// Deal with fast time
		if((flags & TIME_FLAGS_DISP_FAST) && !(flags & TIME_FLAGS_DISP_FAST_HOLD) && fastDecisecs >= 10)
		{
			uint8_t fastTimeSecs;
			ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
			{
				fastTimeSecs = fastDecisecs / 10;
				fastDecisecs -= fastTimeSecs * 10;
			}
			incrementTime(&fastTime, fastTimeSecs);
		}

		// Trigger any scheduled trains
		if( (flags & TIME_FLAGS_DISP_FAST) && !(flags & TIME_FLAGS_DISP_FAST_HOLD) && deadReckoningTime )
		{
			// Only trigger if in fast time mode (and not held), and dead reckoning has not expired
			for(i=0; i<NUM_SIM_TRAINS; i++)
			{
				uint16_t currentTime = (fastTime.hours * 60) + fastTime.minutes;
				if( (simTrain[i].time <= currentTime) && ((simTrain[i].time + simTrainWindow) >= currentTime) )
				{
					// Inside time window for triggering
					if(!(simTrain[i].triggered))
					{
						// Not already triggered
						if(!simulator[simTrain[i].direction].enable)
						{
							// Only if not already enabled - don't stomp on any existing simulated trains from that direction since it might mess up the timers
							simulator[simTrain[i].direction].enable = 1;
							simulator[simTrain[i].direction].approachTime = simTrain[i].approachTime;
							simulator[simTrain[i].direction].totalTime = simTrain[i].totalTime;
							simulator[simTrain[i].direction].sound = simTrain[i].flags | 0x03;
							if(simTrain[i].flags & 0x80)
								interchangeEnable = 1;
							simTrain[i].triggered = 1;
						}
					}
				}
				else
				{
					// Not in time window, untrigger
					simTrain[i].triggered = 0;
				}
			}
		}
		
		// Logic for the automatic interchange
		if(interchangeEnable)
		{
			if(!interchangeOccupancy)
			{
				// Interchange track not occupied, enable relay
				startInterchange();
			}
			else
			{
				// Interchange track occupied, disable relay and disable interchange until next scheduled train
				stopInterchange();
				interchangeEnable = 0;
			}
		}

		// Check for turnout changes
		turnoutChange = 0;
		for(dir=0; dir<NUM_DIRECTIONS; dir++)
		{
			if(turnoutOriginal[dir] != turnout[dir])
				turnoutChange |= _BV(dir);
			turnoutOriginal[dir] = turnout[dir];
		}

		// Process state machines
		for(dir=0; dir<NUM_DIRECTIONS; dir++)
		{
			wdt_reset();

			switch(state[dir])
			{
				case STATE_IDLE:
					if(turnoutChange & _BV(dir))
					{
						lockoutTimer[dir] = 0;  // Clear lockout timer in the event the turnout changed
					}

					if( (approachBlockOccupancy(dir) || simulator[dir].enable) && !lockoutTimer[dir] )
					{
						// Train in approach block (or simulated train) and not in lockout
						if(requestInterlocking(dir))
						{
							// Request for interlocking approved
							state[dir] = STATE_CLEARANCE;
						}
					}
					break;
				case STATE_CLEARANCE:
					if( (turnoutChange & _BV(dir)) || (turnoutChange & _BV(OPPOSITE_DIRECTION(dir))) )
					{
						// Turnout changed, reset
						state[dir] = STATE_CLEAR;
					}
					else if(simulator[dir].enable)
					{
						// Priority given to simulated train
						ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
						{
							simulator[dir].timer = 10 * simulator[dir].totalTime;
						}
						startSound(simulator[dir].sound);
						state[dir] = STATE_TIMER;
					}
					else if(interlockingBlockOccupancy())
					{
						// Train has entered interlocking, proceed
						state[dir] = STATE_OCCUPIED;
					}
					else if(!approachBlockOccupancy(dir))
					{
						// No occupany in approach block, start timeout
						ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
						{
							timeoutTimer[dir] = 10 * timeoutSeconds[dir];
						}
						state[dir] = STATE_TIMEOUT;
					}
					// Wait here if no exit conditions met
					break;
				case STATE_TIMEOUT:
					ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
					{
						temp_uint16 = timeoutTimer[dir];
					}

					// Give priority to turnouts, then occupancy, then timeout
					if( (turnoutChange & _BV(dir)) || (turnoutChange & _BV(OPPOSITE_DIRECTION(dir))) )
					{
						// Turnout changed, reset
						state[dir] = STATE_CLEAR;
					}
					else if(interlockingBlockOccupancy())
					{
						// Train has entered interlocking, proceed
						state[dir] = STATE_OCCUPIED;
					}
					else if(approachBlockOccupancy(dir))
					{
						// Approach detector covered again, go back
						state[dir] = STATE_CLEARANCE;
					}
					else if(!temp_uint16)
					{
						// Timed out.  Reset
						state[dir] = STATE_CLEAR;
					}
					break;
				case STATE_TIMER:
					ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
					{
						temp_uint16 = simulator[dir].timer;
					}
					if( temp_uint16 <= (10 * (simulator[dir].totalTime - simulator[dir].approachTime)) )
					{
						// Approach time has run out, assume simulated train is in interlocking now
						state[dir] = STATE_OCCUPIED;
					}
					break;
				case STATE_OCCUPIED:
					ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
					{
						temp_uint16 = simulator[dir].timer;
					}
					if(!simulator[dir].enable && !interlockingBlockOccupancy())
					{
						// Non-simulated: proceed if interlocking block is clear
						state[dir] = STATE_LOCKOUT;
					}
					else if(simulator[dir].enable && !temp_uint16)
					{
						// Simulated train: proceed once simTime timer expires
						state[dir] = STATE_LOCKOUT;
					}
					break;
				case STATE_LOCKOUT:
					// Set lockout on opposite approach
					temp_uint8 = OPPOSITE_DIRECTION(dir);
					ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
					{
						lockoutTimer[temp_uint8] = 10 * lockoutSeconds;
					}
					turnoutOriginal[temp_uint8] = turnout[temp_uint8];  // Save turnout state for cancelling lockout
					state[dir] = STATE_CLEAR;
					break;
				case STATE_CLEAR:
					interlockingStatus = 0;
					if(simulator[dir].enable)
					{
						simulator[dir].enable = 0;
						stopSound(simulator[dir].sound);
					}
					state[dir] = STATE_IDLE;
					break;
			}
		}

		// Check for state changes
		stateChange = 0;
		for(i=0; i<NUM_DIRECTIONS; i++)
		{
			if(state[i] != oldState[i])
			{
				stateChange = 1;
				if( 
				    ((STATE_OCCUPIED != oldState[i]) && (STATE_OCCUPIED == state[i])) || 
				    (((STATE_CLEARANCE == oldState[i]) || (STATE_TIMEOUT == oldState[i])) && (STATE_CLEAR == state[i]))
				  )
				{
					// Transition into OCCUPIED
					// or...
					// Transition into CLEAR from CLEARANCE or TIMEOUT
					// then...
					// Set TIMELOCK
					ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
					{
						timelockTimer = 10 * timelockSeconds;
					}
				}
			}
			oldState[i] = state[i];
		}

		InterlockingToSignals();

		// Send output
		if (events & EVENT_WRITE_OUTPUTS)
		{
			SignalsToOutputs();
			xioOutputWrite();
			events &= ~(EVENT_WRITE_OUTPUTS);
		}

#ifdef MRBEE
		mrbeePoll();
#endif
		uint8_t txBuffer[MRBUS_BUFFER_SIZE];

		// FIXME: Do the sound outputs also show up in byte 7?  If so, no need to replicate in byte 11, just the OR is needed there

		// 6:  Inputs (Block Detect)
		// 7:  Inputs (Turnout): [<7:4>][/TO2][TO2][/TO0][TO0]
		// 8:  Signal Outputs Bank #1
		// 9:  Signal Outputs Bank #2
		// 10: Signal Outputs Bank #3
		// 11: [Sound#1][Sound#0][Sound#1|Sound#0][<4>][simulator<3:0>.enable]
		// 12: [state<0>][state<1>]
		// 13: [state<2>][state<3>]
		// 14: timelockTimer
		// 15: debounceTimer
		// 16: [<7:4>][lockoutTimer<3:0> Status]
		// 17: [<7:4>][timeoutTimer<3:0> Status]
		// 18: busVoltage

		uint16_t timeTemp;
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
		{
			timeTemp = decisecs;
		}
		
		if ( ((timeTemp >= update_decisecs) || (stateChange) || (sendPacketOnChange)) && !(mrbusPktQueueFull(&mrbusTxQueue)) )
		{
			sendPacketOnChange = 0;
			ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
			{
				decisecs = 0;
			}
			txBuffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
			txBuffer[MRBUS_PKT_DEST] = 0xFF;
			txBuffer[MRBUS_PKT_LEN] = 19;
			txBuffer[5]  = 'S';
			// Debounced input status
			txBuffer[6]  = debounced_inputs[0];
			uint8_t turnoutTemp = debounced_inputs[1];
			if(turnoutTemp & 0x01)
				turnoutTemp &= ~(0x02);
			else
				turnoutTemp |= 0x02;
			if(turnoutTemp & 0x04)
				turnoutTemp &= ~(0x08);
			else
				turnoutTemp |= 0x08;
			txBuffer[7]  = turnoutTemp;

			// Signal outputs
			uint32_t signalHeadsTemp = 0;
			for(i=0; i<sizeof(signalHeads); i++)
			{
				signalHeadsTemp |= (uint32_t)signalHeads[i] << (3*i);
			}
			txBuffer[8]  = (signalHeadsTemp >> 16) & 0xFF;
			txBuffer[9]  = (signalHeadsTemp >> 8) & 0xFF;
			txBuffer[10] = (signalHeadsTemp >> 0) & 0xFF;

			temp_uint8 = xio1Outputs[4] & 0xC0;  // Grab sound output bits
			if(temp_uint8)
				temp_uint8 |= 0x20;  // If either sound enabled, set global sound bit
			for(i=0; i<NUM_DIRECTIONS; i++)
			{
				// Add in simulator enables
				if(simulator[i].enable)
					temp_uint8 |= _BV(i);
			}
			txBuffer[11] = temp_uint8;           // Sound bits + simulator enables
			
			txBuffer[12] = ((state[0] << 4) & 0xF0) | (state[1] & 0x0F);  // State machine states
			txBuffer[13] = ((state[2] << 4) & 0xF0) | (state[3] & 0x0F);  // State machine states

			ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
			{
				temp_uint16 = timelockTimer;
			}
			txBuffer[14] = (temp_uint16 + 5) / 10;     // Timelock timer (seconds)

			ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
			{
				temp_uint16 = interlockingDebounceTimer;
			}
			txBuffer[15] = (temp_uint16 + 5) / 10;     // Debounce timer (seconds)

			temp_uint8 = 0;
			for(i=0; i<NUM_DIRECTIONS; i++)
			{
				ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
				{
					temp_uint16 = lockoutTimer[i];
				}
				if(temp_uint16)
					temp_uint8 |= _BV(i);
			}
			txBuffer[16] = temp_uint8;           // Lockout timer statuses

			temp_uint8 = 0;
			for(i=0; i<NUM_DIRECTIONS; i++)
			{
				ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
				{
					temp_uint16 = timeoutTimer[i];
				}
				if(temp_uint16)
					temp_uint8 |= _BV(i);
			}
			txBuffer[17] = temp_uint8;           // Timeout timer statuses

			txBuffer[18] = busVoltage;           // Bus voltage
			
			mrbusPktQueuePush(&mrbusTxQueue, txBuffer, txBuffer[MRBUS_PKT_LEN]);
		}

		if (mrbusPktQueueDepth(&mrbusTxQueue))
		{
			uint8_t fail = mrbusTransmit();

			// If we're here, we failed to start transmission due to somebody else transmitting
			// Given that our transmit buffer is full, priority one should be getting that data onto
			// the bus so we can start using our tx buffer again.  So we stay in the while loop, trying
			// to get bus time.

			// We want to wait 20ms before we try a retransmit to avoid hammering the bus
			// Because MRBus has a minimum packet size of 6 bytes @ 57.6kbps,
			// need to check roughly every millisecond to see if we have a new packet
			// so that we don't miss things we're receiving while waiting to transmit
			if (fail)
			{

#ifndef MRBEE
				uint8_t bus_countdown = 20;
				while (bus_countdown-- > 0 && !mrbusIsBusIdle())
				{
					wdt_reset();
					_delay_ms(1);
					if (mrbusPktQueueDepth(&mrbusRxQueue))
						PktHandler();
				}
#endif
			}
		}
	}
}



