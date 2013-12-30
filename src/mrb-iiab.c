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

// interlockingStatus currently limits this to a maximum of 7
#define NUM_DIRECTIONS 4

// Assumptions:
//   "Direction" pairs are connected.  0/1, 2/3, 4/5, etc
//   Even numbered directions can have turnouts.  Wired off for "main" if they don't exist.  Signals are separate frog-end signals.
//   Odd numbered directions have no turnouts.  Signals are 2-headed point-end signals using the turnout position from across the diamond.

typedef enum
{
	STATE_IDLE,
	STATE_REQUEST,
	STATE_CLEARANCE,
	STATE_TIMEOUT,
	STATE_TIMER,
	STATE_OCCUPIED,
	STATE_CLEAR,
	STATE_LOCKOUT,
} InterlockState;

InterlockState state[NUM_DIRECTIONS];

typedef struct
{
	uint8_t enable;
	uint16_t timer;  // decisecs
	uint8_t approachTime;
	uint8_t totalTime;
	uint8_t sound;
} Simulator;

Simulator simulator[NUM_DIRECTIONS];

uint16_t timeout[NUM_DIRECTIONS];  // decisecs
uint8_t timeoutSeconds = 5;  // FIXME: load value from EEPROM
uint8_t lockoutSeconds = 5;  // FIXME: load value from EEPROM

#define OCCUPANCY_MAIN   0x01
#define OCCUPANCY_SIDING 0x02

uint8_t approachOccupancy[NUM_DIRECTIONS];
uint8_t interlockingOccupancy;

#define TURNOUT_MAIN   0
#define TURNOUT_SIDING 1

uint8_t turnout[NUM_DIRECTIONS];

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

uint8_t signalMain[NUM_DIRECTIONS];
uint8_t signalSecondary[NUM_DIRECTIONS];


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
volatile uint16_t update_decisecs=10;

void initialize100HzTimer(void)
{
	// Set up timer 1 for 100Hz interrupts
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
	if (++ticks >= 10)
	{
		uint8_t i;
		ticks = 0;
		decisecs++;
		for(i=0; i<NUM_DIRECTIONS; i++)
		{
			// Manage timers
			if(timeout[i])
				timeout[i]--;
			if(simulator[i].timer)
				simulator[i].timer--;
		}
	}
}

// End of 100Hz timer

// **** Bus Voltage Monitor
/*
// Uncomment this block (and the ADC initialization in the init() function) if you want to continuously monitor bus voltage

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
*/


void PktHandler(void)
{
	uint16_t crc = 0;
	uint8_t i;
	uint8_t rxBuffer[MRBUS_BUFFER_SIZE];
	uint8_t txBuffer[MRBUS_BUFFER_SIZE];

	if (0 == mrbusPktQueuePop(&mrbusRxQueue, rxBuffer, sizeof(rxBuffer)))
		return;


	//*************** PACKET FILTER ***************
	// Loopback Test - did we send it?  If so, we probably want to ignore it
	if (rxBuffer[MRBUS_PKT_SRC] == mrbus_dev_addr) 
		goto	PktIgnore;

	// Destination Test - is this for us or broadcast?  If not, ignore
	if (0xFF != rxBuffer[MRBUS_PKT_DEST] && mrbus_dev_addr != rxBuffer[MRBUS_PKT_DEST]) 
		goto	PktIgnore;
	
	// CRC16 Test - is the packet intact?
	for(i=0; i<rxBuffer[MRBUS_PKT_LEN]; i++)
	{
		if ((i != MRBUS_PKT_CRC_H) && (i != MRBUS_PKT_CRC_L)) 
			crc = mrbusCRC16Update(crc, rxBuffer[i]);
	}
	if ((UINT16_HIGH_BYTE(crc) != rxBuffer[MRBUS_PKT_CRC_H]) || (UINT16_LOW_BYTE(crc) != rxBuffer[MRBUS_PKT_CRC_L]))
		goto	PktIgnore;
		
	//*************** END PACKET FILTER ***************


	//*************** PACKET HANDLER - PROCESS HERE ***************

	// Just smash the transmit buffer if we happen to see a packet directed to us
	// that requires an immediate response
	//
	// If we're in here, then either we're transmitting, then we can't be 
	// receiving from someone else, or we failed to transmit whatever we were sending
	// and we're waiting to try again.  Either way, we're not going to corrupt an
	// in-progress transmission.
	//
	// All other non-immediate transmissions (such as scheduled status updates)
	// should be sent out of the main loop so that they don't step on things in
	// the transmit buffer
	
	if ('A' == rxBuffer[MRBUS_PKT_TYPE])
	{
		// PING packet
		txBuffer[MRBUS_PKT_DEST] = rxBuffer[MRBUS_PKT_SRC];
		txBuffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
		txBuffer[MRBUS_PKT_LEN] = 6;
		txBuffer[MRBUS_PKT_TYPE] = 'a';
		mrbusPktQueuePush(&mrbusTxQueue, txBuffer, txBuffer[MRBUS_PKT_LEN]);
		goto PktIgnore;
	} 
	else if ('W' == rxBuffer[MRBUS_PKT_TYPE]) 
	{
		// EEPROM WRITE Packet
		// FIXME: Accept more than 1 byte
		txBuffer[MRBUS_PKT_DEST] = rxBuffer[MRBUS_PKT_SRC];
		txBuffer[MRBUS_PKT_LEN] = 8;			
		txBuffer[MRBUS_PKT_TYPE] = 'w';
		eeprom_write_byte((uint8_t*)(uint16_t)rxBuffer[6], rxBuffer[7]);
		txBuffer[6] = rxBuffer[6];
		txBuffer[7] = rxBuffer[7];
		if (MRBUS_EE_DEVICE_ADDR == rxBuffer[6])
			mrbus_dev_addr = eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_ADDR);
		txBuffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
		mrbusPktQueuePush(&mrbusTxQueue, txBuffer, txBuffer[MRBUS_PKT_LEN]);
		goto PktIgnore;	
	}
	else if ('R' == rxBuffer[MRBUS_PKT_TYPE]) 
	{
		// EEPROM READ Packet
		// FIXME: Accept more than 1 byte
		txBuffer[MRBUS_PKT_DEST] = rxBuffer[MRBUS_PKT_SRC];
		txBuffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
		txBuffer[MRBUS_PKT_LEN] = 8;			
		txBuffer[MRBUS_PKT_TYPE] = 'r';
		txBuffer[6] = rxBuffer[6];
		txBuffer[7] = eeprom_read_byte((uint8_t*)(uint16_t)rxBuffer[6]);			
		mrbusPktQueuePush(&mrbusTxQueue, txBuffer, txBuffer[MRBUS_PKT_LEN]);
		goto PktIgnore;
	}
	else if ('V' == rxBuffer[MRBUS_PKT_TYPE]) 
	{
		// Version
		txBuffer[MRBUS_PKT_DEST] = rxBuffer[MRBUS_PKT_SRC];
		txBuffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
		txBuffer[MRBUS_PKT_LEN] = 16;
		txBuffer[MRBUS_PKT_TYPE] = 'v';
		txBuffer[6]  = MRBUS_VERSION_WIRED;
		txBuffer[7]  = 0; // Software Revision
		txBuffer[8]  = 0; // Software Revision
		txBuffer[9]  = 0; // Software Revision
		txBuffer[10]  = 0; // Hardware Major Revision
		txBuffer[11]  = 0; // Hardware Minor Revision
		txBuffer[12] = 'I';
		txBuffer[13] = 'I';
		txBuffer[14] = 'A';
		txBuffer[15] = 'B';
		mrbusPktQueuePush(&mrbusTxQueue, txBuffer, txBuffer[MRBUS_PKT_LEN]);
		goto PktIgnore;
	}
	else if ('X' == rxBuffer[MRBUS_PKT_TYPE]) 
	{
		// Reset
		cli();
		wdt_reset();
		MCUSR &= ~(_BV(WDRF));
		WDTCSR |= _BV(WDE) | _BV(WDCE);
		WDTCSR = _BV(WDE);
		while(1);  // Force a watchdog reset
		sei();
	}
	
	// FIXME: Train Sim Command Packet
	// if(STATE_IDLE == state[direction]) { state[direction] = STATE_REQUEST }
	// Advance to request state only if that direction is currently idle
	/*	<direction> <simApproachTime> <simTotalTime> <sound> */
	/*	<direction> = 0-3 representing direction from which the simulated train is approaching*/
	/*	<simApproachTIme> = time in seconds between getting green and train crossing the diamond (setting the signal red).*/
	/*	<simTotalTime> = time in seconds from getting green until the diamond is cleared.  Sound plays for this entire interval.*/
	/*	simTotalTime should be > simApproachTime (enforce this)*/
	/*	<sound> = 0:no sound, 1-2: sound output to enable */

	//*************** END PACKET HANDLER  ***************

	
	//*************** RECEIVE CLEANUP ***************
PktIgnore:
	// Yes, I hate gotos as well, but sometimes they're a really handy and efficient
	// way to jump to a common block of cleanup code at the end of a function 

	// This section resets anything that needs to be reset in order to allow us to receive
	// another packet.  Typically, that's just clearing the MRBUS_RX_PKT_READY flag to 
	// indicate to the core library that the mrbus_rx_buffer is clear.
	return;	
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

	// Initialize MRBus address from EEPROM address 1
	mrbus_dev_addr = eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_ADDR);
	// Bogus addresses, fix to default address
	if (0xFF == mrbus_dev_addr || 0x00 == mrbus_dev_addr)
	{
		mrbus_dev_addr = 0x03;
		eeprom_write_byte((uint8_t*)MRBUS_EE_DEVICE_ADDR, mrbus_dev_addr);
	}
	
	update_decisecs = (uint16_t)eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_UPDATE_L) 
		| (((uint16_t)eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_UPDATE_H)) << 8);

	// This line assures that update_decisecs is at least 1
	update_decisecs = max(1, update_decisecs);
	
	// FIXME: This line assures that update_decisecs is 2 seconds or less
	// You probably don't want this, but it prevents new developers from wondering
	// why their new node doesn't transmit (uninitialized eeprom will make the update
	// interval 64k decisecs, or about 110 hours)  You'll probably want to make this
	// something more sane for your node type, or remove it entirely.
	update_decisecs = min(20, update_decisecs);

	// Reset state machines
	for(i=0; i<NUM_DIRECTIONS; i++)
	{
		state[i] = STATE_IDLE;
	}
	
/*
// Uncomment this block to set up the ADC to continuously monitor the bus voltage using a 3:1 divider tied into the ADC7 input
// You also need to uncomment the ADC ISR near the top of the file
	// Setup ADC
	ADMUX  = 0x47;  // AVCC reference; ADC7 input
	ADCSRA = _BV(ADATE) | _BV(ADIF) | _BV(ADPS2) | _BV(ADPS1); // 128 prescaler
	ADCSRB = 0x00;
	DIDR0  = 0x00;  // No digitals were harmed in the making of this ADC

	busVoltage = 0;
	ADCSRA |= _BV(ADEN) | _BV(ADSC) | _BV(ADIE) | _BV(ADIF);
*/

	// FIXME: Arduino specific debug ports
	PORTC &= 0xF0;  // PC0-PC3 as outputs, driven low
	DDRC |= 0x0F;
	
	DDRD &= 0x00;
	PORTD |= 0xFF;  // PD0-PD7 as inputs, pull-up enabled

	// Reset all occupancy to 0
	for(i=0; i<NUM_DIRECTIONS; i++)
	{
		approachOccupancy[i] = 0;
		turnout[i] = TURNOUT_MAIN;
	}
	interlockingOccupancy = 0;
	
	interlockingStatus = 0;
	
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
	if(interlockingStatus & INTERLOCKING_LOCKED)
		return 0;  // Already locked
	else if(interlockingBlockOccupancy())
		return 0;  // Interlocking occupied
	
	// Everything good.  Take it.
	interlockingStatus = INTERLOCKING_LOCKED | _BV(direction);
	return 1;
}

void startSound(uint8_t sound)
{
	// FIXME: Do something real
}

void readInputs(void)
{
	// Read occupancy into occupancy[] array and interlockOccupancy.  Force siding occupancy = 0 on directions without a siding.
	// Debounce
	// Read turnout positions; force directions without turnouts to main
	
	if(PIND & _BV(PD0))
		turnout[0] = TURNOUT_SIDING;
	else
		turnout[0] = TURNOUT_MAIN;

	// Approach blocks
	if(PIND & _BV(PD7))
		approachOccupancy[1] |= OCCUPANCY_MAIN;
	else
		approachOccupancy[1] &= ~OCCUPANCY_MAIN;

	if(PIND & _BV(PD6))
		approachOccupancy[0] |= OCCUPANCY_SIDING;
	else
		approachOccupancy[0] &= ~OCCUPANCY_SIDING;

	if(PIND & _BV(PD5))
		approachOccupancy[0] |= OCCUPANCY_MAIN;
	else
		approachOccupancy[0] &= ~OCCUPANCY_MAIN;

	if(PIND & _BV(PD4))
		approachOccupancy[2] |= OCCUPANCY_MAIN;
	else
		approachOccupancy[2] &= ~OCCUPANCY_MAIN;

	if(PIND & _BV(PD3))
		approachOccupancy[3] |= OCCUPANCY_MAIN;
	else
		approachOccupancy[3] &= ~OCCUPANCY_MAIN;

	// Interlocking block
	if(PIND & _BV(PD2))
		interlockingOccupancy |= OCCUPANCY_MAIN;
	else
		interlockingOccupancy &= ~OCCUPANCY_MAIN;

}

void InterlockingToSignals(void)
{
	uint8_t i;
	
	for(i=0; i<NUM_DIRECTIONS; i++)
	{
		if(interlockingStatus & _BV(i))
		{
			// Direction 'i' cleared for interlocking; set signals appropriately
			if(i % 2)
			{
				// Odd direction = point end
				if(turnout[i-1])
				{
					// Opposite turnout set for siding
					signalMain[i] = ASPECT_RED;
					signalSecondary[i] = ASPECT_YELLOW;
				}
				else
				{
					// Opposite turnout set for main
					signalMain[i] = ASPECT_GREEN;
					signalSecondary[i] = ASPECT_RED;
				}
			}
			else
			{
				// Even direction = frog end
				if(turnout[i])
				{
					// Turnout set for siding
					signalMain[i] = ASPECT_RED;
					signalSecondary[i] = ASPECT_GREEN;
				}
				else
				{
					// Turnout set for main
					signalMain[i] = ASPECT_GREEN;
					signalSecondary[i] = ASPECT_RED;
				}
			}
		}
		else
		{
			// Default to most restrictive
			signalMain[i] = ASPECT_RED;
			signalSecondary[i] = ASPECT_RED;
		}
	}
}

void SignalsToOutputs(void)
{
	uint8_t i;
	
	for(i=0; i<NUM_DIRECTIONS; i++)
	{
		if(ASPECT_GREEN == signalMain[i])
			PORTC |= _BV(i);
		else
			PORTC &= ~_BV(i);
	}
}


#define MRBUS_TX_BUFFER_DEPTH 4
#define MRBUS_RX_BUFFER_DEPTH 8
MRBusPacket mrbusTxPktBufferArray[MRBUS_TX_BUFFER_DEPTH];
MRBusPacket mrbusRxPktBufferArray[MRBUS_RX_BUFFER_DEPTH];

int main(void)
{
	uint8_t i;
	uint16_t temp16;
	
	// Application initialization
	init();

	// Initialize a 100 Hz timer.  See the definition for this function - you can
	// remove it if you don't use it.
	initialize100HzTimer();

	// Initialize MRBus core
	mrbusPktQueueInitialize(&mrbusTxQueue, mrbusTxPktBufferArray, MRBUS_TX_BUFFER_DEPTH);
	mrbusPktQueueInitialize(&mrbusRxQueue, mrbusRxPktBufferArray, MRBUS_RX_BUFFER_DEPTH);
//	mrbusInit();

	sei();	

	while (1)
	{
		wdt_reset();
		
		readInputs();
		
/*		for(i=0; i<NUM_DIRECTIONS; i++)*/
/*		{*/
/*			if(approachOccupancy[i])*/
/*				PORTC |= _BV(i);*/
/*			else*/
/*				PORTC &= ~_BV(i);*/
/*		}*/
/*		if(interlockingOccupancy)*/
/*			PORTC |= _BV(PC0);*/
/*		else*/
/*			PORTC &= ~_BV(PC0);*/

		// Process state machines
		for(i=0; i<NUM_DIRECTIONS; i++)
		{
			wdt_reset();

			switch(state[i])
			{
				case STATE_IDLE:
					if(simulator[i].enable)
					{
						// Simulated train enabled, so proceed
						state[i] = STATE_REQUEST;
					}
					else if(approachBlockOccupancy(i))
					{
						// Train in approach block, proceed
						state[i] = STATE_REQUEST;
					}
					break;
				case STATE_REQUEST:
					if(requestInterlocking(i))
					{
						// Request for interlocking approved; signals will get set by interlockingStatus
						state[i] = STATE_CLEARANCE;
					}
					// Stay here if request not approved
					break;
				case STATE_CLEARANCE:
					if(simulator[i].enable)
					{
						// Priority given to simulated train
						ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
						{
							simulator[i].timer = 10 * simulator[i].totalTime;
						}
						startSound(simulator[i].sound);
						state[i] = STATE_TIMER;
					}
					else if(!approachBlockOccupancy(i))
					{
						// No occupany in approach block, start timeout
						ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
						{
							timeout[i] = 10 * timeoutSeconds;
						}
						state[i] = STATE_TIMEOUT;
					}
					else if(interlockingBlockOccupancy())
					{
						// Train has entered interlocking, proceed
						state[i] = STATE_OCCUPIED;
					}
					// Wait here if no exit conditions met
					break;
				case STATE_TIMEOUT:
					ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
					{
						temp16 = timeout[i];
					}
					if(!temp16)
					{
						// Timed out.  Reset
						state[i] = STATE_CLEAR;
					}
					else if(approachBlockOccupancy(i))
					{
						// Approach detector covered again, go back
						state[i] = STATE_CLEARANCE;
					}
					else if(interlockingBlockOccupancy())
					{
						// Train has entered interlocking, proceed
						state[i] = STATE_OCCUPIED;
					}
					break;
				case STATE_TIMER:
					ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
					{
						temp16 = simulator[i].timer;
					}
					if( temp16 <= (10 * (simulator[i].totalTime - simulator[i].approachTime)) )
					{
						// Approach time has run out, assume simulated train is in interlocking now
						state[i] = STATE_OCCUPIED;
					}
					break;
				case STATE_OCCUPIED:
					// Set home signal to "stop" but keep locked
					interlockingStatus = INTERLOCKING_LOCKED;

					ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
					{
						temp16 = simulator[i].timer;
					}
					if(!simulator[i].enable && !interlockingBlockOccupancy())
					{
						// Non-simulated: proceed if interlocking block is clear
						state[i] = STATE_CLEAR;
					}
					else if(simulator[i].enable && !temp16)
					{
						// Simulated train: proceed once simTime timer expires
						state[i] = STATE_CLEAR;
					}
					break;
				case STATE_CLEAR:
					// Clear interlocking lock and any signals still set to proceed (in the event of a timeout)
					interlockingStatus = 0;

					simulator[i].enable = 0;
					ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
					{
						timeout[i] = 10 * lockoutSeconds;
					}
					state[i] = STATE_LOCKOUT;
					break;
				case STATE_LOCKOUT:
					if(!timeout[i])
						state[i] = STATE_IDLE;
					break;
			}
		}

		InterlockingToSignals();
		SignalsToOutputs();


#ifdef MRBEE
		mrbeePoll();
#endif
		// Handle any packets that may have come in
		if (mrbusPktQueueDepth(&mrbusRxQueue))
			PktHandler();
			
		// FIXME: Do any module-specific behaviours here in the loop.
		
		if (decisecs >= update_decisecs && !(mrbusPktQueueFull(&mrbusTxQueue)))
		{
			uint8_t txBuffer[MRBUS_BUFFER_SIZE];

			txBuffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
			txBuffer[MRBUS_PKT_DEST] = 0xFF;
			txBuffer[MRBUS_PKT_LEN] = 7;
			txBuffer[5] = 'S';
			txBuffer[6] = pkt_count++;
			mrbusPktQueuePush(&mrbusTxQueue, txBuffer, txBuffer[MRBUS_PKT_LEN]);
			decisecs = 0;
		}	

/*
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
*/
	}
}



