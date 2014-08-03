/*************************************************************************
Title:    MRBus Generic Indicator Module v2 (MRB-GIM2)
Authors:  Nathan D. Holmes <maverick@drgw.net>
File:     mrb-gim2.c
License:  GNU General Public License v3

LICENSE:
    Copyright (C) 2014 Nathan Holmes

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
#include <util/atomic.h>
#include <string.h>

#include "mrbus.h"

// Global Variables
uint8_t mrbus_dev_addr = 0;

#define MRBUS_TX_BUFFER_DEPTH 4
#define MRBUS_RX_BUFFER_DEPTH 16

// MRB-IIAB state defines
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

MRBusPacket mrbusTxPktBufferArray[MRBUS_TX_BUFFER_DEPTH];
MRBusPacket mrbusRxPktBufferArray[MRBUS_RX_BUFFER_DEPTH];

//uint8_t indication[8] = {0,0,0,0,0,0,0,0};

uint8_t localIndication[8] = {0,0,0,0,0,0,0,0};
uint8_t setIndication[8] = {0,0,0,0,0,0,0,0};
uint8_t useSetIndication[8] = {0,0,0,0,0,0,0,0};
uint8_t oldIndication[8] = {0,0,0,0,0,0,0,0};
uint8_t blinkIndication[8] = {0,0,0,0,0,0,0,0};
//indication = (useSetInd & setIndication) | ((~useSetInd) & localIndication)

volatile uint8_t blinkInterval;
uint8_t lampTest=0;
uint8_t changed=0;
volatile uint16_t decisecs=0;
uint16_t update_decisecs=0;
volatile uint8_t force7221Refresh=0;
volatile uint8_t blinkState = 0;
uint8_t ledPktRcvd=0;
uint8_t status = 0;

#define STATUS_LAMP_TEST     0x01
#define STATUS_SEND_ON_CHG   0x02
#define STATUS_SEND_ON_TIME  0x04

#define EE_OCC_ADDR       0x10
#define EE_OCC_PKT        0x50
#define EE_OCC_BITBYTE    0x90
#define EE_OCC_SUBTYPE    0xD0


volatile uint16_t busVoltageAccum=0;
volatile uint16_t busVoltage=0;
volatile uint8_t busVoltageCount=0;

void initializeADC()
{
	// Setup ADC for bus voltage monitoring
	ADMUX  = 0x46;  // AVCC reference; ADC6 input
	ADCSRA = _BV(ADATE) | _BV(ADIF) | _BV(ADPS2) | _BV(ADPS1); // 128 prescaler
	ADCSRB = 0x00;
	DIDR0  = 0x00;  // No digitals were harmed in the making of this ADC

	busVoltage = 0;
	busVoltageAccum = 0;
	busVoltageCount = 0;
	ADCSRA |= _BV(ADEN) | _BV(ADSC) | _BV(ADIE) | _BV(ADIF);
}

ISR(ADC_vect)
{
	busVoltageAccum += ADC;
	if (++busVoltageCount >= 64)
	{
		busVoltageAccum = busVoltageAccum / 64;
        //At this point, we're at (Vbus/3) / 5 * 1024
        //So multiply by 150, divide by 1024, or multiply by 75 and divide by 512
        busVoltage = ((uint32_t)busVoltageAccum * 75) / 512;
		busVoltageAccum = 0;
		busVoltageCount = 0;
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

void initialize100HzTimer(void)
{
	// Set up timer 1 for 100Hz interrupts
	TCNT0 = 0;
	OCR0A = 0xC2;
	decisecs = 0;
	TCCR0A = _BV(WGM01);
	TCCR0B = _BV(CS02) | _BV(CS00);
	TIMSK0 |= _BV(OCIE0A);
}

ISR(TIMER0_COMPA_vect)
{
	static uint8_t blinkCntr = 0;
	static uint8_t ticks=0;	

	if (++blinkCntr >= blinkInterval)
	{
		blinkCntr = 0;
		blinkState ^= 0xFF;
	}

	if (++ticks >= 10)
	{
		ticks = 0;
		decisecs++;
		force7221Refresh++;
	}
}

// End of 100Hz timer

void initializeSPI()
{
	DDRB |= _BV(PB2) | _BV(PB5) | _BV(PB3); // Set SS, SCK, and MOSI as an output
	PORTB |= _BV(PB2); // Set slave select high
	SPCR = _BV(SPE) | _BV(MSTR) | _BV(SPR0);
	SPSR = 0;
}

void MAX7221_SendData(uint8_t address, uint8_t data)
{
	PORTB &= ~_BV(PB2); // Clear the enable line
	SPDR = address;
	while (!(SPSR & _BV(SPIF))); // Wait for the address to send
	SPDR = data;
	while (!(SPSR & _BV(SPIF))); // Wait for the data to send
	PORTB |= _BV(PB2);  // Reset the enable line
}

void MAX7221_Initialize()
{
	// No decode
	MAX7221_SendData(0x09, 0x00);

	/* Clear display bits */
	MAX7221_SendData(0x01, 0x00);
	MAX7221_SendData(0x02, 0x00);
	MAX7221_SendData(0x03, 0x00);
	MAX7221_SendData(0x04, 0x00);
	MAX7221_SendData(0x05, 0x00);
	MAX7221_SendData(0x06, 0x00);
	MAX7221_SendData(0x07, 0x00);
	MAX7221_SendData(0x08, 0x00);
	
	/* Maximum Intensity */
	MAX7221_SendData(0x0A, 0x0F);
	
	/* Scan all eight digits */
	MAX7221_SendData(0x0B, 0x07);
	
	/* Turn that baby on */
	MAX7221_SendData(0x0C, 0x01);
	MAX7221_SendData(0x0F, 0x00);	
}


void PktHandler(void)
{
	uint8_t i;
	uint8_t rxBuffer[MRBUS_BUFFER_SIZE];
	uint8_t txBuffer[MRBUS_BUFFER_SIZE];

	if (0 == mrbusPktQueuePop(&mrbusRxQueue, rxBuffer, sizeof(rxBuffer)))
		return;

	//*************** PACKET FILTER ***************
	// Loopback Test - did we send it?  If so, we probably want to ignore it
	if (rxBuffer[MRBUS_PKT_SRC] == mrbus_dev_addr) 
		return;

	// Destination Test - is this for us or broadcast?  If not, ignore
	if (0xFF != rxBuffer[MRBUS_PKT_DEST] && mrbus_dev_addr != rxBuffer[MRBUS_PKT_DEST]) 
		return;
	
	// CRC16 Test - is the packet intact?
	if (!mrbusIsCrcValid(rxBuffer))
		return;
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
	else if ('W' == rxBuffer[MRBUS_PKT_TYPE] && 0xFF != rxBuffer[MRBUS_PKT_DEST]) 
	{
		// EEPROM WRITE Packet
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
		txBuffer[12] = 'G';
		txBuffer[13] = 'I';
		txBuffer[14] = 'M';
		txBuffer[15] = '2';
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
	else if ('C' == rxBuffer[MRBUS_PKT_TYPE] && (rxBuffer[MRBUS_PKT_LEN] > 6)) 
	{
		switch(rxBuffer[MRBUS_PKT_SUBTYPE])
		{
			case 'C':
				// Character to position
				
				break;
			case 'S':
				// Get status
				changed = 1;
				break;
				
			case 'T':
				// Test
				// lamp test of 0x01-0x08 means turn on all bits 0-7 respectively
				// lamp test of 0xFF means all on
				// lamp test of 0xFE means turn everything off - lamp test, all indications
				// lamp test of 0x00 returns to normal operation
				switch(rxBuffer[7])
				{
					case 0x00:
						lampTest = 0;
						status &= ~_BV(STATUS_LAMP_TEST);
						break;
				
					case 0x01:
					case 0x02:
					case 0x03:
					case 0x04:
					case 0x05:
					case 0x06:
					case 0x07:
					case 0x08:
						lampTest = _BV(0x07 & (rxBuffer[7]-1));
						status |= _BV(STATUS_LAMP_TEST);
						ledPktRcvd = 1;
						break;

					case 0xFF:
						lampTest = 0xFF;
						status |= _BV(STATUS_LAMP_TEST);
						ledPktRcvd = 1;
						break;

					case 0xFE:
						lampTest = 0;
						status |= _BV(STATUS_LAMP_TEST);
						ledPktRcvd = 1;
						break;
				
				}
				
				txBuffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
				txBuffer[MRBUS_PKT_DEST] = rxBuffer[MRBUS_PKT_SRC];
				txBuffer[MRBUS_PKT_LEN] = 9;	
				txBuffer[MRBUS_PKT_TYPE] = 'c';
				txBuffer[6] = 'T';
				txBuffer[7] = (status & _BV(STATUS_LAMP_TEST))?1:0;
				txBuffer[8] = lampTest;
				mrbusPktQueuePush(&mrbusTxQueue, txBuffer, txBuffer[MRBUS_PKT_LEN]);
				goto PktIgnore;				
								
			case 'L':
			{
				// lamp cmd of 0 means return all to local logic control
				// lamp cmd of 1-64 followed by a value turns it on (1) or off (0) or back to local control (2)
				// lamp cmd of 0x90 should be followed by 8 bytes of bitmask representing all lamps
				i = rxBuffer[7];
				if (i >=1 && i <= 64)
				{
					i--;
					if (rxBuffer[MRBUS_PKT_LEN] > 7)
					{
						uint8_t mask = 1<<(i%8);
						uint8_t byteNum = i/8;

						if ('1' == rxBuffer[8] || 1 == rxBuffer[8])
						{
							setIndication[byteNum] |= mask;
							useSetIndication[byteNum] |= mask;
						}
						else if ('0' == rxBuffer[8] || 0 == rxBuffer[8])
						{
							setIndication[byteNum] &= ~(mask);
							useSetIndication[byteNum] |= mask;
						}
						else if ('X' == rxBuffer[8])
						{
							setIndication[byteNum] &= ~(mask);
							useSetIndication[byteNum] &= ~(mask);
						}
					}
					ledPktRcvd = 1;
				}
				else if (0 == i)
				{
					memset(useSetIndication, 0, sizeof(useSetIndication));
					memset(setIndication, 0, sizeof(setIndication));
					ledPktRcvd = 1;				
				}
				else if (0x80 == i && rxBuffer[MRBUS_PKT_LEN] >= 16)
				{
					memset(useSetIndication, 0xFF, sizeof(useSetIndication));				
					memcpy(setIndication, (uint8_t*)rxBuffer, 8);
					ledPktRcvd = 1;
				}


			}
			break;

		}	
	}



	/*************** PACKET SUCCESS - PROCESS HERE ***************/

	/* BITBYTE is computed as follows:
		x = bit = 0-7
		y = byte = byte in data stream (6 is first data byte)
		xxxyyyy
	*/
	
	for (i=0; i<(EE_OCC_PKT - EE_OCC_ADDR); i++)
	{
		if (rxBuffer[MRBUS_PKT_SRC] == eeprom_read_byte((uint8_t*)(i+EE_OCC_ADDR)))
		{
			if (rxBuffer[MRBUS_PKT_TYPE] == eeprom_read_byte((uint8_t*)(i+EE_OCC_PKT)))
			{
				uint8_t byteNum = eeprom_read_byte((uint8_t*)(i+EE_OCC_BITBYTE));
				uint8_t bitNum = (byteNum>>5) & 0x07;
				byteNum &= 0x1F;
	
				if (21 == byteNum)
				{
					uint8_t stateNibble = 0;
					// Special stuff for IIAB
					switch(i)
					{
						case 8:
							stateNibble = 0x0F & (rxBuffer[13]>>4);
							break;
						case 9:
							stateNibble = 0x0F & (rxBuffer[12]);
							break;
						case 10:
							stateNibble = 0x0F & (rxBuffer[12]>>4);
							break;
						case 11:
							stateNibble = 0x0F & (rxBuffer[13]);
							break;
					}
					
					switch(stateNibble)
					{
						case STATE_TIMEOUT:
							blinkIndication[i/8] |= 1<<(i%8);
							localIndication[i/8] |= 1<<(i%8);
							break;

						case STATE_CLEARANCE:
						case STATE_TIMER:
						case STATE_OCCUPIED:
							blinkIndication[i/8] &= ~(1<<(i%8));
							localIndication[i/8] |= 1<<(i%8);
							break;

						default:
							blinkIndication[i/8] &= ~(1<<(i%8));
							localIndication[i/8] &= ~(1<<(i%8));
							break;
					}
				} else {
					if (rxBuffer[byteNum] & (1<<bitNum))
						localIndication[i/8] |= 1<<(i%8);
					else
						localIndication[i/8] &= ~(1<<(i%8));
					blinkIndication[i/8] &= ~(1<<(i%8));

				}
				ledPktRcvd = 1;
			}
		}
	}


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

	initializeADC();

	initializeSPI();  // Set up the SPI interface

	// Initialize MRBus address from EEPROM address 0
	mrbus_dev_addr = eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_ADDR);
	// Bogus addresses, fix to default address
	if (0xFF == mrbus_dev_addr || 0x00 == mrbus_dev_addr)
	{
		mrbus_dev_addr = 0x03;
		eeprom_write_byte((uint8_t*)MRBUS_EE_DEVICE_ADDR, mrbus_dev_addr);
	}
	update_decisecs = (uint16_t)eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_UPDATE_L) 
		| (((uint16_t)eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_UPDATE_H)) << 8);


#define MRBUS_EE_BLINK_DECISECS 4

	blinkInterval = eeprom_read_byte((uint8_t*)MRBUS_EE_BLINK_DECISECS);
	if (0xFF == blinkInterval || 0x00 == blinkInterval)
	{
		blinkInterval = 50;
		eeprom_write_byte((uint8_t*)MRBUS_EE_BLINK_DECISECS, blinkInterval);	
	}

	// If update time is not zero, send status packets on time intervals
	if (0 != update_decisecs && 0xFFFF != update_decisecs)
	 status |= _BV(STATUS_SEND_ON_TIME);

	MAX7221_Initialize();
	
	// Initialize MRBus core
	mrbusPktQueueInitialize(&mrbusTxQueue, mrbusTxPktBufferArray, MRBUS_TX_BUFFER_DEPTH);
	mrbusPktQueueInitialize(&mrbusRxQueue, mrbusRxPktBufferArray, MRBUS_RX_BUFFER_DEPTH);
	mrbusInit();
}

// In no decode mode, the segments are mapped as follows:
// G=D0
// F=D1
// E=D2
// D=D3
// C=D4
// B=D5
// A=D6
// DP=D7
// On the board, they're A-DP, so we need to reorder things to 
// make it sane to the user

uint8_t indicationTransmuter(uint8_t indicationBits)
{
	uint8_t retval = 0;
	if (lampTest)
		indicationBits = lampTest;

	retval = indicationBits & 0x80;

	if (indicationBits & 0x01)
		retval |= _BV(6);	
	if (indicationBits & 0x02)
		retval |= _BV(5);	
	if (indicationBits & 0x04)
		retval |= _BV(4);	
	if (indicationBits & 0x08)
		retval |= _BV(3);	
	if (indicationBits & 0x10)
		retval |= _BV(2);	
	if (indicationBits & 0x20)
		retval |= _BV(1);	
	if (indicationBits & 0x40)
		retval |= _BV(0);	

	return(retval);
}

int main(void)
{
	uint8_t oldBlinkState = 0;
	// Application initialization
	init();

	// Initialize a 100 Hz timer.  See the definition for this function - you can
	// remove it if you don't use it.
	initialize100HzTimer();

	sei();	

	while(1)
	{
		wdt_reset();

		// Handle any packets that may have come in
		if (mrbusPktQueueDepth(&mrbusRxQueue))
			PktHandler();

		// If we haven't gotten a packet in a while, just fake an update to the LEDs to keep them refreshed
		if (force7221Refresh >= 10)
		{
			force7221Refresh = 0;
			ledPktRcvd = 1;
		}
		
		// If the blink state changed, refresh
		if (blinkState != oldBlinkState)
		{
			oldBlinkState = blinkState;
			ledPktRcvd = 1;
		}
		
		// This stuff only fires if we've received an update that would affect LED status
		// Also can be triggered virtually from the timeout above
		if (ledPktRcvd)
		{
			uint8_t i;
			for (i=0; i<8; i++)
			{
				uint8_t indication = 0;
				if (status & _BV(STATUS_LAMP_TEST))
					indication = lampTest;
				else
				{
					indication = (useSetIndication[i] & setIndication[i]) 
						| ((~(useSetIndication[i])) & 
							(localIndication[i] & (~blinkIndication[i])) | (localIndication[i] & blinkIndication[i] & blinkState));
				}
				
				MAX7221_SendData(i+1, indicationTransmuter(indication));

				if ((status & _BV(STATUS_SEND_ON_CHG)) && oldIndication[i] != indication)
					changed = 1;

				oldIndication[i] = indication;
			}
		}

		if ((changed || (decisecs >= update_decisecs && status & _BV(STATUS_SEND_ON_TIME))) 
			&& !(mrbusPktQueueFull(&mrbusTxQueue)))
		{
			uint8_t i;
			uint8_t txBuffer[MRBUS_BUFFER_SIZE];

			txBuffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
			txBuffer[MRBUS_PKT_DEST] = 0xFF;
			txBuffer[MRBUS_PKT_LEN] = 15;
			txBuffer[5] = 'S';

			for (i=0; i<8; i++)
				txBuffer[6+i] = oldIndication[i];
			
			txBuffer[14] = (uint8_t)busVoltage;


			mrbusPktQueuePush(&mrbusTxQueue, txBuffer, txBuffer[MRBUS_PKT_LEN]);
			ATOMIC_BLOCK(ATOMIC_FORCEON)
			{
				decisecs -= update_decisecs;
				changed = 0;
			}
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
				uint8_t bus_countdown = 20;
				while (bus_countdown-- > 0 && !mrbusIsBusIdle())
				{
					wdt_reset();
					_delay_ms(1);
					if (mrbusPktQueueDepth(&mrbusRxQueue))
						PktHandler();
				}
			}
		}
	}
}



