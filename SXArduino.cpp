/*
 * SXAduino.cpp
 *
 *  Version:    0.5
 *  Copyright:  Gerard van der Sel
 *
 *  Changed on: 30.11.2015
 *  Version: 0.5
 *  Changes: Reading and writing to multiple addresses in one cycle, resolved timing issues.
 *
 *  Changed on: 14.11.2015
 *  Version: 0.4
 *  Changes: Reading and writing to multiple addresses in one cycle.
 *
 *  Changed on: 27.10.2015
 *  Version: 0.3
 *  Changes: onWait() added to synchronise with the SXbus.
 *
 *  Changed on: 27.09.2015
 *  Version: 0.2
 *  Changes: Minor changes 
 *
 *  Changed on: 10.07.2015
 *  Version: 0.1
 *  Changes: Initial version
 *
 *  interface hardware needed !

 Interface SX-bus
 - SX T0 (Clock) must be connected to Pin 3 (IOL, INT1);
 - SX T1 must be connected to Pin 5 (IOL, T1);
 - SX D must be connected to Pin 6 (IOL, AIN0). 
 
 SX-bus interface (NEM 682)

De clock lijn (T0) is verbonden met een interruptingang, zodat op
 de flanken van dit signaal een interrupt gegenereerd kan worden.
 Hierna kan data gelezen worden van T1 of data geschreven worden naar D.

 Klok:
  --    ----------------    ----------------    ----------------    ------
    |  |                |  |                |  |                |  |
     --                  --                  --                  -- 

 Data:
  -- ------------------- ------------------- ------------------- ---------
    X                   X                   X                   X
  -- ------------------- ------------------- ------------------- ---------

       ^                   ^                   ^                   ^
       P                   P                   P                   P

Opbouw telegram (96 bits):
  0  0 0  1  S 1 A3 A2 1 A1 A0 1     synchronisatie 'byte'
 D0 D1 1 D2 D3 1 D4 D5 1 D6 D7 1
 D0 D1 1 D2 D3 1 D4 D5 1 D6 D7 1
 D0 D1 1 D2 D3 1 D4 D5 1 D6 D7 1         7 data 'bytes'
 D0 D1 1 D2 D3 1 D4 D5 1 D6 D7 1   ieder 'byte' is de inhoud
 D0 D1 1 D2 D3 1 D4 D5 1 D6 D7 1         van een adres
 D0 D1 1 D2 D3 1 D4 D5 1 D6 D7 1
 D0 D1 1 D2 D3 1 D4 D5 1 D6 D7 1

 0 = Logische 0
 1 = Logische 1
 S = Spanning rails (0 = uit, 1= aan)
 Ax = Gezamelijk het nummer van het telegram
 Dx = D0 t/m D7 vormen de data op een Selectrix adres.

 Verdeling adressen over de verschillende telegrammen:
 telegram  '0' : 111, 95, 79, 63, 47, 31, 15
 telegram  '1' : 110, 94, 78, 62, 46, 30, 14
 telegram  '2' : 109, 93, 77, 61, 45, 29, 13
 telegram  '3' : 108, 92, 76, 60, 44, 28, 12
 telegram  '4' : 107, 91, 75, 59, 43, 27, 11
 telegram  '5' : 106, 90, 74, 58, 42, 26, 10
 telegram  '6' : 105, 89, 73, 57, 41, 25,  9
 telegram  '7' : 104, 88, 72, 56, 40, 24,  8
 telegram  '8' : 103, 87, 71, 55, 39, 23,  7
 telegram  '9' : 102, 86, 70, 54, 38, 22,  6
 telegram '10' : 101, 85, 69, 53, 37, 21,  5
 telegram '11' : 100, 84, 68, 52, 36, 20,  4
 telegram '12' :  99, 83, 67, 51, 35, 19,  3
 telegram '13' :  98, 82, 66, 50, 34, 18,  2
 telegram '14' :  97, 81, 65, 49, 33, 17,  1
 telegram '15' :  96, 80, 64, 48, 32, 16,  0

This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.
 
 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.
 
 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

 */


#include <Arduino.h> 

#include "SXArduino.h"


SXArduino::SXArduino() {
    pinMode(SX_T0, INPUT);      // SX-T0 is an input, no pull up to simulate tri-state
    pinMode(SX_T1, INPUT);      // SX-T1 is also an input
    pinMode(SX_D, INPUT);       // SX-D is also an input when not writing to allow other devices to write
}

void SXArduino::init() {
     // initialize function
     // initialize pins and variables

	pinMode(SX_T0, INPUT);      // SX-T0 is an input, no pull up to simulate tri-state
	pinMode(SX_T1, INPUT);      // SX-T1 is also an input
	pinMode(SX_D, INPUT);       // SX-D is also an input when not writing to allow other devices to write

	for (int i = 0; i < SX_ADDRESS_NUMBER; i++) {
		_sxbusrcev[i] = 0;                 // reset sx received data to zero
		_sxbussnd[i] = NO_WRITE;           // set sx data to send to NO_WRITE
	}
	initVar();
}

void SXArduino:: initVar() {
	// start always with search for header
	_sx_state = SYNC;
	_sx_dataFrameCount = SX_DATACOUNT;
	_sx_sepCount = SX_SEPLEN;
	_sx_byteCount = SX_STOP;
	_sx_numFrame = 0;
	_sx_index = 0;
	
	// no writing to the SX bus
	_sx_writing = 0;

	// Powerbit send and receive
	_sx_newPWRBit = 2;
	_sx_PWRBit = 0;                   // At start no power
	
	// reset sync bit
	_sx_sync = 0;
}

// interrupt service routine (AVR INT1)
// driven by RISING EDGES of the clock signal T0 (SX pin 1)
void SXArduino::isr() {
	_sx_bit = (SX_T1_PINREG & _BV(SX_T1_PORTPIN)) > 0;   // read pin

	switch (_sx_state) {
		// Find sync pattern "0001" to start reading and writing
		case SYNC:
			if (_sx_bit == LOW) {               // Sync bits "0"
				if (_sx_byteCount > 0) {
					_sx_byteCount--;
				}
			} else {
				if (_sx_byteCount == 0) {
					_sx_state = PWR;
					_sx_sepCount = SX_SEPLEN - 1;    // Set _sx_sepCount and continue
				}
				_sx_byteCount = SX_STOP;             // Set counter to restart
			}
			break;
		// Read (and write) the power bit.
		case PWR:
			_sx_sepCount--;
			if (_sx_sepCount == 0) {
				bitWrite(SX_D_DDR, SX_D_PORTPIN, LOW);			// Switch pin to input
				_sx_state = ADDR;
				_sx_numFrame = 0;
				_sx_byteCount = SX_BYTELEN / 2;
				_sx_sepCount = SX_SEPLEN;
			} else {
				if (_sx_newPWRBit != 2) {
					bitWrite(SX_D_DDR, SX_D_PORTPIN, HIGH);			// Switch pin to output
					bitWrite(SX_D_PORT, SX_D_PORTPIN, _sx_newPWRBit);	//  and write the newPWRBit
					_sx_newPWRBit = 2;
				}   // end if _sx_newPWRBit 
				_sx_PWRBit = _sx_bit;
			}				
			break;
		// Read the address bits.
		case ADDR:  
			_sx_sepCount--;
			if (_sx_sepCount == 0) {
				_sx_sepCount = SX_SEPLEN;
			} else {
				_sx_numFrame = (_sx_numFrame * 2) + _sx_bit;
			}
			_sx_byteCount--;
			if (_sx_byteCount == 0) {
				// Advance to the next state
				_sx_state = DATA;
				_sx_byteCount = SX_BYTELEN;
				// Frame-number is ready, calculate the index
				_sx_index =  _sx_numFrame * 7;   
				if (_sxbussnd[_sx_index] < NO_WRITE) {
					_sx_write_data = _sxbussnd[_sx_index];
					_sx_writing = 1;
					_sxbussnd[_sx_index] |= NO_WRITE;
				} else {
					_sx_writing = 0;
				}
				// Signal frame "0"
				if (_sx_numFrame == 0) {
					_sx_sync = 1;
				}
			}
			break;
		// Read (and write) the data bits
		case DATA: 
			_sx_sepCount--;
			if (_sx_sepCount == 0) {
				bitWrite(SX_D_DDR, SX_D_PORTPIN, LOW);      // Switch pin to input
				_sx_sepCount = SX_SEPLEN;
			} else {
				if (_sx_writing == 1)  {
					bitWrite(SX_D_DDR, SX_D_PORTPIN, HIGH);		// Switch pin to output
					bitWrite(SX_D_PORT, SX_D_PORTPIN, bitRead(_sx_write_data, 0));
					_sx_write_data = _sx_write_data / 2;
				}
				_sx_read_data = (_sx_read_data / 2);
				bitWrite(_sx_read_data, 7, _sx_bit);
			}
			_sx_byteCount--;
			if (_sx_byteCount == 0) {
				// save read _data
				_sxbusrcev[_sx_index] = _sx_read_data;
				_sx_read_data = 0;
				// Setup for next read/write
				_sx_byteCount = SX_BYTELEN;
				_sx_index++;
				// Decrement dataFrameCount
				// check, if we already reached the last DATA block - in this
				// case move on to the next SX-Datenpaket, i.e. look for SYNC
				_sx_dataFrameCount--;
				if (_sx_dataFrameCount == 0) {
					// Move on to find SYNC pattern
					_sx_dataFrameCount = SX_DATACOUNT;
					_sx_state = SYNC;
					_sx_byteCount = SX_STOP;
					_sx_writing = 0;
				} else {
					// Check if we want to write
					if (_sxbussnd[_sx_index] < NO_WRITE) {
						_sx_write_data = _sxbussnd[_sx_index];
						_sx_writing = 1;
						_sxbussnd[_sx_index] |= NO_WRITE;
					} else {
						_sx_writing = 0;
					}
				}
			}
			break;
		default:
			bitWrite(SX_D_DDR, SX_D_PORTPIN, LOW);			// Switch pin to input
			initVar();
			break;
	}  //end switch/case _sx_state
}


// Convert from SX-bus addresses to index in array.
uint8_t SXArduino::calcIndex(int adr) {
	uint8_t frame = 15 - (adr & 15);    // Get the frame number
	uint8_t offset = 6 - (adr >> 4);    // Get the offset in the frame
	return frame * 7 + offset;          // Calculate the index in the array
}

// functions 'accessing' the SX-bus

// Read data from the array, filled by the isr.
uint8_t SXArduino::get(uint8_t adr) {
     // returns the value of a SX address
	if (adr < SX_ADDRESS_NUMBER)  {
	   return _sxbusrcev[calcIndex(adr)];
	} else  {
	   return 0;
	}
}

// Write data to the array, copying to the SX-bus is done by the isr.
// Check if invalid address.
uint8_t SXArduino::set(uint8_t adr, uint8_t dt) {
	if (adr < SX_ADDRESS_NUMBER)  {
		_sxbussnd[calcIndex(adr)] = dt;
		return 0;    // success
	}
	return 1;    // address out of range
}

// Checks if the isr has written the data to the SX-bus
uint8_t SXArduino::isSet(uint8_t adr) {
	if (adr < SX_ADDRESS_NUMBER)  {
		if (_sxbussnd[calcIndex(adr)] < NO_WRITE) {
			return 1;   // Not written yet
		} else {
			return 0;   // Data written
		}
	}
	return 2;    // address out of range
}

// Read POWER status from the SX-bus
uint8_t SXArduino::getPWR() {
    return _sx_PWRBit;
}

// Write POWER status to the SX-bus and control a connected central.
void SXArduino::setPWR(uint8_t val) {
	if (val == 0 || val == 1) {
		_sx_newPWRBit = val;
	}
}

// Every time frame 0 is passed sync bit is set.
uint8_t SXArduino::inSync() {
	if (_sx_sync == 1) {
		_sx_sync = 0;              // reset sync bit to check for next pass
		return 1;                  // report frame 0 found
	}
	return 0;
}
