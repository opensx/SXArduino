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

	for (int i = 0; i < MAX_ADDRESS_NUMBER; i++) {
		_sxbusrcev[i] = 0;                 // reset sx received data to zero
		_sxbussnd[i] = NO_WRITE;           // set sx data to send to NO_WRITE
	}

	// start always with search for header
	_sx_state = HEADER;
	_sx_dataFrameCount = 0;
	_sx_bitCount = 0;
	_sx_numFrame = 0;
	
	// no writing to the SX bus
	_sx_writing = 0;

	// Powerbit send and receive
	_sx_newPWRBit = 2;
	_sx_PWRBit = 0;                   // At start no power
	
	// reset sync bit
	_sx_sync = 0;
}

// Read the bit that indicates if there is power on the track and store it.
// After reading the bit start processing the address.
void SXArduino::processHeader()  {
	switch (_sx_bitCount) {
// Fimd sync pattern "0001" to start reading and writing
	case 0:                              // Sync bits "0"
	case 1:
	case 2:
		if (_sx_bit == LOW) {
			_sx_bitCount++;
		} else {
			_sx_bitCount = 0;            // reset _sx_bitCounter and start over if high
		}  
	case 3:
		if (_sx_bit == HIGH) {
			_sx_bitCount++;              //  ...0001: Sync part found continue
		}  
		break;
// Read and write the power bit.
		case 4:
		if (_sx_newPWRBit != 2) {
			bitWrite(SX_D_DDR, SX_D_PORTPIN, HIGH);			// Switch pin to output
			bitWrite(SX_D_PORT, SX_D_PORTPIN, _sx_newPWRBit);	//  and write the newPWRBit
			_sx_newPWRBit = 2;
		}   // end if _sx_newPWRBit 
		_sx_PWRBit = (SX_T1_PINREG & _BV(SX_T1_PORTPIN)) > 0;
		_sx_bitCount++;
		break;
	case 5:  // "Trenbit"
		bitWrite(SX_D_DDR, SX_D_PORTPIN, LOW);			// Switch pin to input
		_sx_bitCount++;
		break;
// Read the address bits.
	case 6:  // A3
		bitWrite(_sx_numFrame, 3, _sx_bit);
		_sx_bitCount++;
		break;
	case 7:  // A2
		bitWrite(_sx_numFrame, 2, _sx_bit);
		_sx_bitCount++;
		break;
	case 8:  // "Trenbit"
		_sx_bitCount++;
		break;
	case 9:  // A1
		bitWrite(_sx_numFrame, 1, _sx_bit);
		_sx_bitCount++;
		break;
	case 10:  // A0
		bitWrite(_sx_numFrame, 0, _sx_bit);
		_sx_bitCount++;
		break;
	case 11: // last "Trenbit"
		// advance to DATA state 
		// we are processing the 7 data bytes (i.e. 7 SX addresses)
		_sx_state = DATA;  
		_sx_bitCount = 0;
// Frame-number is read, calculate the real address and check if we must write 
		//  _dataFrameCount is 0.
		_sx_address = (MAX_ADDRESS_NUMBER - 1) - _sx_numFrame;
		if (_sxbussnd[_sx_address] < NO_WRITE) {
			_sx_write_data = _sxbussnd[_sx_address];
			_sx_writing = 1;
			_sxbussnd[_sx_address] |= NO_WRITE;
		} else {
			_sx_writing = 0;
		}
		// Signal frame "0"
		if (_sx_numFrame == 0) {
			_sx_sync = 1;
		}
		break;
	default:
		bitWrite(SX_D_DDR, SX_D_PORTPIN, LOW);			// Switch pin to input
		_sx_state = HEADER;     // Error restart
		_sx_dataFrameCount = 0;
		_sx_bitCount = 0;
		_sx_numFrame = 0;
		_sx_writing = 0;
		_sx_newPWRBit = 2;
		_sx_PWRBit = 0;
		break;
	}  //end switch/case _sx_bitCount
}

// read 12 bits and convert it to a databyte.
// Read 7 databytes to read all data
// After reading 7 data bytes start searching for HEADER
void SXArduino::processData()  {
	// continue read (and write) data
	// a total of 7 data blocks will be received
    // for a certain frame number
	switch(_sx_bitCount) {
	// Read (and write) the data bits
	case 2:  // "Trenn_bits"
	case 5:
	case 8:
		bitWrite(SX_D_DDR, SX_D_PORTPIN, LOW);      // Switch pin to input
		_sx_bitCount++;
		break; // ignore
	case 0:   // D0
		if (_sx_writing == 1)  {
			bitWrite(SX_D_DDR, SX_D_PORTPIN, HIGH);		// Switch pin to output
			bitWrite(SX_D_PORT, SX_D_PORTPIN, bitRead(_sx_write_data, 0));
		}
		bitWrite(_sx_read_data, 0, _sx_bit);
		_sx_bitCount++;
		break;
	case 1:   // D1
		if (_sx_writing == 1)  {
			bitWrite(SX_D_PORT, SX_D_PORTPIN, bitRead(_sx_write_data, 1));
		}
		bitWrite(_sx_read_data, 1, _sx_bit);
		_sx_bitCount++;
		break;
	case 3:   // D2
		if (_sx_writing == 1)  {
			bitWrite(SX_D_DDR, SX_D_PORTPIN, HIGH);		// Switch pin to output
			bitWrite(SX_D_PORT, SX_D_PORTPIN, bitRead(_sx_write_data, 2));
		}
		bitWrite(_sx_read_data, 2, _sx_bit);
		_sx_bitCount++;
		break;
	case 4:   // D3
		if (_sx_writing == 1)  {
			bitWrite(SX_D_PORT, SX_D_PORTPIN, bitRead(_sx_write_data, 3));
		}
		bitWrite(_sx_read_data, 3, _sx_bit);
		_sx_bitCount++;
		break;
	case 6:   // D4
		if (_sx_writing == 1)  {
			bitWrite(SX_D_DDR, SX_D_PORTPIN, HIGH);		// Switch pin to output
			bitWrite(SX_D_PORT, SX_D_PORTPIN, bitRead(_sx_write_data, 4));
		}
		bitWrite(_sx_read_data, 4, _sx_bit);
		_sx_bitCount++;
		break;
	case 7:   // D5
		if (_sx_writing == 1)  {
			bitWrite(SX_D_PORT, SX_D_PORTPIN, bitRead(_sx_write_data, 5));
		}
		bitWrite(_sx_read_data, 5, _sx_bit);
		_sx_bitCount++;
		break;
	case 9:   // D6
		if (_sx_writing == 1)  {
			bitWrite(SX_D_DDR, SX_D_PORTPIN, HIGH);		// Switch pin to output
			bitWrite(SX_D_PORT, SX_D_PORTPIN, bitRead(_sx_write_data, 6));
		}
		bitWrite(_sx_read_data, 6, _sx_bit);
		_sx_bitCount++;
		break;
	case 10:  // D7
		if (_sx_writing == 1)  {
			bitWrite(SX_D_PORT, SX_D_PORTPIN, bitRead(_sx_write_data, 7));
		}
		bitWrite(_sx_read_data, 7, _sx_bit);
		_sx_bitCount++;
		break;
	case 11: // Last "Trenn_bit"
		bitWrite(SX_D_DDR, SX_D_PORTPIN, LOW);      // Switch pin to input
		_sx_bitCount = 0;
		// save read _data
		_sxbusrcev[_sx_address] = _sx_read_data;
        // increment dataFrameCount to move on the next DATA byte
        // check, if we already reached the last DATA block - in this
        // case move on to the next SX-Datenpaket, i.e. look for SYNC
		_sx_dataFrameCount++;
		if (_sx_dataFrameCount < MAX_DATACOUNT) {
			// setup for next read/write
			// calc sx address from numFrame and dataFrameCount
			// _sx_address = (6-_sx_dataFrameCount) * 16 + (15 - _sx_numFrame)
			_sx_address = ((6 - _sx_dataFrameCount) << 4) + (15 - _sx_numFrame);
			// check if we want to write
			if (_sxbussnd[_sx_address] < NO_WRITE) {
				_sx_write_data = _sxbussnd[_sx_address];
				_sx_writing = 1;
				_sxbussnd[_sx_address] |= NO_WRITE;
			} else {
				_sx_writing = 0;
			}
		} else {
			// or move on to HEADER state
			_sx_dataFrameCount = 0;
			_sx_state = HEADER;
			_sx_writing = 0;
		}
		break;
	default:
		bitWrite(SX_D_DDR, SX_D_PORTPIN, LOW);			// Switch pin to input
		_sx_state = HEADER;     // Error restart
		_sx_dataFrameCount = 0;
		_sx_bitCount = 0;
		_sx_numFrame = 0;
		_sx_writing = 0;
		_sx_newPWRBit = 2;
		_sx_PWRBit = 0;
		break;
	}  //end switch/case _sx_bitCount
}

// interrupt service routine (AVR INT1)
// driven by LEVEL CHANGES of the clock signal T0 (SX pin 1)
// LOW: writing
// HIGH: reading and control
void SXArduino::isr() {
	_sx_bit = (SX_T1_PINREG & _BV(SX_T1_PORTPIN)) > 0;   // read pin

     // 2 different states are distinguished
     //     1. HEADER = look for sync bits, pwr and framenumber
     //     2. DATA = read (and write) the data bytes (7 in total)
	switch(_sx_state) {
	case HEADER:                                        // Search for 0 0 0 1 and read header
		processHeader();
		break;
	case DATA:				 							// Read all data
		processData();
		break;
	default:
		bitWrite(SX_D_DDR, SX_D_PORTPIN, LOW);			// Switch pin to input
		_sx_state = HEADER;     // Error restart
		_sx_dataFrameCount = 0;
		_sx_bitCount = 0;
		_sx_numFrame = 0;
		_sx_writing = 0;
		_sx_newPWRBit = 2;
		_sx_PWRBit = 0;
		break;
	}  //end switch/case _sx_state
}

// functions 'accessing' the SX-bus

// Read data from the array, filled by the isr.
uint8_t SXArduino::get(uint8_t adr) {
     // returns the value of a SX address
	if (adr < MAX_ADDRESS_NUMBER)  {
	   return _sxbusrcev[adr];
	} else  {
	   return 0;
	}
}

// Write data to the array, copying to the SX-bus is done by the isr.
// Check if invalid address.
uint8_t SXArduino::set(uint8_t adr, uint8_t dt) {
	if (adr < MAX_ADDRESS_NUMBER)  {
		_sxbussnd[adr] = dt;
		return 0;    // success
	}
	return 1;    // address out of range
}

// Checks if the isr has written the data to the SX-bus
uint8_t SXArduino::isSet(uint8_t adr) {
	if (adr < MAX_ADDRESS_NUMBER)  {
		if (_sxbussnd[adr] < NO_WRITE) {
			return 1;   // Not written yet
		} else {
			return 0;   // Data written
		}
	}
	return 2;    // address out of range
}

// Read POWER status from the SX-bus
uint8_t SXArduino::getPWRBit() {
    return _sx_PWRBit;
}

// Write POWER status to the SX-bus and control a connected central.
void SXArduino::setPWRBit(uint8_t val) {
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
