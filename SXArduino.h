/*
 * SXArduino.h
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
 *  interface hardware needed ! see 

 Read SX Signal - SX Clock must be connected to Pin3 = INT1 and
 SX Data must be connected to Pin 5. Both are connected through a resistor off 22 kilo ohm.
 Pin 6 can be connected via a 100 ohm resistor to the write line 
 of the SX bus
 
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

#ifndef SXArduino_H_
#define SXArduino_H_

#include <Arduino.h>

// define arduino pins, ports and bits
// depends on the hardware used.
#define SX_T0			3
#define SX_T0_DDR		DDRD
#define SX_T0_PORT		PORTD
#define SX_T0_PORTPIN 	PORTD3
#define SX_T0_PINREG	PIND

#define SX_T1			5
#define SX_T1_DDR		DDRD
#define SX_T1_PORT		PORTD
#define SX_T1_PORTPIN 	PORTD5
#define SX_T1_PINREG	PIND

#define SX_D			6
#define SX_D_DDR		DDRD
#define SX_D_PORT		PORTD
#define SX_D_PORTPIN	PORTD6
#define SX_D_PINREG		PIND


// defines for state machine
#define HEADER	0
#define DATA	1


// defines for Selectrix constants
#define MAX_DATACOUNT    7    // 7 dataframes in 1 SYNC Channel
#define MAX_DATABITCOUNT 12   // 12 bits in 1 frame

#define MAX_ADDRESS_NUMBER 112   // SX channels

#define NO_WRITE 256   // Na data to write

class SXArduino {
public:
	SXArduino();
	void init(void);	
	uint8_t get(uint8_t);
	uint8_t set(uint8_t, uint8_t);
	uint8_t isSet(uint8_t);
    uint8_t getPWRBit(void);
	void setPWRBit(uint8_t);
	void isr(void);
	uint8_t inSync(void);
	
private:
	void processHeader(void);
	void processData(void);

	uint8_t _sx_numFrame;                 // number frame
	uint8_t _sx_dataFrameCount;           // frame counting
	uint8_t _sx_address;                  // current address on the bus
    uint8_t _sx_state;
	uint8_t _sx_bitCount;                 // bit counting
	
	uint8_t _sx_PWRBit;                   // current state of POWER on track
	uint8_t _sx_newPWRBit;                // command POWER on track

	uint8_t _sx_read_data;                // read data
    uint8_t _sx_write_data;  			  // data to write
	uint8_t _sx_writing;				  // active during the actual writing
	
	uint8_t _sx_bit;                      // value data bit (T1)
	uint8_t _sx_sync;                     // set if Frame 0 is processed
	
	uint8_t _sxbusrcev[MAX_ADDRESS_NUMBER];   // to store the received SX data
	uint16_t _sxbussnd[MAX_ADDRESS_NUMBER];   // to store the SX data to send

	/* SX Timing
	 1   Bit             50 us
	 1   Kanal          600 us (= 12 Bit)
	 1   Grundrahmen    ca. 4,8 ms
	 1   Gesamtrahmen   ca.  80 ms (= 16 Grundrahmen)  
	 0  0  0  1  S   1  A3  A2  1  A1  A0  1 == sync frame of 12 bits
	 */
};

#endif /* SXArduino_H_ */
