/*
 * SXArduino.h
 *
 *  Version:    3.2
 *  Copyright:  Gerard van der Sel (Debug version)
 *
 *  Changed on: 06.12.2017
 *  Version: 	3.2
 *  Changes: 	Pin configurable from outside the class
 *              Split read and write to prepare for SX2
 *
 *  Changed on: 27.12.2015
 *  Version: 	3.1
 *  Changes: 	Added 3 and 4 pin interface.
 *
 *  Changed on: 27.12.2015
 *  Version: 	3.1
 *  Changes: 	Added 3 and 4 pin interface.
 *
 *  Changed on: 19.12.2015
 *  Version: 	3.0
 *  Changes: 	Added some comment. Given its version number.
 *
 *  Changed on: 30.11.2015
 *  Version: 	0.5
 *  Changes: 	Reading and writing to multiple addresses in one cycle, resolved timing issues.
 *
 *  Changed on: 14.11.2015
 *  Version: 	0.4
 *  Changes: 	Reading and writing to multiple addresses in one cycle.
  *
 *  Changed on: 27.10.2015
 *  Version: 	0.3
 *  Changes: 	onWait() added to synchronise with the SXbus.
 *
 *  Changed on: 27.09.2015
 *  Version: 	0.2
 *  Changes: 	Minor changes 
 *
 *  Changed on: 10.07.2015
 *  Version: 	0.1
 *  Changes: 	Initial version
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
// define arduino pins, ports and bits
// depends on the hardware used.
#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

//#if defined (__AVR_ATmega328P__) ||  defined (__AVR_ATmega48P__) || defined (__AVR_ATmega88P__)  || defined (__AVR_ATmega168P__) // default settings for Arduino Nano 3.0 5V 16MHz
//#else
//  #error "--> CPU settings not supported in SX2Arduino.h <--"
//#endif



#define TRI_STATE           3

// defines for state machine
#define DATA	            0              // For performance DATA first
#define SYNC	            1              // (Gives fastest code)
#define PWR                 2
#define ADDR                3

// defines for Selectrix constants
#define SX_STOP             3              // 3 "0" bits achter elkaar
#define SX_DATACOUNT        7              // 7 dataframes in 1 frame
#define SX_SEPLEN           3              // 3 bit in a separated part
#define SX_BYTELEN         12              // 12 bits for one byte

#define SX_ADDRESS_NUMBER 112              // max number SX channels

#define WRITE             256              // Data to write

class SXArduino {
public:
	SXArduino(uint8_t, uint8_t, uint8_t, uint8_t); // 4 wire interface
	SXArduino(uint8_t, uint8_t, uint8_t);		   // 3 wire interface

	bool init(void);	
	int read(uint8_t);
	uint8_t write(uint8_t, uint8_t);
	uint8_t isWritten(uint8_t);
    uint8_t readPWR(void);
	void writePWR(uint8_t);
	void isr(void);
	uint8_t inSync(void);
	
private:
	void initVars();
	uint8_t calcIndex(uint8_t adr);
    uint8_t readT1();
    void writeD(uint8_t val);

	uint8_t _sx_busFlag;                   // Various flags
#define SXBIT               0              //     Value readbit
#define SXWRITING           1              //     Flag signalling writing is in progress
#define SXPWR               2              //     Value powerbit
#define SXSYNC              3              //     Flag signaling all frames are processed
#define SX4LINE             4              //     Flag signaling to use 4 line interface
#define SXPINS              5              //     Flag signaling pins are initialised
	uint8_t _sx_numFrame;                  // Number of frame 
	uint8_t _sx_dataFrameCount;            // frame counting
	uint8_t _sx_T0_state;
	uint8_t _sx_T0_sepCount;               // bit counting (seperator)
	uint8_t _sx_T0_byteCount;              // bit counting (byte)
	uint8_t _sx_T0_index;                  // current index in the array T0
	uint8_t _sx_D_state;
	uint8_t _sx_D_sepCount;                // bit counting (seperator)
	uint8_t _sx_D_byteCount;               // bit counting (byte)
	uint8_t _sx_D_index;                   // current index in the array D
	
	uint8_t _sx_newPWR;                    // command POWER on track

	uint8_t _sx_read_data;                 // read data
    uint8_t _sx_write_data;  	           // data to write

    // Array for storing SX_bus data
	uint16_t _sxbus[SX_ADDRESS_NUMBER];    // SX data
	
    // Addresses and masks for Memorymapped IO 
    // Clock (T0)
//    uint8_t SX_T0_MASK;
//    uint8_t *SX_T0_OUT;
//    uint8_t *SX_T0_IN;
//    uint8_t *SX_T0_DIR;
    // Data in (T1)
    uint8_t SX_T1_MASK;
//    uint8_t *SX_T1_OUT;
    uint8_t *SX_T1_IN;
//    uint8_t *SX_T1_DIR;
    // Data out (D)
    // For the 3 line interface
    uint8_t SX_D_MASK;
    uint8_t *SX_D_OUT;
//    uint8_t *SX_D_IN;
    uint8_t *SX_D_DIR;
    // For the 4 line interface
    uint8_t SX_D_LOW_MASK;
    uint8_t *SX_D_LOW_OUT;
//    uint8_t *SX_D_LOW_IN;
//    uint8_t *SX_D_LOW_DIR;
    uint8_t SX_D_HIGH_MASK;
    uint8_t *SX_D_HIGH_OUT;
//    uint8_t *SX_D_HIGH_IN;
//    uint8_t *SX_D_HIGH_DIR;

	/* SX Timing
	 1   Bit             50 us
	 1   Kanal          600 us (= 12 Bit)
	 1   Grundrahmen    ca. 4,8 ms
	 1   Gesamtrahmen   ca.  80 ms (= 16 Grundrahmen)  
	 0  0  0  1  S   1  A3  A2  1  A1  A0  1 == sync frame of 12 bits
	 */
};

#endif /* SXArduino_H_ */
