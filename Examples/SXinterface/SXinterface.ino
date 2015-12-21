/*
   SX interface (66824 or 66842)

   Creator: Gerard van der Sel
   Version: 1.0
   Date: 19-12-2015
   
   This software emulates a SX interface on the Arduino.
   Extra hardware needed is de SX-Arduino board which contains the hardware to communicate with the SX-bus.

   Protocol used by the SX-Interface (66824 or 66842)
   2 byte message:
   First byte containing command and address:
      Command is located in bit 7 of first byte:
      0: Read address
      1: Write address
      Address: 0 to 111 (valid adresses on the SX-bus)
               127 controls track power
   Second byte containing data (range 0 to 255)
      in case of address 127: 0 trackpower off
                              128 trackpower on
      in case of a read: value is random and discarded

   Note: For this sketch to run, don't solder switch and LED on de SXArduino board
*/

#include "Arduino.h"
#include "SXArduino.h"

SXArduino SXbus;         // Interface to the SX-bus
boolean cmdAdr;          // Received byte is a command
boolean cmdWrite;        // Write command
uint8_t address;         // Address for reading or writing
uint8_t cmdRcvd;
uint8_t dtRcvd;

void sxisr(void) {
    // if you want to understand this, see:
    // http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1239522239   
    SXbus.isr();
} 


void setup() {
  // put your setup code here, to run once:
    // initialize serial:
    Serial.begin(19200);

    // initialize SX-bus
    SXbus.init();
    // Rising edge on INT1 triggers the interrupt routine sxisr (see above)
    attachInterrupt(1, sxisr, RISING); 

    // initialize application
    cmdAdr = true;
    cmdWrite = false;
}

void serialEvent() {
    // Read all the data
    while (Serial.available()) {
        // First byte is a command, decode it
            cmdRcvd = (uint8_t)Serial.read();
            // If byte value > 127 a write command, data in second byte
            if (cmdRcvd > 127) {
                cmdWrite = true;
                address = cmdRcvd - 128;  // address is command - 128
            } else {  // Read command, perform it
                if (cmdRcvd < 112) {    // Get address data
                    Serial.print((char)SXbus.get(cmdRcvd));
                } else {   // Illegal address, power?
                    if (cmdRcvd == 127) {
                        Serial.print((char)(SXbus.getPWR() * 128));
                    }
                }
            }
            cmdAdr = false;
        } else {
            // Second byte data
            dtRcvd = (uint8_t)Serial.read();
            if (cmdWrite) {
                if (address < 112) {
                    SXbus.set(address, dtRcvd);
                } else {
                    if (address == 127) {
                        if ((dtRcvd & 128) == 0) {
                            SXbus.setPWR(0);
                        } else {
                            SXbus.setPWR(1);
                        }
                    }
                }
                cmdWrite = false;
            }
            cmdAdr = true;
        }
    }
}

void loop() {
  // put your main code here, to run repeatedly:
  // Nothing to do.
}
