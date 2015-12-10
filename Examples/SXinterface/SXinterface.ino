/*
  SX interface (66842)

   This software emulates a SX interface on the ARduino.
   Extra hardware needed is de SX-Arduino board which contains the hardware to communicate with the SX-bus.

   Protocol used by the SX-Interface (66842)
   2 byte message:
   First byte containing command and address:
      Command is located in bit 7 of first byte:
      0: Read address
      1: Write address
      Address: 0 to 111 (valid adresses on the SX-bus)
               127 controls track power
   Second byte containing data (range 0 to 255)
      in case of address 127: 0 trackpower off
                              1 trackpower on
      in case of a read value is random and discarded

   Note: For this sketch to run, don't mount switch and LED on de SXArduino board
*/
#include <SXArduino.h>

SXArduino SXbus;         // Interface to the SX-bus
boolean command;         // Received byte is a command
boolean cmdWrite;        // Write command
uint_8 address;             // Address for reading or writing

void sxisr(void) {
    // if you want to understand this, see:
    // http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1239522239   
    SXbus.isr();
} 


void setup() {
    // initialize serial:
    Serial.begin(19200);

    // initialize SX-bus
    SXbus.init();
    // CHANGE on INT1 triggers the interrupt routine sxisr (see above)
    attachInterrupt(1, sxisr, CHANGE); 

    // initialize application
    command = true;
    cmdWrite = false;
}

void serialEvent() {
  while (Serial.available()) {
    // see if there is some data in input buffer
    if (Serial.available() > 0) {
        if (command) {
            // first byte a command
            uint_8 cmdRcvd = uint_8(Serial.read());
            // If byte value > 127 a write command, data in second byte
            if (cmdRcvd > 127) {
                cmdWrite = true;
                address = cmdRcvd - 128;  // address is command - 128
            } else {  // Read command, perform it
                if (cmdRcvd < 112) {    // Get address data
                    Serial.print(SXbus.get(cmdRcvd));
                } else {   // Illegal address, power?
                    if (cmdRcvd == 127) {
                        Serial.print(SXbus.getPWR());
                    }
                }
            }
            command = false;
        } else {
            // second byte data
            uint_8 dtRcvd = int(Serial.read());
            if (cmdWrite) {
                if (address < 112) {
                    SXbus.set(address, dtRcvd);
                } else {
                    if (address == 127) {
                        if ((dtRcvd & 1) == 0) {
                            SXbus.setPWR(0);
                        } else {
                            SXbus.setPWR(1);
                        }
                    }
                }
                cmdWrite = false;
            }
            command = true;
        }
    }
}

void Loop() {
	// Nothing to do.
}
  

