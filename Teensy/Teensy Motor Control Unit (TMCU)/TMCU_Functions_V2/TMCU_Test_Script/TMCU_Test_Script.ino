/*
TMCU Test Script
Max Engel 6/22/22

Script for a second arduino board to communicate to the teensy. 
Simulates the connection between the teensy and the M4 or (potentially) the Jetson

*/

#include <SoftwareSerial.h>

const int RX_PIN = 10;
const int TX_PIN = 11;
SoftwareSerial serial (RX_PIN, TX_PIN);

void setup() {
  serial.begin(9600);
  Serial.begin(9600);
}

void loop() {
  //Tests that the opcode/data combination works.
  //Test works for pins 2-12. Issues encountered when tested on pins 13, 14  
  uint8_t data = 0;
  uint8_t opcode = 0xC2;
  for(; data<180; data = data + 3){
     serial.write(opcode);
     serial.write(data);

     Serial.write(opcode);
     Serial.write(data);
     
     delay(500);
  }
}
