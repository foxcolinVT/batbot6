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
  delay(1000);
}

void loop() {
  //Tests that the opcode/data combination works.

  //TODO TESTS:
    //Test multiple servo control
    //Test with M4 script
    //Looks like we won't need steppers. Only test them if we are told we will need them

  //DONE TESTS:
    //Test with various delays between opcode and data send
      //Works with delay of 100ms
      //Works with delay fo 1000ms. Probably no need to test beyond this.
    //Test invalid inputs
      //Wrong opcode doesn't allow servo control
    //Test it works for all pins.
      //Issue seemed to stem from use of analog pins. Teensy pins 0-13 can be used for servos.

      
  uint8_t data = 40;
  uint8_t opcode1 = 0b00100010;
  uint8_t opcode2 = 0b00010010;
  for(; data<180; data = data + 50){
     serial.write(opcode1);
     serial.write(data);
     delay(100);
     serial.write(opcode2);
     serial.write(data);  

     Serial.println(opcode1);
     Serial.println(data);
     Serial.println(opcode2);
     Serial.println(data);

     
     delay(100);
  }
}
