/*
 *  Batbot 6 Teensy Motor Control Unit (TMCU)
 *  Controls Servos and Steppers off of a Teensy 4.0
 *  Download teensy software at https://www.pjrc.com/teensy/td_download.html
 *  
 *  Adapted by Maximilien Engel 6/7/22  
 *  
 *  If left, change SERVO_OPCODE to 0x02. If right, change SERVO_OPCODE to 0x05
 */

#include <Servo.h> 
#include <Stepper.h> 
#define M4SERIAL Serial3                            //Bugs out if the teensy software isn't installed. Tx on 14, Rx on 15

//Masks
const uint8_t OPCODE_MASK = 0b00001111;             //Mask for isolating opcode info from opcode
const uint8_t SERVO_INDEX_MASK = 0b11110000;        //Mask for isolating index info from opcode

//Settings
const uint8_t SERVO_OPCODE = 0x05;                  //Decides whether this handles right or left teensy. 0x02 = left, 0x05 = right.
const uint8_t SERVO_MAX_TRAVEL = 150;               //Sets servo max end of travel (in degrees)
const uint8_t SERVO_MIN_TRAVEL = 30;                //Sets servo min end of travel (in degrees)

//Constants 
const int MAX_SERVOS_PER_TEENSY = 12;               //Maximum number of servos per teensy

//servo declarations
Servo servoList[MAX_SERVOS_PER_TEENSY];             //array of servos

void setup() 
{
  M4SERIAL.begin(9600);                             //Used for communication with M4. Tx on 14, Rx on 15
  Serial.begin(9600);
  
  //Servo Initializations
  for(int i = 0; i < MAX_SERVOS_PER_TEENSY; i++){
    servoList[i].attach(i);                         //Starts servo pins at pin 0. Pins 14, 15 used for serial
  }
} 

//Movement of servo
//passes 1 byte to indicate which servo, 1 byte of position data
void moveServo(uint8_t opcode, uint8_t data){
  uint8_t index = (uint8_t)((opcode & SERVO_INDEX_MASK) >> 4);                    //isolate index
  if(data < SERVO_MAX_TRAVEL && data > SERVO_MIN_TRAVEL){                         //Check the desired position is possible
    servoList[index].write((uint8_t)data);                     //Move servo at "index" to position indicated by "data"
  } 
}

//Main function, runs in background
void loop() 
{    
    /*
      Opcode documentation found at
      https://docs.google.com/document/d/1cmILsPNfWTc15lZilK4X1TophI3p4mWjxnR8-Ee94GQ/edit
    */
  
  if (M4SERIAL.available() > 0) {             //Requires at least 1 byte
    //Upper 4 bits of this opcode describe which motor to move (for a max of 14 servos). Lower 4 bits reserved for instructions
    uint8_t opcode = M4SERIAL.peek();         //Normal servo operation requires 2 bytes. Some other instructions require 1. Peeks in case we need to run servo op.
    
    if((opcode & OPCODE_MASK) == SERVO_OPCODE && M4SERIAL.available() > 1){ //Only runs if there's another byte waiting for it
      M4SERIAL.read();                        //Clears the opcode byte from the serial buffer
      uint8_t data = M4SERIAL.read();         //Only retrieve next byte if opcode is valid
      moveServo(opcode, data);
    }
    
    else if((opcode & OPCODE_MASK) == 0x03){
      //Move Stepper. Legacy opcode if we don't end up using steppers. If we need the stepper code again, find it on github before changes pushed on 6/23/22
    }
    else if((opcode & OPCODE_MASK) == 0x04){
      //Send sensory data to m4
      //Legacy opcode. No sensory data to get as far as I know.
    }
    //0x05 reserved for "SERVO_OPCODE"
    else if((opcode & OPCODE_MASK) == 0x06){
      for(int i = 0; i < 12; i++){
        servoList[i].write(SERVO_MAX_TRAVEL);                     //Move servos to max position
      }
    }
    else if((opcode & OPCODE_MASK) == 0x07){
      for(int i = 0; i < 12; i++){
        servoList[i].write(SERVO_MIN_TRAVEL);                     //Move servos to min position
      }
    }
  }
}
