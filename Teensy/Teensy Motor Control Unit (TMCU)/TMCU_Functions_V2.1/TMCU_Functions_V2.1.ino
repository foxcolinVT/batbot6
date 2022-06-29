/*
 *  Batbot 6 Teensy Motor Control Unit (TMCU)
 *  Controls Servos and Steppers off of a Teensy 4.0
 *  Download teensy software at https://www.pjrc.com/teensy/td_download.html
 *  
 *  Adapted by Maximilien Engel 6/7/22  
 *  
 *  
 *  Version 2.1 - Tests whether sending just 1 byte fixes issue
 */

#include <Servo.h> 
#include <Stepper.h> 
#define M4SERIAL Serial3                            //Bugs out if the teensy software isn't installed. Tx on 14, Rx on 15

//Masks
const uint8_t SERVO_INDEX_MASK = 0b00000111;        //Mask for isolating index info from opcode
const uint8_t SERVO_POSD_MASK = 0b11111000;         //Mask for isolating desired servo position

//Settings
const uint8_t SERVO_MAX_TRAVEL = 150;               //Sets servo max end of travel (in degrees)
const uint8_t SERVO_MIN_TRAVEL = 40;                //Sets servo min end of travel (in degrees)

//Constants 
const int MAX_SERVOS_PER_TEENSY = 12;               //Maximum number of servos per teensy

//servo declarations
Servo servoList[MAX_SERVOS_PER_TEENSY];             //array of servos

void setup() 
{
  M4SERIAL.begin(9600);                             //Used for communication with M4 
  
  //Servo Initializations
  for(int i = 0; i < MAX_SERVOS_PER_TEENSY; i++){
    servoList[i].attach(i);                         //Starts servo pins at pin 0. Pins 14, 15 used for serial
  }
} 

//Movement of servo
//passes 1 byte to indicate which servo, 1 byte of position data
void moveServo(uint8_t data){
  uint8_t index = data & SERVO_INDEX_MASK;          //Isolate index
  uint8_t posD = map((data & SERVO_POSD_MASK) >> 3, 0, 0b11111, SERVO_MIN_TRAVEL, SERVO_MAX_TRAVEL);  //Isolate desired servo position, map to range of motion
  
  servoList[index].write(posD);                     //Move servo at "index" to position indicated by "data"
}

//Main function, runs in background
void loop() 
{    
    /*
      Opcode documentation found at
      https://docs.google.com/document/d/1cmILsPNfWTc15lZilK4X1TophI3p4mWjxnR8-Ee94GQ/edit
    */
  
  if (M4SERIAL.available() > 0) {                   //Requires 1 byte- opcode and data
    uint8_t data  = M4SERIAL.read();                //Upper 5 bits of this opcode describe which motor to move (for a max of 14 servos). Lower 3 bits reserved for "which servo"
    moveServo(data);
  }
}
