/*
    Batbot 6 Teensy Motor Control Unit (TMCU)
    Controls Servos and Steppers off of a Teensy 4.0
    Download teensy software at https://www.pjrc.com/teensy/td_download.html

    Adapted by Maximilien Engel 6/24/22

    This script does not work.
    It was intended to demonstrate "switching" servos from active to inactive.
    Unfortunately, servos do not appear to hold their position after being switched to inactive. 
*/

#include <Servo.h>
#include <Stepper.h>
#define M4SERIAL Serial1                            //Bugs out if the teensy software isn't installed

//Masks
const uint8_t OPCODE_MASK = 0b00001111;             //Mask for isolating opcode info from opcode
const uint8_t SERVO_INDEX_MASK = 0b11110000;        //Mask for isolating index info from opcode
const uint8_t STEPPER_LR_MASK = 0b10000000;         //Mask for isolating l/r stepper motor info from opcode
const uint8_t STEPPER_DIRECTION_MASK = 0b01000000;  //Mask for isolating l/r motor direction info from opcode

//Constants
const int SERVO_MAX  = 180;                         //Servo max range of motion
const int NUM_SERVOS = 15;                          //Number of servos implemented (on pins 0 to NUM_SERVOS)
const int STEPS = 200;                              //Stepper number of steps
const int STEPPER_SPEED  = 120;
const int SERVO_MAX_TRAVEL = 180;                   //Sets servo end of travel
const int SERVO_MIN_TRAVEL = 10;                    //Sets servo end of travel


//servo declarations
Servo servoList[NUM_SERVOS];                        //array of servos

void setup()
{
  Serial.begin(9600);                                 //Used for debugging
  M4SERIAL.begin(9600);                               //Used for communication with M4

  //Servo Initializations
  for (int i = 0; i < NUM_SERVOS; i++) {
    servoList[i].attach(i + 2);                       //Starts servo pins at pin 2. Pins 0, 1 used for serial

    /*Serial.print(i); //debug
      Serial.print(": ");
      Serial.println(servoList[i].attached());*/
  }
}

bool activateServo(uint8_t servo) {
  for (int i = 0; i < NUM_SERVOS; i++) {
    if (servoList[i].attached()) {                    //detach exactly 1 servo, starting with the lowest attached servo
      servoList[i].detach();
      break;
    }
  }

  servoList[servo].attach(servo + 2);                 //attach servo to its corresponding pin
}

//Movement of servo
//Passes servo by reference, passes 1 byte of position data
void moveServo(uint8_t opcode, uint8_t data) {
  uint8_t index = (uint8_t)((opcode & SERVO_INDEX_MASK) >> 4);//isolate index

  //Che ck servo is attached. Make it attached
  if (!servoList[index].attached()) {
    activateServo(index);
  }

  //Move servo if movement is within defined bounds
  if (data < SERVO_MAX_TRAVEL && data > SERVO_MIN_TRAVEL) {
    servoList[index].write((uint8_t)data);                  //Move servo at "index" to position indicated by "data"
  }

  //debug
  delay(500);
  servoList[index].detach();
  delay(500);
  servoList[index].attach(index + 2);
  /*Serial.println((uint8_t)((opcode & SERVO_INDEX_MASK) >> 4));
    Serial.println((uint8_t)data);*/

}

//Main function, runs in background
void loop()
{
  /*
    Arduino serial allows up to 63 characters.
    .available() returns the number of characters available
    Thus we can have up to 63 motors controlled in one command

    Opcode documentation found at
    https://docs.google.com/document/d/1cmILsPNfWTc15lZilK4X1TophI3p4mWjxnR8-Ee94GQ/edit
  */

  if (M4SERIAL.available() > 1) {             //Requires 2 bytes- opcode and data
    uint8_t opcode = M4SERIAL.read();         //Upper 4 bits of this opcode describe which motor to move (for a max of 14 servos). Lower 4 bits reserved for instructions
    if ((opcode & OPCODE_MASK) == 0x02) {
      uint8_t data = M4SERIAL.read();         //Only retrieve next byte if opcode is valid
      moveServo(opcode, data);
    }
    else if ((opcode & OPCODE_MASK) == 0x03) {
      //Move Stepper. Legacy opcode if we don't end up using steppers. If we need the stepper code again, find it on github before changes pushed on 6/23/22
    }
    else if ((opcode & OPCODE_MASK) == 0x04) {
      //Send sensory data to m4
      //Legacy opcode. No sensory data to get as far as I know.
    }
  }
}
