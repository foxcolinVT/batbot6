#include <Servo.h> 
#include <Stepper.h> 
#define M4SERIAL Serial1    //Bugs out if the teensy software isn't installed

//Masks
const uint8_t OPCODE_MASK = 0b111;                  //Mask for isolating opcode info from opcode
const uint8_t SERVO_INDEX_MASK = 0b11110000;        //Mask for isolating index info from opcode
const uint8_t STEPPER_LR_MASK = 0b10000000;         //Mask for isolating l/r stepper motor info from opcode
const uint8_t STEPPER_DIRECTION_MASK = 0b01000000;  //Mask for isolating l/r motor direction info from opcode

//Constants
const int SERVO_MAX  = 180;                         //Serrvo max range of motion
const int NUM_SERVOS = 11;
const int STEPS = 200;                              //Stepper number of steps
const int STEPPER_SPEED  = 60;


//servo declarations
Servo servoList[NUM_SERVOS];                        //array of servos

//List of servos in order
/*
Servo earRA;  //Right ear servo A
Servo earRB; //Right ear servo B
Servo earRC;  //Right ear servo C
Servo earRD; //Right ear servo D

Servo earLA;  //Left ear servo A
Servo earLB; //Left ear servo B
Servo earLC;  //Left ear servo C
Servo earLD; //Left ear servo D

Servo mouthA;  //Mouth servo A
Servo mouthB; //Mouth servo B
Servo mouthC;  //Mouth servo C
*/

//Stepper declarations
struct StepperStruct{
  Stepper S = Stepper(STEPS, 23, 22, 21, 20);         //Default values to make arduino happy. Will be overwritten in setup
  int pos = 0;
};

StepperStruct leftStepper;                            //Declare stepper struct
StepperStruct rightStepper;                           //Declare stepper struct

void setup() 
{ 
  //Servo Initializations
  for(int i = 0; i < NUM_SERVOS; i++){
    servoList[i].attach(i+2);                         //Starts servo pins at pin 2 (pins 0 and 1 are used for serial)
  }

  //Stepper Initializations
  leftStepper.S = Stepper(STEPS, 19, 18, 17, 16);
  leftStepper.S.setSpeed(STEPPER_SPEED);
  rightStepper.S = Stepper(STEPS, 23, 22, 21, 20);
  
  Serial.begin(9600);                                 //Used for debugging
  M4SERIAL.begin(9600);                               //Used for communication with M4 
} 

//Movement of servo
//Passes servo by reference, passes 1 byte of position data
//Since we're no longer mapping to 
void moveServo(uint8_t opcode, uint8_t data){
  int index = (int)((opcode & SERVO_INDEX_MASK) >> 4);//isolate index
  servoList[index].write((int)data);                  //Move servo at "index" to position indicated by "data"
}

//Movement of stepper motor
//Passes motor by reference, passes 1 byte of position data
void moveStepper(uint8_t opcode, uint8_t data) {
    //Decide which direction to move
    //TODO update m4 and Jetson to use 0bX1XXXXXX as move left and 0bX0XXXXXX as move right
    int stepsToMove = data;                           //Defaults to moving in the positive (left) direction
    if((opcode & STEPPER_DIRECTION_MASK) != STEPPER_DIRECTION_MASK){
      stepsToMove = -stepsToMove;                     //If bit is 0, move in the negative (right) direction
    }

    //Decide which motor to move
    //TODO update m4 and Jetson to use 0b1XXXXXXX as left and 0b0XXXXXXX as right
    if((opcode & STEPPER_LR_MASK) == STEPPER_LR_MASK){           
      leftStepper.S.step(stepsToMove);                //Move TODO check that it indeed moves left at amount indicated
      leftStepper.pos = leftStepper.pos + stepsToMove;//update current position  
    }
    else {                                        
      rightStepper.S.step(stepsToMove);               //Move TODO check that it indeed moves right at amount indicated
      rightStepper.pos = rightStepper.pos + stepsToMove;  //update current position  
    }
  
  /*
  Note that this is very sensitive to change since there is no positional feedback
  from the steppers to the teensy.

  This could be remedied by either getting motors with feedback or by 
  engineering a reference point for the steppers to index off of.
  (like the end of travel sensors for 3D printers).
*/
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
    uint8_t opcode = M4SERIAL.read();         //Upper 4 bits of this opcode describe which motor to move (for a max of 16 motors). Lower 4 bits reserved for instructions
    if((opcode & OPCODE_MASK) == 0x02){
      uint8_t data = M4SERIAL.read();         //Only retrieve next byte if opcode is valid
      moveServo(opcode, data);
    }
    else if((opcode & OPCODE_MASK) == 0x03){
      uint8_t data = M4SERIAL.read();         //Only retrieve next byte if opcode is valid
      moveStepper(opcode, data);
    }
    else if((opcode & OPCODE_MASK) == 0x04){
      //Send sensory data to m4
      //Legacy opcode. No sensory data to get as far as I know.
    }
  } 
}
