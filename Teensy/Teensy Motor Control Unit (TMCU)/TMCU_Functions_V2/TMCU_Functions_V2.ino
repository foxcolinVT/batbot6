#include <Servo.h> 
#include <Stepper.h> 
#define M4SERIAL Serial1    //Bugs out if the teensy software isn't installed in the 

//Constants
const uint8_t OPCODE_MASK = 0b111;
const int NUM_SERVOS = 2;
const int STEPS = 200;
const int STEPPER_SPEED  = 60;
const uint8_t STEPPER_LR_MASK = 0b10000000; //Masks for whether we're talking about left or right stepper motor

/*
//Struct for continuous servos
struct ContServoStruct {
  Servo S;
  int pos = 0;              //Records servo's current position. Only necessary for continuous servos
}
*/

//Stepper declarations
struct StepperStruct{
  Stepper S = Stepper(STEPS, 23, 22, 21, 20);     //Default values to make arduino happy. Will be overwritten
  int pos = 0;
};

StepperStruct leftStepper;   //Declare stepper struct
StepperStruct rightStepper;   //Declare stepper struct

//servo declarations
Servo servoList[NUM_SERVOS];  //array of servos

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

void setup() 
{ 
  //Stepper Initializations
  leftStepper.S = Stepper(STEPS, 19, 18, 17, 16);
  leftStepper.S.setSpeed(STEPPER_SPEED);
  rightStepper.S = Stepper(STEPS, 23, 22, 21, 20);
  
  
  //Servo Initializations
  for(int i = 0; i < NUM_SERVOS; i++){
    servoList[i].attach(i+2);   //Starts servo pins at pin 2 (pins 0 and 1 are used for serial)
  }
  
  /*earRA.attach(1);
  earRB.attach(1);
  earRC.attach(1);
  earRD.attach(1);

  earLA.attach(1);
  earLB.attach(1);
  earLC.attach(1);
  earLD.attach(1);

  mouthA.attach(1);
  mouthB.attach(1);
  mouthC.attach(1);*/
  
  Serial.begin(9600);                         //Used for debugging
  //commented out for debug
  M4SERIAL.begin(9600);                       //Used for communication with M4 
} 

//Movement of stepper motor
void moveStepper(StepperStruct* motor, uint8_t posD) {
  posD = map(posD, 0, 0b01111111, 0, STEPS);  //map desired position from 0 to 200
  motor->S.step(posD - motor->pos);           //Move motor by difference between current and desired pos
  motor->pos = posD;                          //update current position

  /*
  Note that this is very sensitive to change since there is no positional feedback
  from the steppers to the teensy.

  This could be remedied by either getting motors with feedback or by 
  engineering a reference point for the steppers to index off of.
  (like the end of travel for 3D printers).
*/
}

//Main function, runs in background
void loop() 
{ 
    
    //TODO  
    
    /*
      Arduino serial allows up to 63 characters. 
      .available() returns the number of characters available
      Thus we can have up to 63 motors controlled in one command

      Opcode documentation found at
      https://docs.google.com/document/d/1cmILsPNfWTc15lZilK4X1TophI3p4mWjxnR8-Ee94GQ/edit
    */
  
  if (M4SERIAL.available() > 1) { //Requires opcode and at least one byte of data (depending on implementation)
    uint8_t opcode = M4SERIAL.read();
    uint8_t data = M4SERIAL.read();
    if(opcode && OPCODE_MASK == 0x02){           
      if(data
      //TODO run servo
      //TODO Get information on which servo (either by bitmasking opcode or getting an extra character)
      //TODO calc angle from information
      //moveStdServo(servos[servoNum], angle) //pass servo by pointer
    }
    else if(opcode && OPCODE_MASK == 0x03){
      //TODO run stepper
      //TODO find out how the stepper is wired on the m4 breakout
    }
    else if(opcode && OPCODE_MASK == 0x04){
      //TODO send sensory data to m4
      //when would we use this?
      //Would we use this?
    }
  }
  
}

  /** TEST BLOCK - use for debug if needed
 
  if (count <= 4) {
    moveleftservo(ang);
    delay(1000);
    ang = ang + 10;
    count++;
    
  END TEST BLOCK **/
