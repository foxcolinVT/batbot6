#include <Servo.h> 
#include <Stepper.h> 
#define M4SERIAL Serial1

//Constants
const uint8_t OPCODE_MASK = 0b111;
const int NUM_SERVOS = 2;
/*
//Struct for continuous servos
struct ContServoStruct {
  Servo S;
  int pos = 0;              //Records servo's current position. Only necessary for continuous servos
}
*/

struct StepperStruct {
  Stepper S;                //might be unnecessary if the m4 drives the steppers
  int pos = 0;
  //The original code had these three pins. I don't know if this is right. 
  //While the actual driver requires 5-6 pins input, the M4 shield might only need data from 3 pins
  int stepPin;              //pulse pin
  int dirPin;               //direction pin
  int enPin;                //enable pin
};

//Stepper declarations
//StepperStruct leftStepper;  //Left stepper object variable 
//StepperStruct rightStepper; //Right stepper object variable

//servo declarations
Servo[NUM_SERVOS] servoList;  //array of servos

servoList[0] = Servo earRA;  //Right ear servo A
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


void setup() 
{ 
  //Servo Initializations
  earRA.attach(1);
  earRB.attach(1);
  earRC.attach(1);
  earRD.attach(1);

  earLA.attach(1);
  earLB.attach(1);
  earLC.attach(1);
  earLD.attach(1);

  mouthA.attach(1);
  mouthB.attach(1);
  mouthC.attach(1);

  //Stepper initializations
  //defines pins for left stepper motors
  leftStepper.stepPin = 18;    //PUL -Pulse
  leftStepper.dirPin = 19;     //DIR -Direction
  leftStepper.enPin =  20;     //ENA -Enable
  //defines pins for right stepper motors
  rightStepper.stepPin = 21;   //PUL -Pulse
  rightStepper.dirPin = 22;    //DIR -Direction
  rightStepper.enPin = 23;     //ENA -Enable

  //Set pin modes for steppers
  pinMode(leftStepper.stepPin,OUTPUT); 
  pinMode(leftStepper.dirPin,OUTPUT);
  pinMode(leftStepper.enPin,OUTPUT);
  digitalWrite(leftStepper.enPin,LOW);
  pinMode(rightStepper.stepPin,OUTPUT); 
  pinMode(rightStepper.dirPin,OUTPUT);
  pinMode(rightStepper.enPin,OUTPUT);
  digitalWrite(right.enPin,LOW);
  
  M4SERIAL.begin(9600);
} 

//Movement of stepper motor
void moveStepper(StepperStruct* motor, /*TODO what parameters do I need*/) {
  
  //bitwise of beginning to determine direction
  for(int x = 0; x < 200; x++) {
    digitalWrite(LstepPin,HIGH);
    delayMicroseconds(500);
    digitalWrite(LstepPin,LOW);
    delayMicroseconds(500);
  }
  /*
  //TODO make sense of this
  //Enables the motor direction to move
  digitalWrite(LdirPin,HIGH);
  //Makes 200 Pulses for making one full cycle rotation
  for(int x = 0; x < 200; x++){
    digitalWrite(LstepPin,HIGH); 
    delayMicroseconds(500); 
    digitalWrite(LstepPin,LOW); 
    delayMicroseconds(500); 
  }
  
  //One second delay 
  delay(1000);

  //Changes the rotations direction
  digitalWrite(LdirPin,LOW);
  // Makes 200 pulses for making one full cycle rotation
  for(int x = 0; x < 200; x++) {
    digitalWrite(LstepPin,HIGH);
    delayMicroseconds(500);
    digitalWrite(LstepPin,LOW);
    delayMicroseconds(500);
  }
  
  //One second delay
  delay(1000); 

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
    if(opcode && OPCODE_MASK == 0x02){           
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
