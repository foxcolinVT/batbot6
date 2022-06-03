#include <Servo.h> 
#include <Stepper.h> 
#define M4SERIAL Serial1

//Constants
const uint8_t OPCODE_MASK = 0b111;
const int NUM_SERVOS = 2;

//Struct for continuous servos
struct ContServoStruct {
  Servo S;
  int pos = 0;              //Records servo's current position. Only necessary for continuous servos
}

struct StepperStruct {
  Stepper S;                //might be unnecessary if the m4 drives the steppers
  int pos = 0;
  //The original code had these three pins. I don't know if this is right. 
  //While the actual driver requires 5-6 pins input, the M4 shield might only need data from 3 pins
  int stepPin;              //pulse pin
  int dirPin;               //direction pin
  int enPin;                //enable pin
}

//Servo declarations;
ServoStruct[NUM_SERVOS] servos;  //array of servos

//Stepper declarations
StepperStruct leftStepper;  //Left stepper object variable 
StepperStruct rightStepper; //Right stepper object variable


void setup() 
{ 
  //Servo Initializations
  for(int i = 0; i < NUM_SERVOS / 2; i++){
    //Lower half: left servos
    servos[i].S.attach(i + 2);                                       // attaches the left servos starting on pin 2 (1 and 0 reserved for serial)
    //Upper half: right servos
    servos[i + NUM_SERVOS/2].S.attach((i + NUM_SERVOS / 2) + 2);     // attaches the right servos starting on pin (2 + num of left servos)
  }


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
 
//Movement of standard servo
void moveStdServo(Servo& S, angle) {      //pass servo by reference
  S.write(angle);
  //pos recording not necessary for standard servos
  delay(10);
}

//Movement of stepper motor
void moveStepper(StepperStruct* motor, /*TODO what parameters do I need*/) {
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

    /*uint8_t opcode = M4SERIAL.read();
    // Fix all of this below .
    if (opcode == 0x02) {
      uint8_t byteservo = M4SERIAL.read();
      if (byteservo & 0x40 != 0){
        uint8_t valueB = 0x3F && byteservo;
        int value = int(valueB);
        int angle = (180/64)*value;
        //move right servo by angle;
      }
      if (byteservo & 0x80 != 0){
        uint8_t valueB = 0x3F && byteservo;
        int value = int(valueB);
        int angle = (180/64)*value;  // The binary value to pass is the angle*64/180
        //move left servo by angle;
      }
    }
  }*/
  
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
