#include <Servo.h> 
#include <Stepper.h> 
#define M4SERIAL Serial1

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
int NUM_SERVOS = 12;
ContServoStruct[NUM_SERVOS / 2] leftContServos;  //array of left servos
ContServoStruct[NUM_SERVOS / 2] rightContServos; //array of right servos

//Stepper declarations
StepperStruct leftStepper;  //Left stepper object variable 
StepperStruct rightStepper; //Right stepper object variable


void setup() 
{ 
  //Servo Initializations
  for(int i = 0; i < NUM_SERVOS / 2; i++){
    leftservos[i].S.attach(i + 2);                                  // attaches the left servos starting on pin 2 (1 and 0 reserved for serial)
    rightservos[i + NUM_SERVOS/2].S.attach(i + NUM_SERVOS / 2);     // attaches the right servos starting on pin (2 + num of left servos)
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
void moveStdServo(ContServoStruct* servos, int servoIndex, int angle) {
  servos[servoIndex].S.write(angle);
  servos[servoIndex].pos = angle;     //pos recording not necessary for standard servos
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
  if (M4SERIAL.available()) {
    
    
    //TODO
    
    
    /*
      Arduino serial allows up to 63 characters. 
      .available() returns the number of characters available
      Thus we can have up to 63 motors controlled in one command

      Opcode documentation found at
      https://docs.google.com/document/d/1cmILsPNfWTc15lZilK4X1TophI3p4mWjxnR8-Ee94GQ/edit
    */

    
    uint8_t opcode = M4SERIAL.read();
    // Fix all of this below .
    if (opcode == 0x02) {
      uint8_t byteservo = M4SERIAL.read();
      if (byteservo & 0x40 != 0){
        uint8_t valueB = 0x3F && byteservo;
        int value = int(valueB);
        int angle = (180/64)*value;
        moverightservo(angle);
      }
      if (byteservo & 0x80 != 0){
        uint8_t valueB = 0x3F && byteservo;
        int value = int(valueB);
        int angle = (180/64)*value;  // The binary value to pass is the angle*64/180
        moveleftservo(angle);
      }
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
