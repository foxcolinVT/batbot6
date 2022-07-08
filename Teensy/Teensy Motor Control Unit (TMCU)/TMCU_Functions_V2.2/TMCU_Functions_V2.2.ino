/*
 *  Batbot 6 Teensy Motor Control Unit (TMCU)
 *  Controls Servos and Steppers off of a Teensy 4.0
 *  Download teensy software at https://www.pjrc.com/teensy/td_download.html
 *  
 *  Adapted by Maximilien Engel 6/7/22  
 *  
 *  If left, change SERVO_OPCODE to 0x02. If right, change SERVO_OPCODE to 0x05
 *  
 *  Extends 2.0 code to add a start and end byte, adds CRC
 *  Untested
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
const uint8_t SERVO_MIN_TRAVEL = 40;                //Sets servo min end of travel (in degrees)

//Constants 
const int MAX_SERVOS_PER_TEENSY = 12;               //Maximum number of servos per teensy
const uint8_t SOP = 0xFF;                           //Byte that indicates start of phrase 
const uint8_t EOP = 0xFE;                           //Byte that indicates end of phrase

//Servo declarations
Servo servoList[MAX_SERVOS_PER_TEENSY];             //Array of servos

//Variables for serial messaging
uint8_t SerialCommand[5];                           //Array of characters from serial
int charactersReceived = 0;                         //Keeps track of characters in current message
enum serialState {STDBY, MOVING, DONE};            //Dictates what we are doing with new serial inputs
enum serialState state;                             //Instanstiates state variable
state = STDBY;                                      //Set initial state
int MSG_LENGTH = 4;                                 //Length of a message (currently 4 - 2 for msg, 2 for checksum)
uint8_t currentMsg[MSG_LENGTH];                     //Set array of current message bytes

void setup() 
{
  M4SERIAL.begin(9600);                             //Used for communication with M4 
  Serial.begin(9600);
  
  //Servo Initializations
  for(int i = 0; i < MAX_SERVOS_PER_TEENSY; i++){
    servoList[i].attach(i);                         //Starts servo pins at pin 0. Pins 14, 15 used for serial
  }
} 

//Uses Fletcher's Algorithm from https://www.tutorialspoint.com/cyclic-redundancy-check-crc-in-arduino
//CRC libraries exist if we decide we want to use one
uint16_t checksumCalculator(uint8_t * data)
{
   uint8_t sum1 = (uint8_t) curr_crc;
   uint8_t sum2 = (uint8_t) (curr_crc >> 8);
   
   for(int i = 0; i < MSG_LENGTH; i++){
      sum1 = (sum1 + data[i]) % 255;
      sum2 = (sum2 + sum1) % 255;
   }
   return ((uint16_t) sum2 << 8) | sum1;
}

//Checks that read is ok
//If false, return false
bool isReadOk(){
  //Check message was terminated
  if(M4_SERIAL.read() != EOP){
    return false;
  }

  //Check that received sum and calculated sum match
  uint16_t sum = ((uint16_t)(currentMsg[3] << 8)) | currentMsg[2];
  else if(checksumCalculator(currentMsg) != sum){
    return false;
  }
}

//This function runs every time a serial event occurs.
//Pull elements from serial buffer as they arrive
//Decide whether to throw an error
//Sets FSM state
void serialEvent1(){
  //Flush out irrelevant bytes
  while(M4SERIAL.available() > 0 && M4SERIAL.peek() != SOP){
    //TODO this should bring up an error- resend current set of movement
    M4SERIAL.read();
  }

  //If there are enough bytes, place relevant bytes in array
  if(M4SERIAL.available() > MSG_LENGTH + 2 && M4_SERIAL.read() == SOP){  //Ensures msg, start byte and end byte are all in buffer
    //Read message
    for(int i = 0; i < MSG_LENGTH; i++){
      currentMsg[i] = M4SERIAL.read();
    }   
    
    //Check message
    if(!isReadOk()){
      //ask for new data
    }
    
    //Everything good, go ahead and move
    else{
      state = MOVING;
    }
  } 
}

//Movement of servo
//passes 1 byte to indicate which servo, 1 byte of position data
void moveServo(uint8_t opcode, uint8_t data){
  uint8_t index = (uint8_t)((opcode & SERVO_INDEX_MASK) >> 4);                    //isolate index
  if(data < SERVO_MAX_TRAVEL && data > SERVO_MIN_TRAVEL){                         //Check the desired position is possible
    servoList[index - servoIndexOffset].write((uint8_t)data);                     //Move servo at "index" to position indicated by "data"
  } 
}

//Main function, runs in background
void loop() 
{    
    /*
      Opcode documentation found at
      https://docs.google.com/document/d/1cmILsPNfWTc15lZilK4X1TophI3p4mWjxnR8-Ee94GQ/edit
    */
  
  switch(state){
    case STDBY:
      //Wait for serialEvent to collect data and allow motor control
    break;
    case MOVING:
      uint8_t opcode = currentMsg[0];         //Upper 4 bits of this opcode describe which motor to move (for a max of 14 servos). Lower 4 bits reserved for instructions
      uint8_t data = currentMsg[1];           //Only retrieve next byte if opcode is valid
      
      //If servo opcode, move servo. Having an opcode only makes sense 
      //if we have more than just 1 type of command (which we might not have)
      if((opcode & OPCODE_MASK) == SERVO_OPCODE){  
        moveServo(opcode, data);
      }
      state = STBY;
    break;
  }
}
