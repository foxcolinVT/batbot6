//Teensy Serial Communication Test code
//Meant to test the reliability of the serial communication
//Acts as a platform for serial communication improvements

//What this script does
  //Complaints if a byte is dropped
   
//What it does not do
  //Move motors
  //Offer a recourse if a byte is dropped

//What it should do
  //Expect Motor address, followed by position data
  //If out of order (ie, 2 addresses in a row or 2 data in a row), complain
  //If complaint thrown, then we know that we should implement some kind of feedback- like CRC

enum SerialStatus {StartState, ExpectAddress, ExpectData, Error, End};
enum SerialStatus state = StartState;
unsigned long startTime;

byte currData;
  
void setup() {
  Serial.begin(9600);
  delay(1000);
  startTime = millis();
}

void loop() {
  if(Serial.available() > 0){
    currData = Serial.read();
    switch(state){
      //Start state serves to "sync" up the address/position data bytes
      case StartState:          
        if((currData>>7) == 1)  //Decide what the next byte's type should be based on the current byte's flag bit
          state = ExpectData;
        else
          state = ExpectAddress;
      break;
      
      //When the next byte should be an address
      case ExpectAddress:      
        if((currData>>7)!=1)      //If the flag bit is incorrect, complain
          state = Error;
        else
          state = ExpectData;     //Else if the flag bit is correct, expect data afterwards
      break;
      
      //When the next byte should be positional data
      case ExpectData:
        if((currData>>7) == 1)    //If the flag bit is incorrect, complain
          state = Error;
        else
          state = ExpectAddress;  //If the flag bit is correct, expect an address byte
      break;
      
      //Complain when an unexpected byte is received
      case Error:
        Serial.print("Error: unexpected byte after "); 
        Serial.print(millis() - startTime);
        Serial.println(" ms");
        state = End;
      break;
    }
  }
}
