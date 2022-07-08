//Teensy Serial Communication Test code
//Meant to test the reliability of the serial communication
//Acts as a platform for serial communication improvements

//Issues with current iteration
  //Does not give feedback if bytes are received out of order
  //Does not give feedback if bytes are dropped (missing)
  //Semantics: isNew and cmd should be organized as a struct since they are used in conjunction

//What this script does
  //Allows motor control only if both bytes are of the right type. 
  //Motors will never receive an address for a positional data (or vice versa)

//What it does not do
  //Give feedback when error occurs
  //Account for dropped bytes:
    //If the motor address is dropped, the wrong motor will be given the motor control data
    //Similarly, if the position data is dropped, the motor will receive the wrong data

//What it should do
  //Expect Motor address, followed by position data
  //If out of order (ie, 2 addresses in a row or 2 data in a row), complain
  //If complaint thrown, then we know that we should implement some kind of feedback- like CRC

enum SerialStatus {StartState, ExpectAddress, ExpectData, Error, End};
enum state = StartState;
unsigned long = startTime;

byte currData;
  
void setup() {
  Serial.begin(9600);
  delay 1000;
  startTime = millis();
}

void loop() {
  if(Serial.available() > 0){
    currData = Serial.read();
    switch(state){
      case StartState:          //Start state serves to "sync" up the address/position data bytes
        if((currData>>7) == 1)  //Decide what the next byte's type should be
          state = ExpectData;
        else
          state = ExpectAddress;
      break;
      case ExpectAddress:      
        if((currData>>7)!=1)
          state = Error;
        else
          state = ExpectData;
      break;
      case ExpectData:
        if((currData>>7) == 1)
          state = Error;
        else
          state = ExpectAddress;
      break;
      case Error:
        Serial.print("Error: dropped byte after "); 
        Serial.print(millis() - startTime);
        Serial.println(" ms");
        state = End;
      break;
    }
  }
}
