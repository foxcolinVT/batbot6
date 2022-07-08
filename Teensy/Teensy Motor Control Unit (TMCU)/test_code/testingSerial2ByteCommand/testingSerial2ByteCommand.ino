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
  
boolean isNew[2]={false, false};
byte cmd[2]={0, 0};
  
void setup() {
  Serial.begin(9600);
}

void loop() {
  if (Serial.available()) {
    byte currData=Serial.read();
    //see if flag bit (most significant bit) is true, if so then it is header command
    if((currData>>7)==1)
    {
       isNew[0] = true;
       cmd[0] = currData;
       Serial.println("recieved first byte");

       //can also check if second is true
       //throw away this first one and next one if second is true, also set isNew correctly
    }
    else
    {
       isNew[1]=true;
       command[1]=currData;
       Serial.println("recieved second byte");

       //can also check if first is true
       //throw away this second one if first isn't true, also set isNew correctly
    }

    
    //maybe bool arr for if new command
    //then int8 arr for each subcommand
    //if bool arr is both true then start motor control and 0 out bool arr
    if(isNew[0]==true&&isNew[1]==true)
    {
      //print curr command 
      Serial.println("Recieved a full command, is as follows:");
      Serial.println(command[0]);
      Serial.println(command[1]);
      Serial.println("\n");
      //start motor control
      isNew[0]=false;
      isNew[1]=false;
    }
  }
}
