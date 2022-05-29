#include "Valve.h"

Valve pinna1;
Valve pinna2;
Valve pinna3;

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // put your setup code here, to run once:
  pinna1.setPins(6, 4);
  pinna2.setPins(7, 5);
  pinna3.setPins(2, 3);
  //---------------------------------------
  pinna1.deflate();
  pinna2.deflate();
  pinna3.deflate();

  Serial.println("Starting Test");
  delay(2000);

  /*
    for (int i = 0; i < 4; i++)
    {
      pinna2.runTestSequence();
      pinna1.runTestSequence();
      pinna3.runTestSequence();
      delay (1000);
    }
  */

  
  
  int testArrayLength = 32;
  int testingArray[testArrayLength] = {1, 2, 3 , 4, 5, 6, 7, 8, 9, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120, 130, 140, 150};
  
  for (int i = 0; i < testArrayLength; i++)
  {
    Serial.println("Waiting for next test");
    while (Serial.available() == 0)
    {
      ;
    }
    Serial.print("Running test for: ");
    Serial.println(testingArray[i]);
    Serial.clear();
    pinna1.timedInflate(testingArray[i], false);
  }
  Serial.println("TESTING COMPLETE");

//  int delayTime = 30;
//  for (int i = 0; i < 1000; i++)
//    {
//      pinna1.inflate();
//      pinna2.deflate();
//      delay(delayTime * 1.5);
//      pinna1.deflate();
//      pinna2.inflate();
//      delay(delayTime * .75);
//      pinna1.deflate();
//      pinna2.deflate();
//      delay(delayTime * 2);
//    }
//pinna1.timedInflate(60, true);
//int delayTime = 50;

//----------------- Test 1, indiviudal inflating/deflating -------------------------------
//  for (int i = 0; i <= 150; i += 2)
//  {
//    Serial.println(i);
//    //pinna2.timedInflate(i, true);
//    pinna1.inflate();
//    pinna2.inflate();
//    pinna3.inflate();
//    delay(i);
//    pinna1.closeAll();
//    pinna2.closeAll();
//    pinna3.closeAll();
//    delay(4000);
//    pinna1.deflate();
//    pinna2.deflate();
//    pinna3.deflate();
//    delay(1000);
//  }
//----------------- Test 1, indiviudal inflating/deflating -------------------------------
//  for (int i = 0; i <= 75; i += 2)
//  {
//    Serial.println(i);
//    pinna2.timedInflate(i, true);
//  }
//pinna1.timedInflate(30, true);
//pinna1.timedInflate(70, true);
//pinna1.timedInflate(100, true);

//----------------- Test 2, all inflating/deflating -------------------------------
//  for (int i = 0; i <= 120; i += 1)
//  {
//    Serial.println(i);
//    inflateAllPinna();
//    delay(i);
//    closeAllPinna();
//    delay(1000);
//    deflateAllPinna();
//    delay(500);
//  }
//--------------- Test 3, Rapid Movement ---------------------------------------------
//  for (int i = 0; i <= 1000; i++)
//  {
//    Serial.println(i);
//    inflateAllPinna();
//    delay(20);
//    deflateAllPinna();
//    delay(30);
//  }
}

void loop() {

}

void inflateAllPinna()
{
  pinna1.inflate();
  pinna2.inflate();
  pinna3.inflate();
}

void closeAllPinna()
{
  pinna1.closeAll();
  pinna2.closeAll();
  pinna3.closeAll();
}

void deflateAllPinna()
{
  pinna1.deflate();
  pinna2.deflate();
  pinna3.deflate();
}
