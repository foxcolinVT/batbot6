//constructor
//binaryOn
//binaryOff
//analogWrite
//digitalWrite


#ifndef Valve_h
#define Valve_h
#include "Arduino.h"

class Valve
{
  public:
    Valve();
    void setPins(int pin1, int pin2);
    void inflate();
    void deflate();
    void closeAll();
    void runTestSequence();
    void numWrite(int input);
    void timedInflate(int timing);
    void timedInflate(int timing, boolean pausing);
    void maxRunTime(int lengthOfTime);
    int getTimingFromPos(int posDeg);
    void moveBasedOnFormula(int posDeg);
  private:
    int lowerRange = 900; //upper range of PWM
    int upperRange = 2000; //Lower Range of PWM
    int _pin1; //intake
    int _pin2; //outtake
    int _lengthOfTime;
    int safteyThreshold = 175; //Saftey Threshold
};

#endif
