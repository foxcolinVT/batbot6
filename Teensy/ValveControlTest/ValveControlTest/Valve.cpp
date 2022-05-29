#include "Valve.h"
//configuration of a valve is "Normally Closed"

Valve::Valve()
{

}

void Valve::maxRunTime(int lengthOfTime)
{
  _lengthOfTime = lengthOfTime;
}

void Valve::setPins(int pin1, int pin2)
{
  _pin1 = pin1;
  _pin2 = pin2;
  pinMode(pin1, OUTPUT);
  pinMode(pin2, OUTPUT);
}

void Valve::inflate()
{
  digitalWrite(_pin1, HIGH);
  digitalWrite(_pin2, LOW);
}

void Valve::closeAll()
{
  digitalWrite(_pin1, LOW);
  digitalWrite(_pin2, LOW);
}

void Valve::deflate()
{
  digitalWrite(_pin1, LOW);
  digitalWrite(_pin2, HIGH);
}

void Valve::numWrite(int input)
{
  analogWrite(_pin1, input);
  digitalWrite(_pin2, LOW);
}

void Valve::runTestSequence()
{
  this->inflate();
  delay(200);
  this->closeAll();
  delay(10);
  this->deflate();
  delay(30);
}

void Valve::timedInflate(int timing)
{
  timedInflate(timing, false);
}

void Valve::timedInflate(int timing, boolean pausing)
{
  if (timing < safteyThreshold)
  {
    this->inflate();
    delay(timing);
    if (pausing)
    {
    this->closeAll();
    delay(2000);
    }
    this->deflate();
    delay(200);
  }
  else
  {
    Serial.println("Timing Exceeds Saftey Threshold. This can be changed within the Valve.h class, but do so with caution!");
  }
}

void Valve::moveBasedOnFormula(int posDeg)
{
  this->timedInflate(getTimingFromPos(posDeg), true);
}

int Valve::getTimingFromPos(int posDeg)
{
  return posDeg * 2;
}
