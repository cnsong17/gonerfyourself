/*
  StepperControl.cpp - Library for Stepper motor 
  Created by Chloe Song, Zachary Kendrick 
  CAR LAB
 */

#include "StepperControl.h"
#include "Arduino.h"

StepperControl::StepperControl(int STEP, int DIR, int MS1, int MS2, int MS3,
      int EN)
{
  pinMode(STEP, OUTPUT);
  pinMode(DIR, OUTPUT);
  pinMode(MS1, OUTPUT);
  pinMode(MS2, OUTPUT);
  pinMode(MS3, OUTPUT);
  pinMode(EN, OUTPUT);

  _step = STEP;
  _dir = DIR;
  _ms1 = MS1;
  _ms2 = MS2;
  _ms3 = MS3;
  _en = EN;
}

void StepperControl::step(int numSteps)
{
  for(int x = 1; x < numSteps; x++)  //Loop the forward stepping enough times for motion to be visible
  {
    digitalWrite(_step,HIGH); //Trigger one step forward
    delay(1);
    digitalWrite(_step,LOW); //Pull step pin low so it can be triggered again
    delay(1);
  }
}

void StepperControl::resetBEDPins()
{
  digitalWrite(_step, LOW);
  digitalWrite(_dir, LOW);
  digitalWrite(_ms1, LOW);
  digitalWrite(_ms2, LOW);
  digitalWrite(_ms3, LOW);
  digitalWrite(_en, HIGH);
}

void StepperControl::FullSteps(int numSteps)
{
  digitalWrite(_ms1, LOW);
  digitalWrite(_ms2, LOW);
  digitalWrite(_ms3, LOW);

  step(numSteps);
}

void StepperControl::HalfSteps(int numSteps)
{
  digitalWrite(_ms1, HIGH);
  digitalWrite(_ms2, LOW);
  digitalWrite(_ms3, LOW);

  step(numSteps);
}

void StepperControl::QuarterSteps(int numSteps)
{
  digitalWrite(_ms1, LOW);
  digitalWrite(_ms2, HIGH);
  digitalWrite(_ms3, LOW);

  step(numSteps);
}

void StepperControl::EigthSteps(int numSteps)
{
  digitalWrite(_ms1, HIGH);
  digitalWrite(_ms2, HIGH);
  digitalWrite(_ms3, LOW);

  step(numSteps);
}

void StepperControl::SixteenthSteps(int numSteps)
{
  digitalWrite(_ms1, HIGH);
  digitalWrite(_ms2, HIGH);
  digitalWrite(_ms3, HIGH);

  step(numSteps);
}
