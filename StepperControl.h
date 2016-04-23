/*
  StepperControl.h - Library for Stepper motor 
  Created by Chloe Song, Zachary Kendrick 
  CAR LAB
 */

#ifndef StepperControl_h
#define StepperControl_h

#include "Arduino.h"

class StepperControl
{
  public:
    StepperControl(int STEP, int DIR, int MS1, int MS2, int MS3,
    	int EN);
    void resetBEDPins();
    void FullSteps(int numSteps);
    void HalfSteps(int numSteps);
    void QuarterSteps(int numSteps);
    void EigthSteps(int numSteps);
    void SixteenthSteps(int numSteps);
  private:
    int _step;
    int _dir;
    int _ms1;
	  int _ms2;
	  int _ms3;
	  int _en;
	  void step(int numSteps);
};

#endif
