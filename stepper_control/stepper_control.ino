#include "StepperControl.h"
#include <Pixy.h>
#include <SPI.h>
#include <AccelStepper.h>

//Declare pin functions on Arduino
#define stp 2
#define dir 3
#define MS1 4
#define MS2 5
#define MS3 6
#define EN  7

//Declare variables for functions
#define X_CENTER        ((PIXY_MAX_X-PIXY_MIN_X)/2)       
#define Y_CENTER        ((PIXY_MAX_Y-PIXY_MIN_Y)/2)
#define k_p             10
char user_input;
int x;
int y;
int state;
int numSteps;
double prev_servoVal;

// declare stepper and pixy objects
StepperControl stpr(stp, dir, MS1, MS2, MS3, EN);
AccelStepper stepper(1, stp, dir);
Pixy pixy;

// setup pan and tilt servo loops
class ServoLoop
{
public:
  ServoLoop(int32_t pgain, int32_t dgain);

  void update(int32_t error);
   
  int32_t m_pos;
  int32_t m_prevError;
  int32_t m_pgain;
  int32_t m_dgain;
};

ServoLoop panLoop(300, 500);
ServoLoop tiltLoop(40,700); // 500, 700

ServoLoop::ServoLoop(int32_t pgain, int32_t dgain)
{
  m_pos = PIXY_RCS_CENTER_POS;
  m_pgain = pgain;
  m_dgain = dgain;
  m_prevError = 0x80000000L;
}

void ServoLoop::update(int32_t error)
{
  long int vel;
  char buf[32];
  if (m_prevError!=0x80000000)
  {  
    vel = (error*m_pgain + (error - m_prevError)*m_dgain)>>10;
    //sprintf(buf, "%ld\n", vel);
    //Serial.print(buf);
    m_pos += vel;
    if (m_pos>PIXY_RCS_MAX_POS) 
      m_pos = PIXY_RCS_MAX_POS; 
    else if (m_pos<PIXY_RCS_MIN_POS) 
      m_pos = PIXY_RCS_MIN_POS;
  }
  m_prevError = error;
}

// convert the servo value to the corresponding stepper input
int servoToStepper(int servoVal)
{
  double error = servoVal;
  int rpm = (int) k_p*error;   
  
  //Serial.println(error);
  if(error < 5 && error > -5){
    rpm = 0;
    stepper.setSpeed(rpm);
    stepper.runSpeed();
  } 

  // set direction of the stepper 
  else if (error > 0)
  {
    digitalWrite(dir, LOW); //Pull direction pin low to move "forward"
    digitalWrite(EN, LOW); //Pull enable pin low to set FETs active and allow motor control
    stepper.setSpeed(rpm);
    stepper.runSpeed();
    //stepper.EigthSteps(24);
    //stpr.resetBEDPins();
    //Serial.println(error);
  }
  else 
  { 
    //error = -error;
    digitalWrite(dir, HIGH); //Pull direction pin low to move "forward"
    digitalWrite(EN, LOW); //Pull enable pin low to set FETs active and allow motor control
    stepper.setSpeed(rpm);
    stepper.runSpeed();
    //stepper.EigthSteps(24);
    //stpr.resetBEDPins();
    //Serial.println(error);
  }
}

void setup() {
  stpr.resetBEDPins(); //Set step, direction, microstep and enable pins to default states
  Serial.begin(115200); //Open Serial connection for debugging
  Serial.println("Begin motor control");
  Serial.println();
  //Print function list for user selection
  Serial.println("Enter number for control option:");
  Serial.println("1. One revolution in quarterstep mode.");
  Serial.println();

  // initialize pixy
  pixy.init();

  stpr.QuarterSteps(1); //make the steps quarter steps
  stepper.setMaxSpeed(6000); //set the maximum speed
}

void loop() {
  //pixy control
  static int i = 0;
  int j, stepperVal;
  uint16_t blocks;
  char buf[32]; 
  int32_t panError, tiltError;

  i++;
  blocks = pixy.getBlocks();
  if (blocks)
    i++;
  
  if (i%10 == 0) // this mod changes the rate at which the servos are updated
  {
    panError = X_CENTER-pixy.blocks[0].x;
    tiltError = pixy.blocks[0].y-Y_CENTER;
    
    panLoop.update(panError);
    tiltLoop.update(tiltError);
    
    pixy.setServos(panLoop.m_pos, tiltLoop.m_pos);
    
  }

  if (i%10 == 0) {// this mod changes the rate at which the servos are updated
    servoToStepper(X_CENTER-pixy.blocks[0].x);
    i = 0;
  }
    
    
    
    // do this (print) every 50 frames because printing every
    // frame would bog down the Arduino
    /*if (i%50==0) 
    {
      sprintf(buf, "servoVal %d:\t", panLoop.m_pos);
      Serial.print(buf);
    }

    // get pan servo value and convert to stepper value
    stepperVal = servoToStepper(panLoop.m_pos);
    digitalWrite(EN, LOW); //Pull enable pin low to set FETs active and allow motor control
  stepper.QuarterSteps(4);
  stepper.resetBEDPins();

    if (i%50==0) 
    {
      sprintf(buf, "stepperVal %d:\n", stepperVal);
      Serial.print(buf);
    }*/
   

  // stepper control
  /*digitalWrite(EN, LOW); //Pull enable pin low to set FETs active and allow motor control
    stepper.QuarterSteps(0);
    stepper.resetBEDPins();
*/
  /*while(Serial.available()){
      user_input = Serial.read(); //Read user input and trigger appropriate function

      digitalWrite(EN, LOW); //Pull enable pin low to set FETs active and allow motor control
      if (user_input =='1')
      {
         stepper.QuarterSteps(1600);
      }
      else
      {
        Serial.println("Invalid option entered.");
      }
      stepper.resetBEDPins();
  }*/
}
