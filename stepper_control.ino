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
#define X_CENTER        ((PIXY_MAX_X-PIXY_MIN_X)/2) // x-coord center of frame
#define Y_CENTER        ((PIXY_MAX_Y-PIXY_MIN_Y)/2) // y-coord center of frame
#define k_p             25 // proportional gain // 20 //25
#define k_d             2.5 // differential gain //2 //2.5

// global variables
int x; // x-coordinate
int y; // y-coordinate
int prev_X;; // the previous servo value

// declare stepper and pixy objects
StepperControl stpr(stp, dir, MS1, MS2, MS3, EN); //used to set the size of the steps that we want
AccelStepper stepper(1, stp, dir); //used to control the speed of the stepper motor
Pixy pixy; // instance of pixy

// setup pan and tilt servo loops (this is from the pixy examples)
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
ServoLoop tiltLoop(500 ,700); // 40,700

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
  int deriv_term = pixy.blocks[0].x - prev_X;
  int servoVal_P = X_CENTER-pixy.blocks[0].x;
  int servoVal_D = X_CENTER-deriv_term;
  
  // PID loop for stepper motor
  int rpm = (int) k_p*servoVal_P + k_d*servoVal_D;
  prev_X = pixy.blocks[0].x;

  stepper.setSpeed(-rpm);
  stepper.runSpeed();
  
  // set direction of the stepper 
// if (servoVal > 0)
//  {
//    digitalWrite(dir, HIGH); //Pull direction pin low to move "forward" 
//    stepper.setSpeed(rpm);
//    stepper.runSpeed();
//  }
//  else 
//  {
//    digitalWrite(dir, LOW); //Pull direction pin low to move "forward" 
//    stepper.setSpeed(-rpm);
//    stepper.runSpeed();
//  }
}

void setup() {
  digitalWrite(EN, LOW); //Pull enable pin low to set FETs active and allow motor control
  Serial.begin(115200); //Open Serial connection for debugging
  pixy.init(); // initialize pixy
  stpr.EigthSteps(1); //make the steps quarter steps
  stepper.setMaxSpeed(1000); //set the maximum speed - Speeds of more than 1000 steps per second are unreliable. 
}

void loop() {
  //pixy control
  static int i = 0;
  uint16_t blocks;
  //char buf[32];
  int32_t panError, tiltError;
  const int sampleRate = 5;
  

  blocks = pixy.getBlocks();
//  if (blocks)
//    i++;
//  i++;

  // i%5 slows it down, also need to include blocks
  //if (blocks && i%5 == 0) // this mod changes the rate at which the servos are updated
  if (blocks)
  {
    //panError = X_CENTER-pixy.blocks[0].x;
    tiltError = pixy.blocks[0].y-Y_CENTER;
    
    //panLoop.update(panError);
    tiltLoop.update(tiltError);
    
    pixy.setServos(panLoop.m_pos, tiltLoop.m_pos);
  }

//  if (i%sampleRate == 0)  // we might want this if it becomes too jittery
//  {// this mod changes the rate at which the servos are updated
    servoToStepper(pixy.blocks[0].x);
//    i = 0;
//  }
}
