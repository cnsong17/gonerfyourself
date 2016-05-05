 #include "StepperControl.h"
#include <Pixy.h>
#include <SPI.h>
#include <AccelStepper.h>

//Declare pin functions for stepper motor
#define stp 2
#define dir 3
#define MS1 4
#define MS2 5
#define MS3 6
#define EN  7

//Declare pin functions for laser
#define LASER 53
#define RELAY 47

//Declare variables for functions
#define X_CENTER        ((PIXY_MAX_X-PIXY_MIN_X)/2) // x-coord center of frame
#define Y_CENTER        ((PIXY_MAX_Y-PIXY_MIN_Y)/2) // y-coord center of frame
#define k_p             14 // proportional gain // 20 //25 
#define k_d             1 // differential gain //2 //2.5 //3

// global variables
int x; // x-coordinate
int y; // y-coordinate
int prev_X; // the previous X-coords center of blob
int prev_Y; // the previous Y-coords center of blob

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
void servoToStepper(int blocks_X)
{
  int deriv_term = blocks_X - prev_X;
  int servoVal_P = X_CENTER-blocks_X;
  int servoVal_D = X_CENTER-deriv_term;
  
  // PID loop for stepper motor
  int rpm = (int) k_p*servoVal_P + k_d*servoVal_D;
  prev_X = blocks_X;

//  if(rpm < 1000){
  stepper.setSpeed(-rpm);
//  }
//  else stepper.setSpeed(1000);
//  Serial.println(-rpm);
  stepper.runSpeed();
}

// find the peak of the trajectory
void findPeakOfFlight(int blocks_Y) {
  int deriv_Threshold = 0; //the derivative threshold
  int relayThreshold = 500; // the threshold for the trigger
  
  //find when the derivative is equal to or less than zero
  int deriv_term = Y_CENTER - blocks_Y;
  if (deriv_term <= deriv_Threshold && tiltLoop.m_pos <= relayThreshold) { 
    digitalWrite(RELAY,HIGH);
  }
  if (tiltLoop.m_pos >= relayThreshold){
    digitalWrite(RELAY,LOW);
    digitalWrite(LASER, LOW);
  }
  else {
    digitalWrite(LASER,HIGH);
  }
    
}  

void setup() {
  digitalWrite(EN, LOW); //Pull enable pin low to set FETs active and allow motor control
  Serial.begin(115200); //Open Serial connection for debugging
  pixy.init(); // initialize pixy
  stpr.EigthSteps(1); //make the steps quarter steps
  stepper.setMaxSpeed(1000); //set the maximum speed - Speeds of more than 1000 steps per second are unreliable. 
  pinMode(LASER, OUTPUT); // laser output pin
  pinMode(RELAY, OUTPUT); // relay output pin
  digitalWrite(LASER,LOW); //initialize laser to low
  digitalWrite(RELAY,LOW); //initialize relay to low

  // find pin with high voltage output
//  pinMode(49, OUTPUT);
//  pinMode(47, OUTPUT);
//  pinMode(45, OUTPUT);
//  pinMode(43, OUTPUT);
//  pinMode(41, OUTPUT);
//  pinMode(39, OUTPUT);
//  pinMode(37, OUTPUT);
//  pinMode(35, OUTPUT);
//  pinMode(33, OUTPUT);
//  pinMode(31, OUTPUT);
//  
//  digitalWrite(49, HIGH);
//  digitalWrite(47, HIGH);
//  digitalWrite(45, HIGH);
//  digitalWrite(43, HIGH);
//  digitalWrite(41, HIGH);
//  digitalWrite(39, HIGH); 
//  digitalWrite(37, HIGH);
//  digitalWrite(35, HIGH);
//  digitalWrite(33, HIGH);
//  digitalWrite(31, HIGH);
}

void loop() {
  //pixy control
  static int i = 0;
  uint16_t blocks;
  //char buf[32];
  int32_t panError, tiltError;
  //const int sampleRate = 5;
  

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
    //Serial.println(tiltLoop.m_pos);
    
    pixy.setServos(panLoop.m_pos, tiltLoop.m_pos);
  }

//  if (i%sampleRate == 0)  // we might want this if it becomes too jittery
//  {// this mod changes the rate at which the servos are updated
    servoToStepper(pixy.blocks[0].x);
    findPeakOfFlight(pixy.blocks[0].y);
//    i = 0;
//  }
}
