

#include <Wire.h>
#include "sensorbar.h"
#include <PID_v1.h>
#include <TimerOne.h>

//-------------------------PID variables--------------------------
#define RightMotor        5
#define LeftMotor         6
#define R_dir1            10
#define R_dir2            11
#define L_dir1            8
#define L_dir2            9
#define enc1              3
#define enc2              2
double RightPWM     = 0 ;
double RightSpeed   = 0.0;
double LeftPWM      = 0 ;
double LeftSpeed    = 0.0;
double Setpoint_R   = 50;
double Setpoint_L   = 50;

float RightKp = 2.0, RightKi = 0   , RightKd = 0.05;    //2.8 0.001
float LeftKp = 2.0 ,LeftKi = 0 , LeftKd = 0.05;

PID RighrRPM(&RightSpeed, &RightPWM, &Setpoint_R, RightKp, RightKi , RightKd, DIRECT ) ;
PID LeftRPM(&LeftSpeed, &LeftPWM, &Setpoint_L, LeftKp, LeftKi , LeftKd, DIRECT ) ;

volatile unsigned long RightCounter = 0;
volatile unsigned long LeftCounter = 0;
const float PPR = 384.0/10;

//---------------------IR Variables-------------------------------
/* Include the standard Arduino SPI library */
#include <SPI.h>
/* Include the RFID library */
#include <RFID.h>

/* Define the DIO used for the SDA (SS) and RST (reset) pins. */
#define SDA_DIO 53
#define RESET_DIO 48
/* Create an instance of the RFID library */
RFID RC522(SDA_DIO, RESET_DIO); 

// Uncomment one of the four lines to match your SX1509's address
//  pin selects. SX1509 breakout defaults to [0:0] (0x3E).
const uint8_t SX1509_ADDRESS = 0x3E;  // SX1509 I2C address (00)
//const byte SX1509_ADDRESS = 0x3F;  // SX1509 I2C address (01)
//const byte SX1509_ADDRESS = 0x70;  // SX1509 I2C address (10)
//const byte SX1509_ADDRESS = 0x71;  // SX1509 I2C address (11)

SensorBar mySensorBar(SX1509_ADDRESS);



//Define motor polarity for left and right side -- USE TO FLIP motor directions if wired backwards
#define RIGHT_WHEEL_POL -1
#define LEFT_WHEEL_POL 1

//Define the states that the decision making machines uses:
#define IDLE_STATE 0
#define READ_LINE 1
#define GO_FORWARD 2
#define GO_LEFT 3
#define GO_RIGHT 4

#define GO_LEFT_min 5
#define GO_RIGHT_min 6

uint8_t state=1 ;

#define R_en    5
#define R_fwd   10
#define R_rev   11
#define L_en    6
#define L_fwd   9
#define L_rev   8
#define in      13

char cmd = 'o';
char st = 'z';
  
void setup()
{  
  Serial.begin(115200);
  while (st != 's'){
    st=Serial.read();
    //Serial.println("waiting for s");
    }
//  pinMode(r,OUTPUT);
//  pinMode(a,INPUT);
   //Serial.begin(9600);
  /* Enable the SPI interface */
  SPI.begin(); 
  /* Initialise the RFID reader */
  RC522.init();
  //-----------PID-------------
  pinMode(RightMotor, OUTPUT);
  pinMode(R_dir1, OUTPUT);
  pinMode(R_dir2, OUTPUT);
  pinMode(LeftMotor, OUTPUT);
  pinMode(L_dir1, OUTPUT);
  pinMode(L_dir2, OUTPUT);
  Timer1.initialize(100000);
  attachInterrupt(digitalPinToInterrupt(enc1),int_count1, RISING);
  attachInterrupt(digitalPinToInterrupt(enc2),int_count2, RISING);
  RighrRPM.SetMode(AUTOMATIC);
  LeftRPM.SetMode(AUTOMATIC);
  Timer1.attachInterrupt(int_timer);
  
  //--------IR----------------
    // start serial for output
  //Serial.println("Program started.");
  //Serial.println();
  pinMode (in,INPUT);
  //Default: the IR will only be turned on during reads.
  mySensorBar.setBarStrobe();
  //Other option: Command to run all the time
  //mySensorBar.clearBarStrobe();

  //Default: dark on light
  mySensorBar.clearInvertBits();
  //Other option: light line on dark
  //mySensorBar.setInvertBits();
  
  //Don't forget to call .begin() to get the bar ready.  This configures HW.
  uint8_t returnStatus = mySensorBar.begin();
  if(returnStatus)
  {
   // Serial.println("sx1509 IC communication OK");
  }
  else
  {
    //Serial.println("sx1509 IC communication FAILED!");
  }

//------------waiting for pi command-------

}

void loop()
{ 

  if ( cmd == 'o' ){

  uint8_t nextState = state;
  if(digitalRead(in)==0 ){
  switch (state) {
  case IDLE_STATE:
    mstop();       // Stops both motors
    nextState = READ_LINE;
    break;
  case READ_LINE:
    if( mySensorBar.getDensity() < 7 )
    {
      nextState = GO_FORWARD;
      if( mySensorBar.getPosition() < -25 )
      {
      nextState = GO_LEFT_min;
      }
        
        if( mySensorBar.getPosition() < -100 )
      {
        nextState = GO_LEFT;
      }
      if( mySensorBar.getPosition() > 25 )
      {
        nextState = GO_RIGHT_min;
      }
      if( mySensorBar.getPosition() > 100 )
      {
        nextState = GO_RIGHT;
      }
      
    }
    else
    {
      nextState = IDLE_STATE;
    }
    //Serial.println("stuck here");
    break;
  case GO_FORWARD:
    driveBot(100);
    nextState = READ_LINE;
    break;
  case GO_LEFT:
    turnBot(-.3);
    nextState = READ_LINE;
    break;
  case GO_RIGHT:
    turnBot(.3);
    nextState = READ_LINE;
    break;
     case GO_RIGHT_min:
    driveTurnBot(100,.4);
    nextState = READ_LINE;
    break;
  case GO_LEFT_min:
    driveTurnBot(100,-.4);
    nextState = READ_LINE;
    break;
  default:
    mstop();       // Stops both motors
   
    break;
  }
  state = nextState;
  }
  else {
    mstop();
  }
  if(Serial.read()=='a'){
    cmd='a';
  }
  }
 
 if (cmd == 'a'){
  mstop();
  while(RC522.isCard()!=1){
    }

  cmd = 'o';
  if(Serial.read()=='t'){
    cmd='t';
  }
 }
 if (cmd == 't'){
  mstop();
  while(1){}
 }
}

//--------------------IR Functions------------------------
//When using driveBot( int16_t driveInput ), pass positive number for forward and negative number for backwards
//driveInput can be -255 to 255
void driveBot( int16_t driveInput )
{
  int16_t rightVar;
  int16_t leftVar;
  rightVar = driveInput * RIGHT_WHEEL_POL;
  leftVar = -1 * driveInput * LEFT_WHEEL_POL;
  rightMotor(rightVar);
  leftMotor(leftVar);
  
}

//When using ( int16_t driveInput, float turnInput ), pass + for forward, + for right
//driveInput can be -255 to 255
//turnInput can be -1 to 1, where '1' means turning right, right wheel stopped
void driveTurnBot( int16_t driveInput, float turnInput )
{
  int16_t rightVar;
  int16_t leftVar;
  //if driveInput is negative, flip turnInput
  if( driveInput < 0 )
  {
    turnInput *= -1;
  }
  
  //If turn is positive
  if( turnInput > 0 )
  {
    rightVar = driveInput * RIGHT_WHEEL_POL * ( 1 - turnInput );
    leftVar = -1 *driveInput * LEFT_WHEEL_POL;
  }
  else
  {
    rightVar = driveInput * RIGHT_WHEEL_POL;
    leftVar = -1 *driveInput * LEFT_WHEEL_POL * (1 - ( turnInput * -1));
  }

  rightMotor(rightVar);
  leftMotor(leftVar);
    delay(5);
    
}

//When using turnBot( float turnInput ), pass + for spin right.
//turnInput can be -1 to 1, where '1' means spinning right at max speed
void turnBot( float turnInput )
{
  int16_t rightVar;
  int16_t leftVar;
  //If turn is positive
  if( turnInput > 0 )
  {
    rightVar = -1 * 250 * RIGHT_WHEEL_POL * turnInput;
    leftVar = -1 * 250 * LEFT_WHEEL_POL * turnInput;
  }
  else
  {
    rightVar = 250 * RIGHT_WHEEL_POL * turnInput * -1;
    leftVar = 250 * LEFT_WHEEL_POL * turnInput * -1;
  }

  rightMotor(rightVar);
  leftMotor(leftVar);
  delay(5);
  
}

void rightMotor ( int sp ){
  if(sp>0){
    digitalWrite(R_fwd,HIGH);
    digitalWrite(R_rev,LOW);
    Setpoint_R = sp;
  }
  else if(sp<0){
    digitalWrite(R_fwd,LOW);
    digitalWrite(R_rev,HIGH);
    Setpoint_R = -sp;
  }
}
void leftMotor ( int sp ){
  if(sp<0){
    digitalWrite(L_fwd,HIGH);
    digitalWrite(L_rev,LOW);
    Setpoint_L = -sp;
  }
  else if(sp>0){
    digitalWrite(L_fwd,LOW);
    digitalWrite(L_rev,HIGH);
    Setpoint_L = sp;
  }
}
void mstop( void ){
  digitalWrite(L_fwd,LOW);
  digitalWrite(L_rev,LOW);
  Setpoint_L = 0;
  digitalWrite(R_fwd,LOW);
  digitalWrite(R_rev,LOW);
  Setpoint_R = 0;
}


//--------------------PID Functions & Interrupts-------------------
void int_count1(){
  RightCounter++;
}

void int_count2(){
  LeftCounter++;
}

void int_timer(){
  Timer1.detachInterrupt();
  RightSpeed = (RightCounter/PPR)*60.0;
  RighrRPM.Compute();
  analogWrite (RightMotor , RightPWM );
  RightCounter = 0;
  LeftSpeed = (LeftCounter/PPR)*60.0 ;
  LeftRPM.Compute();
  analogWrite(LeftMotor , LeftPWM );
  LeftCounter = 0;
  Timer1.attachInterrupt(int_timer);
}
