// Latest Version of PIDDYBOT Self Balancing Program 1/14/14
// Program Written and Pieced together by Sean Hodgins.
// The program was morphed from many programs.
// With that being said If you feel you see something that
// is your work and want credit, feel free to contact me.
// Http://Idlehandsproject.com
// This is free to be shared, altered, and used. 
// It is in no way "Finished".
// Find the Second target angle and tune for your bot, it may be different.
// LIBRARIES
#include <Wire.h>
#include <FreeSixIMU.h>
#include <FIMU_ADXL345.h>
#include <FIMU_ITG3200.h>

FreeSixIMU sixDOF = FreeSixIMU();

int STBY = 10; //Motor Standby Pin

//Motor A
int PWMA = 3; //Speed control 
int AIN1 = 9; //Direction
int AIN2 = 8; //Direction

//Motor B
int PWMB = 5; //Speed control
int BIN1 = 11; //Direction
int BIN2 = 12; //Direction

// Potentiometers
static int pot1 = A0;
static int pot2 = A1;
static int pot3 = A2;

float distancecount = 0;
float correction = 0;
float returnto = 0;
float angleAdjust;
int prevforward = 0;
int moveDir = 1;

// PID CONSTANTS
float Kp = 0; 
float Ki = 0;  
float Kd = 0;  

const int AvgAngles = 3;
 float prevTargetAngle = 0;
 float targetAngle = 0;


float angles[5];

float currAngle, prevAngle;
float prevAngles[AvgAngles];
int prevAngleI = 0;
int motorSpeed;
float rotate = 1;
int forward = 0;
int coursetiming = 0;
int dir = 0;
// time vars
int currTime = 0; 
int prevTime = 0; 



float errorSum = 0;
float currError = 0;
float prevError = 0;
float iTerm = 0;
float dTerm = 0;
float pTerm = 0;

//Location PID CONTROL - These are the PID control for the robot trying to hold its location.
  float Lp = 0.5;
  float Li = 0.05;
  float Ld = 0.4;
  float offsetLoc = 0;
  float pT,iT,dT = 0;
  float errorS = 0;
  float prevE = 0;

void setup() {
  //MOTOR SETUP
  pinMode(STBY, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  
  pinMode(13, OUTPUT);
  
  // Serial Connection
  Serial.begin(9600);

  // IMU Connection
  Wire.begin(); 

  delay(5);
  sixDOF.init(); //Begin the IMU
  delay(5);

}


void loop() {
  //testMotors(); //This can be used to see the speed of your motors and which motor is 1 and which is 2. 
  //movement(); //Sub-routine used to program paths for robot. See Movement below. 
  targetAngle = 2; //Change this angle to your BOT! Its the "Zero" angle it will be slightly different for each. Basically the tipping angle of the bot.
  
  //Serial.println(correction);
  targetAngle = targetAngle + correction; //This is how you can make it move, by adding or subtracting the target angle will make it want to moveforward or backwards. 
  
  updateAngle();
  
  Kp = map(analogRead(pot1), 0, 1023, 0, 5000);  //Tuning Pots for P I and D term.
  Ki = map(analogRead(pot2), 0, 1023, 0, 500);
  Kd = map(analogRead(pot3), 0, 1023, 0, 5000);
  Kp = Kp / 100;
  Ki = Ki / 100;
  Kd = Kd / 100;
  
  /*Serial.print("P term: ");
  Serial.print(Kp);
  Serial.print("I term: ");
  Serial.print(Ki);
  Serial.print("D term: ");
  Serial.println(Kd);
  */
  updateSpeed();
  updateMotor();
  updateLocation();
  
}
void testMotors(){
  for (int x=0; x<255; x++){
    Serial.println(x);
    move(1, x, 1, 1); 
    delay(100);
  }
  move(1, 0, 1, 1); 
  for (int y=0; y<255; y++){
    Serial.println(y);
    move(2, y, 1, 1);
    
    delay(100);
  }
}
  
void updateAngle() {
  sixDOF.getYawPitchRoll(angles);
  prevAngles[prevAngleI] = angles[1];
  prevAngleI = (prevAngleI + 1) % AvgAngles;
  float sum = 0;
  for (int i = 0; i < AvgAngles; i++)
      sum += prevAngles[i];
  currAngle = sum / AvgAngles;
  prevAngle = currAngle;
  
}

void updateSpeed() {
  
  float K = 1;
  currError = targetAngle - currAngle;
  pTerm = Kp * currError;
  errorSum += currError;
  errorSum = constrain(errorSum, -200, 200);
  //Serial.println(errorSum);
  iTerm = Ki * errorSum; 
  dTerm = Kd * (currError - prevError);
  prevError = currError;
  motorSpeed = constrain(K*(pTerm + iTerm + dTerm), -255, 255);
  if (currError > 50 || currError < -50)
    motorSpeed = 0;
}

void updateMotor() {

  if (motorSpeed < 0){
    dir = 0;
    
    distancecount =  (motorSpeed+12) * 0.01;
    if (distancecount > 0){
      distancecount = 0;
      
    }
    //correction = correction + distancecount;
  }
  if (motorSpeed > 0){
    dir = 1;
    distancecount = (motorSpeed - 12) * 0.01;
    //correction = correction + distancecount;
    if (distancecount < 0){
      distancecount = 0;
      
    }
  }
   returnto += distancecount;
   if (returnto > 5){
     //forward = 2;
     digitalWrite(13, HIGH);
   }
   if (returnto < 5){
     //forward = -2;
     digitalWrite(13, HIGH);
   }
   if (returnto < 5 && returnto > -5) {
   digitalWrite(13, LOW);
   //forward = 0;
   }
  
  if (motorSpeed < 0){
    motorSpeed = motorSpeed * -1;
  }
  
  move(1, motorSpeed, dir, rotate); 
  move(2, motorSpeed, dir, rotate);
  if (motorSpeed == 0){
    stop();
  }
}


void move(int motor, int speed, int direction, float turn){
//Move specific motor at speed and direction
//motor: 0 for B 1 for A
//speed: 0 is off, and 255 is full speed
//direction: 0 clockwise, 1 counter-clockwise

  digitalWrite(STBY, HIGH); //disable standby
  int leftspeed, rightspeed;
  leftspeed = constrain((speed*turn*1.005) , 0, 255);
  rightspeed = constrain((speed/turn) , 0, 255);
  boolean inPin1 = LOW;
  boolean inPin2 = HIGH;

  if(direction == 1){
    inPin1 = HIGH;
    inPin2 = LOW;
  }

  if(motor == 1){
    digitalWrite(AIN1, inPin1);
    digitalWrite(AIN2, inPin2);
    analogWrite(PWMA, leftspeed);
  }else{
    digitalWrite(BIN1, inPin1);
    digitalWrite(BIN2, inPin2);
    analogWrite(PWMB, rightspeed);
  }
}

void stop(){
//Enable Standby  
  digitalWrite(STBY, LOW); 
}

void updateLocation(){
  
  
  offsetLoc = constrain(returnto, -10, 10);
  pT = Lp * offsetLoc;
  errorS += offsetLoc;
  errorS = constrain(errorS, -200, 200);
  //Serial.println(errorS);
  iT = Li * errorSum; 
  dT = Ld * (offsetLoc - prevE);
  prevE = offsetLoc;
  correction = constrain((pT + iT + dT), -7, 7);
  
}

//You can Experiment with setting paths for the robot by changing either the returnto value so it thinks
//its far away from its target or by adding a variable such a "forward" below.
//If uncommented above this is a simple way of making the robot move forward about 10cm
//then back 10cm in some arbitrary number of time.
void movement(){
  
  //int forward = 0;
  
  if (coursetiming == 500){
    returnto = -50 * moveDir;
    rotate = 1;
  }
  if (coursetiming > 1200){
    coursetiming = 499; 
    moveDir *= -1;
  }
  forward = 0.05*(forward) + 0.95*(prevforward);
  prevforward = forward;
  coursetiming = coursetiming + 1;
  Serial.println(offsetLoc);
}
  
  
  
