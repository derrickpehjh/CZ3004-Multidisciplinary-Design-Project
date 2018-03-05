//import the library in the sketch
#include <DualVNH5019MotorShield.h>
#include <RunningMedian.h>
#include <EnableInterrupt.h>

DualVNH5019MotorShield md;
#define motor_encoder_left 3      //left motor
#define motor_encoder_right 11    //right motor

//the model of the short range sensor is "GP2Y0A21YK0F"
//The model of the long range sensor is "GP2Y0A20YK0F"

#define front_right_sensor_pin 0  //MDP BOARD PIN PS1
#define front_middle_sensor_pin 1  //MDP BOARD PIN PS2
#define front_left_sensor_pin 2  //MDP BOARD PIN PS3
#define left_front_sensor_pin 3  //MDP BOARD PIN PS4
#define left_back_sensor_pin 4  //MDP BOARD PIN PS5
#define right_front_long_range_sensor_pin 5 //MDP BOARD PIN PS6
#define motor_encoder_left 3      //left motor
#define motor_encoder_right 11    //right motor
#define small_delay_before_reading_sensor 100
#define small_delay_between_moves 30
#define small_delay_after_align 20


//---------------Declare variables--------------------------
String stringToSend, command;
char character;
volatile int encoder_R_value = 0 , encoder_L_value = 0 ;
int stringIndex, tickError, error, grids = 0;
bool explorationEnabled, fastestPathEnabled = false;


void setup()
{
  pinMode(motor_encoder_right, INPUT);
  pinMode(motor_encoder_left, INPUT);
  enableInterrupt(motor_encoder_left, LeftEncoderInc, CHANGE);
  enableInterrupt(motor_encoder_right, RightEncoderInc, CHANGE);
  Serial.begin(9600); //Enable the serial comunication
  md.init();
}


/*----------------------------------------------
  Commands from ALGO/RPI to Arduino:
  F -> Move Forward                  
  L -> Rotate Left
  R -> Rotate Right
  C -> Calibrate Robot before start(exploration/fastest path)
  X -> Starts Fastest Path                        
  Z -> Starts Exploration
  A -> Checklists evaluation
  
  ----------------------------------------------*/


void loop()
{
  while (Serial.available())  
  {
    character = Serial.read();
    if (character == '\n' || character == '\0')       //Meaning end of line / no new line and indicates no further input received
      break;
    else 
      command += character;                 //Else read in new commands received.
  }

  while (command.length() > stringIndex)        //If there are any valid comments, perform/execute- operations on it.
  {         
      while (command[stringIndex] != '\n' && command[stringIndex] != '\0') //While not end of string
      {  
        switch (command[stringIndex])         //Switch-case multiple scenarios, handle diff poss scenario inputs.
        {           
            case 'F':                 //Command to move forward 1 grid 
                  {
//                    while (command[stringIndex] == 'F')
//                    { stringIndex++; grids++; } 
                    moveForward();
                    break;  
                  }

            case 'L':               //Command to move rotate left 90 degrees
                  {
                    rotateLeft();
                    break;
                  }

            case 'R':                 //Command to move rotate right 90 degrees   
                  { 
                    rotateRight();
                    break;
                  }

            case 'C':               //Command to callibrate robot before/after
                  {                
                    calibrate();
                    break;
                  }
            case 'A':                     //Read Sensors
                  {
//                    int a = getMedianDistance(0,1080);
//                     int b = getMedianDistance(1,1080);
//                      int c = getMedianDistance(2,1080);
//                       int d = getMedianDistance(3,1080);
//                        int e = getMedianDistance(4,1080);
//                         int f = getMedianDistance(5,20150);
//                         Serial.println(a);
//                         Serial.println(b);
//                         Serial.println(c);
//                         Serial.println(d);
//                         Serial.println(e);
//                         Serial.println(f);
                    readAllSensors();
                    break;
                  }

            default:
                  {
                    Serial.println("from arduino:invalid command");
                    break;
                  }
        }
       stringIndex++;
       grids=0;
      }                       
    stringIndex = 0;
    command = "";
  }
}

void moveForward()              
{  
  forward(25, 100);
  forward(450, 400);
  md.setBrakes(400,400);
  forwardCorrection(575);
  gotWallThenAlign();
//  Serial.print("left");
//  Serial.println(encoder_L_value);
//  Serial.print("right");
//  Serial.println(encoder_R_value);
}

void forwardCorrection(int practicalValue) {
  int Speed = 80;
  while ( encoder_R_value < practicalValue && encoder_L_value < practicalValue ) {    
    tickError = 5 * tuneWithPID();
    md.setSpeeds(-(Speed + tickError), Speed - tickError);
    delay(10);  
    md.setBrakes(400, 400);
    delay(5);
  }
  md.setBrakes(400, 400);
  while (encoder_L_value < practicalValue ) {
    md.setSpeeds(-Speed, 0);
    delay(10);  
    md.setBrakes(400, 400);
    delay(5);
  }
  md.setBrakes(400, 400);
  while (encoder_R_value < practicalValue) {
    md.setSpeeds(0, Speed);
    delay(10);  
    md.setBrakes(400, 400);
    delay(5);
  }
}

void rotateLeft()              
{  
  left(647, 400);   //695 at lower voltage
  md.setBrakes(400, 400);
  turnLeftCorrection(785);    
  gotWallThenAlign();
}

void rotateRight()              //for exploration
{  
  right(644, 400);   //695 at lower voltage
  md.setBrakes(400, 400);
  turnRightCorrection(776);    
  gotWallThenAlign();
}

void forward(int value, int Speed)
{
    resetEncoderValues();
    while ( encoder_R_value < value || encoder_L_value < value ) {    
    tickError = 5 * tuneWithPID();
    md.setSpeeds(-(Speed + tickError), Speed - tickError);
    }
}

void left(int encoderValue, int Speed) {
  resetEncoderValues();
  while (encoder_R_value < encoderValue || encoder_L_value < encoderValue) {
    tickError = 5 * tuneWithPID();
    md.setSpeeds((Speed + tickError), (Speed - tickError));
  }
}

void turnLeftCorrection(int practicalValue) {
  int Speed = 80;

  while ( encoder_R_value < practicalValue && encoder_L_value < practicalValue ) {    
    tickError = 3 * tuneWithPID();
    md.setSpeeds(Speed + tickError, (Speed - tickError) );
    delay(10);  
    md.setBrakes(400, 400);
    delay(5);
  }

  while (encoder_L_value < practicalValue ) {
    md.setSpeeds(Speed, 0);
    delay(10); 
    md.setBrakes(400, 400);
    delay(5);
  }

  while (encoder_R_value < practicalValue) {
    md.setSpeeds(0, Speed);
    delay(10); //20
    md.setBrakes(400, 400);
    delay(5);
  }
}

void right(int encoderValue, int Speed) {
  resetEncoderValues();
  while (encoder_R_value < encoderValue || encoder_L_value < encoderValue) {
    tickError = 5 * tuneWithPID();
    md.setSpeeds(-(Speed + tickError), -(Speed - tickError));
  }
}

void turnRightCorrection(int practicalValue) {
  int Speed = 80;

  while ( encoder_R_value < practicalValue && encoder_L_value < practicalValue ) {    
    tickError = 3 * tuneWithPID();
    md.setSpeeds(-(Speed + tickError), -(Speed - tickError) );
    delay(10);  
    md.setBrakes(400, 400);
    delay(5);
  }

  while (encoder_L_value < practicalValue ) {
    md.setSpeeds(-Speed, 0);
    delay(10); 
    md.setBrakes(400, 400);
    delay(5);
  }

  while (encoder_R_value < practicalValue) {
    md.setSpeeds(0, -Speed);
    delay(10); //20
    md.setBrakes(400, 400);
    delay(5);
  }
}

double tuneWithPID() {
  error = encoder_R_value - encoder_L_value;
  return error;
}

void LeftEncoderInc() {
  encoder_L_value++;
}

void RightEncoderInc() {
  encoder_R_value++;
}

void resetEncoderValues() {
  encoder_R_value = 0;
  encoder_L_value = 0;
}

float readSensor(int IRpin, int model) 
{
  float sensorValue = analogRead(IRpin);
  float distance;
  if (model == 1080)  //for 10-80cm sensor
    distance = 4468.9 * pow(sensorValue, -0.959); // 10-80cm  ~ formula
  if (model == 20150) //for 20-150cm sensor
    distance =  105812 * pow(sensorValue, -1.39275766016713); //20-150cm
  return distance;
}

float getMedianDistance(int IRpin, int model) 
{         
  RunningMedian samples = RunningMedian(9);             //take 9 samples of sensor reading
  for (int i = 0; i < 20; i ++)
   samples.add(readSensor(IRpin, model));           //samples call readSensor() to read in sensor value

 int median = samples.getMedian();

  if(IRpin == front_right_sensor_pin)
    {
      median = median -5;
      if(median <= 14) return median-1;
      else if(median <= 23) return median;      
      else if(median <= 33) return median+3;  
    }
  
  if(IRpin == front_middle_sensor_pin)
    {
      median = median -5;
      if(median <= 14) return median-2;
      else if(median <= 23) return median-1;
      else if(median <= 33) return median ;
    }
  
  if(IRpin == front_left_sensor_pin)
    {
      median = median -5;
      if(median <= 14) return median-1;
      else if(median <= 23) return median; 
      else if(median <= 33) return median+3;    
    }

  if(IRpin == left_front_sensor_pin)
    {
      median = median -5;
      if(median <= 14) return median-2;
      else if(median <= 23) return median-1;
      else if(median <= 33) return median+1;     
    }

   if(IRpin == left_back_sensor_pin)
    {
      median = median -5;
      if(median <= 14) return median-1;
      else if(median <= 23) return median; 
      else if(median <= 33) return median+2;     
    }
  
  if(IRpin == right_front_long_range_sensor_pin)
    {
      median = median -9;
      if(median <= 16) return median +4;  //20
      else if(median <= 27) return median +4;  //30
      else if(median <= 35) return median +6;  //40
      else if(median <= 38) return median +13;  //50
      else if(median <= 45) return median +18;  //60
      
    }
  return median;
}

int getObstacleGridsAway(int pin, int type) 
{
  int distance = getMedianDistance(pin, type);
  if(type == 1080)
  {
  if (distance < 15)        return 1;            
    else if (distance < 25)   return 2;         
    else if (distance < 33)   return 3;          
    else return 0;                    
  }
  else
  {
    if (distance < 25)        return 2;            
    else if (distance < 35)   return 3;         
    else if (distance < 45)   return 4;   
    else if (distance < 55)   return 5;
    else if (distance < 65)   return 6;       
    else return 0;                    
  }
  }

//FR:FM:FL:LF:LB:R
void readAllSensors() 
{      
  delay(small_delay_before_reading_sensor);
  stringToSend += getObstacleGridsAway(front_right_sensor_pin, 1080);
  stringToSend += ":";
  stringToSend += getObstacleGridsAway(front_middle_sensor_pin, 1080);
  stringToSend += ":";
  stringToSend += getObstacleGridsAway(front_left_sensor_pin, 1080);
  stringToSend += ":";
  stringToSend += getObstacleGridsAway(left_front_sensor_pin, 1080);
  stringToSend += ":";
  stringToSend += getObstacleGridsAway(left_back_sensor_pin, 1080);
  stringToSend += ":";
  stringToSend += getObstacleGridsAway(right_front_long_range_sensor_pin, 20150);
  Serial.println(stringToSend);
  stringToSend = "";
}

void calibrate() { 

       gotWallThenAlign(); 
       md.setBrakes(400,400);                                                         
}

boolean frontCanAlign() {
  if ( (getObstacleGridsAway(front_left_sensor_pin, 1080) == 1) && (getObstacleGridsAway(front_right_sensor_pin, 1080) == 1) )
    return true;
  else
    return false;
}

boolean sideCanAlign() {
  if ( (getObstacleGridsAway(left_front_sensor_pin, 1080) == 1) && (getObstacleGridsAway(left_back_sensor_pin, 1080) == 1) )
    return true;
  else
    return false;
}

void gotWallThenAlign() {      //function returns true if can align front or side, false if cannot align
  if (sideCanAlign() || frontCanAlign()) {
    if (sideCanAlign()) {
      alignSideAngle();
    }
    if (frontCanAlign()) {
      alignFrontAngle();
    }
  }
}

void alignSideAngle() { //align left using motor
    int Speed = 80; //speed that you want it to move left / right while adjusting
    int sensorError;
    int sensorErrorAllowance = 1;
  resetEncoderValues();
  while ( ( sensorError = getMedianDistance(left_front_sensor_pin, 1080) - getMedianDistance(left_back_sensor_pin, 1080) ) <= -sensorErrorAllowance ) { //robot tilted left, turn right until acceptable error angle
    tickError = 3 * tuneWithPID();
    md.setSpeeds(-(Speed + tickError), -(Speed - tickError));   //turn right
  }
  md.setBrakes(400,400);
  resetEncoderValues();
  while ( ( sensorError = getMedianDistance(left_front_sensor_pin, 1080) - getMedianDistance(left_back_sensor_pin, 1080) ) >= sensorErrorAllowance ) { //robot tilted right, turn left until acceptable error angle
    tickError = 3 * tuneWithPID();
    md.setSpeeds( Speed + tickError, (Speed - tickError));    //turn left
  }
  md.setBrakes(400, 400);

//    while (getMedianDistance(left_back_sensor_pin,1080) <10)
//    { 
//    right(25,400);
//    md.setBrakes(400,400);
//    Serial.println("a");
//    }
//
//    while (getMedianDistance(left_back_sensor_pin,1080) >10)
//    { 
//    left(25,400);
//    md.setBrakes(400,400);
//    Serial.println("a");
//    }
// 
//
//    while(getMedianDistance(left_front_sensor_pin,1080) >10)
//    {
//      left(25,400);
//      md.setBrakes(400,400);
//      Serial.println("q");
//    }
//
//    while(getMedianDistance(left_front_sensor_pin,1080) <10)
//    {
//      right(25,400);
//      md.setBrakes(400,400);
//      Serial.println("q");
//    }
}

//Need to adjust to find exact encoder and distance
void alignFrontAngle() { //align front using motor

  while (getMedianDistance(front_middle_sensor_pin,1080) > 10)
  {
    forward(25, 100);
  }
   md.setBrakes(400,400); 
}



