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
#define small_delay_before_reading_sensor 200
#define small_delay_between_moves 100
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
  S -> Starts Exploration
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

            case 'C':               //Command to callibrate robot before starting movement
                  {                 //Make sure Robot face west to align, turn right to
                    calibrateBeforeStart();
                    break;
                  }

            case 'X':                     //Command to start with fastest path
                  {
                    fastestPathEnabled = true;
                    explorationEnabled = false; 
                    break;
                  }

            case 'S':                     //Command to start with exploration
                  { 
                    explorationEnabled = true;
                    fastestPathEnabled = false;                      
                    readAllSensors();
                    break;
                  }
            case 'A':                     //Command for checklist(rpie/arduino)
                  {
                    //Serial.println("B");
                    //getDistanceFromRobot();
                    break;
                  }

            default:
                  {
                    Serial.println("from arduino:invalid command");
                    break;
                  }
        }
       stringIndex++;
      }                       
    stringIndex = 0;
    command = "";
  }
   fastestPathEnabled = false;
   explorationEnabled = false;
}

void moveForward()              
{  
  forward(525,400);
  md.setBrakes(400,400);
  if (explorationEnabled == true)
  {
    delay(small_delay_between_moves);
    gotWallThenAlign();
    delay(small_delay_between_moves);
    readAllSensors(); 
  }
}

void rotateLeft()              
{  
  left(725,150);
  md.setBrakes(400,400);     
  if (explorationEnabled == true)
  {
    delay(small_delay_between_moves);
    gotWallThenAlign();
    delay(small_delay_between_moves);
    readAllSensors(); 
  }
}

void rotateRight()              //for exploration
{  
  right(725,150);
  md.setBrakes(400,400);     
  if (explorationEnabled == true)
  {
    delay(small_delay_between_moves);
    gotWallThenAlign();
    delay(small_delay_between_moves);
    readAllSensors(); 
  }
}

void forward(int value, int Speed)
{
    resetEncoderValues();
    while ( encoder_R_value < value || encoder_L_value < value ) {    //run until either one wheel reaches the tick
    tickError = 4 * tuneWithPID();
    md.setSpeeds(-(Speed + tickError), Speed - tickError);
    }
}

void left(int value, int Speed)
{
    resetEncoderValues();
    while ( encoder_R_value < value || encoder_L_value < value ) {    //run until either one wheel reaches the tick
    tickError = 4 * tuneWithPID();
    md.setSpeeds(Speed + tickError, Speed - tickError);
    }
}

void right(int value, int Speed)
{
    resetEncoderValues();
    while ( encoder_R_value < value || encoder_L_value < value ) {    //run until either one wheel reaches the tick
    tickError = 4 * tuneWithPID();
    md.setSpeeds(-(Speed + tickError), -(Speed - tickError));
    }
}

double tuneWithPID() {
  error = encoder_R_value - encoder_L_value;
  //Serial.print("error: ");
  //Serial.println(error);
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
    distance = 104067 * pow(sensorValue, -1.37363); //20-150cm
  return distance;
}

float getMedianDistance(int IRpin, int model) 
{         
  RunningMedian samples = RunningMedian(9);             //take 9 samples of sensor reading
  for (int i = 0; i < 9; i ++)
   samples.add(readSensor(IRpin, model));           //samples call readSensor() to read in sensor value

 int median = samples.getMedian();

  if(IRpin == front_right_sensor_pin)
    {
      median = median -5;
      if(median <= 14) return median-1;
      else if(median <= 23) return median-1;      
      else if(median <= 33) return median;  
      else if(median <= 43) return median+2;
      else if(median <= 53) return median+4;
    }
  
  if(IRpin == front_middle_sensor_pin)
    {
      median = median -5;
      if(median <= 14) return median-2;
      else if(median <= 23) return median-2;
      else if(median <= 33) return median;
      else if(median <= 43) return median; 
      else if(median <= 53) return median;
    }
  
  if(IRpin == front_left_sensor_pin)
    {
      median = median -5;
      if(median <= 14) return median-1;
      else if(median <= 23) return median; 
      else if(median <= 33) return median;
      else if(median <= 43) return median+2; 
      else if(median <= 53) return median;      
    }

  if(IRpin == left_front_sensor_pin)
    {
      median = median -5;
      if(median <= 14) return median;
      else if(median <= 23) return median; 
      else if(median <= 33) return median+3;
      else if(median <= 43) return median+10; 
      else if(median <= 53) return median + 19;      
    }

   if(IRpin == left_back_sensor_pin)
    {
      median = median -5;
      if(median <= 14) return median;
      else if(median <= 23) return median; 
      else if(median <= 33) return median+2;
      else if(median <= 43) return median; 
      else if(median <= 53) return median;      
    }
  
  if(IRpin == right_front_long_range_sensor_pin)
    {
      median = median -9;
      if(median <= 23) return median+1;
      else if(median <= 34) return median-2; 
      else if(median <= 42) return median;
      else if(median <= 47) return median + 7; 
      else if(median <= 58) return median + 10; 
      else if(median <= 70) return median + 3; 
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
    else if (distance < 35)   return 3;          
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

void calibrateBeforeStart() { 
       gotWallThenAlign(); 
       right(725,150);
       md.setBrakes(400,400);
       delay(small_delay_between_moves);
       readAllSensors();                                                                             
       delay(small_delay_between_moves);
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

boolean gotWallThenAlign() {      //function returns true if can align front or side, false if cannot align
  if (sideCanAlign() || frontCanAlign()) {
    if (sideCanAlign()) {
      alignSideAngle();
//      alignmentCount = 0;
      delay(small_delay_after_align);
    }
    if (frontCanAlign()) {
      alignFrontAngle();
//      alignmentCount = 0;
      delay(small_delay_after_align);
    }
    return true;
  }

  else {
    return false;
  }
}


void alignSideAngle() { //align left using motor

}

void alignFrontAngle() { //align front using motor

  
}



















//For checklist To get distance w.r.t robot (not sensor)
//First minus = offset from sensor to robot
//Second minus == sensor inaccuracy offset
void getDistanceFromRobot()
{ 
  int distance0 = getMedianDistance(front_right_sensor_pin,1080);
  Serial.print("Distance for front_right_sensor: ");
  Serial.println(distance0);
  
  int distance1 = getMedianDistance(front_middle_sensor_pin,1080);
  Serial.print("Distance for front_middle_sensor: ");
  Serial.println(distance1);
  
  int distance2 = getMedianDistance(front_left_sensor_pin,1080);
  Serial.print("Distance for front_left_sensor: ");
  Serial.println(distance2);

  
  int distance3 = getMedianDistance(left_front_sensor_pin,1080);
  Serial.print("Distance for left_front_sensor: ");
  Serial.println(distance3);
  
  int distance4 = getMedianDistance(left_back_sensor_pin,1080);
  Serial.print("Distance for left_back_sensor: ");
  Serial.println(distance4);
  

  int distance5 = getMedianDistance(right_front_long_range_sensor_pin,20150);
  Serial.print("Distance for right_front_long_range_sensor: ");
  Serial.println(distance5);
   
   delay(2000);
}
