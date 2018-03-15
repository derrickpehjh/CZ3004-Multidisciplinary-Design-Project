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
#define small_delay_between_moves 30
#define small_delay_before_reading_sensor 50
#define small_delay_after_reading_sensor 20
#define small_delay_after_align 50


//---------------Declare variables--------------------------
String stringToSend, command;
char character;
volatile int encoder_R_value = 0 , encoder_L_value = 0 ;
int stringIndex, tickError, error, grids = 0;
bool explorationEnabled, fastestPathEnabled, rotateCheck = false;


void setup()
{
  pinMode(motor_encoder_right, INPUT);
  pinMode(motor_encoder_left, INPUT);
  enableInterrupt(motor_encoder_left, LeftEncoderInc, CHANGE);
  enableInterrupt(motor_encoder_right, RightEncoderInc, CHANGE);
  Serial.begin(115200); //Enable the serial comunication
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
            delay(small_delay_between_moves);
//            gotWallThenAlign();
            moveForward();
            gotWallThenAlign();
            break;
          }

        case 'L':               //Command to move rotate left 90 degrees
          {
            delay(small_delay_between_moves);
            rotateCheck = true;
            gotWallThenAlign();
            rotateLeft();
            rotateCheck = false;
            break;
          }

        case 'R':                 //Command to move rotate right 90 degrees     //turns right when Front = 1 + Left=1 (LF=1 & LB =1), so cannot turn left as is default action, can only turn right. In this case, then yes, go tLeft wall that can check, so check alignment to front & left first before turning. Then aft turning depending on how much/deg of adjustment, align to wall again if not correct/accurate.
          {
            delay(small_delay_between_moves);
            rotateCheck = true;
            gotWallThenAlign();
            rotateRight();
            gotWallThenAlign();
            rotateCheck = false;
            break;
          }

        case 'C':               //Command to callibrate robot before/after
          {
            calibrate();
            break;
          }
        case 'A':                     //Read Sensors
          {
            Serial.println(getMedianDistance(0, 1080));
            Serial.println(getMedianDistance(1, 1080));
            Serial.println(getMedianDistance(2, 1080));
            Serial.println(getMedianDistance(3, 1080));
            Serial.println(getMedianDistance(4, 1080));
            Serial.println(getMedianDistance(5, 20150));
            readAllSensors();
            break;
          }

        default:
          {
            break;
          }
      }
      stringIndex++;
      grids = 0;
    }
    stringIndex = 0;
    command = "";
  }
}

void moveForward()
{
  forward(25, 100);
  forward(405, 380);
  md.setBrakes(400, 400);
  forwardCorrection(510); //510 original
  Serial.print(encoder_L_value);
  Serial.print(" : ");
  Serial.println(encoder_R_value);
}

void forward(int value, int Speed)
{
  resetEncoderValues();
  if (Speed <= 100) {
    while ( encoder_R_value < value || encoder_L_value < value ) {
      tickError = 1 * tuneWithPID2();
      md.setSpeeds(-(Speed - tickError), Speed + tickError); //lower the right motor speed
    }
  }
  else {
    while ( encoder_L_value < value || encoder_R_value < value ) {
      tickError = 3 * tuneWithPID();
      md.setSpeeds(-(370 + tickError ), 379 - tickError ); //lower the right motor speed 397
    }
  }
}

void backward(int value, int Speed) {
  resetEncoderValues();
  while ( encoder_R_value < value || encoder_L_value < value ) {
    tickError = 2 * tuneWithPID();
    md.setSpeeds(Speed + tickError, -(Speed - tickError));
  }
}

void forwardCorrection(int practicalValue) {
  int Speed = 80;
  while ( encoder_R_value < practicalValue && encoder_L_value < practicalValue ) {
    tickError = 1 * tuneWithPID2();
    md.setSpeeds(-(Speed - tickError), Speed + tickError);
    md.setBrakes(400, 400);
  }
  md.setBrakes(400, 400);
  while (encoder_L_value < practicalValue ) {
    md.setSpeeds(-Speed, 0);
    md.setBrakes(400, 400);
  }
  md.setBrakes(400, 400);
  while (encoder_R_value < practicalValue) {
    md.setSpeeds(0, Speed);
    md.setBrakes(400, 400);
  }
}

void rotateLeft()
{
  left(620, 380);   //620 380 ORIGINAL
  md.setBrakes(400, 400);
  turnLeftCorrection(741); //741 ORIGINAL
  delay(small_delay_between_moves);
  forward(10, 80);
  md.setBrakes(400, 400);
}

void rotateRight()              //for exploration
{
  right(647, 380); //647 380 ORIGINAL
  md.setBrakes(400, 400);
  turnRightCorrection(736);  //736 ORIGINAL
  delay(small_delay_between_moves);
  forward(10, 80);
  md.setBrakes(400, 400);

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
    md.setBrakes(400, 400);
  }

  while (encoder_L_value < practicalValue ) {
    md.setSpeeds(Speed, 0);
    md.setBrakes(400, 400);
  }

  while (encoder_R_value < practicalValue) {
    md.setSpeeds(0, Speed);
    md.setBrakes(400, 400);
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
    md.setBrakes(400, 400);
  }

  while (encoder_L_value < practicalValue ) {
    md.setSpeeds(-Speed, 0);
    md.setBrakes(400, 400);
  }

  while (encoder_R_value < practicalValue) {
    md.setSpeeds(0, -Speed);
    md.setBrakes(400, 400);
  }
}

double tuneWithPID() {
  error = encoder_R_value - encoder_L_value;
  return error;
}

double tuneWithPID2() {
  error = encoder_L_value - encoder_R_value;
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
  RunningMedian samples = RunningMedian(21);             //take 20 samples of sensor reading
  for (int i = 0; i < 20; i ++)
    samples.add(readSensor(IRpin, model));           //samples call readSensor() to read in sensor value

  int median = samples.getMedian();

  if (IRpin == front_right_sensor_pin)
  {
    median = median - 5;
    if (median <= 14) return median - 1;
    else if (median <= 23) return median;
    else if (median <= 33) return median + 3;
  }

  if (IRpin == front_middle_sensor_pin)
  {
    median = median - 5;
    if (median <= 14) return median - 1;
    else if (median <= 23) return median - 1;
    else if (median <= 33) return median ;
  }

  if (IRpin == front_left_sensor_pin)
  {
    median = median - 5;
    if (median <= 14) return median - 1;
    else if (median <= 23) return median;
    else if (median <= 33) return median + 3;
  }

  if (IRpin == left_front_sensor_pin)
  {
    median = median - 5;
    if (median <= 14) return median - 2;
    else if (median <= 23) return median + 1;
    else if (median <= 33) return median;
  }

  if (IRpin == left_back_sensor_pin)
  {
    median = median - 5;
    if (median <= 14) return median - 1;
    else if (median <= 23) return median - 1;
    else if (median <= 33) return median + 2;
  }

  if (IRpin == right_front_long_range_sensor_pin)
  {
    median = median - 9;
    if (median <= 16) return median + 4; //20
    else if (median <= 27) return median + 4; //30
    else if (median <= 35) return median + 6; //40
    else if (median <= 38) return median + 13; //50
    else if (median <= 45) return median + 18; //60
  }
  return median;
}

int getObstacleGridsAway(int pin, int type)
{
  int distance = getMedianDistance(pin, type);
  if (type == 1080)
  {
    if (distance < 13)          return 1;
    else if (distance < 23)   return 2;
    else if (distance < 33)   return 3;
    else return 0;
  }
  else
  {
    if (distance < 15) return 1;
    else if (distance < 25)   return 2;
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
  md.setBrakes(400, 400);
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
  {
    return false;
  }
}

void gotWallThenAlign() {      //function returns true if can align front or side, false if cannot align

  if (sideCanAlign() || frontCanAlign()) {
    if (sideCanAlign()) {
      alignSideAngle();
    }
    if (frontCanAlign()) {
      alignFrontAngle();
    }
    delay(small_delay_after_align);
  }


}

void alignSideAngle() { //align left using motor

  int Speed = 80; //speed that you want it to move left / right while adjusting
  int sensorError;
  int sensorErrorAllowance = 1;
  resetEncoderValues();
  while ( ( (sensorError = getMedianDistance(left_front_sensor_pin, 1080) - getMedianDistance(left_back_sensor_pin, 1080) ) <= -sensorErrorAllowance)) { //robot tilted left, turn right until acceptable error angle
    tickError = 2 * tuneWithPID();
    md.setSpeeds(-(Speed + tickError), -(Speed - tickError));   //turn right
  }
  md.setBrakes(400, 400);
  delay(small_delay_between_moves);
  resetEncoderValues();
  while ( ( sensorError = getMedianDistance(left_front_sensor_pin, 1080) - getMedianDistance(left_back_sensor_pin, 1080) ) >= sensorErrorAllowance ) { //robot tilted right, turn left until acceptable error angle
    tickError = 2 * tuneWithPID();
    md.setSpeeds( Speed + tickError, (Speed - tickError));    //turn left
  }
  md.setBrakes(400, 400);
  delay(small_delay_between_moves);
  if ( rotateCheck == false) {
    if ((getMedianDistance(left_front_sensor_pin, 1080) < 5 && getMedianDistance(left_back_sensor_pin, 1080) < 5) ||
        ((getMedianDistance(left_front_sensor_pin, 1080) > 6 && getMedianDistance(left_back_sensor_pin, 1080) > 6) && getMedianDistance(left_front_sensor_pin, 1080) < 10 && getMedianDistance(left_back_sensor_pin, 1080) < 10))
    {
      rotateLeft();
      delay(small_delay_before_reading_sensor);
      alignFrontAngle();
      delay(small_delay_after_align);
      rotateRight();
      delay(small_delay_between_moves);
      backward(10, 80);
    }
  }
  md.setBrakes(400, 400);
}



void alignFrontAngle() {
  int Speed = 80;
  int sensorError;
  int sensorErrorAllowance = 0;
  resetEncoderValues();
  while (sensorError = getMedianDistance(front_left_sensor_pin, 1080) - getMedianDistance(front_right_sensor_pin, 1080) > sensorErrorAllowance)
  {
    tickError = 2 * tuneWithPID();
    md.setSpeeds(-(Speed + tickError), -(Speed - tickError));
  }
  md.setBrakes(400, 400);
  delay(small_delay_between_moves);
  resetEncoderValues();
  while ( sensorError = getMedianDistance(front_left_sensor_pin, 1080) - getMedianDistance(front_right_sensor_pin, 1080 ) < sensorErrorAllowance ) { //robot tilted right, turn left until acceptable error angle
    tickError = 2 * tuneWithPID();
    md.setSpeeds( Speed + tickError, (Speed - tickError));    //turn left
  }
  md.setBrakes(400, 400);
  delay(small_delay_between_moves);
  while (getMedianDistance(front_middle_sensor_pin, 1080) < 6)
  {
    backward(1, 80);
  }
  md.setBrakes(400, 400);
  delay(small_delay_between_moves);
  while (getMedianDistance(front_middle_sensor_pin, 1080) > 6 && getMedianDistance(front_middle_sensor_pin, 1080) < 10)

  {
    forward(1, 80);
  }
  md.setBrakes(400, 400);


}
