


/*
 * getDistance
 *
 * Example of using SharpIR library to calculate the distance beetween the sensor and an obstacle
 *
 * Created by Giuseppe Masino, 15 June 2016
 * Author URL http://www.facebook.com/dev.hackerinside
 * GitHub URL http://github.com/HackerInside0/Arduino_SharpIR
 *
 * -----------------------------------------------------------------------------------
 *
 * Things that you need:
 * - Arduino
 * - A Sharp IR Sensor
 *
 *
 * The circuit:
 * - Arduino 5V -> Sensor's pin 1 (Vcc)
 * - Arduino GND -> Sensor's pin 2 (GND)
 * - Arduino pin A0 -> Sensor's pin 3 (Output)
 * 
 *
 * See the Sharp sensor datasheet for the pin reference, the pin configuration is the same for all models.
 * There is the datasheet for the model GP2Y0A41SK0F:
 * 
 * http://www.robotstore.it/open2b/var/product-files/78.pdf
 *
 */

//import the library in the sketch
#include <DualVNH5019MotorShield.h>
#include <RunningMedian.h>

DualVNH5019MotorShield md;
#define motor_encoder_left 3      //left motor
#define motor_encoder_right 11    //right motor


//Create a new instance of the library
//Call the sensor "sensor"
//the model of the short range sensor is "GP2Y0A21YK0F"
//The model of the long range sensor is "GP2Y0A20YK0F"

#define front_right_sensor_pin 0  //MDP BOARD PIN PS1
#define front_middle_sensor_pin 1  //MDP BOARD PIN PS2
#define front_left_sensor_pin 2  //MDP BOARD PIN PS3
#define left_front_sensor_pin 3  //MDP BOARD PIN PS4
#define left_back_sensor_pin 4  //MDP BOARD PIN PS5
#define right_front_long_range_sensor_pin 5 //MDP BOARD PIN PS6

int stringIndex;
char character;

String command;


// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  md.init();
}

// the loop routine runs over and over again forever:
void loop() {

  while (Serial.available())  {
    character = Serial.read();
    if (character == '\n' || character == '\0')    // meaning end of line / no new line. Thus indicating no further input received
      break;
    else {
      command += character;     // else read in new commands received.
      delay(2);
    }
  }


  while (command.length() > stringIndex) {      // if there are any valid comments, perform/execute- operations on it.
    while (command[stringIndex] != '\n' && command[stringIndex] != '\0') {  //while not end of string
      switch (command[stringIndex]) {       //switch-case multiple scenarios, handle diff poss scenario inputs from algo.
//        case 'A': //algo sends "AR" infront of string to send message to ARduino, remove the AR.
//          {
//            stringIndex++;
//            break;
//          }

        case 'F':   //command from algo/rpi to arduino to instruct it to move forward.
          {
            stringIndex++;
//            notCalibratedCountNS++;   //start measuring any anomaly/deviation, for calibration to be perform/done ltr?????
//            notCalibratedCountWE++;   //
//            if (command[stringIndex] == '0') {      //first digit is 0
//              stringIndex++;    //index now points to second digit
//              moveForwardGridRamp(command[stringIndex] - 48);       // grid 2-9 for exploration and fastest path      //calls moveForwardGridRamp()
//            }
//            else {            //else first digit is 1
//              stringIndex++;      //index now points to second digit
//              moveForwardGridRamp(10 + command[stringIndex] - 48);    // grid 10-17 for exploration and fastest path    //calls moveForwardGridRamp()
//            }
//            if (!fastestPath) {  //do alignment/calibration if not fastest path
//              calibrateIfExceedTreshold();
//            }
            md.setSpeeds(-200,200);
            while (getObstacleGridsAwayFM() !=1);
            md.setBrakes(400,400);
            break;
          }
        case 'L':
          {
//            turnLeft();
//            facingNorthSouth = !facingNorthSouth;
//            if (!fastestPath) {  //do alignment if not fastest path
//              calibrateIfExceedTreshold();
//            }
//            break;
              md.setSpeeds(200,200);
              delay(1500);
              md.setBrakes(400,400);
          }
//        case 'R':
//          {
//            turnRight();
//            facingNorthSouth = !facingNorthSouth;
//            if (!fastestPath) {   //do alignment if not fastest path
//              calibrateIfExceedTreshold();
//            }
//            break;
//          }
//        case 'C' :    //self_calibrate_on_the_spot
//          {
//            calibrateBeforeExploration();
//            break;
//          }
//        case 'X' :          //X represent fastest path, remaining string should contain all the commands to move
//          {
//            fastestPath = true;
//            break;
//          }
//        case 'Z' :    //ramp_up_accleration. no alignment/calibration performed, just chiong first. Nd counter to check/count num of grids moved.
//          {
//            notCalibratedCountNS++;
//            notCalibratedCountWE++;
//            acceleratedExploration = true;
//            break;
//          }
//        case 'T' :    //move backwards
//          {
//            turnBackward();
//            if (!fastestPath) {   //do alignment if not fastest path
//              calibrateIfExceedTreshold();
//            }
//            break;
//          }
//        case 'S':   //stop
//          {
//            readSensors();
//            break;
//          }
        default :
          {
            Serial.println("from arduino:invalid command");
            break;
          }
      }
      stringIndex++;
//      minusLastForwardValue = 0;
//      minusSecondLastForwardValue = 0;
    } //while not end of string
//
//    fastestPath = false;
//    acceleratedExploration = false;
    stringIndex++;
  } //while not end of string length
  stringIndex = 0;
  command = "";
  //}
}





//
//void setup()
//{
//  Serial.begin(9600); //Enable the serial comunication
////  md.init();
//}
//
//void loop()
//{
////  md.setSpeeds(-400,400);
//  Serial.println("Distance for sensor A2: ");
//  int distance = getMedianDistance(right_front_long_range_sensor_pin,20150);
//  Serial.println(distance); 
////  if (distance<24){
////    md.setBrakes(400,400);
////    delay(5000);
////  }
//
//}
//
  float readSensor(int IRpin, int model) {
  float sensorValue = analogRead(IRpin);
  float distance;
  if (model == 1080)  //for 10-80cm sensor
    distance = 4468.9 * pow(sensorValue, -0.959); // 10-80cm  ~ formula
  if (model == 20150) //for 20-150cm sensor
    distance = 30431 * pow(sensorValue, -1.169); //20-150cm
  return distance;
}
//
//
float getMedianDistance(int IRpin, int model) {   //read each sensor ~5msec
  RunningMedian samples = RunningMedian(9);       //take 9 samples of sensor reading
  for (int i = 0; i < 9; i ++)
    samples.add(readSensor(IRpin, model));    //samples call readSensor() to read in sensor value


  if (IRpin == right_front_long_range_sensor_pin) {       //for model == 20150 "long range sensor"
    if (samples.getMedian() < 35)       return samples.getMedian() + 5;
    else if (samples.getMedian() < 40)  return samples.getMedian() + 6;
    else if (samples.getMedian() < 42)  return samples.getMedian() + 7;
    else if (samples.getMedian() < 48)  return samples.getMedian() + 8;
    else if (samples.getMedian() < 53)  return samples.getMedian() + 9;
    else                                return samples.getMedian() + 10;    // 0.0  =D  so same prob as us w long range reading.
  }
  return samples.getMedian();
}
//  
//
//int getObstacleGridsAwayFL() {   //front left sensor
//  int distance = getMedianDistance(front_left_sensor_pin, 1080);
//  if (distance <= 17)        return 1;    //1 indicates sensor 10cm distance reading.  14cm is from the pov of sensor (4cm inwards) 
//  else if (distance <= 27)   return 2;
//  else return 0;
//}
//
//int getObstacleGridsAwayFR() {   //front right sensor
//  int distance = getMedianDistance(front_right_sensor_pin, 1080);
//  if (distance < 16)        return 1;
//  else if (distance < 28)   return 2;
//  else return 0;
//}
//
int getObstacleGridsAwayFM() {   //front middle sensor
  int distance = getMedianDistance(front_middle_sensor_pin, 1080);
  if (distance < 17)       return 1;
  else if (distance < 28)   return 2;
  else return 0;
  
}
//
//int getObstacleGridsAwayLF() {   //left front sensor
//  int distance = getMedianDistance(left_front_sensor_pin, 1080);
//  if (distance < 17)        return 1;
//  else if (distance < 27)   return 2;
//  else return 0;
//}
//
//int getObstacleGridsAwayLB() {   //left back sensor
//  int distance = getMedianDistance(left_back_sensor_pin, 1080);
//  if (distance < 16)        return 1;
//  else if (distance < 26)   return 2;
//  else return 0;
//}
//
//int getObstacleGridsAwayR() {   //long range sensor
//  int distance = getMedianDistance(right_front_long_range_sensor_pin, 20150);
//  if (distance < 27)        return 1; //20
//  else if (distance < 34)   return 2; //30
//  else if (distance < 45)   return 3; //40
//  else if (distance < 58)   return 4; //50
//  else return 0;
//}
//
//  

