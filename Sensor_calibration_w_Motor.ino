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


void setup()
{
  Serial.begin(9600); //Enable the serial comunication
  md.init();
}

void loop()
{
//  md.setSpeeds(-400,400);


//// Code to readin Sensor Voltage to map against physical distance. Test accuracy of Sensor.
//    int voltage = analogRead(right_front_long_range_sensor_pin);
//    Serial.print("voltage Reading :");
//    Serial.println(voltage);
//    delay(1000);


//// code calling Derrick's func, see below for Code description.
//  getDistanceFromSensor();  ~~ calling Derrick's function, see below


//  if (distance<24){
//    md.setBrakes(400,400);
//    delay(5000);
//  }

}



float getMedianDistance(int IRpin, int model) {   //read each sensor ~5msec
  RunningMedian samples = RunningMedian(9);       //take 9 samples of sensor reading
  for (int i = 0; i < 9; i ++)
    samples.add(readSensor(IRpin, model));    //samples call readSensor() to read in sensor value
//
//
//  if (IRpin == right_front_long_range_sensor_pin) {       //for model == 20150 "long range sensor"
//    if (samples.getMedian() <= 20)       return samples.getMedian() + 0;    //20 = 1 gird
//    else if (samples.getMedian() <= 29)  return samples.getMedian() + 1;    //30 = 2 grid
//    else if (samples.getMedian() <= 39)  return samples.getMedian() + 1;    //40 = 3 grid
//    else if (samples.getMedian() <= 46)  return samples.getMedian() + 4;    //50 = 4 grid
//    else                                return samples.getMedian() + 10;    // 0.0  =D  so same prob as us w long range reading.
//  }
  return samples.getMedian();
}

  
  float readSensor(int IRpin, int model) {
  float sensorValue = analogRead(IRpin);
  float distance;
  if (model == 1080)  //for 10-80cm sensor
    distance = 4468.9 * pow(sensorValue, -0.959); // 10-80cm  ~ formula
  if (model == 20150) //for 20-150cm sensor
    distance = 139310 * pow(sensorValue, -1.4245); //20-150cm
  return distance;
}





//// function just to return distance, using our own formulated dervied formula as indicated above, and test out its accuracy, how much deviation/error. Hopefully not too far off from true value.  Have not yet map to physical num of Grids.  Func Created by Derrick CAA Wed 14 Feb 2018. 
//this function only returns us the distance base on our formula, converting sensor analog voltage, return as distance, BUT havent map to no of Grids yet, so cannot return no. of Grids yet, to inform Algo of num of Obstacles ahead!!
void getDistanceFromSensor(){
  
//  int distance0 = getMedianDistance(front_right_sensor_pin,1080);
//  Serial.print("Distance for front_right_sensor: ");
//  Serial.println(distance0);
//  
//  int distance1 = getMedianDistance(front_middle_sensor_pin,1080);
//  Serial.print("Distance for front_middle_sensor: ");
//  Serial.println(distance1);
//  
//  int distance2 = getMedianDistance(front_left_sensor_pin,1080);
//  Serial.print("Distance for front_left_sensor: ");
//  Serial.println(distance2);
//
//  
//  int distance3 = getMedianDistance(left_front_sensor_pin,1080);
//  Serial.print("Distance for left_front_sensor: ");
//  Serial.println(distance3);
//  
//  int distance4 = getMedianDistance(left_back_sensor_pin,1080);
//  Serial.print("Distance for left_back_sensor: ");
//  Serial.println(distance4);
//  

   int distance5 = getMedianDistance(right_front_long_range_sensor_pin,20150);
   Serial.print("Distance for right_front_long_range_sensor: ");
   Serial.println(distance5);
   
   delay(2000);
} 



//// Taking measurements ONCE w.r.t Robot which is basically getdistancefromSensor() - offset sensor distance to Robot egde.  Prolly define a static int variable for the diff sensor offset values, w.r.t each edge of the robot.
//
//void getDistanceFromRobot(){     //Prep this, just in case Prof wants, cos now ambiguous whether we taking the readings w.r.t Sensor's / Robot's  point-of-view ??
//  
//
//}







//Havent map to no of physical Obstacle Grids yet to return / inform Algo.

int getObstacleGridsAwayFL() {   //front left sensor
  int distance = getMedianDistance(front_left_sensor_pin, 1080);
  if (distance <= 17)        return 1;    //1 indicates sensor 10cm distance reading.  14cm is from the pov of sensor (4cm inwards) 
  else if (distance <= 27)   return 2;
  else return 0;
}

int getObstacleGridsAwayFR() {   //front right sensor
  int distance = getMedianDistance(front_right_sensor_pin, 1080);
  if (distance < 16)        return 1;
  else if (distance < 28)   return 2;
  else return 0;
}

int getObstacleGridsAwayFM() {   //front middle sensor
  int distance = getMedianDistance(front_middle_sensor_pin, 1080);
  if (distance < 17)        return 1;
  else if (distance < 28)   return 2;
  else return 0;
}

int getObstacleGridsAwayLF() {   //left front sensor
  int distance = getMedianDistance(left_front_sensor_pin, 1080);
  if (distance < 17)        return 1;
  else if (distance < 27)   return 2;
  else return 0;
}

int getObstacleGridsAwayLB() {   //left back sensor
  int distance = getMedianDistance(left_back_sensor_pin, 1080);
  if (distance < 16)        return 1;
  else if (distance < 26)   return 2;
  else return 0;
}

int getObstacleGridsAwayR() {   //long range sensor
  int distance = getMedianDistance(right_front_long_range_sensor_pin, 20150);
  if (distance < 27)        return 1; //20
  else if (distance < 34)   return 2; //30
  else if (distance < 45)   return 3; //40
  else if (distance < 58)   return 4; //50
  else return 0;
}




