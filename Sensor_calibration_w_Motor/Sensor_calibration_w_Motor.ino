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


//---------------Declare variables--------------------------
String stringToSend, command;
char character;
volatile int encoder_R_value = 0 , encoder_L_value = 0 ;
int stringIndex, tickError, error, grids = 0;


void setup()
{
  pinMode(motor_encoder_right, INPUT);
  pinMode(motor_encoder_left, INPUT);
  enableInterrupt(motor_encoder_left, LeftEncoderInc, CHANGE);
  enableInterrupt(motor_encoder_right, RightEncoderInc, CHANGE);
  Serial.begin(9600); //Enable the serial comunication
  md.init();
}

void loop()
{
  while (Serial.available())  
  {
    character = Serial.read();
    if (character == '\n' || character == '\0')   		//Meaning end of line / no new line and indicates no further input received
      break;
    else 
      command += character;     						//Else read in new commands received.
  }

	while (command.length() > stringIndex) 				//If there are any valid comments, perform/execute- operations on it.
	{      		
	    while (command[stringIndex] != '\n' && command[stringIndex] != '\0') //While not end of string
	    {  
	     	switch (command[stringIndex]) 				//Switch-case multiple scenarios, handle diff poss scenario inputs.
	     	{       		
		 		    case 'F':   							//Command to move forward 1 grid 
		          		{ 
		          			moveForwardGridRamp();
		          			break;	
		          		}

		 		    case 'L':								//Command to move rotate left 90 degrees
		         		  {
		         			  left(725,150);
                    delay(100);
                    md.setBrakes(400,400);
		         			  readAllSensors();
		         			  break;
		          		}

		        case 'R':   							//Command to move rotate right 90 degrees		
		          		{ 
		          			right(725,150);
                    delay(100);
                    md.setBrakes(400,400);
		          			readAllSensors(); 
		          			break;
		            	}

            case 'B':                 //Command to move rotate right 90 degrees   
                  { 
                    backward(525,400);
                    delay(100);
                    md.setBrakes(400,400);
                    readAllSensors(); 
                    break;
                  }

		        case 'C':								//Command to callibrate robot before starting movement
				          {
                    readAllSensors();
				            calibrateBeforeExploration();
				            break;
				          }

        		case 'X':         						//Command to proceed with fastest path
          				{
            				//Enable FastestPath()
            				break;
          				}

          	case 'S':         						//Command to move read sensors to get distance (Checklist)
          				{
            				getDistanceFromRobot();
            				break;
          				}
            case 'A':                     //Command to test Arduino (Checklist)
                  {
                    Serial.println("B");
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
}

void moveForwardGridRamp()  						//for exploration
{  
    forward(525,400);
    md.setBrakes(400,400);
    delay(100);
    readAllSensors(); 
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

void backward(int value, int Speed)
{
    resetEncoderValues();
    while ( encoder_R_value < value || encoder_L_value < value ) {    //run until either one wheel reaches the tick
    tickError = 4 * tuneWithPID();
    md.setSpeeds(Speed + tickError, -(Speed - tickError));
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
  RunningMedian samples = RunningMedian(9);       			//take 9 samples of sensor reading
  for (int i = 0; i < 9; i ++)
   samples.add(readSensor(IRpin, model));    				//samples call readSensor() to read in sensor value

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
      else if(median <= 33) return median-2;
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

int getObstacleGridsAwayFL() 
{
  int distance = getMedianDistance(front_left_sensor_pin, 1080);
  if (distance < 18)        return 1;    					//if the distance is less than 1 grid away from the obstacle, return 1
  else if (distance < 28)   return 2;	 					//if the distance is less than 2 grid away from the obstacle, return 2
  else if (distance < 38)   return 3;						//if the distance is less than 3 grid away from the obstacle, return 3
  else return 0;						 					//if the distance is more than4 grid away from the obstacle, return 0
}

int getObstacleGridsAwayFR() 
{
  int distance = getMedianDistance(front_right_sensor_pin, 1080);
  if (distance < 17)        return 1;
  else if (distance < 29)   return 2;
  else if (distance < 38)   return 3;
  else return 0;
}

int getObstacleGridsAwayFM() 
{
  int distance = getMedianDistance(front_middle_sensor_pin, 1080);
  if (distance < 18)        return 1;
  else if (distance < 29)   return 2;
  else if (distance < 38)   return 3;
  else return 0;
}

int getObstacleGridsAwayLF() 
{  
  int distance = getMedianDistance(left_front_sensor_pin, 1080);
  if (distance < 18)        return 1;
  else if (distance < 28)   return 2;
  else if (distance < 38)   return 3;
  else return 0;
}

int getObstacleGridsAwayLB() 
{  
  int distance = getMedianDistance(left_back_sensor_pin, 1080);
  if (distance < 17)        return 1;
  else if (distance < 27)   return 2;
  else if (distance < 38)   return 3;
  else return 0;
}

int getObstacleGridsAwayR() 
{ 
  int distance = getMedianDistance(right_front_long_range_sensor_pin, 20150);
  if (distance < 28)        return 1; //20
  else if (distance < 35)   return 2; //30
  else if (distance < 46)   return 3; //40
  else if (distance < 59)   return 4; //50
  else return 0;
}

//FR:FM:FL:LF:LB:R
void readAllSensors() 
{      
  delay(1000);
  stringToSend += getObstacleGridsAwayFR();
  stringToSend += ":";
  stringToSend += getObstacleGridsAwayFM();
  stringToSend += ":";
  stringToSend += getObstacleGridsAwayFL();
  stringToSend += ":";
  stringToSend += getObstacleGridsAwayLF();
  stringToSend += ":";
  stringToSend += getObstacleGridsAwayLB();
  stringToSend += ":";
  stringToSend += getObstacleGridsAwayR();
  Serial.println(stringToSend);
  stringToSend = "";
}

void calibrateBeforeExploration() {

   while(  getMedianDistance(left_front_sensor_pin, 1080)- getMedianDistance(left_back_sensor_pin, 1080) >0)
   {
    md.setSpeeds(70,70);
   }   
   md.setBrakes(400,400);                                                                                             

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
