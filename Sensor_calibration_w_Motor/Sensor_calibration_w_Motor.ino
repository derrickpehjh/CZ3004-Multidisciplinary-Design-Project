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


//---------------Declare variables--------------------------
String stringToSend, command;
int stringIndex = 0;
char character;


void setup()
{
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
		          			moveForwardGridRamp(command[stringIndex]);
                    readAllSensors(); 
		          			break;	
		          		}

		 		    case 'L':								//Command to move rotate left 90 degrees
		         		  {
		         			//turnLeft();
		           			break;
		          		}

		        case 'R':   							//Command to move rotate right 90 degrees		
		          		{ 
		          			//turnRight();
		          			break;
		            	}

		        case 'C' :								//Command to callibrate robot before starting movement
				          {
				            calibrateBeforeExploration();
				            break;
				          }

        		case 'X' :         						//Command to proceed with fastest path
          				{
            				
            				break;
          				}

          	case 'S' :         						//Command to move read sensors to get distance (Checklist)
          				{
            				getDistanceFromRobot();
            				break;
          				}

		        default :
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

void moveForwardGridRamp(int grids)  						//for exploration
{  
  //Move Forward 1 grid and brake
  //Test codes for distance
  md.setSpeeds(-200,200);
	while(getObstacleGridsAwayFM() != 1);
	md.setBrakes(400,400);
}

float readSensor(int IRpin, int model) 
{
  float sensorValue = analogRead(IRpin);
  float distance;
  if (model == 1080)  //for 10-80cm sensor
    distance = 4468.9 * pow(sensorValue, -0.959); // 10-80cm  ~ formula
  if (model == 20150) //for 20-150cm sensor
    distance = 139310 * pow(sensorValue, -1.4245); //20-150cm
  return distance;
}

float getMedianDistance(int IRpin, int model) 
{   			
  RunningMedian samples = RunningMedian(9);       			//take 9 samples of sensor reading
  for (int i = 0; i < 9; i ++)
    samples.add(readSensor(IRpin, model));    				//samples call readSensor() to read in sensor value
  return samples.getMedian();
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
 while( getMedianDistance(left_front_sensor_pin, 1080) != getMedianDistance(left_back_sensor_pin, 1080))
 	{
 		//Do callibration by moving motor values using trigonometry
 	}
}



//For checklist To get distance w.r.t robot (not sensor)
//First minus = offset from sensor to robot
//Second minus == sensor inaccuracy offset
void getDistanceFromRobot()
{ 
int distance0 = getMedianDistance(front_right_sensor_pin,1080) - 6 - 2;
  Serial.print("Distance for front_right_sensor: ");
  Serial.println(distance0);
  
  int distance1 = getMedianDistance(front_middle_sensor_pin,1080) - 6 - 2;
  Serial.print("Distance for front_middle_sensor: ");
  Serial.println(distance1);
  
  int distance2 = getMedianDistance(front_left_sensor_pin,1080) - 6 - 2;
  Serial.print("Distance for front_left_sensor: ");
  Serial.println(distance2);

  
  int distance3 = getMedianDistance(left_front_sensor_pin,1080) - 6 - 2;
  Serial.print("Distance for left_front_sensor: ");
  Serial.println(distance3);
  
  int distance4 = getMedianDistance(left_back_sensor_pin,1080) - 6 - 2;
  Serial.print("Distance for left_back_sensor: ");
  Serial.println(distance4);
  

   int distance5 = getMedianDistance(right_front_long_range_sensor_pin,20150) - 11;
   Serial.print("Distance for right_front_long_range_sensor: ");
   Serial.println(distance5);
   
   delay(2000);
}
