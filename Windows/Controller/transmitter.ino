/*
*
****************************************************************************
* File : Motion control code
* Author : Jun HUR
* Institution : The Hong Kong University of Technology
*
*
 */

#include "NineAxesMotion.h"        //Contains the bridge code between the API and the Arduino Environment
#include <Wire.h>
#include "math.h"

NineAxesMotion mySensor;         //Object that for the sensor 
unsigned long lastStreamTime = 0;     //To store the last streamed time stamp
const int streamPeriod = 20;          //To stream at 50Hz without using additional timers (time period(ms) =1000/frequency(Hz))
bool updateSensorData = true;

//-------------------------joystick-----------------------//
float sw_in, x_in, y_in, z_in;
const int sw_pin = 2;
const int x_pin = 0;
const int y_pin = 1;

//-------------------------button-----------------------//
int up_pin = 8, down_pin = 7, up_in= 0, down_in = 0;

//-------------------------motion shield-----------------------//
int w,x,y,z; //quaternion read
float q1, q2, q3, q4;


void setup() //This code is executed once
{
  //Peripheral Initialization
  Serial.begin(57600);           //Initialize the Serial Port to view information on the Serial Monitor
  I2C.begin();                    //Initialize I2C communication to the let the library communicate with the sensor.
  //Sensor Initialization
  mySensor.initSensor();          //The I2C Address can be changed here inside this function in the library
  mySensor.setOperationMode(OPERATION_MODE_NDOF);   //Can be configured to other operation modes as desired
  mySensor.setUpdateMode(MANUAL);	//The default is AUTO. Changing to MANUAL requires calling the relevant update functions prior to calling the read functions
  //Setting to MANUAL requires fewer reads to the sensor
  mySensor.updateQuat();
  //updateSensorData = true;

  //-------------------------joystick button init-----------------------//
  pinMode(sw_pin, INPUT);
  digitalWrite(sw_pin, HIGH);
  
  //-------------------------button init-----------------------//
  pinMode(up_pin, INPUT);
  digitalWrite(up_pin, HIGH);

  pinMode(down_pin, INPUT);
  digitalWrite(down_pin, HIGH);

  initLoop(); 
}

void initLoop(){
  while(Serial.available()<1){
    Serial.write("T");
  }
  Serial.flush();
  Serial.write("Y");
}

void loop(
{
  if ((millis() - lastStreamTime) >= streamPeriod)
  {
    lastStreamTime = millis();
    mySensor.updateAccel();        //Update the Accelerometer data
    mySensor.updateLinearAccel();  //Update the Linear Acceleration data
    mySensor.updateGravAccel();    //Update the Gravity Acceleration data
    mySensor.updateCalibStatus();  //Update the Calibration Statu
    mySensor.updateQuat();
    updateSensorData = false;
    mySensor.readQuat(w,x,y,z);
    
    q1 = w/16384.0;
    q2 = x/16384.0;
    q3 = y/16384.0;
    q4 = z/16384.0;
    
    sw_in = digitalRead(sw_pin);
    x_in = map(analogRead(x_pin),0, 1022, -502, 500);
    y_in = map(analogRead(y_pin),0, 1024, -524, 500);
    if (x_in < 5 && x_in > -5) {x_in = 0;}
    if (y_in < 5 && y_in > -5) {y_in = 0;}   


    up_in = digitalRead(up_pin);
    down_in = digitalRead(down_pin);
    x_in = x_in*0.5;
    y_in = y_in*0.5;


    Serial.print("@@@");
    Serial.print((char) mapFloat(q1,-1,1,0,255));
    Serial.print((char) mapFloat(q2,-1,1,0,255));
    Serial.print((char) mapFloat(q3,-1,1,0,255));
    Serial.print((char) mapFloat(q4,-1,1,0,255));
    //Serial.print((char) mapFloat(x_in,-514,509,0,255));
    //Serial.print((char) mapFloat(y_in,-524,499,0,255));
    Serial.print((char) mapFloat(x_in,-514,509,0,255));
    Serial.print((char) mapFloat(y_in,-524,499,0,255));
    Serial.print(char(up_in));
    Serial.print(char(down_in));
    
  }
}

float mapFloat(float old, float oldLow, float oldHigh, float newLow, float newHigh){
  float answer = old - oldLow;
  answer/=(oldHigh-oldLow);
  answer*=(newHigh-newLow);
  answer+=newLow;
  return answer;
}
