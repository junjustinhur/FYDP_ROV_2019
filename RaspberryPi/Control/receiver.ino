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
#include <Servo.h>
#include "math.h"

//-------------------------------------------------------motion sensor init -------------------------------------------------------//

NineAxesMotion mySensor;                 //Object that for the sensor
unsigned long lastStreamTime = 0;     //To store the last streamed time stamp
const int streamPeriod = 40;          //To stream at 25Hz without using additional timers (time period(ms) =1000/frequency(Hz))
bool updateSensorData = true;         //Flag to update the sensor data. Default is true to perform the first read before the first stream

//-------------------------------------------------------PID controller -------------------------------------------------------//

float PID_yaw, PID_pitch, PID_roll;
float p_yaw_gain = 3, p_pitch_gain = 1, p_roll_gain = 1;
float i_yaw_gain, i_pitch_gain, i_roll_gain;
float d_yaw_gain=0.5, d_pitch_gain, d_roll_gain;
float p_yaw, p_pitch, p_roll;
float i_yaw, i_pitch, i_roll;
float d_yaw, d_pitch, d_roll;
float PID_pitchAbs;

//-------------------------------------------------------Error calculation -------------------------------------------------------//

int w,x,y,z; //quaternion read
float xGyro,yGyro,zGyro; //gyro read
float q1, q2, q3, q4; // body frame quaternion
float qRef1, qRef2, qRef3, qRef4; // controller quaternion
float qRefCon1, qRefCon2, qRefCon3, qRefCon4;
float qErr1, qErr2, qErr3, qErr4;
float eulX, eulY, eulZ;

//-------------------------------------------------------Motor feed -------------------------------------------------------//

int surge = 0, sway = 0, heave = 0;
int surgeAbs = 0, swayAbs = 0;
float motor1, motor1_neu = 1500, motor1_xy, motor1_z;
float motor2, motor2_neu = 1500, motor2_xy, motor2_z;
float motor3, motor3_neu = 1500, motor3_xy, motor3_z;
float motor4, motor4_neu = 1500, motor4_xy, motor4_z;
float servo14, servo23;
boolean motorInit = false;
float minThrust = 1200;
float maxThrust = 1800;
float motor_c = 0.5;

//-------------------------------------------------------Communication -------------------------------------------------------//

int timeInit = 5000; //in milliseconds
int timeStart;
byte input[22];
byte keyCheck;

Servo motor_1; 
Servo motor_2;
Servo motor_3;
Servo motor_4;
Servo servo_14;
Servo servo_23;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);           //Initialize the Serial Port to view information on the Serial Monitor
  I2C.begin();                    //Initialize I2C communication to the let the library communicate with the sensor.
  
//==========================================================Motor init ==========================================================//

  motor_1.attach(3);
  motor_2.attach(5);
  motor_3.attach(6);
  motor_4.attach(9);
  motor_1.writeMicroseconds(1500);
  motor_2.writeMicroseconds(1500);
  motor_3.writeMicroseconds(1500);
  motor_4.writeMicroseconds(1500);  
  
//==========================================================Sensor init ==========================================================//

  mySensor.initSensor();          //The I2C Address can be changed here inside this function in the library
  mySensor.setOperationMode(OPERATION_MODE_NDOF);   //Can be configured to other operation modes as desired
  mySensor.setUpdateMode(MANUAL);  //The default is AUTO. Changing to manual requires calling the relevant update functions prior to calling the read functions
  //Setting to MANUAL requires lesser reads to the sensor
  mySensor.updateQuat();
  updateSensorData = true;

//==========================================================Communication init ==========================================================//

  initLoop();
  timeStart = millis();
  //acknowledge();
}

void initLoop(){
  while(Serial.available()<1){
    Serial.write("R");
  }
  Serial.flush();
}

void loop() {
  
  if (updateSensorData)  //Keep the updating of data as a separate task
  {
    mySensor.updateAccel();        //Update the Accelerometer data
    mySensor.updateLinearAccel();  //Update the Linear Acceleration data
    mySensor.updateGravAccel();    //Update the Gravity Acceleration data
    mySensor.updateCalibStatus();  //Update the Calibration Status
    mySensor.updateQuat();
    updateSensorData = false;
  }

  mySensor.readQuat(w,x,y,z); //Read the updated quaternion
  q1 = w/16384.0;
  q2 = x/16384.0;
  q3 = y/16384.0;
  q4 = z/16384.0;
  mySensor.readGyro(xGyro,yGyro,zGyro);

  if(Serial.available()>22){ //Read the input from the controller
    input[0] = Serial.read(); //@
    input[1] = Serial.read(); //@
    input[2] = Serial.read(); //@
    input[3] = Serial.read(); //q1
    input[4] = Serial.read(); //q2
    input[5] = Serial.read(); //q3
    input[6] = Serial.read(); //q4
    input[7] = Serial.read(); //x_in
    input[8] = Serial.read(); //y_in
    input[9] = Serial.read(); //up_in
    input[10] = Serial.read(); //down_in
    input[11] = Serial.read(); //@
    input[12] = Serial.read(); //@
    input[13] = Serial.read(); //@
    input[14] = Serial.read(); //q1
    input[15] = Serial.read(); //q2
    input[16] = Serial.read(); //q3
    input[17] = Serial.read(); //q4
    input[18] = Serial.read(); //x_in
    input[19] = Serial.read(); //y_in
    input[20] = Serial.read(); //up_in
    input[21] = Serial.read(); //down_in
    for(int i = 0; i<1+sizeof(input)/2; i++)
      if(input[i] == '@' && input[i+1] == '@' && input[i+2] == '@'){
        qRef1 = mapFloat(input[i+3],0,255,-1,1);
        qRef2 = mapFloat(input[i+4],0,255,-1,1);
        qRef3 = mapFloat(input[i+5],0,255,-1,1);
        qRef4 = mapFloat(input[i+6],0,255,-1,1);
        surge = mapFloat(input[i+7],0,255,-514,509);
        sway = mapFloat(input[i+8],0,255,-524,499);
        heave = int(input[i+9]) + int(input[i+10]);  
      } else {
        Serial.read();
      }
  }

  //-----------------------------------------Error Calculation---------------------------------//
  
  //quaternion error between desired orientation (qRef1, qRef2, qRef3, qRef4) and current orientation (q1, q2, q3, q4)
  qRefCon1 = qRef1;
  qRefCon2 = -1*qRef2;
  qRefCon3 = -1*qRef3;
  qRefCon4 = -1*qRef4;
  qErr1=(qRefCon1*q1-qRefCon2*q2-qRefCon3*q3-qRefCon4*q4);
  qErr2=(qRefCon1*q2+qRefCon2*q1+qRefCon3*q4-qRefCon4*q3);
  qErr3=(qRefCon1*q3-qRefCon2*q4+qRefCon3*q1+qRefCon4*q2);
  qErr4=(qRefCon1*q4+qRefCon2*q3-qRefCon3*q2+qRefCon4*q1); 

  // quaternion to euler
  eulZ=atan2(2.0*(qErr2*qErr3+qErr4*qErr1),(sq(qErr2)-sq(qErr3)-sq(qErr4)+sq(qErr1)));
  eulY=asin(-2.0*(qErr2*qErr4-qErr1*qErr3)/(sq(qErr2)+sq(qErr3)+sq(qErr4)+sq(qErr1)));
  eulX=atan2(2.0*(qErr3*qErr4+qErr1*qErr2),(-sq(qErr2)-sq(qErr3)+sq(qErr4)+sq(qErr1)));
  eulZ=eulZ*180/3.1415; // yaw
  eulY=eulY*180/3.1415; // roll
  eulX=eulX*180/3.1415; // pitch
  if (eulZ < 1 && eulZ > -1) eulZ = 0;
  if (eulY < 5 && eulY > -5) eulY = 0;
  if (eulX < 5 && eulX > -5) eulX = 0;

  //-----------------------------------------PID---------------------------------//
 
  // Proportional
  p_yaw = p_yaw_gain * eulZ;
  p_pitch = p_pitch_gain * eulX;
  p_roll = p_roll_gain * eulY;

  //Integral
  //i_yaw = i_yaw * (i_yaw_gain * eulZ);
  //i_pitch = i_pitch * (i_pitch_gain * eulX);
  //i_roll = i_roll * (i_roll_gain * eulY);

  //Derivative
  d_yaw = -1*d_yaw_gain * zGyro;
  d_pitch = -1*d_pitch_gain * yGyro;
  d_roll = -1*d_roll_gain * xGyro;

  //Integral
  i_yaw = 0;
  i_pitch = 0;
  i_roll = 0;

  //SUM
  PID_yaw = p_yaw + i_yaw + d_yaw;
  PID_pitch = p_pitch + i_pitch + d_pitch ;
  PID_roll = p_roll + i_roll + d_roll;
  

  initialize_feed(); // initialize motor feed

  if ((surge > 10 || surge < -10) || (sway > 10 || sway < -10)){
    }
  else {
    surge = 0;
    sway = 0;
    }

  surge *= 0.8;
  sway *= 0.8;

  surgeAbs = abs(surge);
  swayAbs = abs(sway);

  if (surge == 0 && sway == 0){ // condition to execute heave and stationary mode
    heave_feed();
  }
  else if(surgeAbs >= swayAbs ){// condition to execute surge mode
    surge_feed();
  }
  else if(surgeAbs <= swayAbs ){ // // condition to execute sway mode
    sway_feed();
  }

  if (motor1 >= maxThrust){motor1 = maxThrust;} //motor feed is limited in a range between max allowed feed ans min allowed feed
  else if (motor1 <= minThrust){motor1 = minThrust;}
  else if (motor2 >= maxThrust){motor2 = maxThrust;}
  else if (motor2 <= minThrust){motor2 = minThrust;}
  else if (motor3 >= maxThrust){motor3 = maxThrust;}
  else if (motor3 <= minThrust){motor3 = minThrust;}
  else if (motor4 >= maxThrust){motor4 = maxThrust;}
  else if (motor4 <= minThrust){motor4 = minThrust;}
  else{}
    
  if (servo14 > 90){servo14 = 90;} // servo input is limited in a range from -90 degree to 90 degree
  if (servo23 < -90){servo23 = -90;}
//  servo14 = map(PID_pitch,-90,90,-45,45);
//  servo23 = map(PID_pitch,-90,90,-45,45);
   
killDeadBand();

 
  if(!motorInit){                         // initiate motors
    motor_1.writeMicroseconds(motor1_neu);
    motor_2.writeMicroseconds(motor2_neu);
    motor_3.writeMicroseconds(motor3_neu);
    motor_4.writeMicroseconds(motor4_neu);
    
    Serial.write("@@@");
    Serial.write((char)servo14);
    Serial.write((char)servo23);
    Serial.write((char)mapFloat(motor1_neu,minThrust,maxThrust,0,255));
    Serial.write((char)mapFloat(motor2_neu,minThrust,maxThrust,0,255));
    Serial.write((char)mapFloat(motor3_neu,minThrust,maxThrust,0,255));
    Serial.write((char)mapFloat(motor4_neu,minThrust,maxThrust,0,255));
    Serial.write((char)mapFloat(qRef1,-1,1,0,255));
    Serial.write((char)mapFloat(qRef2,-1,1,0,255));
    Serial.write((char)mapFloat(qRef3,-1,1,0,255));
    Serial.write((char)mapFloat(qRef4,-1,1,0,255));
    Serial.write((char)mapFloat(q1,-1,1,0,255));
    Serial.write((char)mapFloat(q2,-1,1,0,255));
    Serial.write((char)mapFloat(q3,-1,1,0,255));
    Serial.write((char)mapFloat(q4,-1,1,0,255));
    Serial.write((char)mapFloat(p_yaw_gain,0,5.12,0,255));
    Serial.write((char)mapFloat(d_yaw_gain,0,5.12,0,255));
    
    if(millis()-timeStart>timeInit)
      motorInit = true;
  } else {                            // start feeding motors
    motor_1.writeMicroseconds(motor1);
    motor_2.writeMicroseconds(motor2);
    motor_3.writeMicroseconds(motor3);
    motor_4.writeMicroseconds(motor4);
  
    Serial.write("@@@");
    Serial.write((char)servo14);
    Serial.write((char)servo23);
    //Serial.write((char)mapFloat(surge,-513,508,0,255));
    //Serial.write((char)mapFloat(sway,-524,499,0,255));
    Serial.write((char)mapFloat(motor1,minThrust,maxThrust,0,255));
    Serial.write((char)mapFloat(motor2,minThrust,maxThrust,0,255));
    Serial.write((char)mapFloat(motor3,minThrust,maxThrust,0,255));
    Serial.write((char)mapFloat(motor4,minThrust,maxThrust,0,255));
    Serial.write((char)mapFloat(qRef1,-1,1,0,255));
    Serial.write((char)mapFloat(qRef2,-1,1,0,255));
    Serial.write((char)mapFloat(qRef3,-1,1,0,255));
    Serial.write((char)mapFloat(qRef4,-1,1,0,255));
    Serial.write((char)mapFloat(q1,-1,1,0,255));
    Serial.write((char)mapFloat(q2,-1,1,0,255));
    Serial.write((char)mapFloat(q3,-1,1,0,255));
    Serial.write((char)mapFloat(q4,-1,1,0,255));
   // Serial.write((char)mapFloat(p_yaw_gain,0,5.12,0,255));
   // Serial.write((char)mapFloat(d_yaw_gain,0,5.12,0,255));
  }

updateSensorData = true;
}

float mapFloat(float old, float oldLow, float oldHigh, float newLow, float newHigh){
  float answer = old - oldLow;
  answer/=(oldHigh-oldLow);
  answer*=(newHigh-newLow);
  answer+=newLow;
  return answer;
}

void killDeadBand(){
  if (motor1 < 1505 && motor1 > 1473){
    motor1 = 1500;}
  else{motor1 -= 11;
  }

  if (motor2 < 1505 && motor2 > 1473){
    motor2 = 1500;}
  else{motor2 -= 11;
  }

  if (motor3 < 1505 && motor3 > 1473){
    motor3 = 1500;}
  else{motor3 -= 11;
  }

  if (motor4 < 1505 && motor4 > 1473){
    motor4 = 1500;}
  else{motor4 -= 11;
  }
 
  }
  // ======================================================= Heave mode =======================================================//
void heave_feed(){
  servo14 = 90;
  servo23 = 90;

  motor1_z = -heave - PID_yaw - PID_pitch - PID_roll;
  motor2_z = -heave + PID_yaw - PID_pitch + PID_roll;
  motor3_z = heave + PID_yaw - PID_pitch - PID_roll;
  motor4_z = heave - PID_yaw - PID_pitch + PID_roll;
/*
  if (motor1_z > 0){motor1_z = motor_c*motor1_z;}
  if (motor2_z > 0){motor2_z = motor_c*motor2_z;}
  if (motor3_z > 0){motor3_z = motor_c*motor3_z;}
  if (motor4_z > 0){motor4_z = motor_c*motor4_z;}
  */    
  motor1 = motor1_neu + motor1_z;
  motor2 = motor2_neu + motor2_z;
  motor3 = motor3_neu + motor3_z;
  motor4 = motor4_neu + motor4_z;
 }
// ======================================================= Surge mode =======================================================//
void surge_feed(){
  if (surge > 0){
    motor1_xy = surge - PID_yaw;
    if (motor1_xy < 0){motor1_xy = 0;}
    motor1_z = PID_pitch - PID_roll;
    motor1 = motor1_neu + sqrt(sq(motor1_xy)+sq(motor1_z));
    servo14 = 180*atan2(motor1_z, motor1_xy)/PI ;

    motor2_xy = surge + PID_yaw;
    if (motor2_xy < 0){motor2_xy = 0;}
    motor2_z = PID_pitch + PID_roll;
    motor2 = motor2_neu + sqrt(sq(motor2_xy)+sq(motor2_z));
    servo23 = 180*atan2(motor2_z, motor2_xy)/PI ;
  }
  
  else if (surge < 0){
    motor3_xy = -surge - PID_yaw;
    if (motor3_xy < 0){motor3_xy = 0;}
    motor3_z = -PID_pitch - PID_roll;
    motor3 = motor3_neu + sqrt(sq(motor3_xy)+sq(motor3_z));
    servo23 = 180*atan2(motor3_z, motor3_xy)/PI ;

    motor4_xy = -surge + PID_yaw;
    if (motor4_xy < 0){motor4_xy = 0;}
    motor4_z = -PID_pitch + PID_roll;
    motor4 = motor4_neu + sqrt(sq(motor4_xy)+sq(motor4_z));
    servo14 = 180*atan2(motor4_z, motor4_xy)/PI ;
  }
}

// ======================================================= Sway mode================================================//
void sway_feed(){ 
  
  if (sway > 0){
    motor3_xy = sway + PID_yaw;
    if (motor3_xy < 0){motor3_xy = 0;}
    motor3_z = PID_pitch;
    motor3 = motor3_neu - sqrt(sq(motor3_xy)+sq(motor3_z));
    servo23 = 180*atan2(motor3_z, motor3_xy)/PI ;

    motor2_xy = sway - PID_yaw;
    if (motor2_xy < 0){motor2_xy = 0;}
    motor2_z = PID_pitch;
    motor2 = motor2_neu - sqrt(sq(motor2_xy)+sq(motor2_z));
    }
    
  else if (sway < 0){
    motor1_xy = -sway + PID_yaw;
    if (motor1_xy < 0){motor1_xy = 0;}
    motor1_z = PID_pitch;
    motor1 = motor1_neu - sqrt(sq(motor1_xy)+sq(motor1_z));
    servo14 = 180*atan2(motor1_z, motor1_xy)/PI ;

    motor4_xy = -sway - PID_yaw;
    if (motor4_xy < 0){motor4_xy = 0;}
    motor4_z = PID_pitch;
    motor4 = motor4_neu - sqrt(sq(motor4_xy)+sq(motor4_z));
  }
}
// ======================================================= feed initialization ===================================//
void initialize_feed(){
  motor1 = motor1_neu;
  motor2 = motor2_neu;
  motor3 = motor3_neu;
  motor4 = motor4_neu;
  motor1_xy = 0;
  motor2_xy = 0;
  motor3_xy = 0;
  motor4_xy = 0;
  motor1_z = 0;
  motor2_z = 0;
  motor3_z = 0;
  motor4_z = 0;
}
