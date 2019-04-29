/*
 * RoboPeak RPLIDAR Arduino Example
 * This example shows the easy and common way to fetch data from an RPLIDAR
 * 
 * You may freely add your application code based on this template
 *
 * USAGE:
 * ---------------------------------
 * 1. Download this sketch code to your Arduino board
 * 2. Connect the RPLIDAR's serial port (RX/TX/GND) to your Arduino board (Pin 0 and Pin1)
 * 3. Connect the RPLIDAR's motor ctrl pin to the Arduino board pin 3 
 */
 
/* 
 * Copyright (c) 2014, RoboPeak 
 * All rights reserved.
 * RoboPeak.com
 *
 * Redistribution and use in source and binary forms, with or without modification, 
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, 
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, 
 *    this list of conditions and the following disclaimer in the documentation 
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY 
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES 
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT 
 * SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR 
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
 
// This sketch code is based on the RPLIDAR driver library provided by RoboPeak
#include <RPLidar.h>

// You need to create an driver instance 
RPLidar lidar;

#define RPLIDAR_MOTOR 2 // The PWM pin for control the speed of RPLIDAR's motor.
                        // This pin should connected with the RPLIDAR's MOTOCTRL signal 
                       
                        
void setup() {
  // bind the RPLIDAR driver to the arduino hardware serial
  lidar.begin(Serial3);
  Serial.begin(9600);
  
  // set pin modes
  pinMode(RPLIDAR_MOTOR, OUTPUT);
  //analogWrite(RPLIDAR_MOTOR,HIGH);
  Serial.println("starting");
}
typedef struct LidarPoint
{
  double milTime;
  float distance;
  float angle;
  byte quality;
  bool startBit;
};
typedef struct DetectedObject
{
  float firstAngle;
  float firstDistance;
  float secondAngle;
  float secondDistance;
};
const int DISTANCE_MOE = 300;
const int DISTANCE_LIMIT = 3000;
const int SIG_DIF = DISTANCE_MOE;
const int ARRAY_SIZE = 100;

struct DetectedObject* findObjects(struct LidarPoint* points){
  DetectedObject objects[10];
  int numObjects = 0;
  LidarPoint point1;
  LidarPoint point2;
  LidarPoint point3;
  LidarPoint point4;
  bool cornerFound = 0;
  for(int j=0; j<(ARRAY_SIZE - 3); j++){
    point1 = points[j];
    point2 = points[j+1];
    point3 = points[j+2];
    point4 = points[j+3];
    //find first corner
    if(abs(point1.distance - point2.distance) < DISTANCE_MOE &&
       abs(point3.distance - point4.distance) < DISTANCE_MOE &&
       point3.distance < DISTANCE_LIMIT &&
       point2.distance-point3.distance > SIG_DIF&&
       !cornerFound)
       {
        /*
        Serial.print("Corner found  ");
        Serial.print(point1.distance);
        Serial.print("  ");
        Serial.print(point2.distance);
        Serial.print("  ");
        Serial.print(point3.distance);
        Serial.print("  ");
        Serial.print(point4.distance);
        Serial.print("  ");
        Serial.print(point1.angle);
        Serial.print("  ");
        Serial.print(point2.angle);
        Serial.print("  ");
        Serial.print(point3.angle);
        Serial.print("  ");
        Serial.println(point4.angle);
        */
        cornerFound = 1;
        objects[numObjects].firstAngle = point3.angle;
        float ang = objects[numObjects].firstAngle;
        objects[numObjects].firstDistance = point3.distance;
       }
       //find second corner
    if(abs(point1.distance - point2.distance) < DISTANCE_MOE &&
       abs(point3.distance - point4.distance) < DISTANCE_MOE &&
       point2.distance -point3.distance < SIG_DIF&&
       cornerFound)
       {
        cornerFound = 0;
        objects[numObjects].secondAngle = point2.angle;
        objects[numObjects].secondDistance = point2.distance;
        printObject(&objects[numObjects]);
        numObjects++;
       }
  }
  return objects;
}
void printObject(DetectedObject* object){
  DetectedObject obj = *object;
  Serial.print("Object: ");
  Serial.print(obj.firstAngle);
  Serial.print(" degrees, ");
  Serial.print(obj.firstDistance);
  Serial.print("mm away to ");
  Serial.print(obj.secondAngle);
  Serial.print(" degrees at ");
  Serial.print(obj.secondDistance);
  Serial.println("mm\n");
}

LidarPoint data[ARRAY_SIZE];
int i=0;
void loop(){
  i=0;
  while(i<ARRAY_SIZE){
    getData();
  }
  
  for(i=0;i<ARRAY_SIZE;i++){
  
    if(data[i].quality > 0&&data[i].angle >0 && data[i].angle < 360){
    Serial.print("distance ");
    Serial.print(data[i].distance);
    Serial.print(" angle ");
    Serial.print(data[i].angle);
    Serial.print(" quality ");
    Serial.print(data[i].quality);
    Serial.print(" start bit ");
    Serial.print(data[i].startBit);
    Serial.print(" time: ");
    Serial.println(data[i].milTime);
    }
  }
  
  findObjects(data);
}

int getData() {
  /*
  if (IS_OK(lidar.waitPoint())) {
    for(int i=0;i<100;i++){
    data[i].distance = lidar.getCurrentPoint().distance; //distance value in mm unit
    data[i].angle    = lidar.getCurrentPoint().angle; //anglue value in degree
    data[i].startBit = lidar.getCurrentPoint().startBit; //whether this point is belong to a new scan
    data[i].quality  = lidar.getCurrentPoint().quality; //quality of the current measurement

    
  }
  for(int i=0;i<100;i++){
    Serial.print("distance ");
    Serial.print(data[i].distance);
    Serial.print(" angle ");
    Serial.print(data[i].angle);
    Serial.print(" quality ");
    Serial.print(data[i].quality);
    Serial.print(" start bit ");
    Serial.println(data[i].startBit);
  }
    }
    
   else {
    Serial.println("not ok");
    analogWrite(RPLIDAR_MOTOR, 0); //stop the rplidar motor
    
    // try to detect RPLIDAR... 
    rplidar_response_device_info_t info;
    if (IS_OK(lidar.getDeviceInfo(info, 100))) {
       // detected...
       lidar.startScan();
       
       // start motor rotating at max allowed speed
       analogWrite(RPLIDAR_MOTOR, 255);
       delay(1000);
    }
  }
  */
  //Serial.println(lidar.waitPoint());
  
  if (IS_OK(lidar.waitPoint())) {
    /*
    float distance = lidar.getCurrentPoint().distance; //distance value in mm unit
    float angle    = lidar.getCurrentPoint().angle; //anglue value in degree
    bool  startBit = lidar.getCurrentPoint().startBit; //whether this point is belong to a new scan
    byte  quality  = lidar.getCurrentPoint().quality; //quality of the current measurement
*/
if(lidar.getCurrentPoint().quality > 0){
    data[i].distance = lidar.getCurrentPoint().distance; //distance value in mm unit
    data[i].angle    = lidar.getCurrentPoint().angle; //anglue value in degree
    data[i].startBit = lidar.getCurrentPoint().startBit; //whether this point is belong to a new scan
    data[i].quality  = lidar.getCurrentPoint().quality; //quality of the current measurement
    data[i].milTime = millis();
    //perform data processing here... 
    if(1){//quality > 10 && distance != 0 || 1){
      /*
    Serial.print("distance ");
    Serial.print(distance);
    Serial.print(" angle ");
    Serial.print(angle);
    Serial.print(" quality ");
    Serial.print(quality);
    Serial.print(" start bit ");
    Serial.print(startBit);
    Serial.print(" time: ");
    
    Serial.println(millis());
    */
    
    }
    i++;
}
  } else {
    Serial.println("not ok");
    analogWrite(RPLIDAR_MOTOR, 0); //stop the rplidar motor
    
    // try to detect RPLIDAR... 
    rplidar_response_device_info_t info;
    if (IS_OK(lidar.getDeviceInfo(info, 100))) {
       // detected...
       lidar.startScan();
       
       // start motor rotating at max allowed speed
       analogWrite(RPLIDAR_MOTOR, 255);
       delay(1000);
    }
  }
  
}
