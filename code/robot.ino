#include <RPLidar.h>
#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include <SPI.h>  
#include <Pixy.h>
#define redpin 3
#define greenpin 5
#define bluepin 6
#define commonAnode true


Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_4X);
//------------MOTOR SHIELD CONSTANTS------------------------
//B is right, A is left
int const A_FORWARD = LOW;
int const B_FORWARD = HIGH;
int const A_REVERSE = !A_FORWARD;
int const B_REVERSE = !B_FORWARD;
int const A_DIRECTION_PIN = 12;
int const A_BRAKE_PIN = 9;
int const A_SPEED_PIN = 3;
int const B_DIRECTION_PIN = 13;
int const B_BRAKE_PIN = 8;
int const B_SPEED_PIN = 11;
int const FULL_SPEED = 255;
int const FORWARD_SPEED = 70;
int const TURNING_SPEED = 75;
int const REVERSE_SPEED = 55;


int const END_TIME = 20000;
//-------------SENSORS DECLARATIONS--------------------------------
Pixy pixy;
struct Sensor
{
  int pin;
  float value;

};
const int LIGHT_PIN = A10;


//QUAFFLE DETECTION CONSTS
int const TARGET_BUF = 30;
int const ALLOWABLE_BUF = 100;

//State variables
int quaffleDetectedRight;
int quaffleDetectedLeft;
int quaffleDetected;
int quaffleX;
int quaffleY;


//Sensor sensors[] = {irSensor};
Sensor sensors[] = {{A7, 0},{A8,0},{A9,0},{LIGHT_PIN,0}};
Sensor irSensor = sensors[0];

const int SENSOR_QUANT = sizeof(sensors)/sizeof(sensors[0]);



void setup() {
  delay(500);
  pinMode(irSensor.pin, INPUT);
  pinMode(A_DIRECTION_PIN, OUTPUT);
  pinMode(B_DIRECTION_PIN, OUTPUT);
  pinMode(A_BRAKE_PIN,OUTPUT);
  pinMode(B_BRAKE_PIN, OUTPUT);
  Serial.begin(9600);
  tcs.begin();
  pixy.init();

  #if defined(ARDUINO_ARCH_ESP32)
  ledcAttachPin(redpin, 1);
  ledcSetup(1, 12000, 8);
  ledcAttachPin(greenpin, 2);
  ledcSetup(2, 12000, 8);
  ledcAttachPin(bluepin, 3);
  ledcSetup(3, 12000, 8);
#else
  pinMode(redpin, OUTPUT);
  pinMode(greenpin, OUTPUT);
  pinMode(bluepin, OUTPUT);
#endif
  forward(FORWARD_SPEED);
  quaffleDetectedRight = 0;
  quaffleDetectedLeft = 0;
  
}

void loop() {
  float red, green, blue;
  float h, s, v;
  
  
  while(millis()<END_TIME){
  tcs.setInterrupt(false);

  //generic loop counter
  static int i = 0;
  int j;
  int quaffleSeenThisLoop = 0;
  
  //From pixy source---------------
  //pixy blocks
  uint16_t blocks;
  char buf[32]; 
  // grab blocks!
  blocks = pixy.getBlocks();
  //end pixy source-----------------
  
  updateSensors();
  //Serial.println(getLightVal(sensors[3]));
  
  //from HSV source -----------------
  tcs.getRGB(&red, &green, &blue);
  RGBtoHSV(red,green,blue, &h, &s, &v);
  //end HSV source ------------------
  //Serial.print("hue");
  //Serial.println(h);
  /*
  if(h<90&&h>50){
    Serial.println("yellow detected");
    turn_right();
  }
  
  if(getIrVal(sensors[0])<6){
    Serial.println("Ir right");
    turn_right();
  }
  if(getIrVal(sensors[1])<6){
    Serial.println("Ir left");
    turn_right();
  }
  */
  if(getLightVal(sensors[3])>200){
    Serial.print("Light ");
    Serial.println(getLightVal(sensors[3]));
    excited();
  }
  //from pixy source ----------------
  if (blocks)
  {
    i++;
    
    // do this (print) every 50 frames because printing every
    // frame would bog down the Arduino
    if (i%5==0)
    {
      sprintf(buf, "Detected %d:\n", blocks);
      //Serial.print(buf);
      for (j=0; j<blocks; j++)
      {
        //end from pixy source--------------
        /*
        sprintf(buf, "  block %d: ", j);
        Serial.print(buf); 
        pixy.blocks[j].print();*/
        
        if(pixy.blocks[j].signature == 2){
           excited();
           
           if (!quaffleDetected) {
            Serial.print("Green ball ");
            Serial.print(millis());
            Serial.print(" ");
            Serial.println(i);
            
           }
           
           quaffleSeenThisLoop = 1;
           quaffleDetected = 1;        
           quaffleX = pixy.blocks[j].x;
           quaffleY = pixy.blocks[j].y;
           //if quaffle is within buffer begin turning
           
           if(quaffleX>(155+ALLOWABLE_BUF)&& quaffleDetectedRight ==0) {
            forwardRight(TURNING_SPEED);
            Serial.println("Quaffle detected to the right, turning");
            quaffleDetectedRight = 1;
           }
           if(quaffleX<(155-ALLOWABLE_BUF)&& quaffleDetectedLeft ==0 && !quaffleDetectedRight) {
            forwardLeft(TURNING_SPEED);
            Serial.println("Quaffle detected to the left, turning");
            quaffleDetectedLeft = 1;
           }

           
           
           if(quaffleX<155+TARGET_BUF && quaffleDetectedRight==1 ) {
            forward(FORWARD_SPEED);
            quaffleDetectedRight = 0;
            Serial.println("Quaffle centered");
           }
           if(quaffleX>155-TARGET_BUF && quaffleDetectedLeft==1) {
            forward(FORWARD_SPEED);
            quaffleDetectedLeft = 0;
            Serial.println("Quaffle centered");
           }
           
        
        }
        
      }
      
      if((!quaffleSeenThisLoop||quaffleY > 175 )&& quaffleDetected){
        Serial.print("Ball disappeared ");
        Serial.print(millis());
        Serial.print(" ");
        Serial.println(i);
        quaffleDetectedRight = 0;
        quaffleDetectedLeft = 0;
        quaffleDetected = 0;
        forward(FORWARD_SPEED);
      }
      
      
    }
  }
  }
  brake();
  }


//-----------------SENSOR FUNCTIONS--------------------
float getIrVal(Sensor sensor){
  //return sensor.value * sensor.modifier;
 // return 37-sensor.value;
 return 13*pow(analogRead(sensor.pin)*0.0048828125, -1);
}
float getSonarVal(Sensor sensor) {
  return analogRead(sensor.pin)/5;
}
float getLightVal(Sensor sensor) {
  return pow(10, analogRead(sensor.pin)*5.0/1024);
}
int updateSensors() {
  for(int i=0; i < SENSOR_QUANT;i++){
    //Serial.print("Update Sensors ");
    //Serial.println(analogRead(sensors[i].pin));
    sensors[i].value = analogRead(sensors[i].pin);
    //Serial.print("updateSensors val: ");
    //Serial.println(sensors[i].value);
  }
}

/*int greenBlockDetected( blocks){
  int blockDetected = 0;
  for(int i=0;i<blocks;i++){
    if(blocks[i].signature == 2) blockDetected = 1;
  }
}
*/
