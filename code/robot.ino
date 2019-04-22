#include <Adafruit_TCS34725softi2c.h>

#include <RPLidar.h>
#include <Wire.h>
#include <Servo.h>
#include "Adafruit_TCS34725.h"
#include <SPI.h>
#include <Pixy.h>
#define redpin 3
#define greenpin 5
#define bluepin 6
#define commonAnode true
#define SDApin1 4
#define SCLpin1 5
#define SDApin2 6
#define SCLpin2 7
#define SDApin3 22
#define SCLpin3 24


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
int const FORWARD_SPEED = 150;
int const TURNING_SPEED = 150;
int const REVERSE_SPEED = 120;


int const END_TIME = 20000;

//-------------SERVO DECLARATIONS------------------------
Servo leftServo;
Servo rightServo;
Servo roller;
const int LEFT_SERVO_PIN = 46;
const int RIGHT_SERVO_PIN = 45;
const int ROLLER_PIN = 10;
const int LIFT_ANGLE = 50;
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
//COLOR SIGNATURES
int const GREEN_SIG = 2;
int const YELLOW_SIG = 1;

//State variables
int quaffleDetectedRight;
int quaffleDetectedLeft;
int quaffleDetected;
int quaffleX;
int quaffleY;


//Sensor sensors[] = {irSensor};
Sensor sensors[] = {{A7, 0}, {A8, 0}, {A9, 0}, {LIGHT_PIN, 0}};
Sensor irSensor = sensors[0];

const int SENSOR_QUANT = sizeof(sensors) / sizeof(sensors[0]);
//color sensors
Adafruit_TCS34725softi2c leftRGB = Adafruit_TCS34725softi2c(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_4X, SDApin1, SCLpin1);
Adafruit_TCS34725softi2c rightRGB = Adafruit_TCS34725softi2c(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_4X, SDApin2, SCLpin2);
Adafruit_TCS34725softi2c rearRGB = Adafruit_TCS34725softi2c(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_4X, SDApin3, SCLpin3);


void setup() {
  delay(500);
  pinMode(irSensor.pin, INPUT);
  pinMode(A_DIRECTION_PIN, OUTPUT);
  pinMode(B_DIRECTION_PIN, OUTPUT);
  pinMode(A_BRAKE_PIN, OUTPUT);
  pinMode(B_BRAKE_PIN, OUTPUT);
  Serial.begin(9600);
  leftRGB.begin();
  rightRGB.begin();
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

  initializeServos();
  rollIn();

}

void loop() {
  uint16_t red, green, blue, clear;
  float leftHue, rightHue, s, v;


  while (millis() < END_TIME) {
    leftRGB.setInterrupt(false);
    rightRGB.setInterrupt(false);

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
    leftRGB.getRawData(&red, &green, &blue, &clear);
    RGBtoHSV(red, green, blue, &leftHue, &s, &v);
    rightRGB.getRawData(&red, &green, &blue, &clear);
    RGBtoHSV(red, green, blue, &rightHue, &s, &v);

    //end HSV source ------------------


    if (leftHue < 90 && leftHue > 50) {
      Serial.println("yellow detected left");
      turn_right();
    }
    if (rightHue < 90 && rightHue > 50) {
      Serial.println("yellow detected right");
      turn_left();
    }
    /*
      if(getIrVal(sensors[0])<6){
      Serial.println("Ir right");
      turn_right();
      }

      if(getIrVal(sensors[1])<6){
      Serial.println("Ir left");
      turn_right();
      }

      if(getLightVal(sensors[3])>200){
      //Serial.print("Light ");
      //Serial.println(getLightVal(sensors[3]));
      }*/
    //from pixy source ----------------
    if (blocks)
    {
      i++;

      // do this (print) every 50 frames because printing every
      // frame would bog down the Arduino
      if (i % 5 == 0)
      {
        sprintf(buf, "Detected %d:\n", blocks);
        //Serial.print(buf);
        for (j = 0; j < blocks; j++)
        {
          //end from pixy source--------------
          /*
            sprintf(buf, "  block %d: ", j);
            Serial.print(buf);
            pixy.blocks[j].print();*/

          if (pixy.blocks[j].signature == GREEN_SIG) {
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

            if (quaffleX > (155 + ALLOWABLE_BUF) && quaffleDetectedRight == 0) {
              forwardRight(TURNING_SPEED);
              Serial.println("Quaffle detected to the right, turning");
              quaffleDetectedRight = 1;
            }
            if (quaffleX < (155 - ALLOWABLE_BUF) && quaffleDetectedLeft == 0 && !quaffleDetectedRight) {
              forwardLeft(TURNING_SPEED);
              Serial.println("Quaffle detected to the left, turning");
              quaffleDetectedLeft = 1;
            }



            if (quaffleX < 155 + TARGET_BUF && quaffleDetectedRight == 1 ) {
              forward(FORWARD_SPEED);
              quaffleDetectedRight = 0;
              Serial.println("Quaffle centered");
            }
            if (quaffleX > 155 - TARGET_BUF && quaffleDetectedLeft == 1) {
              forward(FORWARD_SPEED);
              quaffleDetectedLeft = 0;
              Serial.println("Quaffle centered");
            }


          }
          if (pixy.blocks[j].signature == YELLOW_SIG) {

          }

        }

        if ((!quaffleSeenThisLoop || quaffleY > 175 ) && quaffleDetected) {
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
float getIrVal(Sensor sensor) {
  //return sensor.value * sensor.modifier;
  // return 37-sensor.value;
  return 13 * pow(analogRead(sensor.pin) * 0.0048828125, -1);
}
float getSonarVal(Sensor sensor) {
  return analogRead(sensor.pin) / 5;
}
float getLightVal(Sensor sensor) {
  return pow(10, analogRead(sensor.pin) * 5.0 / 1024);
}
int updateSensors() {
  for (int i = 0; i < SENSOR_QUANT; i++) {
    //Serial.print("Update Sensors ");
    //Serial.println(analogRead(sensors[i].pin));
    sensors[i].value = analogRead(sensors[i].pin);
    //Serial.print("updateSensors val: ");
    //Serial.println(sensors[i].value);
  }
}
/*
int greenBlockDetected( blocks){
  int blockDetected = 0;
  for(int i=0;i<blocks;i++){
    if(blocks[i].signature == 2) blockDetected = 1;
  }
  }
*/

//SOURCES:
//PIXY source:
// begin license header
//
// This file is part of Pixy CMUcam5 or "Pixy" for short
//
// All Pixy source code is provided under the terms of the
// GNU General Public License v2 (http://www.gnu.org/licenses/gpl-2.0.html).
// Those wishing to use Pixy source code, software and/or
// technologies under different licensing terms should contact us at
// cmucam@cs.cmu.edu. Such licensing terms are available for
// all portions of the Pixy codebase presented here.
//
// end license header
//HSV source:
// https://www.cs.rit.edu/~ncs/color/t_convert.html
