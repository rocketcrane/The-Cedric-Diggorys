#ifndef SERVOS_H
#define SERVOS_H

#include <Adafruit_TCS34725softi2c.h>

#include <Arduino.h>
#include <RPLidar.h>
#include <Wire.h>
#include <Servo.h>
#include <SPI.h>
#include <Pixy.h>
#include <Adafruit_TCS34725.h>
#include "variables.h"

//------------------------PINS------------------------
//ACTUATORS
int const LEFT_SERVO_PIN = 46;
int const RIGHT_SERVO_PIN = 45;
int const ROLLER_PIN = 10;

//------------------------CONSTANTS------------------------
//ARM CONSTANTS
const int RIGHT_START = 13;
const int LEFT_START = 160 - RIGHT_START;
const int LIFT_ANGLE = 35;

//------------------------DECLARATIONS------------------------
Servo leftServo;
Servo rightServo;
Servo roller;

//------------------------SERVO FUNCTIONS------------------------

// RAISEARM
// ------------------------------------------
// raises arm
void raiseArm() {
  //attaches both arm servos
  leftServo.attach(LEFT_SERVO_PIN);
  rightServo.attach(RIGHT_SERVO_PIN);
  for (int pos = 0; pos <= LIFT_ANGLE; pos++) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    rightServo.write(RIGHT_START + pos); // tell servo to go to position in variable 'pos'
    leftServo.write(LEFT_START - pos);
    delay(15); // waits 15ms for the servo to reach the position
  }
}

// LOWERARM
// ------------------------------------------
// lowers arm
void lowerArm() {
  for (int pos = LIFT_ANGLE; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    rightServo.write(pos + RIGHT_START); // tell servo to go to position in variable 'pos'
    leftServo.write(LEFT_START - pos);
    delay(15); // waits 15ms for the servo to reach the position
  }
  //detaches both arm servos
  leftServo.detach();
  rightServo.detach();
}

// ROLLIN
// ------------------------------------------
// starts roller to collect balls
void rollIn(){
  roller.attach(ROLLER_PIN);
  roller.write(0);
}

// ROLLOUT
// ------------------------------------------
// starts roller to score
void rollOut(){
  roller.attach(ROLLER_PIN);
  roller.write(180);
}

// STOPROLL
// ------------------------------------------
// stops and detaches roller servo
void stopRoll(){
  roller.detach();
}

#endif
