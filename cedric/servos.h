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
int const RIGHT_START = 13;
int const LEFT_START = 160 - RIGHT_START;
int const LIFT_ANGLE = 35;
const int HIGHER_LIFT_ANGLE = 55;

//ROLLER STATE CONSTANTS
int const IN = 1;
int const OUT = 2;
int const STOP = 0;

//------------------------DECLARATIONS------------------------
Servo leftServo;
Servo rightServo;
Servo roller;

//------------------------SERVO FUNCTIONS------------------------

// ROLLIN
// ------------------------------------------
// starts roller to collect balls
void rollIn() {
  roller.attach(ROLLER_PIN);
  roller.write(0);
  //update roller state
  rollerState = IN;
}

// ROLLOUT
// ------------------------------------------
// starts roller to score
void rollOut() {
  roller.attach(ROLLER_PIN);
  roller.write(180);
  //update roller state
  rollerState = OUT;
}

// STOPROLL
// ------------------------------------------
// stops and detaches roller servo
void stopRoll() {
  roller.detach();
  //update roller state
  rollerState = STOP;
}

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

// RAISEARMHIGHER
// ------------------------------------------
// raises arm higher to get over wall
void raiseArmHigher() {
  //attaches both arm servos
  leftServo.attach(LEFT_SERVO_PIN);
  rightServo.attach(RIGHT_SERVO_PIN);

  for (int pos = 0; pos <= HIGHER_LIFT_ANGLE; pos++) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    rightServo.write(RIGHT_START + pos); // tell servo to go to position in variable 'pos'
    leftServo.write(LEFT_START - pos);
    delay(15); // waits 15ms for the servo to reach the position
  }

  if (rollerState == IN) {
    stopRoll();
  }
}

// LOWERARMHIGHER
// ------------------------------------------
// lowers arm from higher pos
void lowerArmHigher() {

  for (int pos = HIGHER_LIFT_ANGLE; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    rightServo.write(pos + RIGHT_START); // tell servo to go to position in variable 'pos'
    leftServo.write(LEFT_START - pos);
    delay(15); // waits 15ms for the servo to reach the position
  }
  //detaches both arm servos
  leftServo.detach();
  rightServo.detach();

  if (rollerState == STOP) {
    rollIn();
  }
}

#endif
