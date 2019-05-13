#ifndef MOTORS_H
#define MOTORS_H

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
//MOTOR SHIELD
int const A_FORWARD = HIGH;
int const B_FORWARD = LOW;
int const A_REVERSE = !A_FORWARD;
int const B_REVERSE = !B_FORWARD;
int const A_DIRECTION_PIN = 12;
int const A_BRAKE_PIN = 9;
int const A_SPEED_PIN = 3;
int const B_DIRECTION_PIN = 13;
int const B_BRAKE_PIN = 8;
int const B_SPEED_PIN = 11;

//------------------------CONSTANTS------------------------
//MOTOR SPEEDS
int const FULL_SPEED = 255;
int const FORWARD_SPEED = 200;
int const TURNING_SPEED = 200;
int const REVERSE_SPEED = 200;
//FOR TURNING
double const SPEED_DIVISOR = 1.9;
//MOTOR SPEEDS FOR FOLLOW LINE AND SCORING
int const LINE_SPEED = 160;
int const SCORING_SPEED = 180;

//CORRECTION FOR MOTOR DRIFT
int const CORRECTION = -20;

//MOTOR STATE CONSTANTS
int const BRAKE     = 0;
int const FORWARD   = 1;
int const REVERSE   = 2;
int const SPINLEFT  = 3;
int const SPINRIGHT = 4;
int const TURNLEFT  = 5;
int const TURNRIGHT = 6;

//------------------------BASIC MOTOR FUNCTIONS------------------------

// BRAKE
// ------------------------------------------
// stop moving
void brake() {
  //Serial.println("Braking");
  digitalWrite(B_BRAKE_PIN, HIGH);
  digitalWrite(A_BRAKE_PIN, HIGH);
  motorState = BRAKE;
  //  delay(50); //stop running code and wait for X milliseconds
}

// FORWARD
// ------------------------------------------
// move forward at SPEED
void forward(int speed) {
  //Serial.print("Forward ");
  //Serial.println(speed);
  digitalWrite(B_DIRECTION_PIN, B_FORWARD);
  digitalWrite(B_BRAKE_PIN, LOW);
  analogWrite(B_SPEED_PIN, speed);
  digitalWrite(A_DIRECTION_PIN, A_FORWARD);
  digitalWrite(A_BRAKE_PIN, LOW);
  analogWrite(A_SPEED_PIN, (CORRECTION + speed));
  motorState = FORWARD;
}

// REVERSE
// ------------------------------------------
// move backwards at SPEED
void reverse(int speed) {
  brake();
  //  Serial.println("Reversing ");
  digitalWrite(B_DIRECTION_PIN, B_REVERSE);
  digitalWrite(B_BRAKE_PIN, LOW);
  analogWrite(B_SPEED_PIN, speed);
  digitalWrite(A_DIRECTION_PIN, A_REVERSE);
  digitalWrite(A_BRAKE_PIN, LOW);
  analogWrite(A_SPEED_PIN, CORRECTION + speed);
  motorState = REVERSE;
}

// LEFT
// ------------------------------------------
// turn left ON THE SPOT at SPEED
void left(int speed) {
  //  Serial.println("left");
  digitalWrite(B_DIRECTION_PIN, B_REVERSE);
  digitalWrite(A_DIRECTION_PIN, A_FORWARD);
  digitalWrite(B_BRAKE_PIN, LOW);
  digitalWrite(A_BRAKE_PIN, LOW);
  analogWrite(B_SPEED_PIN, speed);
  analogWrite(A_SPEED_PIN, speed);
  motorState = SPINLEFT;
}

// RIGHT
// ------------------------------------------
// turn right ON THE SPOT at SPEED
void right(int speed) {
  //  Serial.println("right");
  digitalWrite(B_DIRECTION_PIN, B_FORWARD);
  digitalWrite(A_DIRECTION_PIN, A_REVERSE);
  digitalWrite(B_BRAKE_PIN, LOW);
  digitalWrite(A_BRAKE_PIN, LOW);
  analogWrite(B_SPEED_PIN, speed);
  analogWrite(A_SPEED_PIN, speed);
  motorState = SPINRIGHT;
}

// FORWARDRIGHT
// ------------------------------------------
// turn right WHILE MOVING FORWARD at SPEED
void forwardRight(int speed) {
  //  Serial.println("forward right");
  digitalWrite(B_DIRECTION_PIN, B_FORWARD);
  digitalWrite(A_DIRECTION_PIN, A_FORWARD);
  digitalWrite(B_BRAKE_PIN, LOW);
  digitalWrite(A_BRAKE_PIN, LOW);
  analogWrite(B_SPEED_PIN, speed);
  analogWrite(A_SPEED_PIN, speed / SPEED_DIVISOR);
  motorState = TURNRIGHT;
}

// FORWARDLEFT
// ------------------------------------------
// turn left WHILE MOVING FORWARD at SPEED
void forwardLeft(int speed) {
  //  Serial.println("forward left");
  digitalWrite(B_DIRECTION_PIN, B_FORWARD);
  digitalWrite(A_DIRECTION_PIN, A_FORWARD);
  digitalWrite(B_BRAKE_PIN, LOW);
  digitalWrite(A_BRAKE_PIN, LOW);
  analogWrite(B_SPEED_PIN, speed / SPEED_DIVISOR);
  analogWrite(A_SPEED_PIN, speed);
  motorState = TURNLEFT;
}

// EXCITED
// ------------------------------------------
// FOR TESTING:
// moves forwards and then backwards
void excited() {
  forward(FORWARD_SPEED);
  delay(500);
  reverse(REVERSE_SPEED);
  delay(500);
  brake();
}

//------------------------MOTOR CONTROL WRAPPER FUNCTIONS------------------------

// REVERSETIME
// ------------------------------------------
// Deterministic: moves backwards for TIME,
// then moves forward again.
void reverseTime(unsigned long time) {
  reverse(REVERSE_SPEED);
  //stops roller (if on) while reversing
  if (rollerState == IN) {
    stopRoll();
    delay(time);
    forward(FORWARD_SPEED);
    rollIn();
  } else {
    delay(time);
    forward(FORWARD_SPEED);
  }
}

// SPINLEFTTIME
// ------------------------------------------
// Deterministic: turns left ON THE SPOT for TIME,
// then moves forward again.
void spinLeftTime(unsigned long time) {
  left(TURNING_SPEED);
  delay(time);
  forward(FORWARD_SPEED);
}

// SPINRIGHTTIME
// ------------------------------------------
// Deterministic: turns right ON THE SPOT for TIME,
// then moves forward again.
void spinRightTime(unsigned long time) {
  right(TURNING_SPEED);
  delay(time);
  forward(FORWARD_SPEED);
}

// TURNLEFTTIME
// ------------------------------------------
// Deterministic: moves forward left for TIME,
// then moves forward again.
void turnLeftTime(unsigned long time) {
  forwardLeft(TURNING_SPEED);
  delay(time);
  forward(FORWARD_SPEED);
}

// TURNRIGHTTIME
// ------------------------------------------
// Deterministic: moves forward right for TIME,
// then moves forward again.
void turnRightTime(unsigned long time) {
  forwardRight(TURNING_SPEED);
  delay(time);
  forward(FORWARD_SPEED);
}

// RANDSPIN
// ------------------------------------------
// Deterministic: turns randomly right or left ON THE SPOT for TIME
void randSpin(unsigned long time) {
  int x = random(0, 2);
  if (x == 1) {
    spinRightTime(time);
  } else {
    spinLeftTime(time);
  }
}

// RANDTURN
// ------------------------------------------
// Deterministic: moves randomly forward right or forward left for TIME
void randTurn(unsigned long time) {
  int x = random(0, 2);
  if (x == 1) {
    turnRightTime(time);
  } else {
    turnLeftTime(time);
  }
}

//------------------------NAVIGATION FUNCTIONS------------------------

// SLOWLINEUPWALL
// ------------------------------------------
void slowLineUpWall() {

  while (abs(getIRVal(LEFT_FRONT_IR_PIN) - getIRVal(RIGHT_FRONT_IR_PIN)) > .1) {
    Serial.print(getIRVal(LEFT_FRONT_IR_PIN));
    Serial.print(" ");
    Serial.println(getIRVal(RIGHT_FRONT_IR_PIN));

    while (getIRVal(LEFT_FRONT_IR_PIN) < getIRVal(RIGHT_FRONT_IR_PIN)) {
      Serial.println("rotating left");
      left(TURNING_SPEED);
      delay(30);
      brake();
    }

    while (getIRVal(RIGHT_FRONT_IR_PIN) < getIRVal(LEFT_FRONT_IR_PIN)) {
      Serial.println("rotating right");
      right(TURNING_SPEED);
      delay(30);
      brake();
    }
  }

  //double check if robot has lined up with wall correctly
  if (abs(getIRVal(LEFT_FRONT_IR_PIN) - getIRVal(RIGHT_FRONT_IR_PIN)) > .1) {
    slowLineUpWall();
  }
}

// LINEUPWALL
// ------------------------------------------
void lineUpWall() {

  while (abs(getIRVal(LEFT_FRONT_IR_PIN) - getIRVal(RIGHT_FRONT_IR_PIN)) > .1) {
    Serial.print(getIRVal(LEFT_FRONT_IR_PIN));
    Serial.print(" ");
    Serial.println(getIRVal(RIGHT_FRONT_IR_PIN));

    while (getIRVal(LEFT_FRONT_IR_PIN) < getIRVal(RIGHT_FRONT_IR_PIN)) {
      Serial.println("rotating left");
      left(TURNING_SPEED);
    }

    while (getIRVal(RIGHT_FRONT_IR_PIN) < getIRVal(LEFT_FRONT_IR_PIN)) {
      Serial.println("rotating right");
      right(TURNING_SPEED);
    }
  }
}

#endif
