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
int const FORWARD_SPEED = 180;
int const TURNING_SPEED = 180;
int const REVERSE_SPEED = 170;
//FOR TURNING
double const SPEED_DIVISOR = 1.2;

//CORRECTION FOR MOTOR DRIFT
int const CORRECTION = 15;

//------------------------BASIC MOTOR FUNCTIONS------------------------

// BRAKE
// ------------------------------------------
// stop moving
void brake() {
  Serial.println("Braking");
  digitalWrite(B_BRAKE_PIN, HIGH);
  digitalWrite(A_BRAKE_PIN, HIGH);
  //  delay(50); //stop running code and wait for X milliseconds
}

// FORWARD
// ------------------------------------------
// move forward at SPEED
void forward(int speed) {
  //  Serial.print("Forward ");
  Serial.println(speed);
  digitalWrite(B_DIRECTION_PIN, B_FORWARD);
  digitalWrite(B_BRAKE_PIN, LOW);
  analogWrite(B_SPEED_PIN, speed);
  digitalWrite(A_DIRECTION_PIN, A_FORWARD);
  digitalWrite(A_BRAKE_PIN, LOW);
  analogWrite(A_SPEED_PIN, (CORRECTION + speed));
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
  delay(time);
  forward(FORWARD_SPEED);
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

// RANDSPIN
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

void reverse_right(){
  reverse(REVERSE_SPEED);
    delay(300);
    right(TURNING_SPEED);
    delay(700);
    forward(FORWARD_SPEED);
}
void reverse_left(){
  reverse(REVERSE_SPEED);
    delay(300);
    left(TURNING_SPEED);
    delay(700);
    forward(FORWARD_SPEED);
}
void turn_left(){
    left(TURNING_SPEED);
    delay(300);
    forward(FORWARD_SPEED);
}
void turn_right(){
    right(TURNING_SPEED);
    delay(300);
    forward(FORWARD_SPEED);
}

void turn_right_90(){
    right(TURNING_SPEED);
    delay(300);
    forward(FORWARD_SPEED);
}
void turn_180 (){
    reverse(REVERSE_SPEED);
    delay(750);
    left(TURNING_SPEED);
    delay(1300);
    forward(FORWARD_SPEED);
}
//turns right 180
void right180() {
  right(TURNING_SPEED);
  delay(1000);
  forward(FORWARD_SPEED);
}
void rand_turn() {
  int x = random(0, 2);
  if (x == 1) {
    reverse_left();
  } else {
    reverse_right();
  }
}

#endif
