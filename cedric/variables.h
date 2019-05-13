#ifndef VARIABLES_H
#define VARIABLES_H

#include <Adafruit_TCS34725softi2c.h>

#include <Arduino.h>
#include <RPLidar.h>
#include <Wire.h>
#include <Servo.h>
#include <SPI.h>
#include <Pixy.h>
#include <Adafruit_TCS34725.h>

//------------------------CONSTANTS------------------------
//QUAFFLE DETECTION
int const TARGET_BUF = 30;
int const ALLOWABLE_BUF = 100;

//STATE CONSTANTS
int const COLLECTING = 0;
int const FOLLOW_LINE = 1;
int const SCORING = 2;

//LOCATIONS ON TAPE 
int const BLUE_SIDE = 1;
int const RED_SIDE = 2;
int const RED_TAPE = 3;
int const BLUE_TAPE = 4;

//DIRECTIONS
int const RIGHT = 0;
int const LEFT = 1;

//INTEGRATOR CONSTANTS
double const DECAY = 2;
int const INTEGRATE = 20;
int const SENSITIVITY = 50;
int const RESET = 35;
unsigned long const INTERVAL = 250;

//MISC
int const LOW_BATTERY_LEVEL = 910;

//------------------------VARIABLES------------------------
//state variables
int quaffleDetectedRight;
int quaffleDetectedLeft;
int quaffleDetected;
int quaffleSeenThisLoop;
int quaffleX;
int quaffleY;
int loopsSinceQuaffleSeen = 1000;
int turnCounter = 0;
int state;

//location variables
int location;
int currSide;
int homeSide;
int goalSide;
int scoringWallSidePin;
int lastColorSeen;

//goal detection variables
int goalDetected = 0;
int goalX;
int goalY;
int goalSeenThisLoop = 0;
int loopsSinceGoalSeen = 1000;
int goalCentered = 0;
int goalDetectedLeft;
int goalDetectedRight;
int scoringTurnDirection;

//integrator variables
double leakyIntegrator = 40;
unsigned long previousMillis = 0;

//color sensor variables
uint16_t red, green, blue, clear;
float leftHue, rightHue, rearHue, s, v;

//motor state variables
int motorState;
int rollerState;

#endif
