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

//INTEGRATOR CONSTANTS
double const DECAY = 2;
int const INTEGRATE = 20;
int const SENSITIVITY = 50;
unsigned long const INTERVAL = 250;

//COLLECTING TIMES
unsigned long const COLLECTING_TIME = 20 * 1000L;
unsigned long const QUICK_COLLECTING_TIME = 5 * 1000L;

//MISC
int const LOW_BATTERY_LEVEL = 925;
unsigned long const END_TIME = 60 * 1000L;

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
int state = COLLECTING;

//location variables
int location;
int currSide;
int homeSide;
int goalSide;

//goal detection variables
int goalDetected = 0;
int goalX;
int goalY;
int goalSeenThisLoop = 0;
int goalCentered = 0;
int goalDetectedLeft;
int goalDetectedRight;

//integrator variables
double leakyIntegrator = 40;
unsigned long previousMillis = 0;

//scoring variables
unsigned long scoreTime = COLLECTING_TIME;

#endif
