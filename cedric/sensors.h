#ifndef SENSORS_H
#define SENSORS_H

#include <Adafruit_TCS34725softi2c.h>

#include <Arduino.h>
#include <RPLidar.h>
#include <Wire.h>
#include <SPI.h>
#include <Pixy.h>
#include <Adafruit_TCS34725.h>
#include "servos.h"
#include "variables.h"

//------------------------PINS------------------------
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

//SENSORS
int const LEFT_LIGHT_PIN = A4;
int const RIGHT_LIGHT_PIN = A13;
int const LEFT_FRONT_IR_PIN = A7;
int const RIGHT_FRONT_IR_PIN = A15;
int const LEFT_REAR_IR_PIN = A5;
int const RIGHT_REAR_IR_PIN = A14;
int const LEFT_IR_PIN = A3;
int const RIGHT_IR_PIN = A12;
int const SONAR_PIN = A10;
int const VOLT_PIN = A11;

//------------------------CONSTANTS------------------------
//COLOR SIGNATURES
int const GREEN_SIG = 2;
int const YELLOW_SIG = 1;
int const LIGHT_SIG = 4;

//------------------------DECLARATIONS------------------------
Pixy pixy;
Adafruit_TCS34725softi2c rightRGB = Adafruit_TCS34725softi2c(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_4X, SDApin1, SCLpin1);
Adafruit_TCS34725softi2c leftRGB  = Adafruit_TCS34725softi2c(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_4X, SDApin2, SCLpin2);
Adafruit_TCS34725softi2c rearRGB  = Adafruit_TCS34725softi2c(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_4X, SDApin3, SCLpin3);

//------------------------SENSOR FUNCTIONS------------------------

// MIN
// ------------------------------------------
// Returns minimum of three values
float MIN(float a, float b, float c) {
  float currMin = a;
  if (b < currMin) currMin = b;
  if (c < currMin) currMin = c;
  return currMin;
}

// MAX
// ------------------------------------------
// Returns maximum of three values
float MAX(float a, float b, float c) {
  float currMax = a;
  if (b > currMax) currMax = b;
  if (c > currMax) currMax = c;
  return currMax;
}

// RGBTOHSV
// ------------------------------------------
// Converts RGB values to HSV values
// h = [0,360], s = [0,1], v = [0,1]
// if s == 0, then h = -1 (undefined)
void RGBtoHSV( float r, float g, float b, float *h, float *s, float *v )
{
  r = r / 255;
  g = g / 255;
  b = b / 255;
  float min, max, delta;

  min = MIN( r, g, b );
  max = MAX( r, g, b );
  *v = max;       // v

  delta = max - min;

  if ( max != 0 )
    *s = delta / max;   // s
  else {
    // r = g = b = 0    // s = 0, v is undefined
    *s = 0;
    *h = -1;
    return;
  }

  if ( r == max )
    *h = ( g - b ) / delta;   // between yellow & magenta
  else if ( g == max )
    *h = 2 + ( b - r ) / delta; // between cyan & yellow
  else
    *h = 4 + ( r - g ) / delta; // between magenta & cyan

  *h *= 60;       // degrees
  if ( *h < 0 )
    *h += 360;
}

//------------------------SENSOR CHECK FUNCTIONS------------------------

// ISRED?
// ------------------------------------------
// returns #t if RGB sensor sees red
bool isRed(float hue) {
  return hue < 363 && hue > 353;
}

// ISBLUE?
// ------------------------------------------
// returns #t if RGB sensor sees blue
bool isBlue(float hue) {
  return hue < 202 && hue > 192;
}

// ISYELLOW?
// ------------------------------------------
// returns #t if RGB sensor sees yellow
bool isYellow(float hue) {
  return hue < 95 && hue > 77;
}

// ISGRAY?
// ------------------------------------------
// returns #t if RGB sensor sees gray
bool isGray(float hue) {
  return !isYellow(hue) && !isRed(hue) && !isBlue(hue);
}

// GETIRVAL
// ------------------------------------------
// returns IR value of sensor at PIN
float getIRVal(int pin) {
  //return sensor.value * sensor.modifier;
  // return 37-sensor.value;
  return 13 * pow(analogRead(pin) * 0.0048828125, -1);
}

// GETSONARVAL
// ------------------------------------------
// returns sonar value of sensor at PIN
float getSonarVal(int pin) {
  return analogRead(pin) / 5;
}

// GETLIGHTVAL
// ------------------------------------------
// returns light value of sensor at PIN
float getLightVal(int pin) {
  return pow(10, analogRead(pin) * 5.0 / 1024);
}

// GETVOLTAGE
// ------------------------------------------
// returns analog value of battery at PIN
float getVoltage() {
  return analogRead(VOLT_PIN);
}

#endif
