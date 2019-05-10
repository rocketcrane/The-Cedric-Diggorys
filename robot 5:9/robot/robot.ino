#include <Adafruit_TCS34725softi2c.h>
#include <RPLidar.h>
#include <Wire.h>
#include <Servo.h>
#include <Adafruit_TCS34725.h>
#include <SPI.h>
#include <Pixy.h>

//------------------SENSOR PINS------------------
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

const int LEFT_LIGHT_PIN = A4;
const int RIGHT_LIGHT_PIN = A13;
const int LEFT_FRONT_IR_PIN = A7;
const int RIGHT_FRONT_IR_PIN = A15;
const int LEFT_REAR_IR_PIN = A5;
const int RIGHT_REAR_IR_PIN = A14;
const int LEFT_IR_PIN = A3;
const int RIGHT_IR_PIN = A12;
const int SONAR_PIN = A10;
const int VOLT_PIN = A11;
const int LOW_BATTERY_LEVEL = 920;

//------------MOTOR SHIELD---------------
//B is right motor, A is left motor
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

//max input speed possible is 240, because of 15 correction
int const FORWARD_SPEED = 240;
int const TURNING_SPEED = 240;
int const REVERSE_SPEED = 220;

//----------------RUN DURATION------------------
long const END_TIME = 40 * 1000L;

//-------------SERVO DECLARATIONS-------------------
Servo leftServo;
Servo rightServo;
Servo roller;
const int LEFT_SERVO_PIN = 46;
const int RIGHT_SERVO_PIN = 45;
const int ROLLER_PIN = 10;

//-------------PIXY DECLARATION---------------
Pixy pixy;

//Quaffle Detection Constants
int const TARGET_BUF = 30;
int const ALLOWABLE_BUF = 100;

//Color Signatures
int const GREEN_SIG = 2;
int const YELLOW_SIG = 1;

//State Variables
double leakyIntegrator = 40;
unsigned long previousMillis = 0;
unsigned long interval = 250;

//Integrator Constants
double const DECAY = 2;
int const INTEGRATE = 20;
int const SENSITIVITY = 50;

unsigned long const SCORETIME = 1 * 1000L;

//Color Sensors
Adafruit_TCS34725softi2c leftRGB  = Adafruit_TCS34725softi2c(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_4X, SDApin1, SCLpin1);
Adafruit_TCS34725softi2c rightRGB = Adafruit_TCS34725softi2c(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_4X, SDApin2, SCLpin2);
Adafruit_TCS34725softi2c rearRGB  = Adafruit_TCS34725softi2c(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_4X, SDApin3, SCLpin3);

//-----------------------------------SETUP------------------------------------
void setup() {

  delay(5000);
  //pinmodes
  pinMode(LEFT_FRONT_IR_PIN, INPUT);
  pinMode(RIGHT_FRONT_IR_PIN, INPUT);
  pinMode(LEFT_REAR_IR_PIN, INPUT);
  pinMode(RIGHT_REAR_IR_PIN, INPUT);
  pinMode(LEFT_IR_PIN, INPUT);
  pinMode(RIGHT_IR_PIN, INPUT);
  pinMode(SONAR_PIN, INPUT);
  pinMode(LEFT_LIGHT_PIN, INPUT);
  pinMode(RIGHT_LIGHT_PIN, INPUT);
  pinMode(VOLT_PIN, INPUT);
  pinMode(A_DIRECTION_PIN, OUTPUT);
  pinMode(B_DIRECTION_PIN, OUTPUT);
  pinMode(A_BRAKE_PIN, OUTPUT);
  pinMode(B_BRAKE_PIN, OUTPUT);
  //initialization
  Serial.begin(9600);
  leftRGB.begin();
  rightRGB.begin();
  pixy.init();

  /*#if defined(ARDUINO_ARCH_ESP32)
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
    #endif*/

  //Move!
  forward(FORWARD_SPEED);
  rollIn();
  leakyIntegrator = 40;
}

//-----------------------------------LOOP------------------------------------
void loop() {

  //Declarations
  uint16_t red, green, blue, clear;
  float leftHue, rightHue, s, v;

  //Run for duration
  while (millis() < END_TIME) {
    unsigned long currentMillis = millis(); //grab current time
    //leftRGB.setInterrupt(false);
    //rightRGB.setInterrupt(false);

    //Leaky Integrator decay function -
    //runs if integrator is greater than decay value and if interval of time has passed
    if (leakyIntegrator > DECAY && currentMillis - previousMillis >= interval) {
      leakyIntegrator -= DECAY;
      previousMillis = millis(); //updates time
    }

    //prints integrator
    Serial.print("integrator: ");
    Serial.println(leakyIntegrator);

    //from HSV source -----------------
    leftRGB.getRawData(&red, &green, &blue, &clear);
    RGBtoHSV(red, green, blue, &leftHue, &s, &v);
    rightRGB.getRawData(&red, &green, &blue, &clear);
    RGBtoHSV(red, green, blue, &rightHue, &s, &v);

    if (currentMillis >= SCORETIME && leftHue < 363 && leftHue > 353) {
      brake();
      stopRoll();
      while (1) {
      }
    }

    if (leakyIntegrator > SENSITIVITY) {
      Serial.println("repeated sensor inputs, turning");
      rand_turn();
      leakyIntegrator = 20;
    }

    if (leftHue < 87 && leftHue > 77) {
      Serial.println("yellow detected left");
      turn_right();
      leakyIntegrator += INTEGRATE;
    }

    if (rightHue < 90 && rightHue > 80) {
      Serial.println("yellow detected right");
      turn_left();
      leakyIntegrator += INTEGRATE;
    }

    if (getIrVal(LEFT_FRONT_IR_PIN) < 6) {
      Serial.println("Ir front right");
      reverse_left();
      leakyIntegrator += INTEGRATE;
    }

    if (getIrVal(RIGHT_FRONT_IR_PIN) < 6) {
      Serial.println("Ir front left");
      reverse_right();
      leakyIntegrator += INTEGRATE;
    }

    if (getIrVal(RIGHT_IR_PIN) < 6) {
      Serial.println("Ir right");
      turn_left();
      leakyIntegrator += INTEGRATE;
    }

    if (getIrVal(LEFT_IR_PIN) < 6) {
      Serial.println("Ir left");
      turn_right();
      leakyIntegrator += INTEGRATE;
    }

    if (getIrVal(RIGHT_REAR_IR_PIN) < 6) {
      Serial.println("Ir rear right");
      turn_right();
      leakyIntegrator += INTEGRATE;
    }

    if (getIrVal(LEFT_REAR_IR_PIN) < 6) {
      Serial.println("Ir rear left");
      turn_left();
      leakyIntegrator += INTEGRATE;
    }

    if (leakyIntegrator <= DECAY) {
      Serial.println("random turn, integrator 0");
      rand_turn();
      leakyIntegrator += INTEGRATE;
    }

    if (analogRead(VOLT_PIN) < LOW_BATTERY_LEVEL) {
      Serial.println("LOW BATTERY");
    }
  }
  brake();
  stopRoll();
}
