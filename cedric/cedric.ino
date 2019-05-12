#include <Adafruit_TCS34725softi2c.h>

#include <Arduino.h>
#include <RPLidar.h>
#include <Wire.h>
#include <Servo.h>
#include <SPI.h>
#include <Pixy.h>
#include <Adafruit_TCS34725.h>
#include "sensors.h"
#include "motors.h"
#include "servos.h"
#include "variables.h"

int const STARTINGSIDE = BLUE_SIDE; //CHOOSE STARTING SIDE

void setup() {
  //WAIT FOR X SECONDS
  delay(0 * 1000L);

  //------------------------PIN MODES------------------------
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

  //------------------------START ROBOT------------------------
  Serial.begin(9600);
  Serial.println("-------------------CEDRIC DIGGORY-------------------");
  Serial.println("----------TRIWIZARD TOURNAMENT CO-CHAMPION----------");
  Serial.print("booting");
  for (int i = 0; i < 20; i++) {
    delay(200);
    Serial.print(".");
  }
  Serial.println();

  //START RGB SENSORS AND PIXY
  leftRGB.begin();
  rightRGB.begin();
  pixy.init();

  //RESET VARIABLES
  quaffleDetectedRight = 0;
  quaffleDetectedLeft = 0;
  leakyIntegrator = 40;
  location = 0;
  currSide = 0;
  state = COLLECTING;

  //SET INITIAL STATE
  if (STARTINGSIDE == BLUE_SIDE) {
    Serial.println("BLUE side home");
    currSide = BLUE_SIDE;
    location = BLUE_SIDE;
    homeSide = BLUE_SIDE;
    goalSide = RED_SIDE;
    scoringWallSidePin = RIGHT_IR_PIN;
    scoringTurnDirection = LEFT;
  }
  if (STARTINGSIDE == RED_SIDE) {
    Serial.println("RED side home");
    currSide = RED_SIDE;
    location = RED_SIDE;
    homeSide = RED_SIDE;
    goalSide = BLUE_SIDE;
    scoringWallSidePin = RIGHT_IR_PIN;
    scoringTurnDirection = LEFT;
  }

  //START MOVING
  forward(FORWARD_SPEED);
  rollIn();
}

void loop() {
  //------------------------MAIN ROBOT CODE------------------------
  while (millis() < END_TIME) {

    //UPDATE VARIABLES AND GRAB SENSOR DATA
    unsigned long currentMillis = millis();
    //grab pixy blocks
    uint16_t blocks;
    char buf[32];
    blocks = pixy.getBlocks();
    //generic loop counter for Pixy
    int j;
    quaffleSeenThisLoop = 0;
    //grab RGB sensor data & convert to HSV
    leftRGB.getRawData(&red, &green, &blue, &clear);
    RGBtoHSV(red, green, blue, &leftHue, &s, &v);
    rightRGB.getRawData(&red, &green, &blue, &clear);
    RGBtoHSV(red, green, blue, &rightHue, &s, &v);

    //CHECK BATTERY LEVEL
    if (getVoltage() < LOW_BATTERY_LEVEL) {
      Serial.println("I'LL BE BACK");
    }

    //SWITCH TO SCORING MODE IF TIME HAS PAST
    if (currentMillis >= scoreTime && isRed(rightHue) && state == COLLECTING) {
      if (currSide == RED_SIDE) {
        reverse(REVERSE_SPEED);
        delay(300);
      }
      brake();
      state = FOLLOW_LINE;
      raiseArmHigher();
      Serial.println("found line");
    }

    //------------------------COLLECTING STATE------------------------
    if (state == COLLECTING) {

      //UPDATE LEAKY INTEGRATOR
      if (leakyIntegrator > DECAY && currentMillis - previousMillis >= INTERVAL) {
        leakyIntegrator -= DECAY;
        previousMillis = millis(); //updates time
        //prints integrator values
        //Serial.print("Integrator: ");
        //Serial.println(leakyIntegrator);
      }

      //IF INTEGRATOR IS HIGH, REVERSE A BIT THEN TURN UNTIL YOU SEE OPEN SPACE
      if (leakyIntegrator > SENSITIVITY) {
        //variables to tune behavior
        int turnLoopAmount = 3 * 100; //roughly time in seconds
        int turnRandomTime = 600;
        int backTime = 500;
        //sets leaky integrator back to RESET value
        leakyIntegrator = RESET;
        //prints that integrator is high
        Serial.println("Integrator HIGH");

        //reverses a bit
        reverseTime(backTime);

        //turns the opposite direction to current motor state
        if (motorState == SPINLEFT || motorState == TURNLEFT) {
          right(TURNING_SPEED);
          Serial.println("Turning RIGHT (high)");
          for (int i = 0; i < turnLoopAmount; i++ && getIRVal(LEFT_FRONT_IR_PIN) < 40 && getIRVal(RIGHT_FRONT_IR_PIN) < 40) {
            delay(10);
          }
          forward(FORWARD_SPEED);
        }
        else if (motorState == SPINRIGHT || motorState == TURNRIGHT) {
          left(TURNING_SPEED);
          Serial.println("Turning LEFT (high)");
          for (int i = 0; i < turnLoopAmount; i++ && getIRVal(LEFT_FRONT_IR_PIN) < 40 && getIRVal(RIGHT_FRONT_IR_PIN) < 40) {
            delay(10);
          }
          forward(FORWARD_SPEED);
        }
        //or do a random turn if robot is not currently turning
        else {
          Serial.println("Turning RANDOM (high)");
          randTurn(turnRandomTime);
        }
      }

      //IF INTEGRATOR DECAYS TO ZERO, BACK UP AND THEN TURN IN OPPOSITE DIRECTION TO CURRENT TURNING DIRECTION
      if (leakyIntegrator <= DECAY) {
        //variables to tune behavior
        int turnOppositeTime = 400;
        int turnRandomTime = 600;
        int backTime = 500;
        //reset leaky integrator to RESET
        leakyIntegrator = RESET;
        //prints that integrator is LOW
        Serial.println("Integrator LOW");

        //reverses a bit
        reverseTime(backTime);

        //turns opposite direction to current motor state
        if (motorState == SPINLEFT || motorState == TURNLEFT) {
          Serial.println("Turning RIGHT (low)");
          spinRightTime(turnOppositeTime);
        }
        else if (motorState == SPINRIGHT || motorState == TURNRIGHT) {
          Serial.println("Turning LEFT (low)");
          spinLeftTime(turnOppositeTime);
        }
        //or do a random turn if robot is not currently turning
        else {
          Serial.println("Turning RANDOM (low)");
          randTurn(turnRandomTime);
        }
      }

      //IF LEFT RGB SENSOR OR FRONT LEFT IR SENSOR IS TRIGGERED, BACK UP A BIT AND SPIN RIGHT
      if (isYellow(leftHue) || getIRVal(LEFT_FRONT_IR_PIN) < 6) {
        //variables to tune behavior
        unsigned long backTime = 200;
        unsigned long turnTime = 400;
        //integrate leaky integrator
        leakyIntegrator += INTEGRATE;
        Serial.println("LRGB/FLIR turning RIGHT");

        reverseTime(backTime);
        spinRightTime(turnTime);
      }

      //IF RIGHT RGB SENSOR OR FRONT RIGHT IR SENSOR IS TRIGGERED, BACK UP A BIT AND SPIN LEFT
      if (isYellow(rightHue) || getIRVal(RIGHT_FRONT_IR_PIN) < 6) {
        //variables to tune behavior
        unsigned long backTime = 200;
        unsigned long turnTime = 400;
        //integrate leaky integrator
        leakyIntegrator += INTEGRATE;
        Serial.println("RRGB/FRIR turning LEFT");

        reverseTime(backTime);
        spinLeftTime(turnTime);
      }

      //IF LEFT IR SENSOR OR RIGHT REAR IR SENSOR IS TRIGGERED, TURN RIGHT
      if (getIRVal(LEFT_IR_PIN) < 5 || getIRVal(RIGHT_REAR_IR_PIN) < 5) {
        //variables to tune behavior
        unsigned long turnTime = 500;
        //integrate leaky integrator
        leakyIntegrator += INTEGRATE;
        Serial.println("LIR/RRIR turning RIGHT");

        turnLeftTime(turnTime);
      }

      //IF RIGHT IR SENSOR OR LEFT REAR IR SENSOR IS TRIGGERED, TURN LEFT
      if (getIRVal(RIGHT_IR_PIN) < 5 || getIRVal(LEFT_REAR_IR_PIN) < 5) {
        //variables to tune behavior
        unsigned long turnTime = 500;
        //integrate leaky integrator
        leakyIntegrator += INTEGRATE;
        Serial.println("RIR/RLIR turning RIGHT");

        turnRightTime(turnTime);
      }

      //IF PIXYCAM SEES A GREEN BALL, UPDATE VARIABLES AND TRY TO FOLLOW
      if (blocks) {

        //for each block, check signature
        for (j = 0; j < blocks; j++) {
          if (pixy.blocks[j].signature == GREEN_SIG) {
            //update variables
            loopsSinceQuaffleSeen = 0;
            quaffleSeenThisLoop = 1;
            quaffleDetected = 1;
            //get quaffle X and Y
            quaffleX = pixy.blocks[j].x;
            quaffleY = pixy.blocks[j].y;
            Serial.println("quaffle DETECTED");
          }
        }

        //TRY TO FOLLOW A GREEN BALL IF IT EXISTS
        if (quaffleSeenThisLoop) {
          //if quaffle is within buffer begin turning
          if (quaffleX > (155 + ALLOWABLE_BUF) && quaffleDetectedRight == 0) {
            forwardRight(TURNING_SPEED);
            Serial.println("quaffle detected to the RIGHT");
            quaffleDetectedRight = 1;
          }
          if (quaffleX < (155 - ALLOWABLE_BUF) && quaffleDetectedLeft == 0 && !quaffleDetectedRight) {
            forwardLeft(TURNING_SPEED);
            Serial.println("quaffle detected to the LEFT");
            quaffleDetectedLeft = 1;
          }
          //if quaffle is centered then go forward
          if (quaffleX < 155 + TARGET_BUF && quaffleDetectedRight == 1 ) {
            forward(FORWARD_SPEED);
            quaffleDetectedRight = 0;
            Serial.println("quaffle CENTERED");
          }
          if (quaffleX > 155 - TARGET_BUF && quaffleDetectedLeft == 1) {
            forward(FORWARD_SPEED);
            quaffleDetectedLeft = 0;
            Serial.println("quaffle CENTERED");
          }
        }
      }

      //if no quaffle has been seen this loop, increment counter
      if (!quaffleSeenThisLoop) {
        loopsSinceQuaffleSeen ++;
      }

      //if quaffle hasn't been seen in X loops, update variables
      if (loopsSinceQuaffleSeen > 10 && quaffleDetected) {
        Serial.print("quaffle DISAPPEARED");
        //update variables
        quaffleDetectedRight = 0;
        quaffleDetectedLeft = 0;
        quaffleDetected = 0;
      }
    }

    //------------------------FOLLOW LINE STATE------------------------
    if (state == FOLLOW_LINE) {
      Serial.println("following line");
      Serial.println(rightHue);
      // Serial.println(rightHue);
      if (isRed(rightHue)) {
        Serial.println("red");
        forward(FORWARD_SPEED);
      }
      if (isBlue(rightHue)) {
        Serial.println("blue");
        if (homeSide == BLUE_SIDE) left(TURNING_SPEED);
        else right(TURNING_SPEED);
      }
      if (isGray(rightHue)) {
        Serial.println("gray");
        if (homeSide == BLUE_SIDE) right(TURNING_SPEED);
        else left(TURNING_SPEED);
      }
      if (isYellow(rightHue)) {
        brake();
        Serial.println("Wall detected");
        state = SCORING;
        brake();
        Serial.print(getIRVal(LEFT_FRONT_IR_PIN));
        Serial.print(" ");
        Serial.println(getIRVal(RIGHT_FRONT_IR_PIN));
        Serial.println(abs(getIRVal(LEFT_FRONT_IR_PIN) - getIRVal(RIGHT_FRONT_IR_PIN)));
        //lineUpWall();
        Serial.print(getIRVal(LEFT_FRONT_IR_PIN));
        Serial.print(" ");
        Serial.println(getIRVal(RIGHT_FRONT_IR_PIN));
        //turn right until nothing in front
        float sonarVal = getSonarVal(SONAR_PIN);
        Serial.println(sonarVal);
        while (sonarVal < 15) {
          if (scoringTurnDirection == RIGHT) right(TURNING_SPEED);
          else left(TURNING_SPEED);
          sonarVal = getSonarVal(SONAR_PIN);
          Serial.println(sonarVal);
        }
        Serial.println("done sonar turn");
        while (!isYellow(rightHue)) {
          rightRGB.getRawData(&red, &green, &blue, &clear);
          RGBtoHSV(red, green, blue, &rightHue, &s, &v);
          if (scoringTurnDirection == RIGHT) right(TURNING_SPEED);
          else left(TURNING_SPEED);
        }
        Serial.println("done no yellow turn");
        while (isYellow(rightHue)) {
          Serial.println(rightHue);
          rightRGB.getRawData(&red, &green, &blue, &clear);
          RGBtoHSV(red, green, blue, &rightHue, &s, &v);
          if (scoringTurnDirection == RIGHT) right(TURNING_SPEED);
          else left(TURNING_SPEED);
        }
        Serial.println("done yellow turn");
        brake();
        lowerArmHigher();
      }
    }

    //------------------------SCORING STATE------------------------
    if (state == SCORING) {
      float wallDist = getIRVal(scoringWallSidePin);
      Serial.println(wallDist);
      if (wallDist < 8 || isYellow(rightHue)) {
        Serial.println("too close to wall");
        if (scoringTurnDirection == RIGHT) forwardRight(FORWARD_SPEED);
        else forwardLeft(FORWARD_SPEED);
      }

      else {
        if (wallDist > 9) {
          if (scoringTurnDirection == RIGHT) forwardLeft(FORWARD_SPEED);
          else forwardRight(FORWARD_SPEED);
        }
        else {
          forward(FORWARD_SPEED);
        }
      }
      if (getIRVal(LEFT_FRONT_IR_PIN) < 7 ) {
        Serial.println("found goal");
        Serial.println(wallDist);
        Serial.println(getIRVal(LEFT_FRONT_IR_PIN));
        Serial.println(getIRVal(RIGHT_FRONT_IR_PIN));
        state = COLLECTING;
        scoreTime = currentMillis + COLLECTING_TIME;
        slowLineUpWall();
        delay(1000);
        raiseArm();
        forward(FORWARD_SPEED);
        delay(400);
        brake();
        rollOut();
        delay(2000);
        reverse(FORWARD_SPEED);
        delay(500);
        brake();
        lowerArm();
        rollIn();
      }
    }
  }
  
  //STOP ROBOT
  brake();
  stopRoll();
  Serial.println("-------------------MISSION ACCOMPLISHED-------------------");
  Serial.println("----------------CEDRIC DIGGORY SIGNING OFF----------------");
  delay(100);
  exit(0);
}
