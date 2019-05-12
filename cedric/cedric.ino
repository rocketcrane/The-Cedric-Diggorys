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

void setup() {
  //WAIT FOR X MILLISECONDS
  delay(2 * 1000L);
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

  Serial.begin(9600);
  Serial.println("Starting");
  leftRGB.begin();
  rightRGB.begin();
  pixy.init();

  //set initial side
  float redSideLight = getLightVal(LEFT_LIGHT_PIN);
  float blueSideLight = getLightVal(RIGHT_LIGHT_PIN);
  if (blueSideLight > redSideLight) {
    Serial.println("on blue side");
    currSide = BLUE_SIDE;
    location = BLUE_SIDE;
    homeSide = BLUE_SIDE;
    goalSide = RED_SIDE;
    scoringWallSidePin = RIGHT_IR_PIN;
    scoringTurnDirection = LEFT;
  }
  else {
    Serial.println("on red side");
    delay(1000); //delete for comp =====
    currSide = RED_SIDE;
    location = RED_SIDE;
    homeSide = RED_SIDE;
    goalSide = BLUE_SIDE;
    scoringWallSidePin = RIGHT_IR_PIN;
    scoringTurnDirection = LEFT;
  }

  forward(FORWARD_SPEED);
  quaffleDetectedRight = 0;
  quaffleDetectedLeft = 0;
}

void loop() {
  uint16_t red, green, blue, clear;
  float leftHue, rightHue, s, v;



  while (millis() < END_TIME) {
    unsigned long currentMillis = millis();
    /*
        delay(200);
        float leftVal = getIrVal(LEFT_FRONT_IR_PIN);
        float rightVal = getIrVal(RIGHT_FRONT_IR_PIN);
        Serial.print(leftVal);
        Serial.print("  ");
        Serial.println(rightVal);
    */
    //Serial.println(millis());
    

    //switch states if time has past
    if (currentMillis >= scoreTime && isRed(rightHue) && state == COLLECTING) {
      if(currSide == RED_SIDE){
        reverse(FORWARD_SPEED);
        delay(300);
      }
      brake();
      state = FOLLOW_LINE;
      raiseArmHigher();
      Serial.println("found line");
    }
    //
    //        if (!quaffleSeenThisLoop) loopsSinceQuaffleSeen++;
    //        if (loopsSinceQuaffleSeen > 500) {
    //          stopRoll();
    //
    //        }

    if (leakyIntegrator > DECAY && currentMillis - previousMillis >= INTERVAL) {
      leakyIntegrator -= DECAY;
      previousMillis = millis(); //updates time
    }

    leftRGB.setInterrupt(false);
    rightRGB.setInterrupt(false);

    //generic loop counter
    static int i = 0;
    int j;
    quaffleSeenThisLoop = 0;


    //From pixy source---------------
    //pixy blocks
    uint16_t blocks;
    char buf[32];
    // grab blocks!

    blocks = pixy.getBlocks();
    //end pixy source-----------------


    //from HSV source ----------------
    leftRGB.getRawData(&red, &green, &blue, &clear);
    RGBtoHSV(red, green, blue, &leftHue, &s, &v);
    rightRGB.getRawData(&red, &green, &blue, &clear);
    RGBtoHSV(red, green, blue, &rightHue, &s, &v);

    if (isRed(rightHue)) {
      if (location == BLUE_TAPE) {
        currSide = RED_SIDE;
      }
      location = RED_TAPE;
    }
    if (isBlue(rightHue)) {
      if (location == RED_TAPE) {
        currSide = BLUE_SIDE;
      }
      location = BLUE_TAPE;
    }
    if (isGray(rightHue)) {
      currSide = location;
    }


    //end HSV source ------------------
    //Serial.println(getVoltage());
    Serial.println(state);
    if (state == COLLECTING) {
      Serial.println("in collecting if");
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
    }

    
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
        Serial.print(getIrVal(LEFT_FRONT_IR_PIN));
        Serial.print(" ");
        Serial.println(getIrVal(RIGHT_FRONT_IR_PIN));
        Serial.println(abs(getIrVal(LEFT_FRONT_IR_PIN) - getIrVal(RIGHT_FRONT_IR_PIN)));
        //lineUpWall();
        Serial.print(getIrVal(LEFT_FRONT_IR_PIN));
        Serial.print(" ");
        Serial.println(getIrVal(RIGHT_FRONT_IR_PIN));
        //turn right until nothing in front
        float sonarVal = getSonarVal(SONAR_PIN);
        Serial.println(sonarVal);
        while (sonarVal<15) {
          if (scoringTurnDirection == RIGHT) right(TURNING_SPEED);
          else left(TURNING_SPEED);
          sonarVal = getSonarVal(SONAR_PIN);
          Serial.println(sonarVal);        }
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

      //object in the way
//      if (getIrVal(LEFT_FRONT_IR_PIN) < 5 || getIrVal(RIGHT_FRONT_IR_PIN) < 5) {
//        brake();
//        delay(5000);
//        if (getIrVal(LEFT_FRONT_IR_PIN) < 5 || getIrVal(RIGHT_FRONT_IR_PIN) < 5) {
//          state = COLLECTING;
//          scoreTime = QUICK_COLLECTING_TIME;
//
//        }
//      }

    }

    if (state == SCORING) {
      float wallDist = getIrVal(scoringWallSidePin);
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
      if (getIrVal(LEFT_FRONT_IR_PIN)< 7 ) {
        Serial.println("found goal");
        Serial.println(wallDist);
        Serial.println(getIrVal(LEFT_FRONT_IR_PIN));
        Serial.println(getIrVal(RIGHT_FRONT_IR_PIN));
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

    if (analogRead(VOLT_PIN) < LOW_BATTERY_LEVEL) {
      Serial.println("LOW BATTERY");
    }

    if (getLightVal(LEFT_LIGHT_PIN) > 300) {
      float x = 0;
      const int q = 100;
      for (int i = 0; i < q; i++) {
        x += getLightVal(LEFT_LIGHT_PIN);
      }
      x = x / q;
      Serial.print("Light left ");
      Serial.println(x);
      delay(400);
    }
    if (getLightVal(RIGHT_LIGHT_PIN) > 300) {
      float x = 0;
      const int q = 100;
      for (int i = 0; i < q; i++) {
        x += getLightVal(RIGHT_LIGHT_PIN);
      }
      x = x / q;
      Serial.print("Light right ");
      Serial.println(x);
      delay(400);
    }
    //from pixy source ----------------
    if (blocks)
    {
      i++;
      // do this (print) every 50 frames because printing every
      // frame would bog down the Arduino
      if (i % 1 == 0)
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

          if (pixy.blocks[j].signature == GREEN_SIG && state == COLLECTING) {
            rollIn();
            loopsSinceQuaffleSeen = 0;

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
          if (pixy.blocks[j].signature == YELLOW_SIG && pixy.blocks[j].width * pixy.blocks[j].height > 400 && state == SCORING
              && pixy.blocks[j].height < 160) {
            goalDetected = 1;
            goalSeenThisLoop = 1;
            goalX = pixy.blocks[j].x;
            goalY = pixy.blocks[j].y;
          }

        }

      }
    }

    if (loopsSinceQuaffleSeen > 10 && quaffleDetected && state == COLLECTING) {
      Serial.print("Ball disappeared ");
      Serial.print(millis());
      Serial.print(" ");
      Serial.println(i);
      quaffleDetectedRight = 0;
      quaffleDetectedLeft = 0;
      quaffleDetected = 0;
      forward(FORWARD_SPEED);
    }

    //    if (!goalDetected && (getIrVal(LEFT_FRONT_IR_PIN) < 7 || getIrVal(RIGHT_FRONT_IR_PIN) < 7)&& state == SCORING) {
    //      //object in the way
    //      brake();
    //      delay(5000);
    //      if (getIrVal(LEFT_FRONT_IR_PIN) < 7 || getIrVal(RIGHT_FRONT_IR_PIN) < 7) {
    //        state = COLLECTING;
    //        scoreTime = millis() + QUICK_COLLECTING_TIME;
    //      }
    //    }

    if (!goalSeenThisLoop && goalDetected) {
      Serial.print("Goal disappeared ");
      Serial.print(millis());
      Serial.print(" ");
      Serial.println(i);
      goalDetected = 0;
    }

  }
  brake();
  stopRoll();
}

//----------------NAV FUNC=----------
void slowLineUpWall() {
  while (abs(getIrVal(LEFT_FRONT_IR_PIN) - getIrVal(RIGHT_FRONT_IR_PIN)) > .1) {
    Serial.print(getIrVal(LEFT_FRONT_IR_PIN));
    Serial.print(" ");
    Serial.println(getIrVal(RIGHT_FRONT_IR_PIN));

    while (getIrVal(LEFT_FRONT_IR_PIN) < getIrVal(RIGHT_FRONT_IR_PIN)) {
      Serial.println("rotating left");
      left(TURNING_SPEED);
      delay(30);
      brake();
    }

    while (getIrVal(RIGHT_FRONT_IR_PIN) < getIrVal(LEFT_FRONT_IR_PIN)) {
      Serial.println("rotating right");
      right(TURNING_SPEED);
      delay(30);
      brake();
    }

  }
}
void lineUpWall() {
  while (abs(getIrVal(LEFT_FRONT_IR_PIN) - getIrVal(RIGHT_FRONT_IR_PIN)) > .1) {
    Serial.print(getIrVal(LEFT_FRONT_IR_PIN));
    Serial.print(" ");
    Serial.println(getIrVal(RIGHT_FRONT_IR_PIN));

    while (getIrVal(LEFT_FRONT_IR_PIN) < getIrVal(RIGHT_FRONT_IR_PIN)) {
      Serial.println("rotating left");
      left(TURNING_SPEED);
    }


    while (getIrVal(RIGHT_FRONT_IR_PIN) < getIrVal(LEFT_FRONT_IR_PIN)) {
      Serial.println("rotating right");
      right(TURNING_SPEED);

    }

  }
}
void findPath() {

}

//-----------------SENSOR FUNCTIONS--------------------
float getIrVal(int pin) {
  //return sensor.value * sensor.modifier;
  // return 37-sensor.value;
  return 13 * pow(analogRead(pin) * 0.0048828125, -1);
}
float getSonarVal(int pin) {
  float val = 0;
  for(int i=0; i<30; i++){
    val = val + analogRead(pin);
  }
  val = val/30;
  return val / 5;
}
float getLightVal(int pin) {
  return pow(10, analogRead(pin) * 5.0 / 1024);
}
float getVoltage() {
  return analogRead(VOLT_PIN) / 121.79;
}



/*
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
