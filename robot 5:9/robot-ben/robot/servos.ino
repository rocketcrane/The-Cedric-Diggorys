const int RIGHT_START = 13;
const int LEFT_START = 160 - RIGHT_START;
const int LIFT_ANGLE = 35;

void raiseArm(){
  for (int pos = 0; pos <= LIFT_ANGLE; pos++) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    rightServo.write(RIGHT_START+pos);              // tell servo to go to position in variable 'pos'
    leftServo.write(LEFT_START-pos);
    delay(15);                       // waits 15ms for the servo to reach the position
  }
}

void lowerArm(){
  for (int pos = LIFT_ANGLE; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    rightServo.write(pos+RIGHT_START);              // tell servo to go to position in variable 'pos'
    leftServo.write(LEFT_START-pos);
    delay(15);                       // waits 15ms for the servo to reach the position
  }
}

void rollIn(){
  roller.attach(ROLLER_PIN);
  roller.write(0);
}
void rollOut(){
  roller.attach(ROLLER_PIN);
  roller.write(180);
}
void stopRoll(){
  roller.detach();
}

void initializeServos(){
  leftServo.attach(LEFT_SERVO_PIN);
  rightServo.attach(RIGHT_SERVO_PIN);
  roller.attach(ROLLER_PIN);
  leftServo.write(LEFT_START);
  rightServo.write(RIGHT_START);
}
