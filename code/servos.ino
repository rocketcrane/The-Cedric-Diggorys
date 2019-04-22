void raiseArm(){
  for (int pos = 0; pos <= LIFT_ANGLE; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    leftServo.write(pos);              // tell servo to go to position in variable 'pos'
    rightServo.write(pos);
    delay(15);                       // waits 15ms for the servo to reach the position
  }
}

void lowerArm(){
  for (int pos = LIFT_ANGLE; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    leftServo.write(pos*1);              // tell servo to go to position in variable 'pos'
    rightServo.write(pos);
    delay(15);                       // waits 15ms for the servo to reach the position
  }
}

void rollIn(){
  roller.write(0);
}
void rollOut(){
  roller.write(180);
}

void initializeServos(){
  leftServo.attach(LEFT_SERVO_PIN);
  rightServo.attach(RIGHT_SERVO_PIN);
  roller.attach(ROLLER_PIN);
  leftServo.write(15);
  rightServo.write(40);
}
