//-------------------MOTOR CONTROL FUNCTIONS-----------------------
const int CORRECTION = 15;

void forward(int speed) {
  //Serial.println("Forward");
  digitalWrite(A_DIRECTION_PIN, A_FORWARD);
  digitalWrite(A_BRAKE_PIN, LOW);
  analogWrite(A_SPEED_PIN, speed);
  digitalWrite(B_DIRECTION_PIN, B_FORWARD);
  digitalWrite(B_BRAKE_PIN, LOW);
  analogWrite(B_SPEED_PIN, (CORRECTION + speed));
}

void reverse(int speed) {
  brake();
  //Serial.println("Reverse");
  digitalWrite(A_DIRECTION_PIN, A_REVERSE);
  digitalWrite(A_BRAKE_PIN, LOW);
  analogWrite(A_SPEED_PIN, speed);
  digitalWrite(B_DIRECTION_PIN, B_REVERSE);
  digitalWrite(B_BRAKE_PIN, LOW);
  analogWrite(B_SPEED_PIN, CORRECTION + speed);
}

void brake() {
  //Serial.println("Brake");
  digitalWrite(A_BRAKE_PIN, HIGH);
  digitalWrite(B_BRAKE_PIN, HIGH);
  delay(50);
}

//B is right motor, A is left motor

void left(int speed) {
  brake();
  //Serial.println("left");
  digitalWrite(A_DIRECTION_PIN, A_FORWARD);
  digitalWrite(B_DIRECTION_PIN, B_REVERSE);
  digitalWrite(A_BRAKE_PIN, LOW);
  digitalWrite(B_BRAKE_PIN, LOW);
  analogWrite(A_SPEED_PIN, speed);
  analogWrite(B_SPEED_PIN, speed);
}

void right(int speed) {
  brake();
  //Serial.println("right");
  digitalWrite(A_DIRECTION_PIN, A_REVERSE);
  digitalWrite(B_DIRECTION_PIN, B_FORWARD);
  digitalWrite(A_BRAKE_PIN, LOW);
  digitalWrite(B_BRAKE_PIN, LOW);
  analogWrite(A_SPEED_PIN, speed);
  analogWrite(B_SPEED_PIN, speed);
}

void forwardRight(int speed) {
  //Serial.println("forward right");
  digitalWrite(A_DIRECTION_PIN, A_FORWARD);
  digitalWrite(B_DIRECTION_PIN, B_FORWARD);
  digitalWrite(A_BRAKE_PIN, LOW);
  digitalWrite(B_BRAKE_PIN, LOW);
  analogWrite(A_SPEED_PIN, speed / 1.5);
  analogWrite(B_SPEED_PIN, speed);
}

void forwardLeft(int speed) {
  //Serial.println("forward left");
  digitalWrite(A_DIRECTION_PIN, A_FORWARD);
  digitalWrite(B_DIRECTION_PIN, B_FORWARD);
  digitalWrite(A_BRAKE_PIN, LOW);
  digitalWrite(B_BRAKE_PIN, LOW);
  analogWrite(A_SPEED_PIN, speed);
  analogWrite(B_SPEED_PIN, speed / 1.5);
}

//Cedric is excited! For testing.
void excited() {
  forward(FORWARD_SPEED);
  delay(500);
  reverse(REVERSE_SPEED);
  delay(500);
  brake();
}

//Backs up and turns right
void reverse_right() {
  reverse(REVERSE_SPEED);
  delay(300);
  right(TURNING_SPEED);
  delay(700);
  forward(FORWARD_SPEED);
}

//Backs up and turns left
void reverse_left() {
  reverse(REVERSE_SPEED);
  delay(300);
  left(TURNING_SPEED);
  delay(700);
  forward(FORWARD_SPEED);
}

//Turns left
void turn_left() {
  left(TURNING_SPEED);
  delay(700);
  forward(FORWARD_SPEED);
}

//Turns right
void turn_right() {
  right(TURNING_SPEED);
  delay(700);
  forward(FORWARD_SPEED);
}

//Moves forward to the left
void left() {
  forwardLeft(FORWARD_SPEED);
  delay(700);
  forward(FORWARD_SPEED);
}

//Moves forward to the right
void right() {
  forwardRight(FORWARD_SPEED);
  delay(700);
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
