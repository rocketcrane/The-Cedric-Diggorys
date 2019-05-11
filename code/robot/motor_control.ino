//---------------MOTOR CONTROL FUNCTIONS-----------------------
const int CORRECTION = 15;
void forward(int speed){
  Serial.print("Forward ");
  Serial.println(speed);
  digitalWrite(B_DIRECTION_PIN, B_FORWARD);
  digitalWrite(B_BRAKE_PIN, LOW);
  analogWrite(B_SPEED_PIN, speed);   
  digitalWrite(A_DIRECTION_PIN, A_FORWARD);
  digitalWrite(A_BRAKE_PIN, LOW);  
  analogWrite(A_SPEED_PIN, (CORRECTION+speed)); 
}

void reverse(int speed){
  brake();
  Serial.println("Reversing ");
  digitalWrite(B_DIRECTION_PIN, B_REVERSE);
  digitalWrite(B_BRAKE_PIN, LOW);
  analogWrite(B_SPEED_PIN, speed);   
  digitalWrite(A_DIRECTION_PIN, A_REVERSE);
  digitalWrite(A_BRAKE_PIN, LOW);  
  analogWrite(A_SPEED_PIN, CORRECTION+speed);
}

void brake(){
  Serial.println("Breaking");
  digitalWrite(B_BRAKE_PIN, HIGH);
  digitalWrite(A_BRAKE_PIN, HIGH);
  delay(50);
}

//B is right, A left

//turn left until told otherwise
void left(int speed){
  Serial.println("left");
  digitalWrite(B_DIRECTION_PIN, B_REVERSE);
  digitalWrite(A_DIRECTION_PIN, A_FORWARD);
  digitalWrite(B_BRAKE_PIN, LOW);
  digitalWrite(A_BRAKE_PIN, LOW);
  analogWrite(B_SPEED_PIN, speed);
  analogWrite(A_SPEED_PIN, speed);
}

void right(int speed){
  Serial.println("right");
  digitalWrite(B_DIRECTION_PIN, B_FORWARD);
  digitalWrite(A_DIRECTION_PIN, A_REVERSE);
  digitalWrite(B_BRAKE_PIN, LOW);
  digitalWrite(A_BRAKE_PIN, LOW);
  analogWrite(B_SPEED_PIN, speed);
  analogWrite(A_SPEED_PIN, speed);
}
void forwardRight(int speed){
  Serial.println("forward right");

  digitalWrite(B_DIRECTION_PIN, B_FORWARD);
  digitalWrite(A_DIRECTION_PIN, A_FORWARD);
  digitalWrite(B_BRAKE_PIN, LOW);
  digitalWrite(A_BRAKE_PIN, LOW);
  analogWrite(B_SPEED_PIN, speed);
  analogWrite(A_SPEED_PIN, speed/2);
}
void forwardLeft(int speed){
  Serial.println("forward right");

  digitalWrite(B_DIRECTION_PIN, B_FORWARD);
  digitalWrite(A_DIRECTION_PIN, A_FORWARD);
  digitalWrite(B_BRAKE_PIN, LOW);
  digitalWrite(A_BRAKE_PIN, LOW);
  analogWrite(B_SPEED_PIN, speed/2);
  analogWrite(A_SPEED_PIN, speed);
}
void excited(){
  forward(FORWARD_SPEED);
  delay(500);
  reverse(REVERSE_SPEED);
  delay(500);
  
  brake();
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

