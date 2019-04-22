//---------------MOTOR CONTROL FUNCTIONS-----------------------
const int CORRECTION = 15;
void forward(int speed){
  Serial.print("Forward ");
  Serial.println(speed);
  digitalWrite(A_DIRECTION_PIN, A_FORWARD);
  digitalWrite(A_BRAKE_PIN, LOW);
  analogWrite(A_SPEED_PIN, speed);   
  digitalWrite(B_DIRECTION_PIN, B_FORWARD);
  digitalWrite(B_BRAKE_PIN, LOW);  
  analogWrite(B_SPEED_PIN, (CORRECTION+speed)); 
}

void reverse(int speed){
  brake();
  Serial.println("Reversing ");
  digitalWrite(A_DIRECTION_PIN, A_REVERSE);
  digitalWrite(A_BRAKE_PIN, LOW);
  analogWrite(A_SPEED_PIN, speed);   
  digitalWrite(B_DIRECTION_PIN, B_REVERSE);
  digitalWrite(B_BRAKE_PIN, LOW);  
  analogWrite(B_SPEED_PIN, CORRECTION+speed);
}

void brake(){
  Serial.println("Breaking");
  digitalWrite(A_BRAKE_PIN, HIGH);
  digitalWrite(B_BRAKE_PIN, HIGH);
  delay(50);
}

//B is right, A left

//turn left until told otherwise
void left(int speed){
  Serial.println("left");
  brake();
  digitalWrite(A_DIRECTION_PIN, A_FORWARD);
  digitalWrite(B_DIRECTION_PIN, B_REVERSE);
  digitalWrite(A_BRAKE_PIN, LOW);
  digitalWrite(B_BRAKE_PIN, LOW);
  analogWrite(A_SPEED_PIN, speed);
  analogWrite(B_SPEED_PIN, speed);
}

void right(int speed){
  brake();
  Serial.println("right");
  digitalWrite(A_DIRECTION_PIN, A_REVERSE);
  digitalWrite(B_DIRECTION_PIN, B_FORWARD);
  digitalWrite(A_BRAKE_PIN, LOW);
  digitalWrite(B_BRAKE_PIN, LOW);
  analogWrite(A_SPEED_PIN, speed);
  analogWrite(B_SPEED_PIN, speed);
}
void forwardRight(int speed){
  Serial.println("forward right");

  digitalWrite(A_DIRECTION_PIN, A_FORWARD);
  digitalWrite(B_DIRECTION_PIN, B_FORWARD);
  digitalWrite(A_BRAKE_PIN, LOW);
  digitalWrite(B_BRAKE_PIN, LOW);
  analogWrite(A_SPEED_PIN, speed/2);
  analogWrite(B_SPEED_PIN, speed);
}
void forwardLeft(int speed){
  Serial.println("forward right");

  digitalWrite(A_DIRECTION_PIN, A_FORWARD);
  digitalWrite(B_DIRECTION_PIN, B_FORWARD);
  digitalWrite(A_BRAKE_PIN, LOW);
  digitalWrite(B_BRAKE_PIN, LOW);
  analogWrite(A_SPEED_PIN, speed);
  analogWrite(B_SPEED_PIN, speed/2);
}
void excited(){
  forward(FORWARD_SPEED);
  delay(500);
  reverse(REVERSE_SPEED);
  delay(500);
  
  brake();
}

void turn_right(){
  reverse(REVERSE_SPEED);
    delay(750);
    right(TURNING_SPEED);
    delay(300);
    forward(FORWARD_SPEED);
}
void turn_left(){
  reverse(REVERSE_SPEED);
    delay(750);
    left(TURNING_SPEED);
    delay(300);
    forward(FORWARD_SPEED);
}
