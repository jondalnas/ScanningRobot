const int motorR1 = 9;
const int motorR2 = 10;
const int motorL1 = 11;
const int motorL2 = 6;

void setup() {
  pinMode(motorR1, OUTPUT);
  pinMode(motorR2, OUTPUT);
  pinMode(motorL1, OUTPUT);
  pinMode(motorL2, OUTPUT);
 
  
}

void loop() {
  forward();
  delay(3000);
  stop();
  delay(3000);

}

void forward() {
  digitalWrite(motorR1,HIGH);
  digitalWrite(motorR2,LOW);
  digitalWrite(motorL1,HIGH);
  digitalWrite(motorL2,LOW);
  analogWrite(motorR1, 100);

  
}
void back() {
  digitalWrite(motorR1,LOW);
  digitalWrite(motorR2,HIGH);
  digitalWrite(motorL1,LOW);
  digitalWrite(motorL2,HIGH);
  analogWrite(motorR2,200);
  analogWrite(motorL2,200);
  
}

void right() {
  digitalWrite(motorR1,LOW);
  digitalWrite(motorR2,HIGH);
  digitalWrite(motorL1,HIGH);
  digitalWrite(motorL2,LOW);
  analogWrite(motorR2,200);
  analogWrite(motorL2,100);
}

void left() {
  digitalWrite(motorR1,HIGH);
  digitalWrite(motorR2,LOW);
  digitalWrite(motorL1,HIGH);
  digitalWrite(motorL2,LOW);
  analogWrite(motorR2,100);
  analogWrite(motorL2,200);
}


void stop() {
  digitalWrite(motorR1,LOW);
  digitalWrite(motorR2,LOW);
  digitalWrite(motorL1,LOW);
  digitalWrite(motorL2,LOW);
}
