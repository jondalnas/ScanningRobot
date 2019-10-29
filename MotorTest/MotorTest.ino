#define A 2  //Red
#define B 4  //Yellow
#define AH 3 //Black
#define BH 5 //Green

#define DELAY 200

void setup() {
  pinMode(A, OUTPUT);
  pinMode(B, OUTPUT);
  pinMode(AH, OUTPUT);
  pinMode(BH, OUTPUT);
}

void loop() {
  digitalWrite(A, HIGH);
  digitalWrite(B, LOW);
  digitalWrite(AH, LOW);
  digitalWrite(BH, LOW);
  delay(DELAY);
  
  digitalWrite(A, LOW);
  digitalWrite(B, HIGH);
  digitalWrite(AH, LOW);
  digitalWrite(BH, LOW);
  delay(DELAY);
  
  digitalWrite(A, LOW);
  digitalWrite(B, LOW);
  digitalWrite(AH, HIGH);
  digitalWrite(BH, LOW);
  delay(DELAY);
  
  digitalWrite(A, LOW);
  digitalWrite(B, LOW);
  digitalWrite(AH, LOW);
  digitalWrite(BH, HIGH);
  delay(DELAY);
}
