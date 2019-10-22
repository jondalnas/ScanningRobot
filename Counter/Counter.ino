#define latch 4
#define clk 7
#define data 8
void setup() {
  pinMode(latch, OUTPUT);
  pinMode(clk, OUTPUT);
  pinMode(data,OUTPUT);
  pinMode(9,INPUT);
  
  shiftOut(data, clk, MSBFIRST, 0b1);
  digitalWrite( data, LOW);
  digitalWrite(latch, HIGH);
  digitalWrite(latch, LOW);
}

void loop() {
  delay(1000);
  digitalWrite(latch, LOW);
  digitalWrite(clk, HIGH);
  digitalWrite(clk, LOW);
  digitalWrite(latch, HIGH);
  delay(1000);
}
