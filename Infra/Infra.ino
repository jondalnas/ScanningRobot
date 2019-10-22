#define InPin A0
#define SenOn 8

uint8_t stp = 0; 

struct Coord {
  float dist;
  uint8_t stp;
};

void setup() {
  pinMode(InPin,INPUT);
  pinMode(SenOn,OUTPUT);
}

void sens() {
  digitalWrite(SenOn,HIGH);
  delay(50);
  int x = analogRead(InPin);
  Coord c = {-0.000001304705734*x*x*x+0.001527222903312*x*x-0.692281671319746*x+140.3326638634976,
             stp};
  digitalWrite(SenOn,LOW);
  delay(100);
}


void loop() {
  sens();
  stp ++;
}
