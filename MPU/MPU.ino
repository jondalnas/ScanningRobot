#define MPU 0b1101000
#include <Wire.h>

long accelX, accelY, accelZ;
float gForceX, gForceY, gForceZ;

long gyroX, gyroY, gyroZ;
float rotX, rotY, rotZ;

float angle = 0;
float velocity = 0;
float x = 0, y = 0;

float erroraccelX = 0;
float erroraccelY = 0;
float errorgyroX = 0;
float errorgyroY = 0;
float errorgyroZ =0;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  setupMPU();
  calculate_IMU_error();
}


void loop() {
  recordAccelRegisters();
  recordGyroRegisters();
  calcVector();
  printData();
  delay(100);
}

void setupMPU(){
  Wire.beginTransmission(0b1101000); //This is the I2C address of the MPU (b1101000/b1101001 for AC0 low/high datasheet sec. 9.2)
  Wire.write(0x6B); //Accessing the register 6B - Power Management (Sec. 4.28)
  Wire.write(0b00000000); //Setting SLEEP register to 0. (Required; see Note on p. 9)
  Wire.endTransmission();  
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x1B); //Accessing the register 1B - Gyroscope Configuration (Sec. 4.4) 
  Wire.write(0x00000000); //Setting the gyro to full scale +/- 250deg./s 
  Wire.endTransmission(); 
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x1C); //Accessing the register 1C - Acccelerometer Configuration (Sec. 4.5) 
  Wire.write(0b00000000); //Setting the accel to +/- 2g
  Wire.endTransmission(); 
}

void recordAccelRegisters() {
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x3B); //Starting register for Accel Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); //Request Accel Registers (3B - 40)
  while(Wire.available() < 6);
  accelX = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX
  accelY = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
  accelZ = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ
  processAccelData();
}

void processAccelData(){
  gForceX = (accelX - erroraccelX) / 16384.0;
  gForceY = (accelY - erroraccelY) / 16384.0; 
  //gForceZ = (accelZ - erroraccelZ) / 16384.0;
}

void recordGyroRegisters() {
  Wire.beginTransmission(MPU); //I2C address of the MPU
  Wire.write(0x43); //Starting register for Gyro Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); //Request Gyro Registers (43 - 48)
  while(Wire.available() < 6);
  gyroX = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX
  gyroY = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
  gyroZ = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ
  processGyroData();
}

void processGyroData() {
  rotX = (gyroX - errorgyroX) / 131.0;
  rotY = (gyroY - errorgyroY) / 131.0; 
  rotZ = (gyroZ - errorgyroZ) / 131.0;
}

void calculate_IMU_error() {
  for (int c = 0; c < 200; c++) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    float offsetaccelX = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    float offsetaccelY = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    float offsetaccelZ = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    erroraccelX += ((atan((offsetaccelY) / sqrt(pow((offsetaccelX), 2) + pow((offsetaccelZ), 2))) * 180 / PI));
    erroraccelY += ((atan(-1 * (offsetaccelX) / sqrt(pow((offsetaccelY), 2) + pow((offsetaccelZ), 2))) * 180 / PI));
    float offsetgyroX = (Wire.read() << 8 | Wire.read());
    float offsetgyroY = (Wire.read() << 8 | Wire.read());
    float offsetgyroZ = (Wire.read() << 8 | Wire.read());
    errorgyroX += offsetgyroX/131.0;
    errorgyroY += offsetgyroY/131.0;
    errorgyroZ += offsetgyroZ/131.0;
  }
  erroraccelX /= 200;
  erroraccelY /= 200;
  errorgyroX /= 200;
  errorgyroY /= 200;
  errorgyroZ /= 200;
}

uint32_t last = millis();

void calcVector() {
  float deltaSec = (millis() - last) / 1000.0;
  last = millis();
  angle += (rotZ+2)*deltaSec;

  velocity += gForceY * deltaSec;
  
  x += cos(angle) * velocity * deltaSec;
  y += sin(angle) * velocity * deltaSec;

  Serial.println(gForceY);
}

void printData() {
  /*Serial.print("Gyro (deg)");
  Serial.print(" Z=");
  Serial.print(rotZ+=2);
  Serial.print(" Accel (g)");
  Serial.print(" X=");
  Serial.print(gForceX);
  Serial.print(" Y=");
  Serial.println(gForceY);
  Serial.print("x = ");
  Serial.print(x);
  Serial.print(" y = ");
  Serial.println(y);*/
}
