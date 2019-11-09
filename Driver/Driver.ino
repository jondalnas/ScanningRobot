#include <Wire.h>
#define STEP 1.8
#define MAX_DIST 0.5
#define MPU 0x68 // MPU6050 I2C address
#define SENSOR_IN A0
#define SENSOR_ON 8
#define STEPPER 7
float gyroAngleX, gyroAngleY;
float accErrorX, accErrorY, gyroErrorX, gyroErrorY, gyroErrorZ;
float previousTime;
float velocity = 0;

long accelX, accelY, accelZ;
float gForceX, gForceY, gForceZ;

long gyroX, gyroY, gyroZ;
float rotX, rotY, rotZ;

float erroraccelX, erroraccelY, errorgyroX, errorgyroY, errorgyroZ;

const int motorRb = 9;
const int motorRf = 10;
const int motorLb = 11;
const int motorLf = 6;

const float toRad = PI/180;

//Code for everything we need in a vector
struct Vector {
  float x;
  float y;

  float length() {
    return sqrt(x*x + y*y);
  }

  void normalize() {
    float d = length();
    x /= d;
    y /= d;
  }

  float sqrDist(Vector *v) {
    float dx = v->x - x;
    float dy = v->y - y;

    return dx*dx + dy*dy;
  }
};

//Code for everthing we need to define a range of angles
struct AngleRange {
  byte startAngle;
  byte endAngle;

  float mid() {
    return (startAngle + endAngle) * STEP / 2;
  }
};

//A vector that defines the positon of te robot
Vector pos{};
float angle = 0;

//List of all scanned positions
Vector scannedPoses[0];

void setup() {
  Serial.begin(9600);
  
  //setupMPU();

  //Call recursive method
  //tree();
  
  initStepper();
  
  sense();
}

void loop() {
}

void sense() {
  for (int i = 0; i < 360 / STEP; i++) {
    Serial.print(scan());
    Serial.print(", ");
    Serial.println(i);

    pulsStepper();
    delay(500);
  }
}

//Methode that scannes all angles and checks if it has been scanned before. If it hasn't and it can't scan it, then move robot and scan again
void tree() {
  //Always rotate the robot so it faces 0 angles
  rotate(0);

  //A variable that defines the first angle where the sensor can't see anything
  byte blankStart;
  //List of all blank ranges
  AngleRange ar[0];
  //Loop through all angles
  for (byte i = 0; i < 360 / STEP; i++) {
    //Get distance from scanner
    float distance = scan();

    //If scanner isn't in blanking period and it can't see anything then start blanking period
    if (distance > MAX_DIST && blankStart == -1) {
      blankStart = i;

      //If scanner is in blanking period and it can see something, then stop blanking and add new range to list of ranges
    } else if (distance < MAX_DIST && blankStart != -1) {

      realloc(ar, sizeof(ar)/sizeof(ar[0])+1);
      ar[sizeof(ar)/sizeof(ar[0])] = {blankStart, i};
      
      blankStart = -1;
    } else {
      Serial.print(distance);
      Serial.print(",");
      Serial.println((i * STEP));
    }

    //Rotate head
    pulsStepper();
  }

  //Add current pos to list of scanned poses, so it doesn't get scanned again
  realloc(scannedPoses, sizeof(scannedPoses)/sizeof(scannedPoses[0])+1);
  scannedPoses[sizeof(scannedPoses)/sizeof(scannedPoses[0])] = pos;

  //Loop through all blanking ranges
  for (int i = 0; i < sizeof(ar)/sizeof(ar[0]); i++) {
    float theta = ar[i].mid();

    //Check if the current blanking range has been scanned before
    if (notScanned({cos(theta * toRad) * MAX_DIST, sin(theta * toRad) * MAX_DIST})) {
      //Move robot to the specific angle
      moveRobot(theta);
      //Call new instance of recursive function
      tree();
      //Move robot back again
      moveRobot(360 - theta);
    }
  }
}

//Rotate robot in give direction, moves it forward the maximum distance and sets position and angle to that position and angle
void moveRobot(float toAngle) {
  //Rotate robot
  rotate(toAngle);
  //Move it forward
  forward(MAX_DIST);

  //Update position and angle
  pos = {pos.x + cos(toAngle * toRad) * MAX_DIST, pos.y + sin(toAngle * toRad) * MAX_DIST};
  angle = toAngle;
}

//Rotate robot to match angle theta
void rotate(float theta) {
  //Creates angle direction
  float toAngle = theta - angle;

  //If angle direction is negative then rotate left until angle isn't negative anymore
  //Else do the exact oposite
  if (toAngle < 0) {
    rotateLeft();
    while(toAngle < 0) {
      toAngle += angle - currAngle();
      angle = currAngle();
    }
    stopMotors();
  } else {
    rotateRight();
    while(toAngle > 0) {
      toAngle += angle - currAngle();
      angle = currAngle();
    }
    stopMotors();
  }
}

//Move car forward the distance defined
void forward(float distance) {
  //Move car forward
  forward();
  //Stop driving forward if the distance is greater than zero
  while(distance > 0) distance -= deltaPos();
  stopMotors();
}

//Check if a specific position has been scanned before
bool notScanned(Vector v) {
  //Loop trhough all scanned poses
  for (int i = 0; i < sizeof(scannedPoses)/sizeof(scannedPoses[0]); i++) {
    //Check if current position is within one of the scanned positions
    if (scannedPoses[i].sqrDist(&v) < MAX_DIST*MAX_DIST) return false;
  }
  
  return true;
}

//Read distance from scanner
float scan() {
  //Turn on sensor
  digitalWrite(SENSOR_ON, HIGH);
  delay(50);
  //Read sensor input
  int x = analogRead(SENSOR_IN);
  //Convert into distance
  float dist = -0.00000001304705734*x*x*x+0.00001527222903312*x*x-0.00692281671319746*x+1.403326638634976;
  //Turn off sensor
  digitalWrite(SENSOR_ON, LOW);
  delay(100);
  
  return dist;
}

//Drive car forward
void forward() {
  digitalWrite(motorRf,HIGH);
  digitalWrite(motorRb,LOW);
  digitalWrite(motorLf,HIGH);
  digitalWrite(motorLb,LOW);
  analogWrite(motorRf, 100);
}

//Stop motor
void stopMotors() {
  digitalWrite(motorRf,LOW);
  digitalWrite(motorRb,LOW);
  digitalWrite(motorLf,LOW);
  digitalWrite(motorLb,LOW);
}

//Return z-velocity
float deltaPos() {
  recordAccelRegisters();
  return velocity;
}

//Return pitch
float currAngle() {
  recordGyroRegisters();
  return rotZ;
}

//Rotate car left
void rotateLeft() {
  digitalWrite(motorRf,HIGH);
  digitalWrite(motorRb,LOW);
  digitalWrite(motorLf,HIGH);
  digitalWrite(motorLb,LOW);
  analogWrite(motorRb,100);
  analogWrite(motorLb,200);
}

//Rotate car right
void rotateRight() {
  digitalWrite(motorRf,LOW);
  digitalWrite(motorRb,HIGH);
  digitalWrite(motorLf,HIGH);
  digitalWrite(motorLb,LOW);
  analogWrite(motorRb,200);
  analogWrite(motorLb,100);
}

void pulsStepper() {
  digitalWrite(STEPPER, HIGH);
  delay(50);
  digitalWrite(STEPPER, LOW);
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

uint32_t lastAccel = millis();
void processAccelData() {
  float deltaSec = (millis() - lastAccel) / 1000.0;
  lastAccel = millis();
  
  float velX = (accelX - erroraccelX) / 16384.0 * deltaSec;
  float velY = (accelY - erroraccelY) / 16384.0 * deltaSec;

  velocity += sqrt(velX*velX + velY*velY) * (velY < 0 ? -1 : 1);
}

uint32_t lastGyro = millis();
void processGyroData() {
  float deltaSec = (millis() - lastGyro) / 1000.0;
  lastGyro = millis();
  
  rotX = (gyroX - errorgyroX) / 131.0 * deltaSec;
  rotY = (gyroY - errorgyroY) / 131.0 * deltaSec; 
  rotZ = (gyroZ - errorgyroZ) / 131.0 * deltaSec;
}


void calculate_IMU_error() {
  for (int c = 0; c < 200; c++) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    float offsetaccelX = (Wire.read() << 8 | Wire.read());
    float offsetaccelY = (Wire.read() << 8 | Wire.read());
    float offsetaccelZ = (Wire.read() << 8 | Wire.read());
    erroraccelX += offsetaccelX; //((atan((offsetaccelY) / sqrt(pow((offsetaccelX), 2) + pow((offsetaccelZ), 2))) * 180 / PI));
    erroraccelY += offsetaccelY; //((atan(-1 * (offsetaccelX) / sqrt(pow((offsetaccelY), 2) + pow((offsetaccelZ), 2))) * 180 / PI));
    float offsetgyroX = (Wire.read() << 8 | Wire.read());
    float offsetgyroY = (Wire.read() << 8 | Wire.read());
    float offsetgyroZ = (Wire.read() << 8 | Wire.read());
    errorgyroX += offsetgyroX;
    errorgyroY += offsetgyroY;
    errorgyroZ += offsetgyroZ;
    delay(20);
  }
  erroraccelX /= 200;
  erroraccelY /= 200;
  errorgyroX /= 200;
  errorgyroY /= 200;
  errorgyroZ /= 200;
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

//Puls stepper so we know the next puls is moving the stepper
void initStepper() {
  for (uint8_t i = 0; i < 4; i++) {
    pulsStepper();
  }
}
