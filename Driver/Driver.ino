#include <Wire.h>
#define STEP 1.8
#define MAX_DIST 0.5
#define MPU 0x68 // MPU6050 I2C address
#define SENSOR_IN A0
#define SENSOR_ON 8
#define STEPPER 7
float gyroAngleX, gyroAngleY;
float pitch;
float accErrorX, accErrorY, gyroErrorX, gyroErrorY, gyroErrorZ;
float previousTime;
float velZ = 0;

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
  //Initialize MPU-6050
  Serial.begin(9600);
  /*Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmission
  // Call this function if you need to get the IMU error values for your module
  calculate_IMU_error();*/

  //forward(1);
  
  //moveRobot(30);

  //Call recursive method
  tree();
}

void loop() {
  deltaPos();
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
  calculateAccelerometorAndGyro();
  //Serial.println(velZ);
  return velZ;
}

//Return pitch
float currAngle() {
  calculateAccelerometorAndGyro();
  return pitch;
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

//Recalculate accelerometer and gyro angles and acceleration
void calculateAccelerometorAndGyro() {
  float currentTime = millis();                            // Current time actual time read
  float elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds

  //Serial.println(elapsedTime);

  //Don't run the check more then once a second
  if (elapsedTime < 1.0) return;
  
  previousTime = currentTime;        // Previous time is stored before the actual time read
  // === Read acceleromter data === //
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
  float accX = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
  float accY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
  float accZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value
  // Calculating Roll and Pitch from the accelerometer data
  float accAngleY = (atan(-1 * accX / sqrt(pow(accY, 2) + pow(accZ, 2))) * 180 / PI) + 1.58; // accErrorY ~(-1.58) See the calculate_IMU_error()custom function for more details

  Serial.print(accX);
  Serial.print(", ");
  Serial.print(accY);
  Serial.println(", ");
  Serial.print(accZ);
  
  velZ += (accZ * 9.82) * elapsedTime;
  
  // === Read gyroscope data === //
  Wire.beginTransmission(MPU);
  Wire.write(0x43); // gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 4 registers total, each axis value is stored in 2 registers
  Wire.read();//We only need y, so we waste the other input
  Wire.read();
  float gyroY = (Wire.read() << 8 | Wire.read()) / 131.0; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
  Wire.read();
  Wire.read();
  // Correct the outputs with the calculated error values
  gyroY -= gyroErrorY; // gyroErrorY ~(2)
  // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
  gyroAngleY += gyroY * elapsedTime;
  // Complementary filter - combine acceleromter and gyro angle values
  pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;
}

//Calculate the error of the accelerometer and gyroscope
void calculate_IMU_error() {
  // We can call this funtion in the setup section to calculate the accelerometer and gyro data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
  // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
  // Read accelerometer values 200 times
  float c = 0;
  for (uint8_t c = 0; c < 200; c++) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    float accX = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    float accY = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    float accZ = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    // Sum all readings
    accErrorX += ((atan((accY) / sqrt(pow((accX), 2) + pow((accZ), 2))) * 180 / PI));
    accErrorY += ((atan(-1 * (accX) / sqrt(pow((accY), 2) + pow((accZ), 2))) * 180 / PI));
    c++;
  }
  //Divide the sum by 200 to get the error value
  accErrorX = accErrorX / 200;
  accErrorY = accErrorY / 200;
  c = 0;
  // Read gyro values 200 times
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    float gyroX = Wire.read() << 8 | Wire.read();
    float gyroY = Wire.read() << 8 | Wire.read();
    float gyroZ = Wire.read() << 8 | Wire.read();
    // Sum all readings
    gyroErrorX += (gyroX / 131.0);
    gyroErrorY += (gyroY / 131.0);
    gyroErrorZ += (gyroZ / 131.0);
    c++;
  }
  //Divide the sum by 200 to get the error value
  gyroErrorX = gyroErrorX / 200;
  gyroErrorY = gyroErrorY / 200;
  gyroErrorZ = gyroErrorZ / 200;
}

//Puls stepper so we know the next puls is moving the stepper
void initStepper() {
  for (uint8_t i = 0; i < 4; i++) {
    pulsStepper();
  }
}
