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
};

Vector pos{};
float angle = 0;

void setup() {
  
}

void loop() {
  
}

void rotate(float theta) {
  float toAngle = theta - angle;

  if (toAngle < 0) {
    rotateLeft();
    while(toAngle < 0) toAngle += angleVelocity();
    stopMotor();
  } else {
    rotateRight();
    while(toAngle > 0) toAngle += angleVelocity();
    stopMotor();
  }
}

void forward(float distance) {
  forward();
  while(distance > 0) distance -= velocity();
  stopMotors();
}

void moveRobot(Vector to) {
  Vector dir{to.x - pos.x, to.y - pos.y};
  float toAngle = atan(dir.y / dir.x);
  rotate(toAngle);
  forward(dir.length());

  pos = {pos.x + dir.x, pos.y + dir.y};
}

void forward() {
}

void stopMotors() {
}

float velocity() {
}

void rotateLeft() {
}

void rotateRight() {
}
