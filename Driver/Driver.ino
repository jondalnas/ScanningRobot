#define STEP 1.8
#define MAX_DIST 0.5

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

struct AngleRange {
  byte startAngle;
  byte endAngle;

  float mid() {
    return (startAngle + endAngle) * STEP / 2;
  }
};

Vector pos{};
float angle = 0;

Vector scannedPoses[0];

void setup() {
  Serial.begin(9600);
  tree();
}

void loop() {
}

void tree() {
  rotate(0);
  byte blankStart;
  AngleRange ar[0];
  for (byte i = 0; i < 360 / STEP; i++) {
    float distance = scan();

    if (distance > MAX_DIST && blankStart == -1) {
      blankStart = i;
    } else if (distance < MAX_DIST && blankStart != -1) {

      realloc(ar, sizeof(ar)/sizeof(ar[0])+1);
      ar[sizeof(ar)/sizeof(ar[0])] = {blankStart, i};
      
      blankStart = -1;
    } else {
      Serial.print(distance);
      Serial.print(",");
      Serial.println((i * STEP));
    }
  }

  realloc(scannedPoses, sizeof(scannedPoses)/sizeof(scannedPoses[0])+1);
  scannedPoses[sizeof(scannedPoses)/sizeof(scannedPoses[0])] = pos;

  for (int i = 0; i < sizeof(ar)/sizeof(ar[0]); i++) {
    float theta = ar[i].mid();

    if (notScanned({cos(theta) * MAX_DIST, sin(theta) * MAX_DIST})) {
      moveRobot(theta);
      tree();
      moveRobot(360 - theta);
    }
  }
}

void moveRobot(float toAngle) {
  rotate(toAngle);
  forward(MAX_DIST);

  pos = {pos.x + cos(toAngle) * MAX_DIST, pos.y + sin(toAngle) * MAX_DIST};
  angle = toAngle;
}

void rotate(float theta) {
  float toAngle = theta - angle;

  if (toAngle < 0) {
    rotateLeft();
    while(toAngle < 0) toAngle += deltaAngle();
    stopMotors();
  } else {
    rotateRight();
    while(toAngle > 0) toAngle += deltaAngle();
    stopMotors();
  }
}

void forward(float distance) {
  forward();
  while(distance > 0) distance -= deltaPos();
  stopMotors();
}

bool notScanned(Vector v) {
  for (int i = 0; i < sizeof(scannedPoses)/sizeof(scannedPoses[0]); i++) {
    if (scannedPoses[i].sqrDist(&v) < MAX_DIST*MAX_DIST) return false;
  }
  
  return true;
}

float scan() {
}

void forward() {
}

void stopMotors() {
}

float deltaPos() {
}

float deltaAngle() {
}

void rotateLeft() {
}

void rotateRight() {
}
