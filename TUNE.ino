#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ---------------- SERVO PULSE RANGE ----------------
#define SERVOMIN 150
#define SERVOMAX 650

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver();

// ---------------- LAST SERVOS ANGLE ----------------
int currentAngles[16];

int angleToPulse(float ang) {
  return map((int)ang, 0, 180, SERVOMIN, SERVOMAX);
}

void setup() {
  Serial.begin(9600);
  pca.begin();
  pca.reset();
  
  pca.setPWMFreq(60);

  

  for (int i = 0; i < 16; i++) {
    currentAngles[i] = 90;
  }

  Serial.println("--- Smooth PCA9685 Servo Control ---");
  Serial.println("Enter: [Servo_ID] [Angle]");
}

void loop() {
  if (Serial.available() > 0) {
    int servoNum = Serial.parseInt();
    int targetAngle = Serial.parseInt();

    while (Serial.available() > 0) Serial.read();

    if (targetAngle >= 0 && targetAngle <= 180) {
      Serial.print("Move Servo ");
      Serial.print(servoNum);
      Serial.print(" to ");
      Serial.println(targetAngle);

      smoothMove(servoNum, targetAngle, 0.5); 
    }
    else {
      Serial.println("Invalid angle!");
    }
  }
}

/* ============================================================
    SMOOTH SLOW MOVEMENT FUNCTION
    duration_sec = total time of the movement (seconds)
   ============================================================ */

void smoothMove(int servoNum, int targetAngle, float duration_sec) {
  int startAngle = currentAngles[servoNum];
  const int steps = 40;
  float dt = duration_sec * 1000.0 / steps;

  for (int i = 0; i <= steps; i++) {
    int angle = startAngle + (targetAngle - startAngle) * i / steps;
    pca.setPWM(servoNum, 0, angleToPulse(angle));
    delay(dt);
  }

  currentAngles[servoNum] = targetAngle;
}


/* =======================================================
    Easing FUNCTION MAKES MOTION SMOOTH & NATURAL
   ======================================================= */

float easeInOutQuad(float t) {
  if (t < 0.5) return 2 * t * t;
  return -1 + (4 - 2 * t) * t;
}

/* =======================================================
    Convert angle → PCA9685 pulse
   ======================================================= */

