/*

         ROBOT CONFIGULATION                                  

            90        90                                              
            |    ^    |                                               
    180_____|____|____|_____0                                       
            |0       2|                                       
            |         |                                 
      0_____|4_______6|_____180                 
            |         |                       
            |         |                         
            90        90         


            12 echo13 ultrasonic
            8 to driver servo for sensor                 

*/

String command = "";
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define SERVOMIN 150
#define SERVOMAX 650

#define TRIG_PIN 13
#define ECHO_PIN 12
const int sensorServo = 8;

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver();

enum RobotState {
  STOP,
  WALK_FORWARD,
  WALK_BACKWARD
};

RobotState robotState = STOP;

struct ServoMap {
  int hipLF;
  int hipRF;
  int hipLB;
  int hipRB;
  int legLF;
  int legRF;
  int legLB;
  int legRB;
};

ServoMap leftMap = {
  .hipLF = 0,
  .hipRF = 2,
  .hipLB = 4,
  .hipRB = 6,
  .legLF = 1,
  .legRF = 3,
  .legLB = 5,
  .legRB = 7
};

ServoMap RightMap = {
  .hipLF = 6,
  .hipRF = 4,
  .hipLB = 2,
  .hipRB = 0,
  .legLF = 7,
  .legRF = 5,
  .legLB = 3,
  .legRB = 1
};

int dt = 100;
int lift = 100;
int down = 180;

int RF_LB_standby = 45;
int RF_LB_center = 0;
int LF_RB_standby = 135;
int LF_RB_center = 180;

int rightAngle = 0;
int leftAngle = 90;

int currentAngle = rightAngle;
int step = 5;             // +1 or -1
unsigned long lastMove = 0;
unsigned long scanStepTime = 20; // ms

void setup() {
  Serial.begin(9600);
  pca.begin();
  pca.reset();
  pca.setPWMFreq(60);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  standBy();
  delay(4000);
}

void loop() {
  /*
   unsigned long now = millis();

  // Move servo step-by-step
  if (now - lastMove >= scanStepTime) {
    lastMove = now;

    currentAngle += step;

    // Reverse direction at limits
    if (currentAngle >= leftAngle || currentAngle <= rightAngle) {
      step = -step;
    }

    pca.setPWM(sensorServo, 0, angleToPulse(currentAngle));

    // Read sensor at SAME TIME
    int distance = getDistance();

    Serial.print("Angle: ");
    Serial.print(currentAngle);
    Serial.print("  Distance: ");
    Serial.print(distance);
    Serial.println(" cm");
  }
  */
  
  if (Serial.available() > 0) {
    command = Serial.readStringUntil('\n');
    command.trim();

    if (command == "l") {
        Serial.println("Left");
        for (int i = 0; i < 2; i++) {
          moveLeft();
        }
        safeLeft();
      } else if (command == "r") {
        Serial.println("Right");
        for (int i = 0; i < 2; i++) {
          moveRight();
        }
        safeRight();
      } else if (command == "s") {
        standBy();
        Serial.println("stand");
      }
  }

}




void safeLeft() {
  pca.setPWM(leftMap.hipRF, 0, angleToPulse(90));
  pca.setPWM(leftMap.hipLB, 0, angleToPulse(RF_LB_standby+20));
  pca.setPWM(leftMap.hipRB, 0, angleToPulse(LF_RB_standby-15));
  pca.setPWM(leftMap.hipLF, 0, angleToPulse(90));
  pca.setPWM(leftMap.legLF, 0, angleToPulse(120));
  pca.setPWM(leftMap.legRF, 0, angleToPulse(120));
  pca.setPWM(leftMap.legLB, 0, angleToPulse(down));
  pca.setPWM(leftMap.legRB, 0, angleToPulse(down));
}

void safeRight() {
  pca.setPWM(RightMap.hipRF, 0, angleToPulse(90));
  pca.setPWM(RightMap.hipLB, 0, angleToPulse(RF_LB_standby+20));
  pca.setPWM(RightMap.hipRB, 0, angleToPulse(LF_RB_standby-15));
  pca.setPWM(RightMap.hipLF, 0, angleToPulse(90));
  pca.setPWM(RightMap.legLF, 0, angleToPulse(120));
  pca.setPWM(RightMap.legRF, 0, angleToPulse(120));
  pca.setPWM(RightMap.legLB, 0, angleToPulse(down));
  pca.setPWM(RightMap.legRB, 0, angleToPulse(down));
}

void moveLeft() {
  // --- step 1 --- //
  // lift LF leg
  pca.setPWM(leftMap.legLF, 0, angleToPulse(lift));
  delay(dt);
  // move LF hip to 110 = 90 + 20
  pca.setPWM(leftMap.hipLF, 0, angleToPulse(100));
  delay(dt);
  // place LF leg
  pca.setPWM(leftMap.legLF, 0, angleToPulse(down));
  delay(dt+50);

  // --- step 2 --- //
  // swing LF to standby
  pca.setPWM(leftMap.hipLF, 0, angleToPulse(LF_RB_center));
  // swing LB to standby
  pca.setPWM(leftMap.hipLB, 0, angleToPulse(RF_LB_standby+20));
  delay(dt+100);

  // --- step 3 --- //
  // lift RB leg
  pca.setPWM(leftMap.legRB, 0, angleToPulse(lift));
  delay(dt);
  // move RB hip to center
  pca.setPWM(leftMap.hipRB, 0, angleToPulse(LF_RB_center));
  delay(dt);
  // place RB leg
  pca.setPWM(leftMap.legRB, 0, angleToPulse(down));
  delay(dt+50);

  // --- step 4 --- //
  // lift RF leg
  pca.setPWM(leftMap.legRF, 0, angleToPulse(lift));
  delay(dt);
  // move RF hip to 70 = 90 - 20
  pca.setPWM(leftMap.hipRF, 0, angleToPulse(80));
  delay(dt);
  // place RF leg
  pca.setPWM(leftMap.legRF, 0, angleToPulse(down));
  delay(dt+50);

  // --- step 5 --- //
  // swing RF to standby
  pca.setPWM(leftMap.hipRF, 0, angleToPulse(RF_LB_center));
  // swing RB to standby
  pca.setPWM(leftMap.hipRB, 0, angleToPulse(LF_RB_standby-15));
  delay(dt+100);

  // --- step 6 --- //
  // lift LB leg
  pca.setPWM(leftMap.legLB, 0, angleToPulse(lift));
  delay(dt);
  // move LB hip to center
  pca.setPWM(leftMap.hipLB, 0, angleToPulse(RF_LB_center));
  delay(dt);
  // place LB leg
  pca.setPWM(leftMap.legLB, 0, angleToPulse(down));
  delay(dt+50);

}

void moveRight() {
  // --- step 1 --- //
  // lift LF leg
  pca.setPWM(RightMap.legLF, 0, angleToPulse(lift));
  delay(dt);
  // move LF hip to 110 = 90 + 20
  pca.setPWM(RightMap.hipLF, 0, angleToPulse(100));
  delay(dt);
  // place LF leg
  pca.setPWM(RightMap.legLF, 0, angleToPulse(down));
  delay(dt+50);

  // --- step 2 --- //
  // swing LF to standby
  pca.setPWM(RightMap.hipLF, 0, angleToPulse(LF_RB_center));
  // swing LB to standby
  pca.setPWM(RightMap.hipLB, 0, angleToPulse(RF_LB_standby+20));
  delay(dt+100);

  // --- step 3 --- //
  // lift RB leg
  pca.setPWM(RightMap.legRB, 0, angleToPulse(lift));
  delay(dt);
  // move RB hip to center
  pca.setPWM(RightMap.hipRB, 0, angleToPulse(LF_RB_center));
  delay(dt);
  // place RB leg
  pca.setPWM(RightMap.legRB, 0, angleToPulse(down));
  delay(dt+50);

  // --- step 4 --- //
  // lift RF leg
  pca.setPWM(RightMap.legRF, 0, angleToPulse(lift));
  delay(dt);
  // move RF hip to 70 = 90 - 20
  pca.setPWM(RightMap.hipRF, 0, angleToPulse(80));
  delay(dt);
  // place RF leg
  pca.setPWM(RightMap.legRF, 0, angleToPulse(down));
  delay(dt+50);

  // --- step 5 --- //
  // swing RF to standby
  pca.setPWM(RightMap.hipRF, 0, angleToPulse(RF_LB_center));
  // swing RB to standby
  pca.setPWM(RightMap.hipRB, 0, angleToPulse(LF_RB_standby-15));
  delay(dt+100);

  // --- step 6 --- //
  // lift LB leg
  pca.setPWM(RightMap.legLB, 0, angleToPulse(lift));
  delay(dt);
  // move LB hip to center
  pca.setPWM(RightMap.hipLB, 0, angleToPulse(RF_LB_center));
  delay(dt);
  // place LB leg
  pca.setPWM(RightMap.legLB, 0, angleToPulse(down));
  delay(dt+50);

}

void standBy() {
  pca.setPWM(leftMap.hipRF, 0, angleToPulse(RF_LB_standby));
  pca.setPWM(leftMap.hipLB, 0, angleToPulse(RF_LB_standby+20));
  pca.setPWM(leftMap.hipRB, 0, angleToPulse(LF_RB_standby-15));
  pca.setPWM(leftMap.hipLF, 0, angleToPulse(LF_RB_standby));
  pca.setPWM(leftMap.legLF, 0, angleToPulse(down));
  pca.setPWM(leftMap.legRF, 0, angleToPulse(down));
  pca.setPWM(leftMap.legLB, 0, angleToPulse(down));
  pca.setPWM(leftMap.legRB, 0, angleToPulse(down));
}

int angleToPulse(float ang) {
  return map((int)ang, 0, 180, SERVOMIN, SERVOMAX);
}

int getDistance() {
  long duration;

  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  duration = pulseIn(ECHO_PIN, HIGH, 30000);

  if (duration == 0) return -1;

  return duration / 58;
}

int TEST_MOVE_LEFT_RIGHT() {
  for (int i  = 0; i <= 7; i++) {
    if (i < 4) {
      moveLeft();
    } else if ( i == 4) {
      standBy();
      delay(500);
    } else {
      moveRight();
    }
  }
  standBy();
  delay(500);
}
