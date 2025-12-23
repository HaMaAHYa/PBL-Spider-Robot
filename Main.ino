#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define SERVOMIN 150
#define SERVOMAX 650

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver();


void setup() {
  Serial.begin(9600);
  pca.begin();
  pca.reset();
  pca.setPWMFreq(60);


  standBy();
  delay(4000);
}

int dt = 100;
int lift = 100;
int down = 180;

int RF_LB_standby = 45;
int RF_LB_center = 0;
int LF_RB_standby = 135;
int LF_RB_center = 180;

const int LF_hip = 0;
const int RF_hip = 2;
const int LB_hip = 4;
const int RB_hip = 6;
const int LF_leg = 1;
const int RF_leg = 3;
const int LB_leg = 5;
const int RB_leg = 7;


void loop() {
  // --- step 1 --- //
  // lift LF leg
  pca.setPWM(LF_leg, 0, angleToPulse(lift));
  delay(dt);
  // move LF hip to 110 = 90 + 20
  pca.setPWM(LF_hip, 0, angleToPulse(100));
  delay(dt);
  // place LF leg
  pca.setPWM(LF_leg, 0, angleToPulse(down));
  delay(dt+50);

  // --- step 2 --- //
  // swing LF to standby
  pca.setPWM(LF_hip, 0, angleToPulse(LF_RB_center));
  // swing LB to standby
  pca.setPWM(LB_hip, 0, angleToPulse(RF_LB_standby+20));
  delay(dt+100);

  // --- step 3 --- //
  // lift RB leg
  pca.setPWM(RB_leg, 0, angleToPulse(lift));
  delay(dt);
  // move RB hip to center
  pca.setPWM(RB_hip, 0, angleToPulse(LF_RB_center));
  delay(dt);
  // place RB leg
  pca.setPWM(RB_leg, 0, angleToPulse(down));
  delay(dt+50);

  // --- step 4 --- //
  // lift RF leg
  pca.setPWM(RF_leg, 0, angleToPulse(lift));
  delay(dt);
  // move RF hip to 70 = 90 - 20
  pca.setPWM(RF_hip, 0, angleToPulse(80));
  delay(dt);
  // place RF leg
  pca.setPWM(RF_leg, 0, angleToPulse(down));
  delay(dt+50);

  // --- step 5 --- //
  // swing RF to standby
  pca.setPWM(RF_hip, 0, angleToPulse(RF_LB_center));
  // swing RB to standby
  pca.setPWM(RB_hip, 0, angleToPulse(LF_RB_standby-15));
  delay(dt+100);

  // --- step 6 --- //
  // lift LB leg
  pca.setPWM(LB_leg, 0, angleToPulse(lift));
  delay(dt);
  // move LB hip to center
  pca.setPWM(LB_hip, 0, angleToPulse(RF_LB_center));
  delay(dt);
  // place LB leg
  pca.setPWM(LB_leg, 0, angleToPulse(down));
  delay(dt+50);

}

void standBy() {
  pca.setPWM(RF_hip, 0, angleToPulse(RF_LB_standby));
  pca.setPWM(LB_hip, 0, angleToPulse(RF_LB_standby+20));
  pca.setPWM(RB_hip, 0, angleToPulse(LF_RB_standby-15));
  pca.setPWM(LF_hip, 0, angleToPulse(LF_RB_standby));
  pca.setPWM(RF_leg, 0, angleToPulse(down));
  pca.setPWM(RB_leg, 0, angleToPulse(down));
  pca.setPWM(LF_leg, 0, angleToPulse(down));
  pca.setPWM(LB_leg, 0, angleToPulse(down));

}


int angleToPulse(float ang) {
  return map((int)ang, 0, 180, SERVOMIN, SERVOMAX);
}
