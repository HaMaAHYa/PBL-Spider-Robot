#include <Servo.h>

struct Leg {
  Servo hip;
  Servo knee;
};

enum LegID {
  LF,
  RF,
  LB,
  RB
};

Leg legs[4];

/* --- Use ONCE in setup --- */
#define LF_hip_pin 2
#define LF_knee_pin 3
#define RF_hip_pin 4
#define RF_knee_pin 5
#define LB_hip_pin 6
#define LB_knee_pin 7
#define RB_hip_pin 8
#define RB_knee_pin 9

/* --- Ultrasonic Sensor ---*/
const int TRIG_PIN = 10;
const int ECHO_PIN = 11;

#define TOO_CLOSE_CM 5
#define CLIMBING_RANGE_CM 10

#define HIP_FORWARD 120
#define HIP_CENTER  90
#define HIP_BACK    60

#define KNEE_UP     135
#define KNEE_DOWN   90

#define KNEE_CLIMB_UP 160

int stepDelay = 250;

/* --- Function Decleared ---*/
// Helper Function
void setLeg(LegID leg, int hipAngle, int kneeAngle);
int getDistance();

// Main Function (Robot Status)
void standBy();
void walkForward();
void walkBackward();
void climbStair();

void setup() {
  Serial.begin(9600);
  
  legs[LF].hip.attach(LF_hip_pin);
  legs[LF].knee.attach(LF_knee_pin);
  
  legs[RF].hip.attach(RF_hip_pin);
  legs[RF].knee.attach(RF_knee_pin);
  
  legs[LB].hip.attach(LB_hip_pin);
  legs[LB].knee.attach(LB_knee_pin);
  
  legs[RB].hip.attach(RB_hip_pin);
  legs[RB].knee.attach(RB_knee_pin);
  
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  standBy();
  delay(1000);
  Serial.println("Robot ready.");
}

void loop() {
  int distance = getDistance();
  
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  if (distance > CLIMBING_RANGE_CM) {
    walkForward();
  }
  else if (distance > TOO_CLOSE_CM && distance <= CLIMBING_RANGE_CM) {
    Serial.println("Stair detected in climbing range! Stopping.");
    standBy();
    delay(1000);
    climbStair();
  }
  else {
    Serial.println("Obstacle too close. Walk backward");
    standBy();
    delay(1000);
    walkBackward();
  }
}


void standBy() {
  setLeg(LF, HIP_CENTER, KNEE_DOWN);
  setLeg(RF, HIP_CENTER, KNEE_DOWN);
  setLeg(LB, HIP_CENTER, KNEE_DOWN);
  setLeg(RB, HIP_CENTER, KNEE_DOWN);
}

void walkForward() {
  Serial.println("--- Walking ---");
  setLeg(LF, HIP_CENTER, KNEE_UP);
  setLeg(RB, HIP_CENTER, KNEE_UP);
  delay(stepDelay);

  setLeg(LF, HIP_FORWARD, KNEE_UP);
  setLeg(RB, HIP_FORWARD, KNEE_UP);
  setLeg(RF, HIP_BACK, KNEE_DOWN);
  setLeg(LB, HIP_BACK, KNEE_DOWN);
  delay(stepDelay);

  setLeg(LF, HIP_FORWARD, KNEE_DOWN);
  setLeg(RB, HIP_FORWARD, KNEE_DOWN);
  delay(stepDelay);

  setLeg(RF, HIP_CENTER, KNEE_UP);
  setLeg(LB, HIP_CENTER, KNEE_UP);
  delay(stepDelay);

  setLeg(RF, HIP_FORWARD, KNEE_UP);
  setLeg(LB, HIP_FORWARD, KNEE_UP);
  setLeg(LF, HIP_BACK, KNEE_DOWN);
  setLeg(RB, HIP_BACK, KNEE_DOWN);
  delay(stepDelay);

  setLeg(RF, HIP_FORWARD, KNEE_DOWN);
  setLeg(LB, HIP_FORWARD, KNEE_DOWN);
  delay(stepDelay);
}

void walkBackward() {
  Serial.println("--- Stepping Back ---");
  setLeg(LF, HIP_CENTER, KNEE_UP);
  setLeg(RB, HIP_CENTER, KNEE_UP);
  delay(stepDelay);

  setLeg(LF, HIP_BACK, KNEE_UP);
  setLeg(RB, HIP_BACK, KNEE_UP);
  setLeg(RF, HIP_FORWARD, KNEE_DOWN);
  setLeg(LB, HIP_FORWARD, KNEE_DOWN);
  delay(stepDelay);

  setLeg(LF, HIP_BACK, KNEE_DOWN);
  setLeg(RB, HIP_BACK, KNEE_DOWN);
  delay(stepDelay);

  setLeg(RF, HIP_CENTER, KNEE_UP);
  setLeg(LB, HIP_CENTER, KNEE_UP);
  delay(stepDelay);

  setLeg(RF, HIP_BACK, KNEE_UP);
  setLeg(LB, HIP_BACK, KNEE_UP);
  setLeg(LF, HIP_FORWARD, KNEE_DOWN);
  setLeg(RB, HIP_FORWARD, KNEE_DOWN);
  delay(stepDelay);

  setLeg(RF, HIP_BACK, KNEE_DOWN);
  setLeg(LB, HIP_BACK, KNEE_DOWN);
  delay(stepDelay);
}

void climbStair() {
  Serial.println("--- Starting Climbing ---");

  setLeg(RF, HIP_CENTER, KNEE_CLIMB_UP);
  delay(stepDelay * 2);

  setLeg(RF, HIP_FORWARD, KNEE_CLIMB_UP);
  delay(stepDelay * 2);

  setLeg(RF, HIP_FORWARD, KNEE_DOWN);
  delay(stepDelay * 2);

  setLeg(LF, HIP_CENTER, KNEE_CLIMB_UP);
  delay(stepDelay * 2);

  setLeg(LF, HIP_FORWARD, KNEE_CLIMB_UP);
  delay(stepDelay * 2);

  setLeg(LF, HIP_FORWARD, KNEE_DOWN);
  delay(stepDelay * 2);

  setLeg(LF, HIP_BACK, KNEE_DOWN);
  setLeg(RF, HIP_BACK, KNEE_DOWN);
  setLeg(LB, HIP_BACK, KNEE_DOWN);
  setLeg(RB, HIP_BACK, KNEE_DOWN);
  delay(stepDelay * 3);

  setLeg(RB, HIP_CENTER, KNEE_CLIMB_UP);
  delay(stepDelay * 2);

  setLeg(RB, HIP_CENTER, KNEE_DOWN);
  delay(stepDelay * 2);

  setLeg(LB, HIP_CENTER, KNEE_CLIMB_UP);
  delay(stepDelay * 2);

  setLeg(LB, HIP_CENTER, KNEE_DOWN);
  delay(stepDelay * 2);

  Serial.println("Climb complete!");
  standBy();
  delay(stepDelay * 4);
}

/* --- Helper Function for Get the Distance from Ultrasonic Sensor --- */
int getDistance() {
  long duration;
  int distance_cm;

  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  duration = pulseIn(ECHO_PIN, HIGH, 30000);

  distance_cm = duration / 58;

  if (distance_cm == 0) {
    return 999; 
  }

  return distance_cm;
}

/* --- Helper Function for Control a Leg Movement --- */
void setLeg(LegID leg, int hipAngle, int kneeAngle) {
  legs[leg].hip.write(hipAngle);
  legs[leg].knee.write(kneeAngle);
}
