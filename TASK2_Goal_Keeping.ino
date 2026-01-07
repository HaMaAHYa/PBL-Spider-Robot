/*

  SPIDER ROBOT - SERVO & ULTRASONIC SENSOR STARTER CODE

  This code provides basic setup and control for:
  - Adafruit PWM Servo Driver (16 servo channels)
  - Ultrasonic sensor (HC-SR04) attached to servo for scanning

  Servo Channel Map:
            90        90
            |    ^    |
    180_____|____|____|_____0
            |0       2|
            |         |
      0_____|4_______6|_____180
            |         |
            |         |
            90        90

  Channel 0-7: 8 servo motors (legs and hips)
  Channel 8: Sensor servo (ultrasonic motor)

*/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define SERVOMIN 150
#define SERVOMAX 650

#define TRIG_PIN 13
#define ECHO_PIN 12
const int sensorServo = 8;

#define MAX_ANGLE 180
int distanceMap[MAX_ANGLE + 1];

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver();

struct ServoMap
{
    int hipLF;
    int hipRF;
    int hipLB;
    int hipRB;
    int legLF;
    int legRF;
    int legLB;
    int legRB;
};

ServoMap LeftMap = {
    .hipLF = 0,
    .hipRF = 2,
    .hipLB = 4,
    .hipRB = 6,
    .legLF = 1,
    .legRF = 3,
    .legLB = 5,
    .legRB = 7};

ServoMap RightMap = {
    .hipLF = 6,
    .hipRF = 4,
    .hipLB = 2,
    .hipRB = 0,
    .legLF = 7,
    .legRF = 5,
    .legLB = 3,
    .legRB = 1};

int robotDelayMs = 100;
int robotLegLiftDeg = 100;
int robotLegPlaceDeg = 180;

int RF_LB_standby = 45;
int RF_LB_center = 0;
int LF_RB_standby = 135;
int LF_RB_center = 180;

const int hipLF = 0, legLF = 1;
const int hipRF = 2, legRF = 3;
const int hipLB = 4, legLB = 5;
const int hipRB = 6, legRB = 7;

// Timer-based state tracking
unsigned long lastActionTime = 0;
unsigned long actionInterval = 0;
int currentStep = 0;

// Ultrasonic sensor scanning
int currentAngle = 0;
int step = 5;
unsigned long lastMove = 0;
unsigned long scanStepTime = 5;

int nearestDistance = 999;
int nearestAngle = 0;

bool isContinuousMovement = false;
bool isDriving = false;

// Walking state machine
enum MovementState
{
    MOVE_IDLE,

    // setServo(leg, robotLegLiftDeg);
    MOVE_LEFT_STEP1_LIFT_LEG,
    // setServo(hip, hipAngle);
    MOVE_LEFT_STEP1_MOVE_HIP,
    // setServo(leg, robotLegPlaceDeg);
    MOVE_LEFT_STEP1_PLACE_LEG,

    MOVE_LEFT_STEP2,
    MOVE_LEFT_STEP3_LIFT_LEG,
    MOVE_LEFT_STEP3_MOVE_HIP,
    MOVE_LEFT_STEP3_PLACE_LEG,
    MOVE_LEFT_STEP4_LIFT_LEG,
    MOVE_LEFT_STEP4_MOVE_HIP,
    MOVE_LEFT_STEP4_PLACE_LEG,
    MOVE_LEFT_STEP5,
    MOVE_LEFT_STEP6_LIFT_LEG,
    MOVE_LEFT_STEP6_MOVE_HIP,
    MOVE_LEFT_STEP6_PLACE_LEG,

    MOVE_RIGHT_STEP1_LIFT_LEG,
    MOVE_RIGHT_STEP1_MOVE_HIP,
    MOVE_RIGHT_STEP1_PLACE_LEG,
    MOVE_RIGHT_STEP2,
    MOVE_RIGHT_STEP3_LIFT_LEG,
    MOVE_RIGHT_STEP3_MOVE_HIP,
    MOVE_RIGHT_STEP3_PLACE_LEG,
    MOVE_RIGHT_STEP4_LIFT_LEG,
    MOVE_RIGHT_STEP4_MOVE_HIP,
    MOVE_RIGHT_STEP4_PLACE_LEG,
    MOVE_RIGHT_STEP5,
    MOVE_RIGHT_STEP6_LIFT_LEG,
    MOVE_RIGHT_STEP6_MOVE_HIP,
    MOVE_RIGHT_STEP6_PLACE_LEG
};

MovementState movementState = MOVE_IDLE;
unsigned long movementTimer = 0;

void setup()
{
    Serial.begin(9600);
    pca.begin();
    pca.reset();
    pca.setPWMFreq(60);

    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);

    for (int i = 0; i <= MAX_ANGLE; i++)
    {
        distanceMap[i] = -1;
    }

    standBy();
    delay(4000);
    lastActionTime = millis();
    actionInterval = 4000;
}

void loop()
{
    unsigned long now = millis();

    // Scanning logic for ultrasonic sensor
    if (now - lastMove >= scanStepTime)
    {
        lastMove = now;

        currentAngle += step;

        if (currentAngle >= 90 || currentAngle <= 0)
        {
            step = -step;
        }

        setServo(sensorServo, currentAngle);

        int distance = getDistance();

        if (distance > 0 && distance < nearestDistance)
        {
            nearestDistance = distance;
            nearestAngle = currentAngle;
        }

        Serial.print("Angle: ");
        Serial.print(currentAngle);
        Serial.print("°  Distance: ");
        Serial.print(distance);
        Serial.print("cm  Nearest: ");
        Serial.print(nearestDistance);
        Serial.print("cm @ ");
        Serial.print(nearestAngle);
        Serial.println("°");

        // Goal keeping logic - if object is very close (< 10cm), execute driveLeft or driveRight
        if (nearestDistance < 10 && nearestDistance > 0)
        {
            if (nearestAngle > 60)
            {
                isContinuousMovement = false;
                isDriving = true;
                driveLeft();
            }
            else if (nearestAngle < 40)
            {
                isContinuousMovement = false;
                isDriving = true;
                driveRight();
            }
            else
            {
                isContinuousMovement = false;
                isDriving = false;
                standBy();
            }
        }
        // Only start new movement if idle
        else if (movementState == MOVE_IDLE && now >= movementTimer)
        {
            if (nearestAngle > 60)
            {
                isContinuousMovement = true;
                isDriving = false;
                startMoveLeft();
            }
            else if (nearestAngle > 40)
            {
                isContinuousMovement = false;
                isDriving = false;
                standBy();
            }
            else
            {
                isContinuousMovement = true;
                isDriving = false;
                startMoveRight();
            }
        }
    }

    // Execute movement state machine concurrently with scanning
    processMovement(now);
}

void processMovement(unsigned long now)
{
    if (movementState == MOVE_IDLE || now < movementTimer)
    {
        return;
    }

    switch (movementState)
    {
        // LEFT MOVEMENT

    case MOVE_LEFT_STEP1_LIFT_LEG:
        setServo(LeftMap.legLF, robotLegLiftDeg);
        movementTimer = now + robotDelayMs;
        movementState = MOVE_LEFT_STEP1_MOVE_HIP;
        break;

    case MOVE_LEFT_STEP1_MOVE_HIP:
        setServo(LeftMap.hipLF, LF_RB_center);
        movementTimer = now + robotDelayMs;
        movementState = MOVE_LEFT_STEP1_PLACE_LEG;
        break;

    case MOVE_LEFT_STEP1_PLACE_LEG:
        setServo(LeftMap.legLF, robotLegPlaceDeg);
        movementTimer = now + robotDelayMs + 50;
        movementState = MOVE_LEFT_STEP2;
        break;

    case MOVE_LEFT_STEP2:
        swingHipsImmediate(LeftMap.hipLF, LF_RB_center, LeftMap.hipLB, RF_LB_standby + 20);
        movementTimer = now + robotDelayMs + 100;
        movementState = MOVE_LEFT_STEP3_LIFT_LEG;
        break;

    case MOVE_LEFT_STEP3_LIFT_LEG:
        setServo(LeftMap.legRB, robotLegLiftDeg);
        movementTimer = now + robotDelayMs;
        movementState = MOVE_LEFT_STEP3_MOVE_HIP;
        break;

    case MOVE_LEFT_STEP3_MOVE_HIP:
        setServo(LeftMap.hipRB, LF_RB_center);
        movementTimer = now + robotDelayMs;
        movementState = MOVE_LEFT_STEP3_PLACE_LEG;
        break;

    case MOVE_LEFT_STEP3_PLACE_LEG:
        setServo(LeftMap.legRB, robotLegPlaceDeg);
        movementTimer = now + robotDelayMs + 50;
        movementState = MOVE_LEFT_STEP4_LIFT_LEG;
        break;

    case MOVE_LEFT_STEP4_LIFT_LEG:
        setServo(LeftMap.legRF, robotLegLiftDeg);
        movementTimer = now + robotDelayMs;
        movementState = MOVE_LEFT_STEP4_MOVE_HIP;
        break;

    case MOVE_LEFT_STEP4_MOVE_HIP:
        setServo(LeftMap.hipRF, 80);
        movementTimer = now + robotDelayMs;
        movementState = MOVE_LEFT_STEP4_PLACE_LEG;
        break;

    case MOVE_LEFT_STEP4_PLACE_LEG:
        setServo(LeftMap.legRF, robotLegPlaceDeg);
        movementTimer = now + robotDelayMs + 50;
        movementState = MOVE_LEFT_STEP5;
        break;

    case MOVE_LEFT_STEP5:
        swingHipsImmediate(LeftMap.hipRF, RF_LB_center, LeftMap.hipRB, LF_RB_standby - 15);
        movementTimer = now + robotDelayMs + 100;
        movementState = MOVE_LEFT_STEP6_LIFT_LEG;
        break;

    case MOVE_LEFT_STEP6_LIFT_LEG:
        setServo(LeftMap.legLB, robotLegLiftDeg);
        movementTimer = now + robotDelayMs;
        movementState = MOVE_LEFT_STEP6_MOVE_HIP;
        break;

    case MOVE_LEFT_STEP6_MOVE_HIP:
        setServo(LeftMap.hipLB, RF_LB_center);
        movementTimer = now + robotDelayMs;
        movementState = MOVE_LEFT_STEP6_PLACE_LEG;
        break;

    case MOVE_LEFT_STEP6_PLACE_LEG:
        setServo(LeftMap.legLB, robotLegPlaceDeg);
        movementTimer = now + robotDelayMs + 50;

        // continue moving indefinitely (no targetRepeatCount)
        if (isContinuousMovement)
        {
            movementState = MOVE_LEFT_STEP1_LIFT_LEG;
        }
        else
        {
            movementState = MOVE_IDLE;
        }
        break;

        // RIGHT MOVEMENT

    case MOVE_RIGHT_STEP1_LIFT_LEG:
        setServo(RightMap.legLF, robotLegLiftDeg);
        movementTimer = now + robotDelayMs;
        movementState = MOVE_RIGHT_STEP1_MOVE_HIP;
        break;

    case MOVE_RIGHT_STEP1_MOVE_HIP:
        setServo(RightMap.hipLF, 100);
        movementTimer = now + robotDelayMs;
        movementState = MOVE_RIGHT_STEP1_PLACE_LEG;
        break;

    case MOVE_RIGHT_STEP1_PLACE_LEG:
        setServo(RightMap.legLF, robotLegPlaceDeg);
        movementTimer = now + robotDelayMs + 50;
        movementState = MOVE_RIGHT_STEP2;
        break;

    case MOVE_RIGHT_STEP2:
        swingHipsImmediate(RightMap.hipLF, LF_RB_center, RightMap.hipLB, RF_LB_standby + 20);
        movementTimer = now + robotDelayMs + 100;
        movementState = MOVE_RIGHT_STEP3_LIFT_LEG;
        break;

    case MOVE_RIGHT_STEP3_LIFT_LEG:
        setServo(RightMap.legRB, robotLegLiftDeg);
        movementTimer = now + robotDelayMs;
        movementState = MOVE_RIGHT_STEP3_MOVE_HIP;
        break;

    case MOVE_RIGHT_STEP3_MOVE_HIP:
        setServo(RightMap.hipRB, LF_RB_center);
        movementTimer = now + robotDelayMs;
        movementState = MOVE_RIGHT_STEP3_PLACE_LEG;
        break;

    case MOVE_RIGHT_STEP3_PLACE_LEG:
        setServo(RightMap.legRB, robotLegPlaceDeg);
        movementTimer = now + robotDelayMs + 50;
        movementState = MOVE_RIGHT_STEP4_LIFT_LEG;
        break;

    case MOVE_RIGHT_STEP4_LIFT_LEG:
        setServo(RightMap.legRF, robotLegLiftDeg);
        movementTimer = now + robotDelayMs;
        movementState = MOVE_RIGHT_STEP4_MOVE_HIP;
        break;

    case MOVE_RIGHT_STEP4_MOVE_HIP:
        setServo(RightMap.hipRF, 80);
        movementTimer = now + robotDelayMs;
        movementState = MOVE_RIGHT_STEP4_PLACE_LEG;
        break;

    case MOVE_RIGHT_STEP4_PLACE_LEG:
        setServo(RightMap.legRF, robotLegPlaceDeg);
        movementTimer = now + robotDelayMs + 50;
        movementState = MOVE_RIGHT_STEP5;
        break;

    case MOVE_RIGHT_STEP5:
        swingHipsImmediate(RightMap.hipRF, RF_LB_center, RightMap.hipRB, LF_RB_standby - 15);
        movementTimer = now + robotDelayMs + 100;
        movementState = MOVE_RIGHT_STEP6_LIFT_LEG;
        break;

    case MOVE_RIGHT_STEP6_LIFT_LEG:
        setServo(RightMap.legLB, robotLegLiftDeg);
        movementTimer = now + robotDelayMs;
        movementState = MOVE_RIGHT_STEP6_MOVE_HIP;
        break;

    case MOVE_RIGHT_STEP6_MOVE_HIP:
        setServo(RightMap.hipLB, RF_LB_center);
        movementTimer = now + robotDelayMs;
        movementState = MOVE_RIGHT_STEP6_PLACE_LEG;
        break;

    case MOVE_RIGHT_STEP6_PLACE_LEG:
        setServo(RightMap.legLB, robotLegPlaceDeg);
        movementTimer = now + robotDelayMs + 50;

        if (isContinuousMovement)
        {
            movementState = MOVE_RIGHT_STEP1_LIFT_LEG;
        }
        else
        {
            movementState = MOVE_IDLE;
        }
        break;
    }
}

void startMoveLeft()
{
    movementState = MOVE_LEFT_STEP1_LIFT_LEG;
    movementTimer = millis();
}

void startMoveRight()
{
    movementState = MOVE_RIGHT_STEP1_LIFT_LEG;
    movementTimer = millis();
}

void swingHipsImmediate(int hip1, int ang1, int hip2, int ang2)
{
    setServo(hip1, ang1);
    setServo(hip2, ang2);
}

void driveLeft()
{
    safePose(LeftMap);
}

void driveRight()
{
    safePose(RightMap);
}

void safePose(const ServoMap &map)
{
    setServo(map.hipRF, 90);
    setServo(map.hipLB, RF_LB_standby + 20);
    setServo(map.hipRB, LF_RB_standby - 15);
    setServo(map.hipLF, 90);

    setServo(map.legLF, 120);
    setServo(map.legRF, 45);
    setServo(map.legLB, robotLegPlaceDeg);
    setServo(map.legRB, robotLegPlaceDeg);
}

void standBy()
{
    setServo(hipRF, RF_LB_standby);
    setServo(hipLB, RF_LB_standby + 20);
    setServo(hipRB, LF_RB_standby - 15);
    setServo(hipLF, LF_RB_standby);

    setServo(legLF, robotLegPlaceDeg);
    setServo(legRF, robotLegPlaceDeg);
    setServo(legLB, robotLegPlaceDeg);
    setServo(legRB, robotLegPlaceDeg);
}

void setServo(int channel, int angle)
{
    pca.setPWM(channel, 0, angleToPulse(angle));
}

int angleToPulse(float ang)
{
    return map((int)ang, 0, 180, SERVOMIN, SERVOMAX);
}

int getDistance()
{
    long duration;

    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    duration = pulseIn(ECHO_PIN, HIGH, 30000);

    if (duration == 0)
        return -1;

    return duration / 58;
}
