/*

  SPIDER ROBOT - SERVO & ULTRASONIC SENSOR STARTER CODE

  This code provides basic setup and control for:
  - Adafruit PWM Servo Driver (16 servo channels)
  - Ultrasonic sensor (HC-SR04) attached to servo for scanning

  Servo Channel Map:
            90        90
            |         |
    180_____|_________|_____0
            |0       2|
            |         |-->
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
#define SENSOR_SERVO 8

#define MAX_ANGLE 180
#define SERVO_FREQ 60

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

ServoMap FrontMap = {
    .hipLF = 2,
    .hipRF = 6,
    .hipLB = 0,
    .hipRB = 4,

    .legLF = 3,
    .legRF = 7,
    .legLB = 1,
    .legRB = 5};

ServoMap BackMap = {
    .hipLF = 4,
    .hipRF = 0,
    .hipLB = 6,
    .hipRB = 2,
    .legLF = 5,
    .legRF = 1,
    .legLB = 7,
    .legRB = 3};

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

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver();

int RF_LB_standby = 45;
int RF_LB_center = 0;
int LF_RB_standby = 135;
int LF_RB_center = 180;
int robotDelayMs = 100;
int robotLegLiftDeg = 100;
int robotLegPlaceDeg = 180;

int currentAngle = 0;
int step = 5;
unsigned long lastMove = 0;
unsigned long scanStepTime = 10;

int nearestDistance = 999;
int nearestAngle = 0;

enum MovementState
{
    MOVE_IDLE,
    // FORWARD movement split into lift/moveHip/place phases for steps 1,3,4,6
    MOVE_FORWARD_STEP1_LIFT_LEG,
    MOVE_FORWARD_STEP1_MOVE_HIP,
    MOVE_FORWARD_STEP1_PLACE_LEG,
    MOVE_FORWARD_STEP2,
    MOVE_FORWARD_STEP3_LIFT_LEG,
    MOVE_FORWARD_STEP3_MOVE_HIP,
    MOVE_FORWARD_STEP3_PLACE_LEG,
    MOVE_FORWARD_STEP4_LIFT_LEG,
    MOVE_FORWARD_STEP4_MOVE_HIP,
    MOVE_FORWARD_STEP4_PLACE_LEG,
    MOVE_FORWARD_STEP5,
    MOVE_FORWARD_STEP6_LIFT_LEG,
    MOVE_FORWARD_STEP6_MOVE_HIP,
    MOVE_FORWARD_STEP6_PLACE_LEG,

    // BACKWARD movement split into lift/moveHip/place phases for steps 1,3,4,6
    MOVE_BACKWARD_STEP1_LIFT_LEG,
    MOVE_BACKWARD_STEP1_MOVE_HIP,
    MOVE_BACKWARD_STEP1_PLACE_LEG,
    MOVE_BACKWARD_STEP2,
    MOVE_BACKWARD_STEP3_LIFT_LEG,
    MOVE_BACKWARD_STEP3_MOVE_HIP,
    MOVE_BACKWARD_STEP3_PLACE_LEG,
    MOVE_BACKWARD_STEP4_LIFT_LEG,
    MOVE_BACKWARD_STEP4_MOVE_HIP,
    MOVE_BACKWARD_STEP4_PLACE_LEG,
    MOVE_BACKWARD_STEP5,
    MOVE_BACKWARD_STEP6_LIFT_LEG,
    MOVE_BACKWARD_STEP6_MOVE_HIP,
    MOVE_BACKWARD_STEP6_PLACE_LEG,

    // LEFT movement split into lift/moveHip/place phases for steps 1,3,4,6
    MOVE_LEFT_STEP1_LIFT_LEG,
    MOVE_LEFT_STEP1_MOVE_HIP,
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

    // RIGHT movement split into lift/moveHip/place phases for steps 1,3,4,6
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
    MOVE_RIGHT_STEP6_PLACE_LEG,

    // DANCE states
    DANCE_STEP1,
    DANCE_STEP2,
    DANCE_STEP3,
    DANCE_STEP4,

    // GREET state

};

MovementState movementState = MOVE_IDLE;
unsigned long movementTimer = 0;
bool isContinuousMovement = false;
bool isDancing = false;
int currentCommand = 0;

unsigned long lastMovingAction = 0;
const int MOVING_INTERVAL = 5000;

void setup()
{
    Serial.begin(9600);

    pca.begin();
    pca.reset();
    pca.setPWMFreq(SERVO_FREQ);

    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);

    standBy();
    setServo(SENSOR_SERVO, 45);
    delay(4000);
}

void loop()
{
    unsigned long now = millis();

    if (now - lastMove >= scanStepTime)
    {
        lastMove = now;

        int distance = getDistance();

        if (distance != -1 && distance < nearestDistance)
        {
            nearestDistance = distance;
            nearestAngle = currentAngle;
        }
    }
    // If object detected within 10cm, stop and go to standby

    checkSerialCommand();

    processMovement(now);
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

void standBy()
{
    setServo(LeftMap.hipRF, RF_LB_standby);
    setServo(LeftMap.hipLB, RF_LB_standby + 20);
    setServo(LeftMap.hipRB, LF_RB_standby - 15);
    setServo(LeftMap.hipLF, LF_RB_standby);

    setServo(LeftMap.legLF, robotLegPlaceDeg);
    setServo(LeftMap.legRF, robotLegPlaceDeg);
    setServo(LeftMap.legLB, robotLegPlaceDeg);
    setServo(LeftMap.legRB, robotLegPlaceDeg);

    Serial.println("Robot: Stand By");
}

void dance()
{
    Serial.println("Robot: Dance Time!");

    driveLeft();

    bodyLift();

    driveRight();

    bodyLift();
}

void checkSerialCommand()
{
    if (Serial.available() > 0)
    {
        char command = Serial.read();

        switch (command)
        {
        case 'w':
        case 'W':
            isDancing = false;
            isContinuousMovement = true;
            currentCommand = 1;
            if (movementState == MOVE_IDLE || movementState >= DANCE_STEP1)
            {
                startMoveForward();
                Serial.println("Command: Move Forward (Continuous)");
            }
            break;
        case 's':
        case 'S':
            isDancing = false;
            isContinuousMovement = true;
            currentCommand = 2;
            if (movementState == MOVE_IDLE || movementState >= DANCE_STEP1)
            {
                startMoveBackward();
                Serial.println("Command: Move Backward (Continuous)");
            }
            break;
        case 'a':
        case 'A':
            isDancing = false;
            isContinuousMovement = true;
            currentCommand = 3;
            if (movementState == MOVE_IDLE || movementState >= DANCE_STEP1)
            {
                startMoveLeft();
                Serial.println("Command: Move Left (Continuous)");
            }
            break;
        case 'd':
        case 'D':
            isDancing = false;
            isContinuousMovement = true;
            currentCommand = 4;
            if (movementState == MOVE_IDLE || movementState >= DANCE_STEP1)
            {
                startMoveRight();
                Serial.println("Command: Move Right (Continuous)");
            }
            break;
        case ' ':
            isContinuousMovement = false;
            isDancing = false;
            currentCommand = 0;
            movementState = MOVE_IDLE;
            standBy();
            Serial.println("Command: Stop");
            break;
        case 'p':
        case 'P':
            isContinuousMovement = false;
            currentCommand = 0;
            isDancing = true;
            movementState = DANCE_STEP1;
            movementTimer = 0;
            Serial.println("Command: Dance (Continuous)");
            break;
        }
    }
}

void processMovement(unsigned long now)
{
    if (movementState == MOVE_IDLE || now < movementTimer)
    {
        return;
    }

    switch (movementState)
    {
    case MOVE_FORWARD_STEP1_LIFT_LEG:
        setServo(FrontMap.legLF, robotLegLiftDeg);
        movementTimer = now + robotDelayMs;
        movementState = MOVE_FORWARD_STEP1_MOVE_HIP;
        break;
    case MOVE_FORWARD_STEP1_MOVE_HIP:
        setServo(FrontMap.hipLF, 10);
        movementTimer = now + robotDelayMs;
        movementState = MOVE_FORWARD_STEP1_PLACE_LEG;
        break;
    case MOVE_FORWARD_STEP1_PLACE_LEG:
        setServo(FrontMap.legLF, robotLegPlaceDeg);
        movementTimer = now + robotDelayMs + 50;
        movementState = MOVE_FORWARD_STEP2;
        break;
    case MOVE_FORWARD_STEP2:
        swingHipsImmediate(FrontMap.hipLF, 90, FrontMap.hipLB, 135 + 20);
        movementTimer = now + robotDelayMs + 100;
        movementState = MOVE_FORWARD_STEP3_LIFT_LEG;
        break;
    case MOVE_FORWARD_STEP3_LIFT_LEG:
        setServo(FrontMap.legRB, robotLegLiftDeg);
        movementTimer = now + robotDelayMs;
        movementState = MOVE_FORWARD_STEP3_MOVE_HIP;
        break;
    case MOVE_FORWARD_STEP3_MOVE_HIP:
        setServo(FrontMap.hipRB, 90);
        movementTimer = now + robotDelayMs;
        movementState = MOVE_FORWARD_STEP3_PLACE_LEG;
        break;
    case MOVE_FORWARD_STEP3_PLACE_LEG:
        setServo(FrontMap.legRB, robotLegPlaceDeg);
        movementTimer = now + robotDelayMs + 50;
        movementState = MOVE_FORWARD_STEP4_LIFT_LEG;
        break;
    case MOVE_FORWARD_STEP4_LIFT_LEG:
        setServo(FrontMap.legRF, robotLegLiftDeg);
        movementTimer = now + robotDelayMs;
        movementState = MOVE_FORWARD_STEP4_MOVE_HIP;
        break;
    case MOVE_FORWARD_STEP4_MOVE_HIP:
        setServo(FrontMap.hipRF, 170);
        movementTimer = now + robotDelayMs;
        movementState = MOVE_FORWARD_STEP4_PLACE_LEG;
        break;
    case MOVE_FORWARD_STEP4_PLACE_LEG:
        setServo(FrontMap.legRF, robotLegPlaceDeg);
        movementTimer = now + robotDelayMs + 50;
        movementState = MOVE_FORWARD_STEP5;
        break;
    case MOVE_FORWARD_STEP5:
        swingHipsImmediate(FrontMap.hipRF, 90, FrontMap.hipRB, 45 - 15);
        movementTimer = now + robotDelayMs + 100;
        movementState = MOVE_FORWARD_STEP6_LIFT_LEG;
        break;
    case MOVE_FORWARD_STEP6_LIFT_LEG:
        setServo(FrontMap.legLB, robotLegLiftDeg);
        movementTimer = now + robotDelayMs;
        movementState = MOVE_FORWARD_STEP6_MOVE_HIP;
        break;
    case MOVE_FORWARD_STEP6_MOVE_HIP:
        setServo(FrontMap.hipLB, 90);
        movementTimer = now + robotDelayMs;
        movementState = MOVE_FORWARD_STEP6_PLACE_LEG;
        break;
    case MOVE_FORWARD_STEP6_PLACE_LEG:
        setServo(FrontMap.legLB, robotLegPlaceDeg);
        movementTimer = now + robotDelayMs + 50;
        if (isContinuousMovement && currentCommand == 1)
        {
            movementState = MOVE_FORWARD_STEP1_LIFT_LEG;
        }
        else
        {
            movementState = MOVE_IDLE;
        }
        break;

    case MOVE_BACKWARD_STEP1_LIFT_LEG:
        setServo(BackMap.legLF, robotLegLiftDeg);
        movementTimer = now + robotDelayMs;
        movementState = MOVE_BACKWARD_STEP1_MOVE_HIP;
        break;
    case MOVE_BACKWARD_STEP1_MOVE_HIP:
        setServo(BackMap.hipLF, 10);
        movementTimer = now + robotDelayMs;
        movementState = MOVE_BACKWARD_STEP1_PLACE_LEG;
        break;
    case MOVE_BACKWARD_STEP1_PLACE_LEG:
        setServo(BackMap.legLF, robotLegPlaceDeg);
        movementTimer = now + robotDelayMs + 50;
        movementState = MOVE_BACKWARD_STEP2;
        break;
    case MOVE_BACKWARD_STEP2:
        swingHipsImmediate(BackMap.hipLF, 90, BackMap.hipLB, 135 + 20);
        movementTimer = now + robotDelayMs + 100;
        movementState = MOVE_BACKWARD_STEP3_LIFT_LEG;
        break;
    case MOVE_BACKWARD_STEP3_LIFT_LEG:
        setServo(BackMap.legRB, robotLegLiftDeg);
        movementTimer = now + robotDelayMs;
        movementState = MOVE_BACKWARD_STEP3_MOVE_HIP;
        break;
    case MOVE_BACKWARD_STEP3_MOVE_HIP:
        setServo(BackMap.hipRB, 90);
        movementTimer = now + robotDelayMs;
        movementState = MOVE_BACKWARD_STEP3_PLACE_LEG;
        break;
    case MOVE_BACKWARD_STEP3_PLACE_LEG:
        setServo(BackMap.legRB, robotLegPlaceDeg);
        movementTimer = now + robotDelayMs + 50;
        movementState = MOVE_BACKWARD_STEP4_LIFT_LEG;
        break;
    case MOVE_BACKWARD_STEP4_LIFT_LEG:
        setServo(BackMap.legRF, robotLegLiftDeg);
        movementTimer = now + robotDelayMs;
        movementState = MOVE_BACKWARD_STEP4_MOVE_HIP;
        break;
    case MOVE_BACKWARD_STEP4_MOVE_HIP:
        setServo(BackMap.hipRF, 170);
        movementTimer = now + robotDelayMs;
        movementState = MOVE_BACKWARD_STEP4_PLACE_LEG;
        break;
    case MOVE_BACKWARD_STEP4_PLACE_LEG:
        setServo(BackMap.legRF, robotLegPlaceDeg);
        movementTimer = now + robotDelayMs + 50;
        movementState = MOVE_BACKWARD_STEP5;
        break;
    case MOVE_BACKWARD_STEP5:
        swingHipsImmediate(BackMap.hipRF, 90, BackMap.hipRB, 45 - 15);
        movementTimer = now + robotDelayMs + 100;
        movementState = MOVE_BACKWARD_STEP6_LIFT_LEG;
        break;
    case MOVE_BACKWARD_STEP6_LIFT_LEG:
        setServo(BackMap.legLB, robotLegLiftDeg);
        movementTimer = now + robotDelayMs;
        movementState = MOVE_BACKWARD_STEP6_MOVE_HIP;
        break;
    case MOVE_BACKWARD_STEP6_MOVE_HIP:
        setServo(BackMap.hipLB, 90);
        movementTimer = now + robotDelayMs;
        movementState = MOVE_BACKWARD_STEP6_PLACE_LEG;
        break;
    case MOVE_BACKWARD_STEP6_PLACE_LEG:
        setServo(BackMap.legLB, robotLegPlaceDeg);
        movementTimer = now + robotDelayMs + 50;
        if (isContinuousMovement && currentCommand == 2)
        {
            movementState = MOVE_BACKWARD_STEP1_LIFT_LEG;
        }
        else
        {
            movementState = MOVE_IDLE;
        }
        break;

    case MOVE_LEFT_STEP1_LIFT_LEG:
        setServo(LeftMap.legLF, robotLegLiftDeg);
        movementTimer = now + robotDelayMs;
        movementState = MOVE_LEFT_STEP1_MOVE_HIP;
        break;
    case MOVE_LEFT_STEP1_MOVE_HIP:
        setServo(LeftMap.hipLF, 100);
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
        if (isContinuousMovement && currentCommand == 3)
        {
            movementState = MOVE_LEFT_STEP1_LIFT_LEG;
        }
        else
        {
            movementState = MOVE_IDLE;
        }
        break;

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
        if (isContinuousMovement && currentCommand == 4)
        {
            movementState = MOVE_RIGHT_STEP1_LIFT_LEG;
        }
        else
        {
            movementState = MOVE_IDLE;
        }
        break;
    case DANCE_STEP1:
        driveLeft();
        movementTimer = now + 150;
        movementState = DANCE_STEP2;
        break;
    case DANCE_STEP2:
        bodyLift();
        movementTimer = now + 150;
        movementState = DANCE_STEP3;
        break;
    case DANCE_STEP3:
        driveRight();
        movementTimer = now + 150;
        movementState = DANCE_STEP4;
        break;
    case DANCE_STEP4:
        bodyLift();
        movementTimer = now + 150;
        if (isDancing)
        {
            movementState = DANCE_STEP1;
        }
        else
        {
            standBy();
            movementState = MOVE_IDLE;
        }
        break;
    }
}

void startMoveForward()
{
    movementState = MOVE_FORWARD_STEP1_LIFT_LEG;
    movementTimer = millis();
}

void startMoveBackward()
{
    movementState = MOVE_BACKWARD_STEP1_LIFT_LEG;
    movementTimer = millis();
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

void stepLegImmediate(int leg, int hip, int hipAngle)
{
    setServo(leg, robotLegLiftDeg);
    setServo(hip, hipAngle);
    setServo(leg, robotLegPlaceDeg);
}

void swingHipsImmediate(int hip1, int ang1, int hip2, int ang2)
{
    setServo(hip1, ang1);
    setServo(hip2, ang2);
}

void bodyLift()
{
    setServo(LeftMap.hipRF, 90);
    setServo(LeftMap.hipLB, 90);
    setServo(LeftMap.hipRB, 90);
    setServo(LeftMap.hipLF, 90);

    setServo(LeftMap.legLF, robotLegPlaceDeg);
    setServo(LeftMap.legRF, robotLegPlaceDeg);
    setServo(LeftMap.legLB, robotLegPlaceDeg);
    setServo(LeftMap.legRB, robotLegPlaceDeg);
}

void driveLeft()
{
    drivePose(LeftMap);
}

void driveRight()
{
    drivePose(RightMap);
}

void drivePose(const ServoMap &map)
{
    setServo(map.hipRF, 90);
    setServo(map.hipLB, 90);
    setServo(map.hipRB, 90);
    setServo(map.hipLF, 90);

    setServo(map.legLF, 120);
    setServo(map.legRF, 120);
    setServo(map.legLB, robotLegPlaceDeg);
    setServo(map.legRB, robotLegPlaceDeg);
}