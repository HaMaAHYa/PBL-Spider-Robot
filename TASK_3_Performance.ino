/*

  SPIDER ROBOT - SERVO & ULTRASONIC SENSOR STARTER CODE

  This code provides basic setup and control for:
  - Adafruit PWM Servo Driver (16 servo channels)
  - Ultrasonic sensor (HC-SR04) attached to servo for scanning

  Servo Channel Map:
            90        90
            |     L   |
    180_____|_________|_____0
            |0       2|
            |         |--> F
      0_____|4_______6|_____180
            |         |
            |     R   |
            90        90

  Channel 0-7: 8 servo motors (legs and hips)
  Channel 8: Sensor servo (ultrasonic motor)
  sda 5 scl 4

*/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define SERVOMIN 150
#define SERVOMAX 650

#define TRIG_PIN 13
#define ECHO_PIN 12
#define SENSOR_SERVO 8

#define SERVO_FREQ 60

int robotLegLiftDeg = 100;
int robotLegPlaceDeg = 180;
int robotDelayMs = 75;

unsigned long randomTimer = 0;
unsigned long movementTimer = 0;

// Flag
bool isMoving = false;
bool isRotating = false;
bool isDancing = false;
bool isGreeting = false;
bool isScaring = false;

// Condition
int rotateCount = 0;
int rotateMax = 4;

int danceCount = 0;
int danceMax = 4;

int greetingCount = 0;
int greetingMax = 4;

int moveBackwardCount = 0;
int moveBackwardMax = 4;

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
enum MovementState
{
    MOVE_IDLE,

    /* MOVE FORWARD */
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

    /* MOVE BACKWARD */
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

    /* ROTATE */
    ROTATE_LEFT_STEP1_LIFT_LEG,
    ROTATE_LEFT_STEP1_MOVE_HIP,
    ROTATE_LEFT_STEP1_PLACE_LEG,
    ROTATE_LEFT_STEP1_SWING_HIPS,
    ROTATE_LEFT_STEP2_LIFT_LEG,
    ROTATE_LEFT_STEP2_MOVE_HIP,
    ROTATE_LEFT_STEP2_PLACE_LEG,
    ROTATE_LEFT_STEP2_SWING_HIPS,
    ROTATE_LEFT_STEP3_LIFT_LEG,
    ROTATE_LEFT_STEP3_MOVE_HIP,
    ROTATE_LEFT_STEP3_PLACE_LEG,
    ROTATE_LEFT_STEP3_SWING_HIPS,
    ROTATE_LEFT_STEP4_LIFT_LEG,
    ROTATE_LEFT_STEP4_MOVE_HIP,
    ROTATE_LEFT_STEP4_PLACE_LEG,
    ROTATE_LEFT_STEP4_SWING_HIPS,

    ROTATE_RIGHT_STEP1_LIFT_LEG,
    ROTATE_RIGHT_STEP1_MOVE_HIP,
    ROTATE_RIGHT_STEP1_PLACE_LEG,
    ROTATE_RIGHT_STEP1_SWING_HIPS,
    ROTATE_RIGHT_STEP2_LIFT_LEG,
    ROTATE_RIGHT_STEP2_MOVE_HIP,
    ROTATE_RIGHT_STEP2_PLACE_LEG,
    ROTATE_RIGHT_STEP2_SWING_HIPS,
    ROTATE_RIGHT_STEP3_LIFT_LEG,
    ROTATE_RIGHT_STEP3_MOVE_HIP,
    ROTATE_RIGHT_STEP3_PLACE_LEG,
    ROTATE_RIGHT_STEP3_SWING_HIPS,
    ROTATE_RIGHT_STEP4_LIFT_LEG,
    ROTATE_RIGHT_STEP4_MOVE_HIP,
    ROTATE_RIGHT_STEP4_PLACE_LEG,
    ROTATE_RIGHT_STEP4_SWING_HIPS,

    /* DANCE */
    DANCE_STEP1,
    DANCE_STEP2,
    DANCE_STEP3,
    DANCE_STEP4,

    /* GREET */
    GREET_STEP1,
    GREET_STEP2,
    GREET_STEP3,
};
MovementState movementState = MOVE_IDLE;

void processMovement(unsigned long now)
{
    if (movementState == MOVE_IDLE || now < movementTimer)
    {
        return;
    }
    switch (movementState)
    {

    /* MOVE FORWARD */
    case MOVE_FORWARD_STEP1_LIFT_LEG:
        setServo(FrontMap.legLF, robotLegLiftDeg);
        movementTimer = now + robotDelayMs;
        movementState = MOVE_FORWARD_STEP1_MOVE_HIP;
        Serial.print("Robot: move forward");
        break;
    case MOVE_FORWARD_STEP1_MOVE_HIP:
        setServo(FrontMap.hipLF, 10);
        movementTimer = now + robotDelayMs;
        movementState = MOVE_FORWARD_STEP1_PLACE_LEG;
        Serial.print(".");
        break;
    case MOVE_FORWARD_STEP1_PLACE_LEG:
        setServo(FrontMap.legLF, robotLegPlaceDeg);
        movementTimer = now + robotDelayMs + 50;
        movementState = MOVE_FORWARD_STEP2;
        Serial.print(".");
        break;
    case MOVE_FORWARD_STEP2:
        swingHipsImmediate(FrontMap.hipLF, 90, FrontMap.hipLB, 155);
        movementTimer = now + robotDelayMs + 100;
        movementState = MOVE_FORWARD_STEP3_LIFT_LEG;
        Serial.print(".");
        break;
    case MOVE_FORWARD_STEP3_LIFT_LEG:
        setServo(FrontMap.legRB, robotLegLiftDeg);
        movementTimer = now + robotDelayMs;
        movementState = MOVE_FORWARD_STEP3_MOVE_HIP;
        Serial.print(".");
        break;
    case MOVE_FORWARD_STEP3_MOVE_HIP:
        setServo(FrontMap.hipRB, 90);
        movementTimer = now + robotDelayMs;
        movementState = MOVE_FORWARD_STEP3_PLACE_LEG;
        Serial.print(".");
        break;
    case MOVE_FORWARD_STEP3_PLACE_LEG:
        setServo(FrontMap.legRB, robotLegPlaceDeg);
        movementTimer = now + robotDelayMs + 50;
        movementState = MOVE_FORWARD_STEP4_LIFT_LEG;
        Serial.print(".");
        break;
    case MOVE_FORWARD_STEP4_LIFT_LEG:
        setServo(FrontMap.legRF, robotLegLiftDeg);
        movementTimer = now + robotDelayMs;
        movementState = MOVE_FORWARD_STEP4_MOVE_HIP;
        Serial.print(".");
        break;
    case MOVE_FORWARD_STEP4_MOVE_HIP:
        setServo(FrontMap.hipRF, 170);
        movementTimer = now + robotDelayMs;
        movementState = MOVE_FORWARD_STEP4_PLACE_LEG;
        Serial.print(".");
        break;
    case MOVE_FORWARD_STEP4_PLACE_LEG:
        setServo(FrontMap.legRF, robotLegPlaceDeg);
        movementTimer = now + robotDelayMs + 50;
        movementState = MOVE_FORWARD_STEP5;
        Serial.print(".");
        break;
    case MOVE_FORWARD_STEP5:
        swingHipsImmediate(FrontMap.hipRF, 90, FrontMap.hipRB, 30);
        movementTimer = now + robotDelayMs + 100;
        movementState = MOVE_FORWARD_STEP6_LIFT_LEG;
        Serial.print(".");
        break;
    case MOVE_FORWARD_STEP6_LIFT_LEG:
        setServo(FrontMap.legLB, robotLegLiftDeg);
        movementTimer = now + robotDelayMs;
        movementState = MOVE_FORWARD_STEP6_MOVE_HIP;
        Serial.print(".");
        break;
    case MOVE_FORWARD_STEP6_MOVE_HIP:
        setServo(FrontMap.hipLB, 90);
        movementTimer = now + robotDelayMs;
        movementState = MOVE_FORWARD_STEP6_PLACE_LEG;
        Serial.print(".");
        break;
    case MOVE_FORWARD_STEP6_PLACE_LEG:
        setServo(FrontMap.legLB, robotLegPlaceDeg);
        movementTimer = now + robotDelayMs + 50;
        if (isMoving)
        {
            moveForward();
        }
        else
        {
            stop();
        }
        Serial.println(".");
        break;

    /* MOVE BACKWARD */
    case MOVE_BACKWARD_STEP1_LIFT_LEG:
        setServo(BackMap.legLF, robotLegLiftDeg);
        movementTimer = now + robotDelayMs;
        movementState = MOVE_BACKWARD_STEP1_MOVE_HIP;
        Serial.print("Robot: move backward");
        break;
    case MOVE_BACKWARD_STEP1_MOVE_HIP:
        setServo(BackMap.hipLF, 10);
        movementTimer = now + robotDelayMs;
        movementState = MOVE_BACKWARD_STEP1_PLACE_LEG;
        Serial.print(".");
        break;
    case MOVE_BACKWARD_STEP1_PLACE_LEG:
        setServo(BackMap.legLF, robotLegPlaceDeg);
        movementTimer = now + robotDelayMs + 50;
        movementState = MOVE_BACKWARD_STEP2;
        Serial.print(".");
        break;
    case MOVE_BACKWARD_STEP2:
        swingHipsImmediate(BackMap.hipLF, 90, BackMap.hipLB, 155);
        movementTimer = now + robotDelayMs + 100;
        movementState = MOVE_BACKWARD_STEP3_LIFT_LEG;
        Serial.print(".");
        break;
    case MOVE_BACKWARD_STEP3_LIFT_LEG:
        setServo(BackMap.legRB, robotLegLiftDeg);
        movementTimer = now + robotDelayMs;
        movementState = MOVE_BACKWARD_STEP3_MOVE_HIP;
        Serial.print(".");
        break;
    case MOVE_BACKWARD_STEP3_MOVE_HIP:
        setServo(BackMap.hipRB, 90);
        movementTimer = now + robotDelayMs;
        movementState = MOVE_BACKWARD_STEP3_PLACE_LEG;
        Serial.print(".");
        break;
    case MOVE_BACKWARD_STEP3_PLACE_LEG:
        setServo(BackMap.legRB, robotLegPlaceDeg);
        movementTimer = now + robotDelayMs + 50;
        movementState = MOVE_BACKWARD_STEP4_LIFT_LEG;
        Serial.print(".");
        break;
    case MOVE_BACKWARD_STEP4_LIFT_LEG:
        setServo(BackMap.legRF, robotLegLiftDeg);
        movementTimer = now + robotDelayMs;
        movementState = MOVE_BACKWARD_STEP4_MOVE_HIP;
        Serial.print(".");
        break;
    case MOVE_BACKWARD_STEP4_MOVE_HIP:
        setServo(BackMap.hipRF, 170);
        movementTimer = now + robotDelayMs;
        movementState = MOVE_BACKWARD_STEP4_PLACE_LEG;
        Serial.print(".");
        break;
    case MOVE_BACKWARD_STEP4_PLACE_LEG:
        setServo(BackMap.legRF, robotLegPlaceDeg);
        movementTimer = now + robotDelayMs + 50;
        movementState = MOVE_BACKWARD_STEP5;
        Serial.print(".");
        break;
    case MOVE_BACKWARD_STEP5:
        swingHipsImmediate(BackMap.hipRF, 90, BackMap.hipRB, 30);
        movementTimer = now + robotDelayMs + 100;
        movementState = MOVE_BACKWARD_STEP6_LIFT_LEG;
        Serial.print(".");
        break;
    case MOVE_BACKWARD_STEP6_LIFT_LEG:
        setServo(BackMap.legLB, robotLegLiftDeg);
        movementTimer = now + robotDelayMs;
        movementState = MOVE_BACKWARD_STEP6_MOVE_HIP;
        Serial.print(".");
        break;
    case MOVE_BACKWARD_STEP6_MOVE_HIP:
        setServo(BackMap.hipLB, 90);
        movementTimer = now + robotDelayMs;
        movementState = MOVE_BACKWARD_STEP6_PLACE_LEG;
        Serial.print(".");
        break;
    case MOVE_BACKWARD_STEP6_PLACE_LEG:
        setServo(BackMap.legLB, robotLegPlaceDeg);
        movementTimer = now + robotDelayMs + 50;
        if (isMoving)
        {
            moveBackwardCount++;
            if (moveBackwardCount >= moveBackwardMax)
            {
                moveBackwardCount = 0;
                Serial.println("Reached max backward moves");
                stop();
            }
            else
            {
                moveBackward();
            }
        }

        Serial.println(".");
        break;

    /* ROTATE LEFT*/
    case ROTATE_LEFT_STEP1_LIFT_LEG:
        setServo(FrontMap.legLF, robotLegLiftDeg);
        movementTimer = now + robotDelayMs;
        movementState = ROTATE_LEFT_STEP1_MOVE_HIP;
        Serial.print("Robot: Rotating");
        break;
    case ROTATE_LEFT_STEP1_MOVE_HIP:
        setServo(FrontMap.hipLF, 90);
        movementTimer = now + robotDelayMs;
        movementState = ROTATE_LEFT_STEP1_PLACE_LEG;
        Serial.print(".");
        break;
    case ROTATE_LEFT_STEP1_PLACE_LEG:
        setServo(FrontMap.legLF, robotLegPlaceDeg);
        movementTimer = now + robotDelayMs + 50;
        movementState = ROTATE_LEFT_STEP1_SWING_HIPS;
        Serial.print(".");
        break;
    case ROTATE_LEFT_STEP1_SWING_HIPS:
        setServo(FrontMap.hipLF, 45);
        movementTimer = now + robotDelayMs;
        movementState = ROTATE_LEFT_STEP2_LIFT_LEG;
        Serial.print(".");
        break;
    case ROTATE_LEFT_STEP2_LIFT_LEG:
        setServo(FrontMap.legRF, robotLegLiftDeg);
        movementTimer = now + robotDelayMs;
        movementState = ROTATE_LEFT_STEP2_MOVE_HIP;
        Serial.print(".");
        break;
    case ROTATE_LEFT_STEP2_MOVE_HIP:
        setServo(FrontMap.hipRF, 180);
        movementTimer = now + robotDelayMs + 50;
        movementState = ROTATE_LEFT_STEP2_PLACE_LEG;
        Serial.print(".");
        break;
    case ROTATE_LEFT_STEP2_PLACE_LEG:
        setServo(FrontMap.legRF, robotLegPlaceDeg);
        movementTimer = now + robotDelayMs + 50;
        movementState = ROTATE_LEFT_STEP2_SWING_HIPS;
        Serial.print(".");
        break;
    case ROTATE_LEFT_STEP2_SWING_HIPS:
        setServo(FrontMap.hipRF, 135);
        movementTimer = now + robotDelayMs + 100;
        movementState = ROTATE_LEFT_STEP3_LIFT_LEG;
        Serial.print(".");
        break;
    case ROTATE_LEFT_STEP3_LIFT_LEG:
        setServo(FrontMap.legRB, robotLegLiftDeg);
        movementTimer = now + robotDelayMs;
        movementState = ROTATE_LEFT_STEP3_MOVE_HIP;
        Serial.print(".");
        break;
    case ROTATE_LEFT_STEP3_MOVE_HIP:
        setServo(FrontMap.hipRB, 90);
        movementTimer = now + robotDelayMs;
        movementState = ROTATE_LEFT_STEP3_PLACE_LEG;
        Serial.print(".");
        break;
    case ROTATE_LEFT_STEP3_PLACE_LEG:
        setServo(FrontMap.legRB, robotLegPlaceDeg);
        movementTimer = now + robotDelayMs + 50;
        movementState = ROTATE_LEFT_STEP3_SWING_HIPS;
        Serial.print(".");
        break;
    case ROTATE_LEFT_STEP3_SWING_HIPS:
        setServo(FrontMap.hipRB, 45);
        movementTimer = now + robotDelayMs + 100;
        movementState = ROTATE_LEFT_STEP4_LIFT_LEG;
        Serial.print(".");
        break;
    case ROTATE_LEFT_STEP4_LIFT_LEG:
        setServo(FrontMap.legLB, robotLegLiftDeg);
        movementTimer = now + robotDelayMs;
        movementState = ROTATE_LEFT_STEP4_MOVE_HIP;
        Serial.print(".");
        break;
    case ROTATE_LEFT_STEP4_MOVE_HIP:
        setServo(FrontMap.hipLB, 180);
        movementTimer = now + robotDelayMs;
        movementState = ROTATE_LEFT_STEP4_PLACE_LEG;
        Serial.print(".");
        break;
    case ROTATE_LEFT_STEP4_PLACE_LEG:
        setServo(FrontMap.legLB, robotLegPlaceDeg);
        movementTimer = now + robotDelayMs + 50;
        movementState = ROTATE_LEFT_STEP4_SWING_HIPS;
        Serial.print(".");
        break;
    case ROTATE_LEFT_STEP4_SWING_HIPS:
        setServo(FrontMap.hipLB, 135);
        movementTimer = now + robotDelayMs + 100;
        if (isRotating)
        {

            rotateCount++;
            if (rotateCount >= rotateMax)
            {
                rotateCount = 0;
                stop();
            }
            else
            {
                rotateLeft();
            }
        }
        Serial.println(".");
        break;

    /* ROTATE RIGHT*/
    case ROTATE_RIGHT_STEP1_LIFT_LEG:
        setServo(FrontMap.legLF, robotLegLiftDeg);
        movementTimer = now + robotDelayMs;
        movementState = ROTATE_RIGHT_STEP1_MOVE_HIP;
        Serial.print("Robot: Rotating");
        break;
    case ROTATE_RIGHT_STEP1_MOVE_HIP:
        setServo(FrontMap.hipLF, 0);
        movementTimer = now + robotDelayMs;
        movementState = ROTATE_RIGHT_STEP1_PLACE_LEG;
        Serial.print(".");
        break;
    case ROTATE_RIGHT_STEP1_PLACE_LEG:
        setServo(FrontMap.legLF, robotLegPlaceDeg);
        movementTimer = now + robotDelayMs + 50;
        movementState = ROTATE_RIGHT_STEP1_SWING_HIPS;
        Serial.print(".");
        break;
    case ROTATE_RIGHT_STEP1_SWING_HIPS:
        setServo(FrontMap.hipLF, 45);
        movementTimer = now + robotDelayMs;
        movementState = ROTATE_RIGHT_STEP2_LIFT_LEG;
        Serial.print(".");
        break;
    case ROTATE_RIGHT_STEP2_LIFT_LEG:
        setServo(FrontMap.legRF, robotLegLiftDeg);
        movementTimer = now + robotDelayMs;
        movementState = ROTATE_RIGHT_STEP2_MOVE_HIP;
        Serial.print(".");
        break;
    case ROTATE_RIGHT_STEP2_MOVE_HIP:
        setServo(FrontMap.hipRF, 90);
        movementTimer = now + robotDelayMs + 50;
        movementState = ROTATE_RIGHT_STEP2_PLACE_LEG;
        Serial.print(".");
        break;
    case ROTATE_RIGHT_STEP2_PLACE_LEG:
        setServo(FrontMap.legRF, robotLegPlaceDeg);
        movementTimer = now + robotDelayMs + 50;
        movementState = ROTATE_RIGHT_STEP2_SWING_HIPS;
        Serial.print(".");
        break;
    case ROTATE_RIGHT_STEP2_SWING_HIPS:
        setServo(FrontMap.hipRF, 135);
        movementTimer = now + robotDelayMs + 100;
        movementState = ROTATE_RIGHT_STEP3_LIFT_LEG;
        Serial.print(".");
        break;
    case ROTATE_RIGHT_STEP3_LIFT_LEG:
        setServo(FrontMap.legRB, robotLegLiftDeg);
        movementTimer = now + robotDelayMs;
        movementState = ROTATE_RIGHT_STEP3_MOVE_HIP;
        Serial.print(".");
        break;
    case ROTATE_RIGHT_STEP3_MOVE_HIP:
        setServo(FrontMap.hipRB, 0);
        movementTimer = now + robotDelayMs;
        movementState = ROTATE_RIGHT_STEP3_PLACE_LEG;
        Serial.print(".");
        break;
    case ROTATE_RIGHT_STEP3_PLACE_LEG:
        setServo(FrontMap.legRB, robotLegPlaceDeg);
        movementTimer = now + robotDelayMs + 50;
        movementState = ROTATE_RIGHT_STEP3_SWING_HIPS;
        Serial.print(".");
        break;
    case ROTATE_RIGHT_STEP3_SWING_HIPS:
        setServo(FrontMap.hipRB, 45);
        movementTimer = now + robotDelayMs + 100;
        movementState = ROTATE_RIGHT_STEP4_LIFT_LEG;
        Serial.print(".");
        break;
    case ROTATE_RIGHT_STEP4_LIFT_LEG:
        setServo(FrontMap.legLB, robotLegLiftDeg);
        movementTimer = now + robotDelayMs;
        movementState = ROTATE_RIGHT_STEP4_MOVE_HIP;
        Serial.print(".");
        break;
    case ROTATE_RIGHT_STEP4_MOVE_HIP:
        setServo(FrontMap.hipLB, 90);
        movementTimer = now + robotDelayMs;
        movementState = ROTATE_RIGHT_STEP4_PLACE_LEG;
        Serial.print(".");
        break;
    case ROTATE_RIGHT_STEP4_PLACE_LEG:
        setServo(FrontMap.legLB, robotLegPlaceDeg);
        movementTimer = now + robotDelayMs + 50;
        movementState = ROTATE_RIGHT_STEP4_SWING_HIPS;
        Serial.print(".");
        break;
    case ROTATE_RIGHT_STEP4_SWING_HIPS:
        setServo(FrontMap.hipLB, 135);
        movementTimer = now + robotDelayMs + 100;
        if (isRotating)
        {

            rotateCount++;
            if (rotateCount >= rotateMax)
            {
                rotateCount = 0;
                stop();
            }
            else
            {
                rotateRight();
            }
        }
        Serial.println(".");
        break;

    /* DANCE */
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
            danceCount++;
            if (danceCount >= danceMax)
            {
                danceCount = 0;
                stop();
            }
            else
            {
                dance();
            }
        }
        break;

    /* GREET */
    case GREET_STEP1:
        setServo(FrontMap.legLB, robotLegPlaceDeg - 80); // LB leg down most
        setServo(FrontMap.legLF, robotLegPlaceDeg - 20); // LF leg down
        setServo(FrontMap.legRB, robotLegPlaceDeg - 20); // RB leg down
        setServo(FrontMap.hipRF, 150);                   // RF hip forward
        setServo(FrontMap.legRF, 30);                    // RF leg up
        setServo(SENSOR_SERVO, 120);
        movementTimer = now + 150;
        movementState = GREET_STEP2;
        break;
    case GREET_STEP2:
        setServo(FrontMap.hipRF, 135); // Wave RF leg to
        movementTimer = now + 75;
        movementState = GREET_STEP3;
        break;
    case GREET_STEP3:
        setServo(FrontMap.hipRF, 150); // Wave RF leg right
        movementTimer = now + 75;
        if (isGreeting && !isScaring)
        {
            greetingCount++;
            if (greetingCount >= greetingMax)
            {
                greetingCount = 0;
                setServo(SENSOR_SERVO, 90);
                stop();
            }
            else
            {
                greeting();
            }
        }
        break;
    }
}
Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver();

/* MAIN FUNCTION */

void setup()
{
    Serial.begin(9600);
    pca.begin();
    pca.reset();
    pca.setPWMFreq(SERVO_FREQ);
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    standBy();
    setServo(SENSOR_SERVO, 90);
    delay(2000);

    greeting();
    delay(2000);
    randomTimer = millis();
}

void loop()
{
    unsigned long now = millis();
    processMovement(now);

    int sensorDis = getDistance();
    if (sensorDis > 0 && sensorDis < 30 && !isScaring && !isGreeting)
    {
        // scare
        standBy();
        scare();
    }
    else if (sensorDis > 60 && sensorDis < 70 && !isGreeting && !isScaring)
    {
        standBy();
        greeting();
    }

    if (!isMoving && !isRotating && !isDancing && !isGreeting && !isScaring)
    {
        moveForward();
        randomTimer = now;
    }
    if (now - randomTimer >= 5000) // Every 10 sec
    {
        if (isMoving && !isScaring)
        {
            int randNum = random(1, 11);

            if (randNum >= 7)
            {
                standBy();
                dance();
                Serial.println("Randomly Dance");
                randomTimer = now;
            }
            else if (randNum >= 2)
            {
                Serial.println("Randomly Rotate");
                standBy();
                if (random(1, 3) > 1)
                {

                    rotateLeft();
                }
                else
                {
                    rotateRight();
                }
                randomTimer = now;
            }
        }
    }
}

/* ACTION FUNCTION*/

void standBy()
{
    int RF_standBy = 45;
    int LF_standBy = 135;
    int LB_standBy = 65;
    int RB_standBy = 120;

    setServo(LeftMap.hipRF, RF_standBy);
    setServo(LeftMap.hipLB, LB_standBy);
    setServo(LeftMap.hipRB, RB_standBy);
    setServo(LeftMap.hipLF, LF_standBy);

    setServo(LeftMap.legLF, robotLegPlaceDeg);
    setServo(LeftMap.legRF, robotLegPlaceDeg);
    setServo(LeftMap.legLB, robotLegPlaceDeg);
    setServo(LeftMap.legRB, robotLegPlaceDeg);

    Serial.println("Robot: Stand By");
}

void stop()
{
    standBy();
    isMoving = false;
    isRotating = false;
    isDancing = false;
    isGreeting = false;
    isScaring = false;
    movementState = MOVE_IDLE;
    movementTimer = millis();
}

void moveForward()
{
    isMoving = true;
    isRotating = false;
    isDancing = false;
    isGreeting = false;
    movementState = MOVE_FORWARD_STEP1_LIFT_LEG;
    movementTimer = millis();
}

void scare()
{
    Serial.println("Robot: Scare!");
    isMoving = false;
    isScaring = true;
    setServo(FrontMap.legLF, 100);
    setServo(FrontMap.legRF, 100);

    setServo(FrontMap.hipLF, 20);
    setServo(FrontMap.hipRF, 160);
    setServo(FrontMap.hipLB, 150);
    setServo(FrontMap.hipRB, 30);

    for (int i = 0; i < 3; i++)
    {
        delay(100);
        setServo(FrontMap.hipLF, 40);
        setServo(FrontMap.hipRF, 180);
        setServo(FrontMap.hipLB, 170);
        setServo(FrontMap.hipRB, 50);
        delay(100);
        setServo(FrontMap.hipLF, 20);
        setServo(FrontMap.hipRF, 160);
        setServo(FrontMap.hipLB, 150);
        setServo(FrontMap.hipRB, 30);
    }

    standBy();
    moveBackward();
}

void moveBackward()
{
    isMoving = true;
    isRotating = false;
    isDancing = false;
    isGreeting = false;
    isScaring = true;
    movementState = MOVE_BACKWARD_STEP1_LIFT_LEG;
    movementTimer = millis();
}

void rotateLeft()
{
    isMoving = false;
    isRotating = true;
    isDancing = false;
    isGreeting = false;
    movementState = ROTATE_LEFT_STEP1_LIFT_LEG;
    movementTimer = millis();
}

void rotateRight()
{
    isMoving = false;
    isRotating = true;
    isDancing = false;
    isGreeting = false;
    movementState = ROTATE_RIGHT_STEP1_LIFT_LEG;
    movementTimer = millis();
}

void dance()
{
    isMoving = false;
    isRotating = false;
    isDancing = true;
    isGreeting = false;
    movementState = DANCE_STEP1;
    movementTimer = millis();
}

void greeting()
{
    isMoving = false;
    isRotating = false;
    isDancing = false;
    isGreeting = true;
    movementState = GREET_STEP1;
    movementTimer = millis();
}

/* HELPER FUNCTION */

void setServo(int channel, int angle)
{
    pca.setPWM(channel, 0, map(angle, 0, 180, SERVOMIN, SERVOMAX));
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

    setServo(map.legLF, 150);
    setServo(map.legRF, 150);
    setServo(map.legLB, robotLegPlaceDeg);
    setServo(map.legRB, robotLegPlaceDeg);
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
