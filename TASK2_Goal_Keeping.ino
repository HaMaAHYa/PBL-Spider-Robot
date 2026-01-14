/*

  SPIDER ROBOT - SERVO & ULTRASONIC SENSOR STARTER CODE

  This code provides basic setup and control for:
  - Adafruit PWM Servo Driver (16 servo channels)
  - Ultrasonic sensor (HC-SR04) attached to servo for scanning

  Servo Channel Map:
            90        90
            |         |
    180_____|____ ____|_____0
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

// --- Radar Scanner Variables ---
const int scanAngles[] = {40, 60, 80, 100, 130, 100, 80, 60};
const unsigned long scanInterval = 75; // ms between steps
int scanIndex = 0;
unsigned long lastScanTime = 0;
float distances[5];
bool scanning = true;
bool foundTarget = false;
int foundAngle = -1;
float foundDist = 9999;
unsigned long lastActionTime = 0;
bool waitingAfterMove = false;

int robotLegPlaceDeg = 180;

int moveRightDelay = 90;
int moveLeftDelay = 100;

void moveLeft()
{
    Serial.println("Move Left");
    // TODO: Add your robot's left movement code here

    for (int i = 0; i < 10; i++)
    {
        skippingLeft();
    }

    drivePose(LeftMap);
    delay(2000);
    standBy();
    delay(2000);

    for (int i = 0; i < 10; i++)
    {
        skippingRight();
    }

    standBy();
    delay(2000);
}

void moveRight()
{
    Serial.println("Move Right");

    for (int i = 0; i < 10; i++)
    {
        skippingRight();
    }

    drivePose(RightMap);
    delay(2000);
    standBy();
    delay(2000);

    for (int i = 0; i < 10; i++)
    {
        skippingLeft();
    }

    standBy();
    delay(2000);
    // TODO: Add your robot's right movement code here
}

void standStill()
{
    Serial.println("Stand Still");
    // TODO: Add your robot's stand still code here
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

void skippingLeft()
{
    setServo(LeftMap.legLF, robotLegPlaceDeg - 35);
    setServo(LeftMap.legRF, robotLegPlaceDeg - 35);
    delay(moveLeftDelay);
    setServo(LeftMap.legLF, robotLegPlaceDeg);
    setServo(LeftMap.legRF, robotLegPlaceDeg);
    delay(moveLeftDelay);
}

void skippingRight()
{
    setServo(RightMap.legLF, robotLegPlaceDeg - 40);
    setServo(RightMap.legRF, robotLegPlaceDeg - 40);
    delay(moveRightDelay);
    setServo(RightMap.legLF, robotLegPlaceDeg);
    setServo(RightMap.legRF, robotLegPlaceDeg);
    delay(moveRightDelay);
}

void standBy()
{
    setServo(LeftMap.hipRF, 80);
    setServo(LeftMap.hipLB, 95);
    setServo(LeftMap.hipRB, 85);
    setServo(LeftMap.hipLF, 80);

    setServo(LeftMap.legLF, robotLegPlaceDeg);
    setServo(LeftMap.legRF, robotLegPlaceDeg);
    setServo(LeftMap.legLB, robotLegPlaceDeg);
    setServo(LeftMap.legRB, robotLegPlaceDeg);

    Serial.println("Robot: Stand By");
}

void handleRadarDecision(float dist, int angle)
{
    if (dist > 10 && dist < 50)
    {
        // For scanAngles[5] = {40, 60, 80, 100, 120}
        if (angle <= 60)
        {
            moveLeft();
        }
        else if (angle >= 100)
        {
            moveRight();
        }
        else
        {
            standStill();
        }
    }
    else
    {
        standStill();
    }
}

void radarScanUpdate()
{
    unsigned long now = millis();
    if (waitingAfterMove)
    {
        if (now - lastActionTime >= 5000)
        {
            waitingAfterMove = false;
            scanning = true;
            foundTarget = false;
            scanIndex = 0;
        }
        return;
    }
    if (!scanning)
        return;
    if (scanning && now - lastScanTime >= scanInterval)
    {
        lastScanTime = now;
        int angle = scanAngles[scanIndex];
        setSensorServo(angle);
        delay(20); // allow servo to reach position
        long dist = readUltrasonic();
        Serial.print("Scan ");
        Serial.print(scanIndex);
        Serial.print(": angle ");
        Serial.print(angle);
        Serial.print(", dist ");
        Serial.println(dist);
        if (dist > 5 && dist < 50 && !foundTarget)
        {
            foundTarget = true;
            foundAngle = angle;
            foundDist = dist;
        }
        scanIndex++;
        if (scanIndex >= (int)(sizeof(scanAngles) / sizeof(scanAngles[0])))
        {
            if (foundTarget)
            {
                Serial.print("Target found: ");
                Serial.print(foundDist);
                Serial.print(" cm at ");
                Serial.print(foundAngle);
                Serial.println(" deg");
                handleRadarDecision(foundDist, foundAngle);
                scanning = false;
                waitingAfterMove = true;
                lastActionTime = millis();
            }
            else
            {
                // No valid target, restart scan
                scanIndex = 0;
            }
        }
    }
}

// --- Main Functions --- //

void setup()
{
    Serial.begin(9600);
    pca.begin();
    pca.setPWMFreq(60);
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);

    standBy();
    setSensorServo(90);

    delay(4000);
}

void loop()
{
    radarScanUpdate(); // Non-blocking scan
}

// -------------------- //

// --- Helper Functions --- //
int angleToPulse(int angle)
{
    // Map 0-180 deg to SERVOMIN-SERVOMAX
    return map(angle, 0, 180, SERVOMIN, SERVOMAX);
}

void setSensorServo(int angle)
{
    pca.setPWM(SENSOR_SERVO, 0, angleToPulse(angle));
}

void setServo(int channel, int angle)
{
    pca.setPWM(channel, 0, angleToPulse(angle));
}

long readUltrasonic()
{
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    long duration = pulseIn(ECHO_PIN, HIGH, 20000); // 20ms timeout
    long distance = duration * 0.034 / 2;           // cm
    return distance;
}

// ----------------------- //
