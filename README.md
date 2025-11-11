# 🕷️ Spider Root Stair Climbing

This project features the core Arduino code for four-legged robot designed to autonomously navigate a flat floor and detect and climb stairs using an ultrasonic sensor.

---

## ⚙️ Hardware & Wiring Setup 

This code is written for an Arduino board controlling **8 standard hobby servos** (2 per leg) and one **HC-SR04 Ultrasonic Sensor**.

**Note:** The program is currently in the simulation phase. Pin configurations are subject to change as testing continues.

### **Servo Wiring** (Pins 2 - 9)

| Leg | Joint | Arduino Pin | Note |
| :---: | :---: | :---: | :---: |
| **LF** (Left-Front) | Hip | **2** | 8 Servos require an external power supply |
| | Knee | **3** | |
| **RF** (Right-Front) | Hip | **4** | |
| | Knee | **5** | |
| **LB** (Left-Back) | Hip | **6** | |
| | Knee | **7** | |
| **RB** (Right-Back) | Hip | **8** | |
| | Knee | **9** | |

### **Ultrasonic Sensor Wiring** (Pins 10 - 11)

| Sensor Pin | Arduino Pin |
| :---: | :---: |
| **TRIG** | **10** |
| **ECHO** | **11** |
| **VCC** | **5V** |
| **GND** | **GND** |

---

## 🧠 Robot Logic (The `loop()` Function)

The robot acts as a state machine. It constantly checks the distance to the obstacle in front of it and chooses one of three modes:

| Distance Check | Mode Triggered | Action |
| :---: | :---: | :---: |
| **`distance` > 10 cm** (`CLIMBING_RANGE_CM`) | **Normal Walk** | Executes the `walkForward()` **Trot Gait**. |
| **5 cm < `distance` ≤ 10 cm** | **Stair Climb** | Calls `standBy()`, then executes the `climbStair()` **Crawl Gait** to step over the obstacle. |
| **`distance` ≤ 5 cm** (`TOO_CLOSE_CM`) | **Emergency Retreat** | Calls `standBy()`, then executes `walkBackward()` to safely back up. |

---

## 🚶‍♂️ Gait Functions

The code includes three distinct movement patterns:

### 1. `walkForward()` (Trot Gait)
This function uses a **fast, diagonal trot**. It moves the Left-Front and Right-Back legs simultaneously, followed by the Right-Front and Left-Back legs.

### 2. `walkBackward()` (Reverse Trot Gait)
The reverse of the `walkForward()` gait, used for quickly moving away from obstacles that are too close.

### 3. `climbStair()` (Crawl Gait)
This is a **slow, deliberate crawl** used for maximizing stability during the climb. It moves one leg at a time (RF → LF → RB → LB) and includes a strong body **LUNGE** phase to pull the rear legs onto the step.

| Angle Define | Use | Default Value | Note |
| :---: | :---: | :---: | :---: |
| `KNEE_CLIMB_UP` | Lift foot over step | **160°** | **Crucial for stair climbing.** Must be tuned to lift the foot above the stair height (e.g., 30mm). |
| `HIP_FORWARD` | Hip rotation for swing | **120°** | |
| `HIP_BACK` | Hip rotation for push | **60°** | |

---

## 🛠️ Calibration Notes

* **Angle Values:** The defined angles (`HIP_FORWARD`, `KNEE_UP`, etc.) are starting points. You must calibrate these values to ensure your servos are centered correctly (90°) and that the hip and knee angles provide the desired range of motion for walking and clearing the step.
* **Stair Height:** Adjust the `KNEE_CLIMB_UP` value (`160`) to reliably clear your target stair riser (e.g., if the stair is 3cm high, this angle must lift the foot > 3cm).
* **Distance Thresholds:**
    * `CLIMBING_RANGE_CM` (10 cm): How far away the robot should stop and prepare to climb.
    * `TOO_CLOSE_CM` (5 cm): The minimum safe distance before backing up.

---

## ▶️ How to Use

1. Connect your Arduino board and servos according to the wiring tables above.
2. Open the `main.ino` file in **Arduino IDE**.
3. Select the correct **board** and **port**.
4. Upload the sketch.
5. Place your robot in front of an obstacle and observe its automatic walking and climbing behavior!

---

## 📘 License
This project is open-source for educational and research purposes.  
Feel free to modify, improve, and share with proper credit.

