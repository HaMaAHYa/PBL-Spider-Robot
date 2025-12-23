# 🕷️ Spider Robot – Wave Gait Walking (PCA9685)

This project contains the core **Arduino code for a four-legged spider robot** using a **PCA9685 16-channel servo driver**.
The robot walks forward using a **stable wave gait**, lifting and moving **one leg at a time** for maximum balance.

---

## ⚙️ Hardware & Wiring Setup

This code is designed for an Arduino-compatible board controlling **8 servo motors** via **I2C** using the **Adafruit PCA9685** PWM driver.

### **Main Components**

* Arduino (UNO / Nano / Mega)
* PCA9685 16-Channel Servo Driver
* 8× Standard hobby servos (2 DOF per leg)
* External 5–6V servo power supply (**required**)

---

### 🔌 PCA9685 Connections

| PCA9685 Pin | Arduino Pin |
| ----------- | ----------- |
| **VCC**     | 5V          |
| **GND**     | GND         |
| **SDA**     | SDA         |
| **SCL**     | SCL         |

⚠️ **Important:**
Do **NOT** power servos from the Arduino 5V pin. Use an external power supply connected to **V+** on the PCA9685.

---

### 🦵 Servo Channel Mapping

| Leg                  | Joint | PCA9685 Channel |
| -------------------- | ----- | --------------- |
| **LF** (Left-Front)  | Hip   | `0`             |
|                      | Knee  | `1`             |
| **RF** (Right-Front) | Hip   | `2`             |
|                      | Knee  | `3`             |
| **LB** (Left-Back)   | Hip   | `4`             |
|                      | Knee  | `5`             |
| **RB** (Right-Back)  | Hip   | `6`             |
|                      | Knee  | `7`             |

---

## 🧠 Robot Logic Overview

The robot continuously performs a **wave gait walking sequence** inside the `loop()` function.

### **Key Characteristics**

* One leg moves at a time
* High stability (always 3 legs on ground)
* Suitable for slow walking and uneven terrain
* No sensors required (open-loop motion)

---

## 🚶 Wave Gait Walking Sequence (`loop()`)

The walking cycle consists of **6 ordered steps**:

### **Step 1 – Move Left-Front (LF)**

1. Lift LF knee
2. Swing LF hip forward
3. Place LF foot down

---

### **Step 2 – Body Shift**

* Swing LF hip back to center
* Swing LB hip forward slightly to shift body weight

---

### **Step 3 – Move Right-Back (RB)**

1. Lift RB knee
2. Move RB hip forward
3. Place RB foot down

---

### **Step 4 – Move Right-Front (RF)**

1. Lift RF knee
2. Swing RF hip forward
3. Place RF foot down

---

### **Step 5 – Body Shift**

* Swing RF hip back to center
* Swing RB hip forward slightly

---

### **Step 6 – Move Left-Back (LB)**

1. Lift LB knee
2. Move LB hip forward
3. Place LB foot down

➡️ The cycle then repeats continuously.

---

## 🧍 Standby Position (`standBy()`)

Before walking begins, the robot moves into a **neutral standing pose**:

* All knees go to **DOWN position**
* Hips are angled to form a stable rectangle
* Ensures correct balance before gait execution

This function is automatically called in `setup()`.

---

## 🧩 Important Parameters

### **Servo Pulse Range**

```cpp
#define SERVOMIN 150
#define SERVOMAX 650
```

These values map **0–180°** to your specific servos.
⚠️ Must be calibrated for your hardware.

---

### **Motion Timing**

| Variable | Purpose                 | Default  |
| -------- | ----------------------- | -------- |
| `dt`     | Delay between movements | `100 ms` |
| `lift`   | Knee lift angle         | `100°`   |
| `down`   | Knee down angle         | `180°`   |

---

### **Hip Angles**

| Variable        | Description            |
| --------------- | ---------------------- |
| `RF_LB_standby` | RF & LB default stance |
| `LF_RB_standby` | LF & RB default stance |
| `*_center`      | Neutral hip position   |

These values control **step length**, **body shift**, and **stability**.

---

## 🛠️ Calibration Notes

* **Center your servos** mechanically at 90° before assembly
* Adjust `SERVOMIN` / `SERVOMAX` to avoid servo buzzing
* Tune `lift` angle to ensure foot clears the ground
* Increase `dt` for slower, smoother motion
* Decrease `dt` for faster walking (less stable)

---

## ▶️ How to Use

1. Wire Arduino ↔ PCA9685 via I2C
2. Connect servos according to the channel table
3. Provide external power to servos
4. Install **Adafruit PWM Servo Driver Library**
5. Upload the sketch
6. Place robot on flat ground and power on

The robot will:
➡️ Stand up
➡️ Begin continuous wave-gait walking

---

## 📘 License

This project is open-source for **education and robotics research**.
You are free to modify, optimize, and expand this code with proper credit.

---

If you want, I can also:

* Convert this gait to **trot / ripple**
* Add **inverse kinematics**
* Add **speed control**
* Add **stair climbing**
* Draw a **leg timing diagram**

Just tell me 👍
