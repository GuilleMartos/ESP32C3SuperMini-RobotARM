# ESP32-C3 SuperMini Robotic Arm

![License: CC BY 4.0](https://img.shields.io/badge/License-CC_BY_4.0-lightgrey.svg)
![Platform](https://img.shields.io/badge/Platform-ESP32--C3-blue)
![Connectivity](https://img.shields.io/badge/Connectivity-BLE_4.2-1182c3)

A professional-grade firmware for controlling a 4-Degrees-of-Freedom (4-DOF) robotic arm using an ESP32-C3 SuperMini and an Adafruit PCA9685 PWM Servo Driver. 

This project solves common physical constraints in hobbyist robotics—such as continuous servo drift and mechanical inertia—by implementing **Quadratic Kinematic Compensation** and **Proportional Multi-Axis Interpolation**. It is fully controllable via Bluetooth Low Energy (BLE) and features a dual-bank memory system to record, store, and seamlessly play back complex movement sequences.

## 🦾 Mechanical Design
The 3D-printed mechanical structure for this robotic arm was designed by **luca_dilo**. 
You can find the original 3D models and printing instructions on their MakerWorld profile:
👉 **[luca_dilo on MakerWorld](https://makerworld.com/es/@luca_dilo)**

## ✨ Key Features
* **Quadratic Kinematic Base Compensation:** Uses a non-linear $A \cdot x^2 + B \cdot x + C$ mathematical curve to calculate exact travel times for continuous rotation servos, effectively eliminating overshoot caused by mechanical inertia and cable tension.
* **Dual-Bank Memory System:** Record, clear, and playback up to 50 distinct waypoints per bank. 
* **Smart Home Sequence:** Safely closes the gripper to secure the payload before proportionately centering all axes back to the 0º origin.
* **Smooth Interpolation:** Multi-axis synchronized movements prevent jerky transitions.
* **Wireless BLE Control:** Acts as a BLE Server (`ESP32C3SuperMini-RobotARM`), allowing integration with standard mobile terminals.

## 🛠 Hardware Requirements
1. **Microcontroller:** ESP32-C3 SuperMini.
2. **PWM Driver:** Adafruit PCA9685 16-Channel Servo Driver.
3. **Motors:**
   * 1x Continuous Rotation Servo (Base).
   * 3x Standard 180º Positional Servos (Shoulder, Elbow, Gripper).
4. **Power Supply:** An independent 5V/6V power supply capable of handling peak stall currents (DO NOT power the servos directly from the ESP32 pins).

## 🔌 Pinout & Wiring

| ESP32-C3 Pin | PCA9685 Module |
| :--- | :--- |
| GPIO 6 | SDA |
| GPIO 7 | SCL |
| 3.3V / 5V | VCC (Logic Power) |
| GND | GND |

| PCA9685 Channel | Component | Type |
| :--- | :--- | :--- |
| Channel 0 | Base Motor | Continuous Rotation |
| Channel 1 | Shoulder | 180º Positional |
| Channel 2 | Elbow | 180º Positional |
| Channel 3 | Gripper | 180º Positional |

## 📦 Software Dependencies
Ensure you have the following libraries installed in your Arduino IDE:
* `Wire.h` (Built-in I2C)
* `Adafruit_PWMServoDriver.h` (By Adafruit)
* Built-in ESP32 BLE Libraries (`BLEDevice.h`, `BLEServer.h`, `BLEUtils.h`, `BLE2902.h`)

## 📱 Teach Pendant (Mobile Control)
To control the arm wirelessly, you can use the **[Serial Bluetooth Terminal](https://play.google.com/store/apps/details?id=de.kai_morich.serial_bluetooth_terminal)** app (available on Android). 

Connect to the BLE device named `ESP32C3SuperMini-RobotARM`. You can configure the app's bottom macro buttons to create a fully functional, touchscreen "Teach Pendant" by mapping the custom characters below:

### Main Commands
* `h` / `H` : **Smart Home** (Secures payload, returns to origin).

### Memory Bank 1
* `r` / `R` : **Record** current position to Bank 1.
* `m` / `M` : **Play** Bank 1 sequence.
* `c` / `C` : **Clear** Bank 1 memory.

### Memory Bank 2
* `v` / `V` : **Record** current position to Bank 2.
* `b` / `B` : **Play** Bank 2 sequence.
* `n` / `N` : **Clear** Bank 2 memory.

### Manual Overrides (Step-by-step)
* **Base:** `q` (Left -5º) | `e` (Right +5º)
* **Shoulder:** `w` (Up -3º) | `s` (Down +3º)
* **Elbow:** `d` (Up -3º) | `a` (Down +3º)
* **Gripper:** `o` (Close to 40º) | `p` (Open to 130º)

## 🧮 Tuning the Kinematics
Every physical robot has different tolerances, weight distribution, and cable tension. To ensure perfect base rotation, you can tune the Quadratic Curve variables in the code:

```cpp
// Right Turn Coefficients
float A_DER = -0.02;  // Inertia correction for long travels (reduces time)
float B_DER = 21.0;   // Linear velocity (ms/degree)
float C_DER = 40.0;   // Static friction threshold (base ms delay)
