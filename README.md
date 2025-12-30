# ESP-32 Gesture Control System

A dual ESP32 wireless gesture recognition system using an MPU6050 and ESP-NOW. One ESP32 detects hand gestures and transmits direction and magnitude data, while the second ESP32 provides real-time feedback via LEDs, an 8×8 LED matrix, an LCD, and a servo motor without requiring WiFi or internet access.

---

## Project Overview

This project implements a low-latency, peer-to-peer gesture interface using two ESP32 microcontrollers.

- ESP32 A reads motion data from an MPU6050 over I²C  
- Gestures are detected based on dominant acceleration direction and magnitude  
- Data is sent wirelessly using ESP-NOW  
- ESP32 B receives the data and produces visual and mechanical feedback  

The system is designed to be reliable, responsive, and easy to extend.

---

## System Architecture

### ESP32 A – Gesture Transmitter
- Reads accelerometer X and Y axes
- Detects left, right, forward, and backward gestures
- Computes gesture strength (magnitude)
- Applies thresholding, cooldown timing, and magnitude bucketing
- Sends compact binary packets via ESP-NOW

### ESP32 B – Receiver and Output Controller
- Receives gesture packets wirelessly
- Displays direction text on an I²C LCD
- Shows arrow icons on an 8×8 LED matrix
- Lights LEDs based on gesture strength and direction
- Controls a servo motor for forward and backward gestures

---

## Hardware Used

- ESP32 Dev Module ×2  
- MPU6050 IMU  
- MAX7219 8×8 LED matrix  
- I²C LCD (16×2)  
- Servo motor  
- 7 LEDs with resistors  
- Jumper wires and breadboard  

---

## Gesture Set

The following gestures are supported:

- **Left**
- **Right**
- **Forward**
- **Backward**

Gesture direction is determined by the dominant accelerometer axis. Gesture strength is calculated from the acceleration magnitude and is used to scale LED output and servo motion.

---

## Communication

- ESP-NOW peer-to-peer communication
- No router or internet required
- Fixed-size binary packets for low latency
- Explicit peer MAC addressing

---

## Pin Configuration

### ESP32 A (Transmitter)
- I²C SDA: GPIO 21  
- I²C SCL: GPIO 22  
- MPU6050 connected via I²C  

### ESP32 B (Receiver)
- LEDs: GPIO 12, 14, 25, 26, 27, 32, 33  
- Servo: GPIO 4  
- Matrix DIN: GPIO 23  
- Matrix CLK: GPIO 19  
- Matrix CS: GPIO 18  
- I²C SDA: GPIO 21  
- I²C SCL: GPIO 22  

---

## How to Run

1. Flash the transmitter code to ESP32 A  
2. Flash the receiver code to ESP32 B  
3. Update the receiver ESP32 MAC address in the transmitter code  
4. Power both boards  
5. Perform hand gestures near the MPU6050 to trigger output feedback  

---

## Repository Structure

- `transmitter/` – ESP32 code for gesture detection and ESP-NOW transmission  
- `receiver/` – ESP32 code for gesture reception and output feedback  
- `tools/` – Utility sketches for calibration and MAC discovery  
- `hardware/` – Wiring diagrams and hardware reference files  

---

## Design Notes

- Gestures are tilt-based rather than dynamic swipe-based
- Thresholding and cooldown reduce wireless noise
- Fixed-width data types ensure consistent packet size
- ESP-NOW enables fast, reliable communication

---

## Possible Improvements

- Add gyroscope-based gesture detection
- Implement gesture confidence scoring
- Expand the gesture set
- Add wireless pairing or configuration

---

## Wiring Diagram Notes

The included Fritzing diagram is provided for illustrative purposes only.  
A different ESP32 development board model was used in the diagram, but the pin functions and connections match those used in this project.

GPIO numbers, power connections, and peripheral interfaces (I²C, SPI, PWM) should be followed as defined in the source code rather than inferred solely from the diagram.

---

## License

This project is intended for educational and experimental use.
