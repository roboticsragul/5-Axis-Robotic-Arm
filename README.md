# 5DOF Adaptive Robotic Arm

## Overview

The **5DOF Adaptive Robotic Arm** is a cost-effective, high-precision robotic manipulator developed using Arduino Mega, capable of performing tasks in both manual and automated modes. It uses adaptive gripping technology powered by pressure sensing and high-torque servo motors for precise movement and control.

This project is designed for educational and industrial automation use cases, enabling learners and developers to understand the real-time implementation of motion control, sensor feedback integration, and robotic automation.

---

## Features

- **5 Degrees of Freedom**: Allows versatile and accurate arm positioning.
- **Manual and Auto Modes**: Operated via switch and push-button interface.
- **Adaptive Gripping**: Utilizes a force sensor to adjust grip pressure.
- **OLED Display**: Real-time display of mode, servo positions, and pressure values.
- **High-Torque Servo Motors**: Smooth and powerful movements.
- **Position Recording**: Manual movements can be recorded and replayed automatically.

---

## Components Used

### Electronics:
- **Arduino Mega** – Main controller
- **DS51150 Servo Motors (150kg, 12V)** – 3 Nos
- **DS3218 Servo Motors (20kg, 6V)** – 2 Nos
- **1.3" OLED Display (128x64 SPI)** – 1 No
- **Force Sensor (15mm)** – 1 No
- **10K Potentiometers** – 5 Nos
- **Push Buttons** – 6 Nos
- **Mode Switch (On/Off)** – 1 No
- **12V and 5V Regulated Power Supply**

### Mechanical:
- **Aluminum Arms & Gripper**
- **Mild Steel (MS) Base** for stability

---

## Modes of Operation

### Manual Mode:
- Each joint is controlled via individual potentiometers.
- Allows for freehand positioning and operation.

### Auto Mode:
- Records multiple joint positions using button presses.
- Repeats the movement loop automatically.
- Adaptive gripper control based on object pressure.

---

## Getting Started

### Prerequisites:
- Arduino IDE
- Required Libraries:
  - `Servo.h`
  - `Adafruit_GFX`
  - `Adafruit_SSD1306`

### Uploading the Code:
1. Connect the Arduino Mega to your PC.
2. Open the Arduino sketch.
3. Select the correct board and port.
4. Upload the code.
