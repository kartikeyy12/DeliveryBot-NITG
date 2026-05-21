# 🤖 DeliveryBot NITG

*Autonomous In-Campus Robo-Delivery System using ESP32 and Standalone Web Interface*

![Project Status](https://img.shields.io/badge/Status-Completed-success)
![Platform](https://img.shields.io/badge/Platform-ESP32-blue)
![Language](https://img.shields.io/badge/Language-C++-orange)

## 📌 Overview
This project proposes an autonomous robotic delivery system designed specifically for the National Institute of Technology Goa (NITG) campus. 
To address the logistical delays of traditional delivery methods, we developed a cost-effective, self-navigating rover powered by an ESP32 microcontroller. 
The system operates entirely offline using a standalone network architecture, serving a mobile-optimized web application directly from the ESP32's flash memory.

**Full Demo Video:** [Google Drive Link](https://drive.google.com/file/d/1zHF1fIpplKEKeNxKaXLTZ_2-ds54X5HS/view?usp=sharing)
**Detailed Project Report:** [Read PDF](docs/EC253_Project_Report.pdf)

## ✨ Key Engineering Features
* **Standalone Network Architecture:** The ESP32 operates in Access Point (AP) mode, hosting a local web server that allows users to dispatch the robot without an active internet connection.
* **Non-Blocking State Machine:** Implemented a non-blocking C++ architecture that eliminates processing delays, ensuring the robot can simultaneously serve the web UI, read GPS bytes, and poll ultrasonic echoes without freezing.
* **Custom Communication Protocol:** The system's primary communication step utilizes GFSK modulation and demodulation for robust data transfer.
* **Dynamic Obstacle Detection:** Deployed a sequential ultrasonic array that performs dynamic obstacle detection while actively preventing sensor crosstalk.
* **Waypoint Queuing:** Overcomes the limitations of straight-line GPS navigation on curved campus pathways by utilizing a breadcrumb trail of pre-surveyed coordinates and magnetic heading correction.

## 🛠️ Hardware Architecture

| Component | Role |
| :--- | :--- |
| **ESP32 WROOM-32** | Central dual-core processing unit managing the HTTP server, state machine logic, and hardware abstraction. |
| **NEO-6M GPS** | Interfaces via UART to provide 1Hz geographical coordinate fixes for macro-navigation. |
| **QMC5883L Magnetometer**| Interfaces via I2C to provide continuous 8-point cardinal direction output for real-time heading correction. |
| **3x HC-SR04** | Ultrasonic sensors positioned at the Left, Center, and Right for forward-facing spatial sweep and obstacle detection. |
| **L298N Motor Driver** | Governs the 2WD differential chassis via PWM signals and utilizes an onboard 5V regulator to power the ESP32 logic. |

## ⚙️ Operational Workflow (Mission Phases)
1.  **Initialization & UI Serving:** ESP32 broadcasts its AP network and serves the HTML interface (via Base64 encoding) to the user's smartphone.
2.  **Waypoint Queuing:** Upon destination selection, the system loads an array of intermediate GPS coordinates specific to that route.
3.  **Autonomous Transit:** The non-blocking loop continuously drains GPS bytes to calculate distance, while the magnetometer calculates the required steering angle, dynamically adjusting motor PWM.
4.  **Obstacle Avoidance:** If an obstacle is detected, the state machine instantly overrides drive commands and halts the motors.
5.  **Arrival:** Upon verifying the final coordinate, the system halts, updates the UI, and enters standby.

## 🚀 Future Roadmap
* **Sensor Fusion & Odometry:** Transitioning to precise dead-reckoning by combining motor encoders, RTK GPS, and a 9-DOF IMU through an Extended Kalman Filter (EKF).
* **Dynamic Path Planning:** Integrating a 2D LiDAR and a dedicated computer running ROS to dynamically route around pedestrians.
* **Cloud Connectivity:** Upgrading to a 4G/LTE modem connected to an IoT cloud backbone for centralized fleet management.

---
*Developed as part of EC253: Sensor Technologies at NIT Goa.*
