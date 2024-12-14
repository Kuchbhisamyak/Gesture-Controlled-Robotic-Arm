# 6 DOF Gesture Controlled Robotic Arm

Welcome to the 6 Degrees of Freedom Gesture Controlled Robotic Arm project! This repository contains all the resources, including code, schematics, and documentation, to build and control a robotic arm using hand gestures.

---

## Features

- Gesture Control: Control the robotic arm using hand gestures via MPU6050 sensor.
- 6 Degrees of Freedom: Achieves versatile and precise movements.
- Wireless Communication: Utilizes ESP-NOW for real-time data transmission between the controller and the robotic arm.
- Flex Sensors Integration: Flex sensors are used for finer control of additional functionalities.
- Battery-Powered: Operates using a LiPo 3S battery.
- High Torque Motors: Includes DRV8825 motor drivers and a 16-channel servo driver for robust performance.

---

## Hardware Components

- Robotic Arm: Custom-built with 6 degrees of freedom.
- ESP32 (x2): One for the gesture controller and another for the robotic arm.
- MPU6050: For gesture sensing.
- Flex Sensors (x3): Connected to GPIO 36, 39, and 34 on the ESP32.
- 16-Channel Servo Driver: To control the servos.
- DRV8825 Motor Driver: For stepper motor control.
- LiPo 3S 2200mAh Battery: Powers the system.

---

## Software Stack

- Arduino IDE: For programming ESP32 boards.
- ESP-NOW Protocol: For wireless communication.
- I2C Protocol: For interfacing MPU6050 and flex sensors.
- Servo and Motor Control Libraries: For precise movement.

---

## Repository Structure

```plaintext
├── Code
│   ├── Sender
│   │   ├── sender_code.ino       # Gesture control code for MPU6050
│   ├── Receiver
│   │   ├── receiver_code.ino     # Robotic arm control logic
├── Schematics
│   ├── robotic_arm_circuit.pdf   # Circuit diagrams
├── Models
│   ├── robotic_arm_design.stl    # 3D models for robotic arm
├── Documentation
│   ├── README.md                 # This file
│   ├── setup_guide.md            # Step-by-step setup instructions
│   ├── troubleshooting.md        # Common issues and solutions
```

---

## How to Use

### Prerequisites

1. Install [Arduino IDE](https://www.arduino.cc/en/software) and set up ESP32 boards.
2. Ensure all hardware components are connected correctly as per the provided schematics.

### Setup Instructions

1. Clone this repository:
   ```bash
   [git clone https://github.com/yourusername/6DOF-gesture-controlled-arm.git](https://github.com/Kuchbhisamyak/Gesture-Controlled-Robotic-Arm.git)
   ```
2. Upload the `sender_code.ino` to the ESP32 controlling the MPU6050.
3. Upload the `receiver_code.ino` to the ESP32 on the robotic arm.
4. Power up the system using the LiPo 3S battery.
5. Perform gesture movements to control the robotic arm.

---

## Applications

- Industrial automation
- Assistive robotics
- Educational robotics projects
- Customizable for various robotic tasks

---

## Future Enhancements

- Add inverse kinematics for precise movement control.
- Integrate a camera for visual feedback.
- Enhance gesture recognition using machine learning.
- Improve battery life and optimize power consumption.

---

## Links to Resources

- CAD/3D Printing model : https://www.thingiverse.com/limpsquid/designs
- Working of Project : https://smartbuilds.io/diy-robot-arm-arduino-hand-gestures/

## Contributing

We welcome contributions! Feel free to fork this repository, create a branch, and submit a pull request. For significant changes, please open an issue to discuss what you would like to change.

---

## License

This project is licensed under the MIT License. See the `LICENSE` file for details.

---

## Acknowledgments

- Special thanks to the Robotics and Aviation Club at RCOEM for their support and 3D printing.
- Inspired by various open-source robotics projects in the community.
- CAD design reference from limpsquid.
- Project concept reference from smartbuilds.io.



---

Happy Coding! 😊

