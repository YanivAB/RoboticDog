# Robotic Dog Project
This repository contains the code and resources for our final engineering project: a robotic dog designed and built as part of our Mechanical Engineering and Robotics studies at the Technion. The project integrates biomechanical principles to develop a four-legged robotic prototype that mimics the movements of a real dog.

# Project Overview
## Objective
The primary goal is to design and construct a robotic dog prototype capable of walking independently and transitioning between positions (stand, sit, and lie down). The project integrates:
Mechanical design and kinematics
Mechatronics and software development
Biomechanical concepts for robotic motion

## Key Features
Self-Stabilizing Movement: Robot walks and maintains balance on uneven terrain.
Pose Transitions: Switches between standing, sitting, and lying down positions.
Servo Motor System: Driven by SPT5425LV-W and MG996R servos, enabling precise control of leg joints.
Inverse Kinematics & Trajectory Planning: Ensures smooth movement through calculated joint angles.
Communication: Coordination between Raspberry Pi and Teensy microcontroller for motor control and trajectory planning.

# Hardware Components
Motors: SPT5425LV-W (25 kg/cm torque), MG996R (10 kg/cm torque)
Controller: Teensy microcontroller for PWM motor control
Processor: Raspberry Pi for kinematics calculations and trajectory planning
Structure: 3D-printed parts (PLA) for lightweight and robust assembly

# Software Architecture
Inverse Kinematics: IK(x, y) - Computes joint angles based on desired foot coordinates.
Trajectory Planning: trajectory_plan(x0, y0, R, steps) - Generates smooth movement paths for each leg.
Motor Control: send_angles(angles, pose_name) - Sends angle commands to each servo motor.

# Setup and Installation
## Prerequisites
Hardware: Raspberry Pi, Teensy, servo motors (SPT5425LV-W and MG996R)
Software: Python, Arduino IDE with required libraries for motor control (or custom PWM control)
Installation
Clone this repository:
bash
Copy code
git clone https://github.com/username/robotic-dog-project.git
cd robotic-dog-project
Load and compile the Teensy control code using the Arduino IDE.
Run the main Python scripts on the Raspberry Pi to initiate kinematics and control loops.

# Challenges and Solutions
Servo Durability: Reinforced motor gears with Teflon layers to improve durability under load.
Weight Distribution: Adjusted battery and motor placement for optimal balance.
Custom Movement Control: Developed custom code for efficient motor control and reduced reliance on pre-built libraries.

# Future Enhancements
Advanced Mobility: Additional gaits and obstacle avoidance
Remote Control via ROS: Integration with ROS for wireless operation
AI Capabilities: Vision processing and autonomous navigation

## Technion- Israel Institute of Technology

# Contributors
Natay Baruchis - https://www.linkedin.com/in/nitai-bruchis/
Yaniv Abramov - https://www.linkedin.com/in/yaniv-abramov/
Supervised by: Roman Shamsutdinov






