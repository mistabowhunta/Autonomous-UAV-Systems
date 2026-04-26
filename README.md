# Autonomous UAV Systems & MAVLink Controllers

## Overview
This repository serves as the central hub for custom unmanned aerial vehicle (UAV) control software, autonomous mission logic, and MAVLink telemetry integrations developed under NasonNation Robotics. 

The architecture is designed as a monorepo to support multiple hardware platforms and flight controller environments, segregating hardware-specific tuning and mission logic into dedicated modules while maintaining standard DroneKit and PyMavlink communication protocols.

## Repository Structure

### `/XP5_Platform`
Contains the flight control logic and autonomous mission scripts tailored for the XP5 hardware frame.
* **`XP5Mission.py`**: An object-oriented, Python 3 flight controller script executing automated arming, guided quaternion-based attitude control for ascent, failsafe altitude hovering, and autonomous landing sequences without GPS reliance (`GUIDED_NOGPS`).

### `/Future_Platforms` *(TBD)*
Reserved for upcoming multirotor builds requiring separate kinematic models and tuning parameters.

## Core Technologies
* **Python 3:** Primary control logic.
* **DroneKit-Python:** High-level vehicle state management and mode transitions.
* **PyMavlink:** Low-level MAVLink message encoding (e.g., `set_attitude_target_encode`) for direct quaternion manipulation.
* **Hardware Targets:** Compatible with ArduPilot/PX4 software-in-the-loop (SITL) simulators and physical flight controllers via serial/telemetry radios.

## Usage
Dependencies can be installed via standard requirements:
```bash
pip install dronekit pymavlink
