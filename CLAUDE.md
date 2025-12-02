# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ALBERT is a self-regulating guided rocket project - a "Maturaprojekt" (Austrian graduation project) that aims to develop a fully functional, autonomous guided rocket system. Named after Albert I, the first mammal launched on a rocket (V-2 Rocket "Blossom No. 3") on June 18, 1948.

**Project Goals:**
- Budget: Under €1000
- Timeline: 9 months
- Develop fully functional self-regulating guided rocket
- Autonomous flight path correction through control systems
- Successful parachute recovery system deployment
- Self-designed solid fuel motor (~H-Class)
- Fin-based steering system
- All systems must pass ground tests
- Integrated safety systems in flight computer

## Repository Structure

The repository is organized into 5 main directories:
- `1_Documentation/` - Project documentation (contains Maturaprojekt_Lastenheft.pdf)
- `2_CAD_Structure/` - CAD files and structural designs
- `3_Flight_Computer/` - Flight computer hardware and software
- `4_Simulation/` - Simulation tools and models
- `5_Codebase/` - Main source code

## Team Responsibilities

**TIM:**
- Recovery system (parachute)
- Steering system (fins)
- Structural system

**FLIGHTCOMPUTER (Team Member):**
- Flight computer development
- Control systems & simulation
- Telemetry & ground station

## Technical Specifications

### Flight Computer & Control Systems
**Hardware Requirements:**
- Microcontroller: min. 32-bit, >50MHz
- 9-DOF IMU (gyroscope, accelerometer, magnetometer)
- GPS module for position determination
- Barometer/altimeter for altitude measurement
- Data logging on SD card/flash chip
- Sampling rate >100Hz for sensor data
- Redundancy systems

**Control System Goals (Priority):**
1. Primary: Vertical ascent with attitude control
2. Secondary: Waypoint guidance to GPS coordinates

**Control Requirements:**
- Attitude control: Roll, Pitch, Yaw stabilization (PID controllers)
- Position control: GPS-based waypoint navigation
- Disturbance rejection: Wind, center of gravity shifts
- Fail-safe modes: Automatic stabilization on sensor failure
- Adaptive center of gravity compensation
- Coordinate transformation: Body-frame to Earth-frame
- Sensor fusion: Kalman filter for IMU/GPS data

### Telemetry System
- LoRa radio module for real-time data transmission
- Range: >2km line of sight
- Transmission rate: >10Hz for critical data
- Laptop ground station for data reception and analysis

**Transmitted Data:**
- IMU data (gyroscope, accelerometer, magnetometer)
- GPS position and velocity
- Barometer altitude and velocity
- Fin positions (actual values)
- Control variables and control signals
- System status and error messages
- Battery voltage

### Propulsion System
- Solid fuel motor H-class
- Burn duration: 2-4 seconds
- Electrical ignition system
- Secure motor mounting in fuselage

### Steering System
- 4 controllable canard fins or tail fins
- Servo actuators with >10kg*cm torque
- Deflection: ±20° minimum
- Response time: <50ms
- Robust mounting against aerodynamic forces

### Recovery System
- Parachute sized for <5m/s landing speed
- Pyrotechnic deployment (ejection charge)
- Pressure-tight separation of payload/motor
- Backup timer for parachute deployment
- Recovery lines with >500N tensile strength

### Structure System
- Cardboard tube main fuselage (min. 2mm wall thickness)
- 3D-printed components (nose cone, fin holders)
- Total length: <1m for transportability
- Modular design for easy maintenance
- Load capacity: >20G axial, >10G lateral
- Weight: 500g - 1000g

### Power System
- LiPo battery with min. 30min operating duration
- Voltage monitoring and undervoltage protection
- Arm/disarm switch for safety
- All voltages stabilized and filtered

## Testing Requirements

- All subsystems must be tested individually
- Complete system integration tests before flight test
- Telemetry tests with ground station
- Control tests in simulation before hardware test

## Development Guidelines

- All systems must pass ground tests before integration
- Prioritize safety systems and fail-safe modes
- Implement comprehensive logging for post-flight analysis
- Use simulation extensively before hardware testing
- Follow modular design principles for maintainability

## Current State

The project is in early development phase with the requirements document (Lastenheft) defining the comprehensive technical specifications. The main directories are structured but awaiting implementation of the various subsystems.
