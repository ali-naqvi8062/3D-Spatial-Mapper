# 3D-Spatial-Mapper #

## Overview ##
The 3D Spatial Mapper is an innovative device designed for accurate spatial measurement and mapping. This project leverages a VL53L1X Time-of-Flight (ToF) sensor, an MSP432E401Y microcontroller, and a UNL2003 Stepper Motor Controller to perform 360-degree distance measurements on a single plane. The captured data is transmitted via UART for 3D visualization using Python's Open3D module.

## Key Features ##
- MSP432E401Y Microcontroller: Handles data processing with a 40MHz (up to 120MHz can be configured) bus speed and interfaces with the ToF sensor.
- VL53L1X ToF Sensor: Provides distance measurements up to 4 meters with high accuracy.
- Stepper Motor Controller: Rotates the ToF sensor to capture comprehensive spatial data.
- Data Visualization: Uses Open3D and Python to create 3D models from the captured data, ideal for applications like indoor navigation and layout mapping.

## Functionality ##
The device performs precise 3D mapping by rotating the ToF sensor and capturing distance data at each step. The microcontroller processes this data and transmits it to a Python script, which visualizes the environment in real-time. This setup provides a cost-effective alternative to commercial LiDAR systems, suitable for use in autonomous robotics and spatial analysis.

## Block Diagram ## 
<img width="537" alt="block" src="https://github.com/user-attachments/assets/7e8fbde7-be8d-46fb-a059-c449e5402257">

## Usage ##
- Software Requirements: Python 3.6+, with pyserial, numpy, and open3D libraries.
- Hardware Setup: Assemble the microcontroller, stepper motor, and ToF sensor using the block diagram. Connect the microcontroller to a PC via USB for data transmission.
- Operation: Run the provided Python script to visualize the 3D mapping data. Adjust parameters like rotation angle and x-axis displacement as needed.

## Limitations ##
- The ToF sensor's performance can be affected by ambient light conditions.
- The system is optimized for environments with minimal light interference.
- The microcontroller offloads complex trigonometric calculations to Python to maintain real-time processing capabilities.

## Applications ##
This device is ideal for indoor navigation, autonomous robotics, and environment mapping, offering a practical solution for 3D spatial analysis in various technical and industrial fields.

