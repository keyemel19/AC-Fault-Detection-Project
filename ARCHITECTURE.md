# System Architecture

## I. Introduction

### A. Purpose of the System
1. Overview of the fault detection system for AC power lines.
2. Importance of real-time monitoring and control to prevent power line faults.

### B. Overview of Features
1. Speed control using VFD.
2. Voltage and current measurement for fault detection.
3. Power calculation and display on an LCD.
4. User alerts via buzzer and relay switch control.
5. Teaching and demonstration of AC power concepts.

## II. System Requirements

### A. Hardware Requirements
1. Microcontroller specifications
2. Speed Controller (VFD)
3. Volt Meter and Current Meter
4. Relay Switch
5. Button and Buzzer
6. LCD Display

### B. Software Requirements
1. Programming environment (e.g., STM32CubeIDE)
2. Required libraries and dependencies (e.g., LCD display library)

## III. System Architecture

### A. High-Level Design
1. Main Controller
2. Input and Output Devices
3. Interaction between components

### B. Detailed Design
1. Main Module
   - Central control logic
   - Communication with other modules
2. Speed Control Module
   - Integration with VFD
3. Measurement Modules
   - Volt Meter Reader
   - Current Meter Reader
4. Power Calculation Module
   - RMS calculation
5. Display Module
   - Print to LCD
   - Update and refresh logic
6. Input Handling Module
   - Button input processing
   - Relay switch control
   - Buzzer activation
7. Fault Detection Module
   - Fault detection logic
   - Signal isolation using relay switch
8. TDR Integration Module
   - Handling TSR Pulse for fault location

## IV. Implementation Details

### A. Main Module
1. Initialization of components
2. Main control loop
3. Data flow management

### B. Speed Control Module
1. Function: `adjustSpeed(int newSpeed)`
   - Interface with VFD
2. Function: `getSpeed()`
   - Retrieve current speed

### C. Measurement Modules
1. Volt Meter Reader
   - Function: `readVoltage()`
2. Current Meter Reader
   - Function: `readCurrent()`

### D. Power Calculation Module
1. Function: `calculateRMS(float voltage, float current)`
   - Compute power using RMS

### E. Display Module
1. Function: `printToLCD(String message)`
   - Display data on LCD
2. Function: `updateLCD()`
   - Refresh display content

### F. Input Handling Module
1. Button Input Processing
   - Function: `processButtonPress()`
2. Relay Switch Control
   - Function: `toggleRelay()`
3. Buzzer Activation
   - Function: `activateBuzzer()`
4. Handle TSR pulse signals
   - Function: `processTSRPulse()`

## V. User Interface

### A. Button Interface
1. Button functionalities
2. User feedback mechanisms

### B. LCD Display Interface
1. Display layout
2. Example messages and data

