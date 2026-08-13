#Interactive Air Levitation PID Control
#by : Mohamed Ahmed Abdulhamid Abdellah

## Project Video
[Click here to watch the full system demonstration on youtube](https://www.youtube.com/watch?v=pBzFlaL845w)

#note:>>
#watch the video file first 
.............
.............
#introduction
.............
An Arduino-based pneumatic levitation system that uses a custom PID controller to stabilize a ping-pong ball in a vertical airstream. 
Features a dual-ultrasonic sensor architecture allowing users to dynamically control the levitation height using hand gestures, alongside a LabVIEW supervisory dashboard.


##  System Architecture

*   **Microcontroller:** Arduino (Handles all real-time PID calculations and sensor polling at ~30ms intervals).
*   **Actuation:** 12V Blower Fan driven by an L298N Motor Driver (PWM controlled).
*   **Sensors (Dual HC-SR04):**
    *   *Process Variable Sensor:* Tracks the ping-pong ball's height inside a 40cm tube.
    *   *Interactive Setpoint Sensor:* Reads the user's hand distance to generate a dynamic, real-time target height.
*   **Supervisory Dashboard (LabVIEW):** Communicates with the Arduino via VISA Serial (9600 baud). It plots the system's step response and allows the user to override the hand-sensor with a manual numerical setpoint.

## Control Strategy
The Arduino calculates the error between the desired setpoint (hand position or LabVIEW override) and the actual ball position. The PID algorithm (Kp=6, Ki=0.0796, Kd=1.1996) computes the required adjustment and maps it to a constrained PWM output (180–255) to modulate the fan speed, ensuring smooth aerodynamic stabilization.

## Repository Contents
*   `levitation_controller.ino`: The main Arduino C++ firmware containing the PID loop and sensor logic.
*   `test23_lastone.vi`: The LabVIEW virtual instrument used for serial data acquisition and manual override.
*   `System_Design_Report.pdf`: Hardware schematics and theoretical background.
#author Mohamed Ahmed Abdulhamid Abdellah
