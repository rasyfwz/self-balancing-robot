# Lab 7: Self-Balancing Robot (MinSeg)

This repository contains the modeling, simulation, and implementation files for a miniature Segway (MinSeg) robot designed to balance upright using feedback control.

## Project Overview
The objective of this lab is to control a two-wheeled robot to maintain an upright equilibrium position. The project is divided into three primary phases:
1.  **Modeling**: Deriving the equations of motion using Euler-Lagrange equations.
2.  **Simulation**: Testing the controller in a Python environment using both linear and non-linear models.
3.  **Hardware Implementation**: Deploying the control law to a microcontroller using CircuitPython.

## File Structure
* **`Lab 7.pdf`**: Lab manual and report documenting mathematical derivations, transfer functions, and simulation results [cite: 1-94].
* **`host.py`**: A Python script used to simulate the robot's dynamics, calculate LQR (Linear Quadratic Regulator) gains, and visualize the system response [cite: 106-193].
* **`code.cpy`**: The CircuitPython source code for the robot's microcontroller, handling real-time sensor fusion and motor control [cite: 95-105].

## System Dynamics & Modeling
The robot is modeled as an inverted pendulum on wheels. Key parameters used in the calculations include:
* **Pendulum Mass ($m_p$)**: 0.116 kg
* **Pendulum Length ($l_p$)**: 0.018 m
* **Wheel Radius ($r_w$)**: 0.021 m
* **Motor Torque Constant ($k_t$)**: 0.0948 
* **Back EMF Constant ($k_b$)**: 0.1343 

The system's state vector is defined as $x = [h, \dot{h}, \alpha, \dot{\alpha}]^T$, where $h$ is horizontal displacement and $ lpha$ is the angular position of the pendulum [cite: 16, 101].

## Control Strategy
### 1. Linearization
The non-linear equations derived from the Euler-Lagrange method are linearized around the upright equilibrium point ($\alpha = 0$) using numerical Jacobian differentiation in `host.py`

### 2. LQR Control
A Linear Quadratic Regulator (LQR) is used to find the optimal feedback gains ($K$) [cite: 172]. The weighting matrices are defined as:
* **Q**: `diag([1, 0, 0, 0.02])`
* **R**: `0.1`

The resulting gain vector $K$ used in the implementation is:
`[-3.16227766, -13.62534159, 8.36052798, 0.48831056]`.

## Hardware Implementation
The hardware runs on CircuitPython and utilizes the following logic:
* **Sensor Fusion**: A complementary filter (alpha = 0.99) combines gyroscope integration and accelerometer data to estimate the tilt angle.
* **Safety Features**: Motors are disabled if the tilt angle exceeds 40 degrees ($|\alpha| > 40^\circ$).
* **Motor Control**: PWM duty cycles are updated at 100Hz ($dt = 0.01s$) based on the LQR control law.

## How to Run
1.  **Simulation**: Run `host.py` to perform the LQR design and see the linear vs. non-linear comparison plot.
2.  **Hardware**: Load `code.cpy` onto the MinSeg microcontroller and monitor live telemetry via `usb_cdc`.
