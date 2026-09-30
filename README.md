# 🛰️ Cart-Pole Inverted Pendulum: Dynamics & Control System Design

## 📝 Overview
This project was developed for the **Aerospace Systems Analysis and Simulation** course (BSc in Aerospace Engineering) at Politecnico di Milano. 

The objective was to derive the mathematical model of a cart-pole system and design a control architecture to stabilize the pendulum in its unstable upright position. The analysis bridges theoretical control design with real-world physical constraints.

**Key achievements of this project include:**
* **System Dynamics:** Derivation of the non-linear equations of motion and simulation of the free response, comparing ideal conditions (frictionless) with realistic ones (with friction).
* **Classical Control (PD/PID):** Design of a PD/PID controller for the pole angle, analyzing the system's robustness against model mismatch (designing without friction and testing in a frictional environment).
* **Modern Control (State Feedback & Observer):** Implementation of Pole Placement to control both the pole angle and the cart's linear position. A State Observer was designed to estimate unmeasurable state variables in real-time.
* **Physical Feasibility:** Careful tuning of the control loops to ensure the electrical control effort (motor voltage/current) remained within realistic saturation limits, guaranteeing a physically realizable actuator response.

## Read the Report
The full theoretical background, mathematical models, control tuning procedures, and final results are detailed in the project report:
👉 **[Read the Full PDF Report Here](Report.pdf)**

## 🛠️ Built With
* **MATLAB / Simulink** (For numerical simulation, control loop design, and state estimation)
* **LaTeX** (For documentation and mathematical typesetting)

## 🚀 How to Run the Code
You don't need to manually open the Simulink files; the MATLAB script handles everything automatically.

1. Open **MATLAB**.
2. Navigate to the folder containing the project files.
3. Open and run the main script (e.g., `Matlab_Code.m`).
4. The script will automatically load the required workspace variables, open the `.slx` Simulink models, run the simulations, and generate all the performance plots (time responses, control effort, etc.).
