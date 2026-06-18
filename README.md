# Active-Suspension-Control
Final project for the course **Vehicle Dynamics and Control (RO47017)** at TU Delft.

This project investigates active suspension control for a quarter-car model, comparing two advanced control strategies against a passive baseline. The goal is to improve ride comfort and road holding by minimizing body acceleration and suspension deflection while keeping tire force fluctuations low. All controllers are implemented and simulated in MATLAB.

The project explores:

- H-infinity (H∞) robust control
- µ-synthesis for robustness against model uncertainty
- Model Predictive Control (MPC)
- Linear-Quadratic Regulator (LQR) as a reference controller
- Quarter-car suspension modelling and road disturbance response

## Project Overview

The suspension system is represented by a quarter-car model with a slow-active actuator. Two control architectures are designed and evaluated:

- **H∞ control** is implemented in three driving modes — *comfort*, *balanced*, and *performance*. Each are tuned for a different trade-off between ride comfort and handling. The controller is then robustified with a **µ-synthesis** design to handle model uncertainty, and its stability is analysed.

- **Model Predictive Control (MPC)** optimizes the control action over a prediction horizon subject to state and actuator constraints. Its performance is benchmarked against an **LQR** controller used as a reference.

The strategies are compared on tire force (road holding), and suspension deflection and body acceleration (comfort). The results show that both approaches offer significant improvements over a passive suspension, each with distinct advantages under different conditions.

The repository contains the MATLAB implementations of both controllers together with the [final report](Vehicle_Dynamics_Report.pdf) containing the model derivation, controller design, simulations, plots, and analysis.

## Repository Structure

```
Active-Suspension-Control-Quarter-Car/
│
├── Code/
│   ├── H_infinity_Group10.m          % H∞ and µ-synthesis controllers
│   └── MPC_Controller_Group10.m      % MPC and LQR controllers
│
├── Vehicle_Dynamics_Report.pdf
│
└── README.md

```
---

## Authors

- Sven Rutgers
- Melis Orhun
- Maksymilian Szafer
- Tine Coutuer
- Yogesh Prasanna Kumar Rao