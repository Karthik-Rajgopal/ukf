# UKF-Based Bicycle State Estimator

## Overview

This project implements an Unscented Kalman Filter (UKF) for estimating the state of a bicycle moving in a 2D plane. The goal is to estimate the bike’s position and heading angle over time given pedal speed, steering input, and noisy GPS-like position measurements. The UKF handles the nonlinearities in the system more accurately than an Extended Kalman Filter.

## Features

- Nonlinear state estimation using UKF with sigma-point propagation
- Joint estimation of position, heading, and physical parameters (wheelbase and radius)
- Handling of irregular and noisy position measurements
- Clean modular implementation with only NumPy and SciPy

## System Model

The state includes:
- Rear wheel position \((x_1, y_1)\)
- Heading angle \(\theta\)
- Wheelbase \(B\)
- Wheel radius \(r\)

The velocity is computed from pedal speed as \(v(t) = 5r \cdot \omega(t)\), and dynamics are discretized via Euler integration.

## Measurement Model

Position measurements correspond to the center of the bike, computed as:
\[
p = \begin{bmatrix} x_1 + \frac{1}{2}B \cos\theta \\ y_1 + \frac{1}{2}B \sin\theta \end{bmatrix}
\]
Measurements are noisy and may be missing intermittently.

## UKF Approach

- **State prediction:** Sigma points are propagated using nonlinear bicycle dynamics.
- **Measurement update:** When position is available, update via Kalman gain.
- **Angle handling:** All angles are wrapped to \([-π, π]\) for consistency.
- **Adaptive parameter estimation:** \(B\) and \(r\) are included in the state vector to account for uncertainty.

## Usage

Two functions interface with the system:
- `estInitialize()` – Initializes the UKF with prior state and covariances.
- `estRun(time, dt, internalState, steeringAngle, pedalSpeed, measurement)` – Runs the UKF prediction and update for each timestep.

## File Structure

ukf_bicycle/
├── estInitialize.py # Initializes the UKF
├── estRun.py # Performs UKF time and measurement updates
├── main.py # Runs simulation and plots results
├── utils.py # Helper functions (e.g., angle wrapping)
└── data/
└── bicycle_data.csv # Steering, pedal speed, and GPS measurements

## Results

The UKF provides smooth and accurate estimates of the bicycle's trajectory even with intermittent or noisy measurements. The filter successfully adapts to uncertainties in physical parameters.

## Contributors

- **Karthik Rajgopal** – UKF implementation, data processing, report intro and problem setup
- **Jannik Heinen** – Code logic, structure, and algorithm implementation
- **Mauricio Vergara** – Cleanup, conclusion, and formatting
