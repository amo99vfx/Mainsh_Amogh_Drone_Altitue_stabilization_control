# Mainsh_Amogh_Drone_Altitue_stabilization_control
🚁 MATLAB &amp; Simulink drone stabilization system using analytical PID control with pole placement. Implements multi-axis UAV stabilization, disturbance rejection, root locus, and stability analysis for robust altitude and attitude control under wind disturbances.
[README (2).md](https://github.com/user-attachments/files/27747408/README.2.md)
# 🚁 Drone Multi-Axis Stabilization — PID Control System

**CONTROL CRAFT Hackathon | BNMIT ECE Department × RoboStrata Technologies**  
**Problem Statement 1: Drone Altitude Stabilization — Extended to Full Multi-Axis Control**

---

## 👥 Team Members

| Name | Department |
|------|------------|
| Amogh M S | ECE, BNMIT |
| Manish N  | ECE, BNMIT |

---

## 📌 Problem Statement

Design a control system to stabilize a drone across **all 4 axes** subjected to external disturbances such as wind.

**Given Plant Transfer Function (Altitude):**
```
        1
G(s) = ───────────────
       s² + 2s + 5
```

**Extended to all axes:**

| Axis | Transfer Function | Basis |
|------|-------------------|-------|
| Altitude (Z) | `1 / (s² + 2.0s + 5)` | Given |
| Roll (φ) | `1 / (s² + 1.5s + 4)` | Derived — lighter damping, faster angular dynamics |
| Pitch (θ) | `1 / (s² + 1.5s + 4)` | Derived — symmetric to roll (quadcopter) |
| Yaw (ψ) | `1 / (s² + 0.8s + 2)` | Derived — slowest axis, gyroscopic effect |

---

## Assumptions

1. Drone dynamics are linearized around hover equilibrium
2. The 4 axes are treated as decoupled (decentralized control architecture) — same assumption used in real flight controllers like PX4 and ArduPilot
3. Roll and Pitch have identical dynamics (symmetric quadcopter frame)
4. Yaw has slower dynamics due to gyroscopic and reaction torque effects
5. Each axis has an independent PID controller (4 SISO loops)
6. Disturbances modeled as step inputs at plant input

---

## 🎯 Control Objectives (All Axes)

| Metric | Requirement |
|--------|-------------|
| Overshoot | < 10% |
| Settling Time | < 3 seconds |
| Steady-State Error | ≈ 0 |
| Stability under disturbance | Yes |

---

## 🧠 Design Approach — PID via Pole Placement

**Target closed-loop poles (same for all axes):**
- Damping ratio: zeta = 0.7 → Overshoot ≈ 4.6%
- Natural frequency: wn = 3 → Settling time ≈ 1.9s
- Dominant poles: s = -2.1 ± j2.14
- Third pole (non-dominant): s = -6

**Desired characteristic polynomial:**
```
(s + 6)(s^2 + 4.2s + 9) = s^3 + 10.2s^2 + 34.2s + 54
```

**PID gains computed per axis by matching coefficients:**

| Axis | Kp | Ki | Kd |
|------|----|----|-----|
| Altitude (Z) | 29.2 | 54 | 8.2 |
| Roll (φ) | 30.2 | 54 | 8.7 |
| Pitch (θ) | 30.2 | 54 | 8.7 |
| Yaw (ψ) | 32.2 | 54 | 9.4 |

---

## Results Achieved

| Axis | Overshoot | Settling Time | SS Error | Phase Margin | Result |
|------|-----------|---------------|----------|--------------|--------|
| Altitude (Z) | ~4.6% | ~1.9s | < 0.0001 | > 45° | PASS |
| Roll (φ) | ~4.6% | ~1.9s | < 0.0001 | > 45° | PASS |
| Pitch (θ) | ~4.6% | ~1.9s | < 0.0001 | > 45° | PASS |
| Yaw (ψ) | ~4.6% | ~1.9s | < 0.0001 | > 45° | PASS |

---

## 📊 MATLAB Plots Generated

| Plot | Description |
|------|-------------|
| Open-Loop (2x2) | Uncontrolled step response for all 4 axes |
| Closed-Loop (2x2) | PID vs Open-Loop comparison per axis |
| Combined Step | All 4 axes overlaid on one plot |
| Disturbance Rejection (2x2) | Response to wind/torque disturbances |
| Bode Plots (2x2) | Gain and phase margins per axis |
| Root Locus (2x2) | Pole placement verification per axis |

---

## Simulink Model

The Simulink model (`drone_multiaxis.slx`) implements the same 4-axis decentralized PID architecture.

**Architecture per axis:**
```
Reference -> [Sum] -> [PID Controller] -> [Plant G(s)] -> Output
               ^                                 |
               |______ [Feedback: -1] <__________|
                                          |
                                   [Disturbance Step]
```

**Simulink Blocks Used:**
- `Transfer Fcn` — plant model for each axis
- `PID Controller` — with computed Kp, Ki, Kd
- `Step` — reference input and disturbance injection
- `Sum` — error and disturbance summing
- `Scope` — output visualization

**To run:**
1. Open `drone_multiaxis.slx`
2. Set simulation stop time to `15`
3. Click Run and open Scope blocks

---

## Repository Structure

```
Amogh_MS_Drone_Altitude_Stabilization/
|
|-- drone_multiaxis_main.m    # Full MATLAB script — all 4 axes
|-- drone_multiaxis.slx       # Simulink model — 4-axis PID
|-- README.md
|-- plots/
|   |-- openloop_all_axes.png
|   |-- closedloop_all_axes.png
|   |-- combined_step.png
|   |-- disturbance_all_axes.png
|   |-- bode_all_axes.png
|   `-- rootlocus_all_axes.png
`-- demo_video.mp4
```

---

## How to Run

### Requirements
- MATLAB R2020a or later
- Control System Toolbox

### MATLAB
```matlab
>> drone_multiaxis_main
```
All 6 figure windows open automatically with metrics printed in Command Window.

### Simulink
1. Open MATLAB
2. Open `drone_multiaxis.slx`
3. Set simulation stop time to `15`
4. Run and view Scope outputs

---

## References

- Ogata, K. — Modern Control Engineering, 5th Edition
- BNMIT Hackathon Problem Statement Document
- PX4 Flight Controller Architecture — px4.io
- MATLAB Control System Toolbox Documentation

---

*Submitted for CONTROL CRAFT Hackathon | BNMIT ECE | 2025*  
*Team: Amogh M S & Manish N*
