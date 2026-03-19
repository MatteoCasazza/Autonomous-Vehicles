# Assignment 5 – Parking Gate Control FSM

## 🎯 Problem Statement

The aim of this assignment is to analyze a provided **Vehicle Finite State Machine (FSM)** and improve it by designing a **Parking Gate Control FSM**.  

The parking gate must be modeled for both **raise** and **lower** actions, considering physical limitations:

- Maximum angular speed: |ω| ≤ 4°/s  
- Angular position: 0° ≤ θ ≤ 90°  
- Maximum angular acceleration: |a| ≤ 1°/s²  

The integrated FSM model is simulated for 100 seconds to evaluate the behavior of both the gate and the vehicles.

---

## ⚙️ Methodology

All work was carried out in **Simulink**, using **Stateflow** to design and test the state machine.

### Vehicle FSM Analysis

The provided FSM describes the motion of a vehicle approaching a parking gate and passing through it once the gate is raised.  

**Inputs:**  
- `pos < 0` → random initial position  
- `start = 1` → start signal  
- `gate_up = 1` → gate status  

**Outputs:**  
- `pos` → vehicle position  
- `car_waiting` → waiting flag  
- `car_passed` → passed flag  

The FSM has five sequential states:

1. **Idle:** Initializes vehicle position; `car_waiting = 0`, `car_passed = 0`. Transitions to `Approaching` when `start == 1`.  
2. **Approaching:** Vehicle moves with constant velocity `v`. Transition to `Stop` when `pos ≥ 0`.  
3. **Stop:** Vehicle stops at the gate (`car_waiting = 1`). Transition to `Pass` when `gate_up == 1`.  
4. **Pass:** Vehicle resumes motion (`pos = tmp + v · elapsed(time)`) and resets waiting flag. Transitions to `Passed` when reaching the gate end.  
5. **Passed:** Vehicle continues motion; `car_passed = 1`. After 10 s, FSM resets.  

**Limitation:**  
The original FSM treats gate motion as instantaneous (via Boolean `gate_up`) and ignores physical constraints, preventing realistic synchronization between vehicle and gate.

---

###  Integration of Gate Control

To improve the model, a **Gate Control FSM** was added:

- Vehicle states (`Idle`, `Approaching`, `Stop`, `Pass`) remain unchanged.  
- The **Raise Function** is added after `Stop`, with three phases:  

  1. **Acceleration (raise_Start):**  
     - Gate accelerates from 0°  
     - ω = a · t  
     - θ = θ_init + ½ · a · t²  

  2. **Constant Velocity (raise_Linear):**  
     - Gate moves at ω_max until near target  
     - θ = θ_init + ω_max · t  

  3. **Deceleration (Brake_Raise):**  
     - Gate slows to stop at 90°  
     - ω = max(0, ω_max - a · t)  
     - θ = θ_init + ½ · (ω + ω_max) · t  

- Once the gate is fully raised, `gate_up = 1` and vehicle can pass.  
- After vehicle passes, a **symmetric lowering function** returns the gate to 0°.  

**Velocity Profile:**  
- Trapezoidal, continuous in position and velocity, avoiding unrealistic jumps.

---

## 📊 Results

Simulations of 100 s show vehicle and gate interactions:

### Vehicle Motion

- Vehicle stops at gate (`pos = 0`) and waits for gate to open.  
- Once gate is open, vehicle moves at constant speed `v = 1 m/s`.  
- Vehicle resets after gate closes for 10 s.  

**Observation:**  
Second vehicles follow the same behavior, demonstrating correct vehicle-gate synchronization.

---

### Gate Motion

- Trapezoidal velocity profile for both raising and lowering:  
  - Constant acceleration phase: 4 s  
  - Constant velocity phase: 18.6 s  
  - Constant deceleration phase: 4 s  

- Angular position ranges from 0° → 90° → 0°, motion is smooth and continuous.  
- Total time to raise the gate: 26.6 s, compliant with physical constraints but longer than typical real-world parking gates.

---

## ✅ Conclusion

- The integrated FSM **successfully models vehicle and gate interaction** with physical constraints.  
- The gate motion is realistic, continuous, and synchronized with vehicle arrival.  
- Main limitation: **raising/lowering time** is long compared to real parking scenarios. Improvements could include overshoot handling or mechanical aids.  
- The designed FSM provides a feasible solution under simulation parameters, ensuring both **functional requirements** and **physical feasibility**.

---
