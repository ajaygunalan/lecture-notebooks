# How to Walk: from a (mostly) optimal control perspective

**Optimal Control and Reinforcement Learning, 4.1.2025**

John Zhang
Robotic Exploration Lab
Robotics Institute, Carnegie Mellon University

---

## Logistics

- Guest lecture today, no class on Thursday
- No office hours this evening. I will host office hours after class in my office (NSH 1502)

---

## Historical Timeline of Walking Robots

### 1960s to early 2000s — Honda Asimo

[Image showing Honda Asimo robot walking on a treadmill]

---

### 1960s to early 2000s — CMU → MIT Leg Lab

[Image showing early CMU/MIT Leg Laboratory experimental setup]

---

### 2010s — Boston Dynamics Atlas

[Image showing Boston Dynamics Atlas robot performing parkour/jumping]

---

### 2010s — MIT Cheetah

[Image showing MIT Cheetah quadruped robot on indoor floor]

---

### 2025 — Reinforcement Learning

[Image showing modern humanoid robot in dynamic motion]

---

## History of Walking Robots: A Summary

- First legged robots was built in ~1960s, serious research from ~1980s

- **Modeling approaches**
  - Inverted industrial arm: slow, quasi-static (Honda)
  - Floating-based dynamics: dynamic locomotion (Raibert/CMU/MIT)

- **Actuation methods**
  - Hydraulic actuators used to be very common
  - These days direct-drive electric motors are much better

- **Control strategy**
  - MPC was the state-of-the-art in the 2010s
  - Since ~2021/2022 RL methods have become the standard

---

## Convex MPC

$$
\begin{align}
\min_{u} \quad & \text{quadratic cost} \\
\text{subject to} \quad & \text{linear robot dynamics} \\
& \text{linear constraints}
\end{align}
$$

[3D visualization showing a convex quadratic cost function surface]

---

## Walking with Convex MPC — Single-Rigid Body Model

aka. lumped mass model/potato model etc.

[Image showing quadruped robot with single rigid body representation, force vectors $f_1, f_2, f_3$, position vectors $r_1, r_2, r_3$, coordinate frames ${}^Bx$, ${}^By$, ${}^Bz$, ${}^0x$, ${}^0y$, ${}^0z$, and center of mass position $n_x$ labeled]

---

## Walking with Convex MPC

[System architecture diagram showing:]

```
┌─────────────────┐   position,    ┌─────────────────┐   swing-leg   ┌─────────────────┐
│ state estimator │ → attitude   → │ gait/footstep   │ → trajectory → │  joint-level    │
│                 │                 │    planner      │                │   controller    │
└────────┬────────┘                 └────────┬────────┘                └────────┬────────┘
         ↑                                   ↓                                  ↓
      sensors                          contact mode                       desired motor
         ↑                                   ↓                              torques
         │                           ┌──────────────┐   contact forces       ↓
         │                           │  linear MPC  │ ───────────────────────┘
         │                           └──────────────┘
         │                                                                    ↓
         └───────────────────────────────────────────────────────────────────┤
                                                                         ┌────┴────┐
                                                                         │  robot  │
                                                                         └─────────┘
```

[Small robot diagram shown in top right]

---

## Walking with Convex MPC — Assumptions

- Leg mass and inertia are relative insignificant compare to the body

- Body angular velocities and pitch/roll are small (small angle approximation)

- Foot positions can track the reference pretty well

---

## Walking with Convex MPC

- Computationally inexpensive (hundreds of Hz on on-board computers)

- (Linearized) single-rigid body assumptions can be limiting

- Enhancements include using nonlinear single-rigid body or whole-body models (at the cost of more compute at run time, no longer convex)

---

## Walking with Nonlinear MPC — iLQR

[Two side-by-side images showing simulation and real-world robot running with visualization overlays]

**1x**

---

## Walking with iLQR — Takeaways

- iLQR doesn't naturally handle additional constraints, contacts constraints are implicitly enforced through forward simulation with contact dynamics

- The algorithm dates back to the 1960s, computers just got faster

- We got a little better at modeling robots (rigid body and point-surface contact), but mostly computers got faster

---

## Walking with Nonlinear MPC — Still an Active Area of Research

- Nonlinear single-rigid body model, whole-body dynamics

- iLQR vs SQP

- Differentiating through contact dynamics, reasoning over contact modes in the optimization problem (so called "contact-implicit" methods)

---

## Reinforcement Learning

```
                    agent
                (neural network)
                      ↓
                   action
                      ↓
                 environment
                   (MuJoCo)
                      ↓
                   reward
                      ↑
                      └──────────┘
```

---

## Learning to Walk with (Sim-to-Real) Reinforcement Learning

- Massively parallel simulation on the GPU

[Image showing hundreds of quadruped robots training in parallel in simulation]

---

## Learning to Walk with (Sim-to-Real) Reinforcement Learning

- Massively parallel simulation on the GPU

- Modern robotics simulators are "good enough" for locomotion

- Amortized compute offline, cheap to inference online

```
                    agent
                (neural network)
           ┌─────────┴─────────┐
        reward              action
           ↑                   ↓
           │             environment
           │               (MuJoCo)
           └───────────────────┘
```

---

## Learning to Walk with (Sim-to-Real) Reinforcement Learning

- Massively parallel simulation on the GPU

- Modern robotics simulators are "good enough" for locomotion

- Amortized compute offline, cheap to inference online

- Much richer representation (vision based, domain randomization, sensor to torques, etc.)

---

## Open Research Question

**How can we leverage parallel computation at test time?**

---

## Model-Predictive Path Integral Control (MPPI)

**MPPI algorithm:**
- Sample actions from some distribution
- Evaluate the samples, pick the best one
- Update the distribution mean
- Repeat

[Image showing quadruped robot on checkered floor with goal marker]

---

## MPPI

**Pro:**
- Parallelization friendly
- Derivative free
- Naturally incorporates data-driven black-box models

**Con:**
- Curse of dimensionality

[Image of RC rally car jumping over terrain]

---

## Walking (and Manipulating) with MPPI

- Simulators today give you (almost) everything you can model

- Real-time reasoning over full-order models, nonlinear contact, and whole-body collision, without offline training

[Three simulation screenshots showing: quadruped on stairs, pushing a box, climbing stairs]

**All solves in real time > 100 Hz**

---

## Walking (and Manipulating) with MPPI

- Real-time reasoning over full-order models, nonlinear contact, and whole-body collision, without offline training

- Simulators today give you (almost) everything you can model

[Images showing: simulation with ball, robot climbing over box in real life, robot pushing box in real life]

**Climbing a box** | **Pushing the box**

**All solves in real time > 100 Hz with 30 samples!**

**2x**

---

## Comparison: GPU Simulation vs CPU Simulation

**GPU simulation**

[Video comparison panel showing faster performance]

**CPU simulation**

[Video comparison panel showing slower performance]

---

## Sampling Over Spline Control Points

[Diagram showing multiple trajectory samples with spline control points]

```
u
  │  ●───────────────────●───────────────────●
  │ ╱ ╲               ╱   ╲               ╱   ╲
  │╱   ╲             ╱     ╲             ╱     ╲
  ●─────●───────────●───────●───────────●───────●
  │     │           │       │           │       │
  ├─────┼───────────┼───────┼───────────┼───────┤→ t
```

**Reduce the search space by 10x**

**e.g.**
Direct sampling over controls: 50 time steps

vs.

Sampling over splines: 5 control points

---

## Sampling Over Spline Control Points

**Reduce the search space by 10x**

**Actions are guaranteed to be smooth**

---

## MPPI — Limitations

- Many cases simulated physics diverge from real life

[Left image: simulation showing robot with "Middle Goal" marker]
[Right image: diagram showing robot body tilt discrepancy between expected and actual]

---

## MPPI — Limitations

- Many cases simulated physics diverge from real life

- Only control what you can simulate (fast and in parallel)

[Left image: Aquarium visualization by Lee et al.]
[Right image: Drake simulator contact visualization by TRI]

---

## MPPI — Limitations

- Many cases simulated physics diverge from real life

- Only control what you can simulate (fast and in parallel)

- Unstable systems

**MPPI** | **LQR**
[Comparison images showing scattered/unstable quadrotor trajectory vs. stable quadrotor trajectory]

---

## Parting Thoughts

- Legged locomotion is a largely solved problem

- The next frontier is locomotion + manipulation (so called "loco-manipulation")

[Image showing Boston Dynamics Atlas robot carrying a large object on stairs]

---

## Parting Thoughts

- Legged locomotion is a largely solved problem

- There are many open challenges in locomotion + manipulation for legged robots

- Don't worry about MPC vs RL, think about online vs offline computation

- Robotics algorithms are deeply connected to the robot hardware and compute

- Sim-to-real (or real-to-sim?) is still an open problem
