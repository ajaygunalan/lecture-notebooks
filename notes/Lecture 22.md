# Lecture 22

## Last Time:
- LQG
- Kalman

## Today:
- Rocket Soft Landing

---

## The Rocket Soft-Landing Problem

**Objective:**
- Go from some initial state $X_0$ to some final position $r_f$ with $Z_f = 0$, $V_f \approx 0$
- Minimize some combination of fuel consumption and/or landing position error $||r_f - r_y||$
- Respect thrust limits + safety constraints

**Examples:**
- NASA Curiosity "Sky Crane" (2012)
- SpaceX Falcon, Starship
- NASA Perseverance w/TRN (2021)

---

## The "Full Stack"

```
Position/Velocity → [Position Controller] → acceleration (linear) → [Attitude Controller] → Thrust/gimbal angle → [Rocket]
                                              command                    ↑
                                              ↑                          attitude + angular rate
                                              └──────────────────────────[State Estimator]
```

### State Estimation

**SpaceX:** GPS + IMU with good filtering
- ~1m position accuracy
- <1cm/s velocity
- ~1° attitude

**Mars:** No GPS. IMU + Radar Altimeter + Vision
- ~30 meter Accuracy. Avoid Boulders.

### Decoupled Control Loop:

**High-Level Position Controller:** Uses a point-mass model. Reasons about safety, thrust limits, and fuel. Generating acceleration commands. Runs at ~Hz

**Low-Level Attitude Controller:** Reasons about attitude, flexible modes, fluid slosh, generates thrust + gimbal commands to track desired acceleration. Runs at ~10 Hz

---

## Rocket Dynamics

### Rigid Body
- **Point mass:** $\dot{v} = -g + \frac{T}{m}$
- **Fuel burn:** $\dot{m} = -\alpha||T||$
- **Attitude (fast):** $J\dot{\omega} + \omega \times J\omega = l \times T$
  - Inertia torque

### Key Considerations:

- **Fuel can be 80%+ of initial vehicle mass.** Have to account for this.

- **Fluid Slosh:** Highly nonlinear, time-varying, hard to model. Standard model: pendulum.

- **Flexible Modes:** Rockets are built to be light $\Rightarrow$ not stiff $\Rightarrow$ low-frequency bending modes. First modes ~Hz. Dealt with by adding notch filters to the attitude controller at bending frequencies.

- **Aerodynamic Forces:** Mostly ignore.

- **Lots of model error** $\Rightarrow$ Linear robust control ideas are used in the attitude controller.

---

## Background: Convex Relaxation

Sometimes we have a nonconvex constraint that can be expressed as the boundary of a convex set:

**Original set $S_1$:**
- $S_1 = \{x \mid ||x|| = 1\}$

**Enlarged set $S_2$:**
- $S_2 = \{x \mid ||x|| \leq 1\}$
- $S_1 \subset S_2$

Replacing the original constraint with larger convex one is called "convex relaxation"

Sometimes if the cost is "nice" we can still get the answer to the original problem by solving the relaxed version:

$$
\begin{align}
\min \quad & c^T x \\
\text{s.t.} \quad & ||x|| = 1 \\
& \quad \downarrow \\
& ||x|| \leq 1
\end{align}
$$

When this happens we call it a "tight" relaxation.

---

## Convex Relaxation of Thrust Constraint

### Maximum thrust constraint
- $T \in \mathbb{R}^3$, $||T|| \leq T_{\max}$ (convex)

### Thrust angle constraint
- $\frac{n^T T}{||T||} \leq \cos(\theta_{\max})$ (convex)

### Rocket engines also have a minimum thrust constraint:
- $T_{\min} \leq ||T|| \leq T_{\max}$ **Not Convex!**

---

## Convexification Approach

Let's add a new "slack variable" $\Gamma \in \mathbb{R}$ that equals the thrust magnitude:

**Original constraints:**
1) $||T|| = \Gamma$ $\rightarrow$ Boundary of a convex set (sphere)
2) $T_{\min} \leq \Gamma \leq T_{\max}$ $\rightarrow$ Convex
3) $n^T T \leq \Gamma \cos(\theta_{\max})$

**Now we can convexify the constraint by relaxing 1):**

1*) $||T|| \leq \Gamma$
2) $T_{\min} \leq \Gamma \leq T_{\max}$
3) $n^T T \leq \Gamma \cos(\theta_{\max})$

The paper proves that this relaxation is tight using Pontryagin's minimum principle.
