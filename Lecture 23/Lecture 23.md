# Lecture 23: Autonomous Driving and Game-Theoretic MPC

## Last Time:
- How to land a space ship

## Today:
- How to drive a car
- Game-Theoretic MPC

---

## History

- Primitive demos going back to 1920s-30s
- First modern work in 1980s at CMU + Germany
- Lots of demos in 1990s with 98% autonomy on cross-country highway drives
- DARPA challenge in early 2000s

## The "Full Stack"

```
[Route Planner]
       ↓
[Perception] → [Path Planner] → [MPC] → [Car]
       ↑                                    ↓
       └────────────(Sensors)────────────────┘
```

**Components:**

- **Sensors:** GPS, IMUs, Cameras, RADAR, LIDAR
- **Perception + human-driver prediction** are the hard parts
- **Route planning:** graph search to generate waypoints
- **Path Planner:** Generate a smooth spline curve that is collision free (no dynamics)
- **MPC Controller:** Tracks spline curve while reasoning about vehicle dynamics + constraints

---

## Vehicle Dynamics

- Lots of options with different levels of fidelity
- Most Common: "Bicycle" or "single-track" models

### Kinematic Bicycle Model:

**Inputs:** $V$, $\alpha$ (or $\dot{v}$, $\dot{\alpha}$)

**Assume tires don't slip:**

$$
\begin{align}
\dot{x} &= v \cos(\theta) \\
\dot{y} &= v \sin(\theta) \\
\dot{\theta} &= \frac{v \tan(\alpha)}{L} \\
\dot{v} &= u_1 \\
\dot{\alpha} &= u_2
\end{align}
$$

**State vector:**
$$
X = \begin{bmatrix} x \\ y \\ \theta \\ v \\ \alpha \end{bmatrix}
$$

**Control vector:**
$$
U = \begin{bmatrix} \dot{v} \\ \dot{\alpha} \end{bmatrix}
$$

- Works well for "normal" driving
- Breaks down in more aggressive settings (high acceleration)

---

## More Complex Models:

### "Dynamic" Bicycle Model:
- Model car as a rigid body
- $F = ma$, $\tau = J\dot{\omega}$
- Reason about engine torque, tire forces, braking, etc. explicitly

### "Double-Track" Model:
- 4 tires, full 3D rigid body dynamics, suspension etc.
- Reason about body roll, weight transfer, aggressive cornering, racing, drifting, off-road driving

### Tire Models:
- Can go from simple (no-slip), to medium (Coulomb), to complex (contact patch, deformation, nonlinear friction)

---

## State-of-the-Art Nonlinear MPC

- Dynamic bicycle model with good tire models gets you really far
- Online MPC with IPOPT at $\sim$ 50 Hz

---

## The Frozen Robot Problem:

- We want the MPC controller to reason about coupled behavior with other drivers
- Current systems make lots of assumptions. Simplest: other cars will continue at constant velocity over MPC horizon
- Works well for highway driving
- Breaks down in scenarios with tighter coupling between cars (e.g. ramp merging)

---

## Game-Theoretic Trajectory Optimization

**High-level idea:** Assume other cars are also solving an MPC problem like me.

- Solve a joint optimization problem for all cars simultaneously
- One version of this idea: "Nash Equilibrium"

### Formulation:

Let $\bar{X} = [x^1, x^2, \ldots, x^n]^T$ and $\bar{U} = [u^1, u^2, \ldots, u^n]^T$ $\leftarrow$ States + inputs for all cars stacked

For each player $i$:
$$
\begin{align}
\min_{\bar{X}, u^i} \quad & J^i(\bar{X}, u^i) \quad \leftarrow \text{ cost for player } i \\
\text{s.t.} \quad & D(\bar{X}, \bar{U}) = 0 \quad \} \text{ dynamics + collision} \\
& C(\bar{X}, \bar{U}) \leq 0 \quad \} \text{ constraints for all cars}
\end{align}
$$

- We get $n$ (number of cars) of these problems that we must solve simultaneously
- **Interpretation:** No player can unilaterally improve their cost
- Good model of non-cooperative behavior
- Cost functions can capture driver behavior like aggressiveness etc.

---

## Solution Strategy (ALGAMES)

### Form Augmented Lagrangian for each player's 1st-order necessary conditions:

$$
\begin{align}
L^i(\bar{X}, \bar{U}, \lambda^i, \mu^i) = & J^i(\bar{X}, u^i) + \frac{\rho}{2} ||D(\bar{X}, \bar{U})||_2^2 + (\lambda^i)^T D(\bar{X}, \bar{U}) \\
& + \frac{\rho}{2} ||C(\bar{X}, \bar{U})^+||_2^2 + (\mu^i)^T C(\bar{X}, \bar{U})
\end{align}
$$

$$
\Rightarrow \nabla_{u_i} L^i = 0
$$

### Stack FON conditions for all players:

$$
\begin{bmatrix}
\nabla_{u_1} L^1 \\
\nabla_{u_2} L^2 \\
\vdots \\
\nabla_{u_n} L^n
\end{bmatrix} = 0
$$

- Solve with Newton's method
- With good implementation, can run at $\sim$ 50 Hz for $\sim$ 6 cars
- Generates human-like interaction strategies
