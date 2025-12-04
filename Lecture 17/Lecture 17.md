# Lecture 17

## Last Time:
- LQR with Quaternions

## Today:
- Contact dynamics
- Hybrid systems modeling
- Trajopt for legged systems

---

## Contact Dynamics

### Imagine a bouncing ball

```
z| ⚪
 |
 |________→ x
```

- In the air, dynamics described by smooth ODE: $m\ddot{z} = -mg$
- When the ball hits the ground:

```
     z|              z|
      |  V⁻          |  V⁺
 ⚪   |              |
////////////////////////////////////→ t
```

- Because of discontinuities, can't write down dynamics around impact as an ODE.

---

## Two Options:

### 1) Event-based/hybrid
Integrate ODE while checking for contact events using a "guard function" (e.g. $z = 0$). When contact happens, execute "jump map" that models the discontinuity, then continue integrating ODE.

### 2) Time-stepping/contact-implicit
Solve a constrained optimization problem at each timestep that enforces no interpenetration between objects ($\phi \geq 0$) by solving for contact forces jointly with state (HW2 brick problem)

### Comparison:

**Both are widely used and have pros/cons**

**In control, hybrid formulation is easy to implement:**
- Works with standard algorithms (e.g. DIRCOL)
- **Downsides:** requires enumeration of all possible "contact modes" and/or pre-specification of "mode sequence"
- Very successful in locomotion

**Contact-implicit method:**
- Doesn't need mode sequence pre-specified
- But the optimization problems are much harder

---

## Falling Brick Two Ways:

```
    V₀→
[brick]
    ↘
/////////////////////////
```

### 1) Time-Stepping Method:

**Dynamics:**
$$m\dot{v} = -mg + J^T \lambda$$
where $\lambda$ is the contact force and $J$ is the Contact Jacobian

**State vectors:**
$$g = \begin{bmatrix} 0 \\ 9.8 \end{bmatrix}, \quad \chi = \begin{bmatrix} z \\ v \end{bmatrix}$$

**Signed-distance function:**
$$\phi(q) = \begin{bmatrix} 0 & 1 \end{bmatrix} \begin{bmatrix} q_x \\ q_z \end{bmatrix}$$

**Backward Euler integration:**
$$m\frac{V_{n+1} - V_n}{h} = -mg + J^T \lambda_n$$

$$q_{n+1} = q_n + h V_{n+1}$$

$$\phi(q_{n+1}) \geq 0$$

$$\lambda_n \geq 0 \quad \text{(only pushing, no pulling)}$$

$$\phi(q_{n+1}) \lambda_n = 0 \quad \text{(no force unless you're in contact)}$$

**This is a QP in disguise (KKT conditions)!**

$$\min_{V_{n+1}} \frac{1}{2} m V_{n+1}^T V_{n+1} + m V_{n+1}^T(hg - V_n)$$

$$\text{s.t. } J(q_n + hV_{n+1}) \geq 0$$

**Characteristics:**
- Exact impact time is not resolved (only step)
- Contact forces ($\lambda_n$) are explicitly computed
- Doesn't generalize to higher-order integration (e.g. RK4), need to take small steps
- Widely used: PyBullet, Dart, Gazebo, etc.
- **Key issue for trajopt:** complementarity condition is non-smooth

---

### 2) Hybrid Method:

**Smooth vector field (dynamics):**
$$\dot{x} = f(x) \Rightarrow \begin{bmatrix} \dot{z} \\ \dot{v} \end{bmatrix} = \begin{bmatrix} v \\ -g \end{bmatrix} \quad \text{"smooth vector field" (dynamics)}$$

**Guard function:**
$$\phi(x) \geq 0 \quad \text{"guard function"}$$

**Jump map:**
$$x^+ = g(x) = \begin{bmatrix} z_x \\ z_y \\ v_x \\ 0 \end{bmatrix} \quad \text{"jump map"}$$
(zero out vertical velocity)

**Pseudocode:**
```
while t < t_final
    if φ(x) ≥ 0
        ẋ = f(x)        ← e.g. RK4
    else (φ = 0)
        x⁺ = g(x)       ← jump map simulates non-smooth impact
    end
end
```

**Characteristics:**
- Precise impact time
- Contact forces not explicitly computed
- Can use high accuracy integrators
- Widely used for trajopt/MPC
- **Key insight:** If we know impact times a-priori, we don't need guard function and we can just deal with $f(x)$ and $g(x)$, which are differentiable

---

## Hybrid Traj Opt for Legged Robots:

### One-legged hopper:

```
  (M)^T
    |
   F|
  ⚪|
/////////////////////////
```

**State:**
$$x = \begin{bmatrix} r_b \\ r_f \\ v_b \\ v_f \end{bmatrix} \in \mathbb{R}^8, \quad u = \begin{bmatrix} F \\ \theta \end{bmatrix} \in \mathbb{R}^2$$

### Define jump map to transition between modes:

**Impact:**
$$x^+ = g_{21}(x) = \begin{bmatrix} r_b \\ r_f \\ v_b \\ 0 \end{bmatrix} \quad \text{(impact)}$$
(zero out foot velocity at impact)

**Lift-off:**
$$x^+ = g_{12}(x) = x \quad \text{for this problem (lift-off)}$$

### Assign modes to alternating groups of knot points by enforcing appropriate constraints:

**Mode 1 (Contact):**
```
for k = 1:N₁
    x_{k+1} = f₁(x_k, u_k)
    φ(x_k) = 0
end
```

**Mode 2 (Flight):**
```
for k = (N₁+1):N₂
    x_{k+1} = f₂(x_k, u_k)
    φ(x_k) > 0
end
```

**Transition:**
$$x_{N_2} = g_{21}(x_{N_2-1})$$
$$\phi(x_{N_2}) = 0$$

(repeats)...

---

**End of Lecture 17**
