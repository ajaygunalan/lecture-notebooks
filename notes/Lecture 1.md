# Lecture 1: Continuous-Time Dynamics

## Continuous-Time Dynamics

### Most General Form for a Smooth System

The most general form for a smooth system is:

$$\dot{x} = f(x, u)$$

Where:
- $x$ = state $\in \mathbb{R}^n$ (the "configuration"/"pose" - not always a vector)
- $\dot{x}$ = time derivative of state
- $u$ = input $\in \mathbb{R}^m$ (the "dynamics")

---

### For a Mechanical System

For a mechanical system:

$$x = \begin{bmatrix} q \\ v \end{bmatrix}$$

Where:
- $q$ = "Configuration"/"pose"
- $v$ = "Velocity"

---

### Example: Pendulum

![Pendulum diagram showing angle θ, length l, and mass m]

**Equation of motion:**

$$ml^2 \ddot{\theta} + mgl \sin(\theta) = \tau$$

**State representation:**

$$q = \theta, \quad v = \dot{\theta}, \quad u = \tau$$

$$x = \begin{bmatrix} \theta \\ \dot{\theta} \end{bmatrix} \quad \Rightarrow \quad \dot{x} = \begin{bmatrix} \dot{\theta} \\ -\frac{g}{l} \sin(\theta) + \frac{1}{ml^2}u \end{bmatrix} = f(x,u)$$

**State space:**
- $q \in S^1$ (circle)
- $x \in S^1 \times \mathbb{R}$ (cylinder)

---

## Control-Affine Systems

$$\dot{x} = f_0(x) + B(x)u$$

Where:
- $f_0(x)$ = "drift"
- $B(x)$ = "input Jacobian"

Most systems can be put in this form.

### Pendulum Example:

$$f_0(x) = \begin{bmatrix} \dot{\theta} \\ -\frac{g}{l} \sin(\theta) \end{bmatrix}, \quad B(x) = \begin{bmatrix} 0 \\ \frac{1}{ml^2} \end{bmatrix}$$

---

## Manipulator Dynamics

$$M(q)\dot{v} + C(q,v) = B(q)u + F$$

Where:
- $M(q)$ = inertia matrix
- $C(q,v)$ = dynamic bias (Coriolis + Gravity)
- $B(q)$ = input Jacobian
- $F$ = external forces

This can be rewritten as:

$$\dot{q} = G(q)v$$

$$\dot{x} = f(x,u) = \begin{bmatrix} G(q)v \\ M^{-1}(q)(B(q)u + F - C) \end{bmatrix}$$

### Pendulum Example:

$$M(q) = ml^2, \quad C(q,v) = mgl \sin(\theta), \quad B = I, \quad G = I$$

All mechanical systems can be written this way.

This is just a way of re-writing the Euler-Lagrange equation:

$$L = \frac{1}{2}v^T M(q)v - U(q)$$

Where:
- $\frac{1}{2}v^T M(q)v$ = Kinetic Energy
- $U(q)$ = Potential Energy

---

## Linear Systems

$$\dot{x} = A(t)x + B(t)u$$

- Called **"time invariant"** if $A(t) = A$, $B(t) = B$
- Called **"time varying"** otherwise

### Importance in Control:

- Super important in control
- We often approximate nonlinear systems with linear ones:

$$\dot{x} = f(x,u) \quad \Rightarrow \quad A = \frac{\partial f}{\partial x}, \quad B = \frac{\partial f}{\partial u}$$
