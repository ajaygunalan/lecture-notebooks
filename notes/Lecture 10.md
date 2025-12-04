# Lecture 10: Convexity and Convex Model-Predictive Control

## Last Time:
- Controllability
- Dynamic Programming

## Today:
- Convexity Background
- Convex MPC

---

## Convex Model-Predictive Control

- LQR is very powerful but we often need to reason about constraints
- Often these are simple (e.g. actuator limits)
- Constraints break the Riccati solution, but we can still solve the QP online
- Convex MPC has gotten popular as computers have gotten faster

---

## Background: Convexity

### Convex Set:
- A line connecting any 2 points in the set is fully contained in the set

**Visual representation:**
- Left shape (convex): A line between any two points stays inside
- Right shape (not convex): A line between two points can go outside

### Standard Examples of Convex Sets:
- Linear subspaces: $Ax = b$
- Half space/box/polytope: $Ax \leq b$
- Ellipsoids: $x^T P x \leq 1$, $P \geq 0$
- Cones: $|x_1| = \|x_{2:n}\|_2$
  - "Second-order" cone (like ice cream cone)

---

## Convex Function

- A function $f(x) : \mathbb{R}^n \to \mathbb{R}$ whose epigraph is a convex set

**Visual representation:**
- Left graph: Convex function (bowl-shaped)
- Right graph: Non-convex function (has local minima)

### Standard Examples of Convex Functions:
- Linear: $f(x) = c^T x$
- Quadratic: $f(x) = \frac{1}{2}x^T Q x + q^T x$, $Q \geq 0$
- Norms: $f(x) = |x|$ (any norm)

---

## Convex Optimization Problem
Minimize a convex function over a convex set

### Standard Examples:
- **Linear Program (LP)**: $f(x)$, $c(x)$ both linear
- **Quadratic Program (QP)**: Quadratic $f(x)$, linear $c(x)$
- **Quadratically-Constrained QP (QCQP)**: $"$, ellipsoid $c(x)$
- **Second-order Cone Program (SOCP)**: linear $f(x)$, cone $c(x)$

### Key Properties:
- Convex optimization problems don't have any spurious local optima that satisfy KKT
  - If you find a local KKT solution, you have the solution
- Practically, Newton's method converges really fast and reliably (5-10 iterations max)
  - Can bound solution time for real-time control

---

## Convex MPC

Think "constrained LQR"

### From Dynamic Programming:
Remember from DP, if we have a cost-to-go function $V(x)$, we can get $u$ by solving a one-step problem:

$$u_n = \arg\min_{u} \ell(x,u) + V_{n+1}(f(x,u))$$

$$= \arg\min_{u} \frac{1}{2}u^T R u + (Ax + Bu)^T P_{n+1}(Ax + Bu)$$

- We can add constraints on $u$ to this one-step problem
- But this will perform poorly because $V(x)$ was computed without constraints

### Multi-step Approach:
There's no reason I can't add more steps to the one-step problem:

$$\min_{x_{1:H}, u_{1:H-1}} \sum_{n=1}^{H-1} \frac{1}{2}x_n^T Q x_n + \frac{1}{2}u_n^T R u_n + x_H^T P_H x_H$$

where $x_H^T P_H x_H$ is the LQR cost-to-go

- $H = N$ is called "Horizon"

### Properties:
- With no additional constraints, MPC ("receding horizon") exactly matches LQR for any $H$
- **Intuition**: Explicit constrained optimization over first $H$ steps gets the state close enough to the reference that constraints are no longer active and LQR cost-to-go is valid further into the future

### In General:
- A good approximation of $V(x)$ is important for good performance
- Better $V(x)$ $\Rightarrow$ shorter horizon
- Longer $H$ $\Rightarrow$ less reliance on $V(x)$

---

## Example: Planar Quadrotor

### System Description:
```
     u₁              u₂
      ↑               ↑
      |     m,J       |      y
      └──────⚫──────┘       ↑ θ
           ℓ                 └──→ x
```

### Dynamics:
$$m\ddot{x} = -(u_1 + u_2)\sin(\theta)$$

$$m\ddot{y} = (u_1 + u_2)\cos(\theta) - mg$$

$$J\ddot{\theta} = \frac{\ell}{2}(u_2 - u_1)$$

### Linearize about hover:
$$u_1 = u_2 = \frac{1}{2}mg$$

This gives:

$$\Delta\ddot{x} \approx -g\Delta\theta$$

$$\Delta\ddot{y} \approx \frac{1}{m}(\Delta u_1 + \Delta u_2)$$

$$\Delta\ddot{\theta} \approx \frac{1}{J}\frac{\ell}{2}(\Delta u_2 - \Delta u_1)$$

### State-Space Form:
$$\begin{bmatrix} \Delta\dot{x} \\ \Delta\dot{y} \\ \Delta\dot{\theta} \\ \Delta\ddot{x} \\ \Delta\ddot{y} \\ \Delta\ddot{\theta} \end{bmatrix} = \begin{bmatrix} 0 & 0 & 0 & 1 & 0 & 0 \\ 0 & 0 & 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 0 & 0 & 1 \\ 0 & 0 & -g & 0 & 0 & 0 \\ 0 & 0 & 0 & 0 & 0 & 0 \\ 0 & 0 & 0 & 0 & 0 & 0 \end{bmatrix} \begin{bmatrix} \Delta x \\ \Delta y \\ \Delta\theta \\ \Delta\dot{x} \\ \Delta\dot{y} \\ \Delta\dot{\theta} \end{bmatrix} + \begin{bmatrix} 0 & 0 \\ 0 & 0 \\ 0 & 0 \\ 0 & 0 \\ \frac{1}{m} & \frac{1}{m} \\ -\frac{\ell}{J} & \frac{\ell}{J} \end{bmatrix} \begin{bmatrix} \Delta u_1 \\ \Delta u_2 \end{bmatrix}$$

$$\dot{x} = Ax + Bu$$

### MPC Cost Function:
$$J = \sum_{n=1}^{H-1} \frac{1}{2}(x_n - x_{ref})^T Q(x_n - x_{ref}) + \frac{1}{2}u_n^T R u_n + \frac{1}{2}(x_H - x_{ref})^T P_H(x_H - x_{ref})$$

---

## Summary

Convex MPC combines the power of LQR with the ability to handle constraints by:
1. Using a receding horizon optimization approach
2. Solving a constrained convex optimization problem at each time step
3. Using an LQR-based terminal cost to approximate the infinite-horizon cost-to-go
4. Leveraging fast, reliable convex optimization solvers for real-time control
