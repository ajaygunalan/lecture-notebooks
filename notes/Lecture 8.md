# Lecture 8

## Last Time:
- Deterministic Optimal Control
- Pontryagin
- Indirect Shooting

## Today:
- LQR Problem
- LQR as a QP
- Riccati Recursion

---

## LQR Problem

**Minimize:**

$$\min_{x_{0:N}, u_{0:N-1}} J = \sum_{n=1}^{N-1} \frac{1}{2}x_n^T Q_n x_n + \frac{1}{2}u_n^T R_n u_n + \frac{1}{2}x_N^T Q_N x_N$$

**Subject to:**

$$x_{n+1} = A_n x_n + B_n u_n$$

$$Q_n \geq 0, \quad R_n > 0$$

### Example: "Double Integrator"

**Continuous-time dynamics:**

$$\dot{x} = \begin{bmatrix} \dot{p} \\ \dot{v} \end{bmatrix} = \begin{bmatrix} 0 & 1 \\ 0 & 0 \end{bmatrix} \begin{bmatrix} p \\ v \end{bmatrix} + \begin{bmatrix} 0 \\ 1 \end{bmatrix} u$$

where:
- $p$ = Position
- $v$ = Velocity
- $u$ = Force (acceleration)

Think of this as a brick sliding on ice (no friction)

**Discrete-time version with time step $h$:**

$$x_{n+1} = \begin{bmatrix} 1 & h \\ 0 & 1 \end{bmatrix} \begin{bmatrix} p_n \\ v_n \end{bmatrix} + \begin{bmatrix} \frac{1}{2}h^2 \\ h \end{bmatrix} u_n$$

---

## LQR as a QP

**Assumptions:**
- Assume $x_1$ (initial state) is given (not a decision variable)

**Define decision variable:**

$$z = \begin{bmatrix} u_1 \\ x_2 \\ u_2 \\ x_3 \\ \vdots \\ x_N \end{bmatrix}$$

**Define $H$ matrix:**

$$H = \begin{bmatrix} R_1 & & & & O \\ & Q_2 & & & \\ & & R_2 & & \\ & & & \ddots & \\ O & & & & Q_N \end{bmatrix}$$

such that $J = \frac{1}{2}z^T H z$

**Define $C$ and $d$:**

$$\begin{bmatrix} B_1 & -I & 0 & \cdots & 0 \\ 0 & A_2 & B_2 & -I & 0 & \cdots & 0 \\ & & & & \ddots & \\ & & & A_{N-1} & B_{N-1} & -I \end{bmatrix} \begin{bmatrix} u_1 \\ x_2 \\ u_2 \\ \vdots \\ x_N \end{bmatrix} = \begin{bmatrix} -A_1 x_1 \\ 0 \\ \vdots \\ 0 \end{bmatrix}$$

$$\Rightarrow Cz = d$$

---

## Standard QP Form

Now we can write LQR as a standard QP:

$$\min_z \frac{1}{2}z^T H z$$

$$\text{s.t. } Cz = d$$

**The Lagrangian of this QP is:**

$$L(z, \lambda) = \frac{1}{2}z^T H z + \lambda^T[Cz - d]$$

**KKT Conditions:**

$$\nabla_z L = Hz + C^T \lambda = 0$$

$$\nabla_\lambda L = Cz - d = 0$$

$$\Rightarrow \begin{bmatrix} H & C^T \\ C & 0 \end{bmatrix} \begin{bmatrix} z \\ \lambda \end{bmatrix} = \begin{bmatrix} 0 \\ d \end{bmatrix}$$

**We get the exact solution by solving one linear system!**

### Example:
- Much better than shooting!

---

## A Closer Look at the LQR QP

**Structure:**
- The KKT system for LQR is very sparse (lots of zeros) and has lots of structure

**The KKT system:**

```
┌                                               ┐┌      ┐   ┌        ┐
│ R                    B^T                      ││  u₁  │   │   0    │
│   Q                  -I   A^T                 ││  x₂  │   │   0    │
│     R                      B^T                ││  u₂  │   │   0    │
│       Q                    -I   A^T           ││  x₃  │   │   0    │
│         R                        B^T          ││  u₃  │ = │   0    │
│           Q_N                    -I           ││  x₄  │   │   0    │
│ B  -I                         O               ││  λ₂  │   │ A₁x₁   │
│      A   B  -I                                ││  λ₃  │   │   0    │
│              A   B  -I                        ││  λ₄  │   │   0    │
└                                               ┘└      ┘   └        ┘
```

**Analyzing the structure (working backwards from final time step):**

From the blue circled equation:
$$Q_N x_4 - \lambda_4 = 0 \Rightarrow \lambda_4 = Q_N x_4$$

From the red circled equation:
$$R u_3 + B^T \lambda_4 = R u_3 + B^T Q_N x_4 = 0$$

Plug in dynamics for $x_4$:
$$\Rightarrow R u_3 + B^T Q_N(A x_3 + B u_3) = 0$$

$$\Rightarrow u_3 = -(R + B^T Q_N B)^{-1} B^T Q_N A x_3$$

Define the feedback gain matrix:
$$u_3 = -K_3 x_3$$

where
$$K_3 = (R + B^T Q_N B)^{-1} B^T Q_N A$$

From the green circled equation:
$$Q x_3 - \lambda_3 + A^T \lambda_4 = 0$$

Plug in $\lambda_4$:
$$Q x_3 - \lambda_3 + A^T Q_N x_4 = 0$$

Plug in dynamics:
$$Q x_3 - \lambda_3 + A^T Q_N(A x_3 + B u_3) = 0$$

Plug in $u_3 = -K_3 x_3$:
$$\Rightarrow \lambda_3 = (Q + A^T Q_N(A - B K_3))x_3$$

Define the cost-to-go matrix:
$$\lambda_3 = P_3 x_3$$

where
$$P_3 = Q + A^T Q_N(A - B K_3)$$

---

## Riccati Recursion

Now we have a recursion for $K$ and $P$:

$$P_N = Q_N$$

$$K_n = (R + B^T P_{n+1} B)^{-1} B^T P_{n+1} A$$

$$P_n = Q + A^T P_{n+1}(A - B K_n)$$

**This is called the Riccati equation/recursion**

We can solve the QP by doing a **backward Riccati pass** followed by a **forward rollout** to compute $x_{1:N}$ and $u_{1:N}$.

**Complexity comparison:**

- **General (dense) QP has complexity:** $O([N(n+m)]^3)$
  - $N$ = horizon
  - $n$ = state dim
  - $m$ = control dim

- **Riccati solution is:** $O(N(n+m)^3)$

### Example:
- Riccati exactly matches QP
- Feedback policy lets us change $x_0$ and reject noise/disturbances

---

## Infinite-Horizon LQR

- For time-invariant LQR, $K$ and $P$ converge to constants
- For stabilization problems we usually use constant $K$

**Backward recursion for $P$:**

$$K_n = (R + B^T P_{n+1} B)^{-1} B^T P_{n+1} A$$

$$P_n = Q + A^T P_{n+1}(A - B K_n)$$

**Infinite-horizon limit:** $P_{n+1} = P_n = P_{\infty}$

$\Rightarrow$ Solve as a root-finding/fixed-point problem

Julia/MATLAB/Python "dare" function does this for you
