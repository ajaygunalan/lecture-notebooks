# Lecture 5: Inequality-Constrained Minimization

## Last Time:
- Minimization with Equality Constraints

## Today:
- Inequality Constraints
- Inequality-Constrained Minimization

---

## Inequality-Constrained Minimization

**Problem:**
$$
\begin{align}
\min_{x} \quad & f(x) \\
\text{s.t.} \quad & c(x) \geq 0
\end{align}
$$

### KKT Conditions:

1. **Stationarity:** $\nabla f - \left(\frac{\partial c}{\partial x}\right)^T \lambda = 0$
2. **Primal feasibility:** $c(x) \geq 0$
3. **Dual feasibility:** $\lambda \geq 0$
4. **Complementarity:** $\lambda \circ c(x) = 0$

**Note:** Unlike the equality case, we can't directly solve KKT conditions with Newton.

---

## Lots of Solution Methods:

### ★ Active Set:
- Switch inequality constraints on/off and solve equality-constrained problem
- Works well if you can guess active set well

### ★ Penalty Method:
- Replace constraints with cost terms that penalize violation
- Transform:
$$
\min_{x} f(x) \text{ s.t. } c(x) \geq 0 \quad \rightarrow \quad \min_{x} f(x) + \rho[\min(0, c(x))]^2
$$

- **Pros:**
  - Easy to implement
- **Cons:**
  - Has issues with ill-conditioning (have to crank $\rho \rightarrow \infty$)
  - Can't solve to high accuracy
- **Popular fix:** Estimate $\lambda$ from penalty at each iteration $\rightarrow$ converge with finite $\rho$. Called "Augmented Lagrangian" (also closely related to ADMM)

---

## ★ Interior-Point/Barrier Methods

- Replace inequalities with barrier function in objective:
$$
\min_{x} f(x) \text{ s.t. } x \geq 0 \quad \rightarrow \quad \min_{x} f(x) - \rho \log(x)
$$

- The $-\log(x)$ function acts as a barrier

**Properties:**
- Gold standard for convex problems
- Fast convergence with Newton
- Strong theoretical properties
- Used in IPOPT

### Problem with Standard Interior Point Methods:
$$
\min_{x} f(x) - \rho \log(x) \text{ s.t. } x \geq 0
$$

FOC: $\frac{\partial f}{\partial x} - \frac{\rho}{x} = 0$

This "primal" FON condition blows up as $x \rightarrow 0$.

**Solution:** We can fix this with the "primal-dual trick"

---

## Primal-Dual Interior Point Method

- Introduce new variable: $\lambda = \frac{\rho}{x} \Rightarrow x\lambda = \rho$
$$
\begin{cases}
\frac{\partial f}{\partial x} - \lambda = 0 \\
x \cdot \lambda = \rho
\end{cases}
$$
  This gives us **relaxed complementarity** from KKT!

- Converges to exact KKT solution as $\rho \rightarrow 0$
- We lower $\rho$ gradually as solver converges from $\rho \approx 1$ to $\rho \approx 10^{-6}$
- **Note:** We still need to enforce $x \geq 0$ and $\lambda \geq 0$ (with line search)
- We will use another approach...

---

## Log-Domain Interior-Point Method:

**More general case:**
$$
\begin{align}
\min_{x} \quad & f(x) \\
\text{s.t.} \quad & c(x) \geq 0
\end{align}
$$

### Simplify by introducing a "slack variable":
$$
\begin{align}
\min_{x,s} \quad & f(x) \\
\text{s.t.} \quad & c(x) - s = 0 \\
& s \geq 0
\end{align}
\quad \rightarrow \quad
\begin{align}
\min_{x,s} \quad & f(x) - \rho \log(s) \\
\text{s.t.} \quad & c(x) - s = 0
\end{align}
$$

### Lagrangian:
$$
L(x, s, \lambda) = f(x) - \rho \log(s) - \lambda^T(c(x) - s)
$$

---

## FON Conditions:

$$
\begin{align}
\nabla_x L &= \nabla f - \left(\frac{\partial c}{\partial x}\right)^T \lambda = 0 \\
\nabla_s L &= -\frac{\rho}{s} + \lambda = 0 \quad \Rightarrow \quad s \circ \lambda = \rho \quad \text{(relaxed complementarity)} \\
\nabla_\lambda L &= s - c(x) = 0
\end{align}
$$

### To ensure $s \geq 0$ and $\lambda \geq 0$, introduce change of variables:
$$
s = \sqrt{\rho} \, e^{\sigma} \quad \Rightarrow \quad \lambda = \sqrt{\rho} \, e^{-\sigma}
$$

- Now (relaxed) complementarity is always satisfied!

### Plug back into FON conditions:
$$
\begin{align}
\nabla f - \left(\frac{\partial c}{\partial x}\right)^T \sqrt{\rho} \, e^{-\sigma} &= 0 \\
c(x) - \sqrt{\rho} \, e^{\sigma} &= 0
\end{align}
$$

**We can solve these with (Gauss) Newton:**
$$
\begin{bmatrix}
H & \sqrt{\rho} \, C^T e^{-\sigma} \\
C & -\sqrt{\rho} \, e^{\sigma}
\end{bmatrix}
\begin{bmatrix}
\delta x \\
\delta \sigma
\end{bmatrix}
=
\begin{bmatrix}
-\nabla f + C^T \sqrt{\rho} \, e^{-\sigma} \\
-c(x) + \sqrt{\rho} \, e^{\sigma}
\end{bmatrix}
$$

where $C = \frac{\partial c}{\partial x}$ is the constraint Jacobian.

---

## ★ Example: Quadratic Program

$$
\begin{align}
\min_{x} \quad & \frac{1}{2} x^T Q x + q^T x, \quad Q \geq 0 \\
\text{s.t.} \quad & Ax = b \\
& Cx \geq d
\end{align}
$$

- Super useful in Control
- Can be solved very fast ($\sim O(n^2)$)
