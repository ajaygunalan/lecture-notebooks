# Lecture 7

## Last Time:
- Regularization + Duality
- Merit Functions + line search
- Control History

## Today:
- Deterministic Optimal Control
- Pontryagin
- LQR

---

## Deterministic Optimal Control

### Continuous time:

$$\min_{x(t), u(t)} J(x_0, u(t)) = \int_{t_0}^{t_f} \ell(x(t), u(t)) \, dt + \ell_F(x(t_f))$$

Where:
- **Stage cost**: $\ell(x(t), u(t))$
- **Terminal cost**: $\ell_F(x(t_f))$

Subject to:
$$\dot{x}(t) = f(x(t), u(t))$$
(dynamics constraint)

(possibly other constraints)

**Notes:**
- This is an "infinite-dimensional" optimization problem
- Solutions are open-loop trajectories
- There are a handful of problems with analytic solutions but not many
- We will focus on the discrete-time setting

---

## Discrete Time:

$$\min_{X_{1:N}, U_{1:N-1}} J(X_N, U_N) = \sum_{k=1}^{N-1} \ell(x_k, u_k) + \ell_F(x_N)$$

Subject to:
$$x_{k+1} = f(x_k, u_k)$$

$$u_{\min} \leq u_k \leq u_{\max} \quad \forall k$$ (torque limits)

$$c(x_k) \leq 0 \quad \forall k$$ (obstacle/safety constraints)

**Notes:**
- This is a finite-dimensional problem
- Samples $x_k, u_k$ are often called "knot points"
- Continuous $\rightarrow$ discrete uses integration (e.g., Runge-Kutta)
- Discrete $\rightarrow$ continuous using interpolation

---

## Pontryagin's Minimum Principle

- Also called "Maximum Principle" if you maximize a reward
- First-order necessary conditions for a deterministic optimal control problem
- In discrete time, special case of KKT conditions

### Given:

$$\min_{x_{1:N}, u_{1:N-1}} \sum_{k=1}^{N-1} \ell(x_k, u_k) + \ell_F(x_N)$$

Subject to:
$$x_{k+1} = f(x_k, u_k)$$

We can form the Lagrangian:

$$L = \sum_{k=1}^{N-1} \ell(x_k, u_k) + \lambda_{k+1}^T (f(x_k, u_k) - x_{k+1}) + \ell_F(x_N)$$

### Hamiltonian Formulation

This result is usually stated in terms of the "Hamiltonian":

$$H(x, u, \lambda) = \ell(x, u) + \lambda^T f(x, u)$$

Plug $H$ into $L$:

$$L = H(x_1, u_1, \lambda_2) + \left[\sum_{k=2}^{N-1} H(x_k, u_k, \lambda_{k+1}) - \lambda_k^T x_k\right] + \ell_F(x_N) - \lambda_N^T x_N$$

*Note: change to indexing*

### Taking derivatives w.r.t. $x$ and $\lambda$:

$$\frac{\partial L}{\partial \lambda_k} = \frac{\partial H}{\partial \lambda_k} - x_{k+1} = f(x_k, u_k) - x_{k+1} = 0$$

$$\frac{\partial L}{\partial x_k} = \frac{\partial H}{\partial x_k} - \lambda_k^T = -\frac{\partial \ell}{\partial x_k} + \lambda_{k+1}^T \frac{\partial f}{\partial x_k} - \lambda_k^T = 0$$

$$\frac{\partial L}{\partial x_N} = \frac{\partial \ell_F}{\partial x_N} - \lambda_N^T = 0$$

For $u$, we write the min explicitly to handle torque limits:

$$u_k = \arg\min_u H(x_k, u, \lambda_{k+1})$$

Subject to: $u \in \mathcal{U}$ (shorthand for "in feasible set", e.g., $u_{\min} \leq u \leq u_{\max}$)

---

## Summary:

**Discrete Time:**

$$x_{k+1} = \nabla_\lambda H(x_k, u_k, \lambda_{k+1})$$

$$\lambda_k = \nabla_x H(x_k, u_k, \lambda_{k+1})$$

$$u_k = \arg\min_u H(x_k, u, \lambda_{k+1})$$
subject to $u \in \mathcal{U}$

$$\lambda_N = \frac{\partial \ell_F}{\partial x_N}$$

**Continuous Time:**

$$\dot{x} = \nabla_\lambda H(x, u, \lambda)$$

$$-\dot{\lambda} = \nabla_x H(x, u, \lambda)$$

$$u = \arg\min_{\tilde{u}} H(x, \tilde{u}, \lambda)$$
subject to $\tilde{u} \in \mathcal{U}$

$$\lambda(t_f) = \frac{\partial \ell_F}{\partial x}$$

---

## Some Notes:

- Historically many algorithms were based on integrating ODEs forward + backward to do gradient descent
- These are called "indirect" and/or "shooting" methods
- In continuous time $\lambda(t)$ is called "costate" trajectory
- These methods have largely fallen out of favor as computers have improved

---

## LQR Problem

$$\min_{x_{1:N}, u_{1:N-1}} J = \sum_{k=1}^{N-1} \frac{1}{2}x_k^T Q_k x_k + \frac{1}{2}u_k^T R_k u_k + \frac{1}{2}x_N^T Q_N x_N$$

Subject to:
$$x_{k+1} = A_k x_k + B_k u_k$$

$$Q \geq 0, \quad R > 0$$

**Properties:**
- Can (locally) approximate many nonlinear problems
- Computationally tractable
- Many extensions e.g., infinite horizon, stochastic, etc.
- "Time invariant" if $A_k = A$, $B_k = B$, $Q_k = Q$, $R_k = R$ $\forall k$
- "Time varying" otherwise

---

## LQR with Indirect Shooting:

$$x_{k+1} = \nabla_\lambda H(x_k, u_k, \lambda_{k+1}) = A x_k + B u_k$$

$$\lambda_k = \nabla_x H(x_k, u_k, \lambda_{k+1}) = Q x_k + A^T \lambda_{k+1}$$

$$\lambda_N = Q_N x_N$$

$$u_k = \nabla_u H(x_k, u_k, \lambda_{k+1}) = 0 \Rightarrow -R^{-1} B^T \lambda_{k+1}$$

### Procedure:

1) Start with initial guess $u_{1:N-1}$
2) Simulate/rollout to get $x_{1:N}$
3) Backward pass to get $\lambda$, $\delta u$
4) Rollout with line search on $\delta u$
5) Go to 3 until convergence

---

## Example: "Double Integrator"

$$\dot{x} = \begin{bmatrix} \dot{q} \\ \ddot{q} \end{bmatrix} = \begin{bmatrix} 0 & 1 \\ 0 & 0 \end{bmatrix} \begin{bmatrix} q \\ \dot{q} \end{bmatrix} + \begin{bmatrix} 0 \\ 1 \end{bmatrix} u$$

Where:
- $q$ = position
- $\dot{q}$ = velocity
- $u$ = force (acceleration)

Think of this as a brick sliding on ice (no friction)

**Discrete version:**

$$x_{k+1} = \begin{bmatrix} 1 & h \\ 0 & 1 \end{bmatrix} \begin{bmatrix} q_k \\ \dot{q}_k \end{bmatrix} + \begin{bmatrix} \frac{1}{2}h^2 \\ h \end{bmatrix} u_k$$

Where $h$ = time step
