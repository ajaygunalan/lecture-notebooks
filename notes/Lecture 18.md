# Lecture 18: Iterative Learning Control

## Last Time:
- Hybrid methods for legged locomotion

## Today:
- Iterative Learning Control

## What happens when our model has errors?

- Models are approximate
- Simpler models are often preferred even if they're less accurate
- Feedback (e.g. LQR/MPC) can often compensate for model errors
- Sometimes that isn't enough (e.g. very tight constraints, performance, Safety)

## Several Options:

### 1) Parameter Estimation: Classical "System ID" / "gray-box" modeling
Fit e.g. masses, lengths, etc. in your model from data.

**Pros:**
- Very sample efficient
- Generalizes well
- Interpretable

**Cons:**
- Assumes model structure

### 2) Learn Model
Fit generic function approximator to the full dynamics or residual. Classical "black-box" modeling/System ID

**Pros:**
- Doesn't assume model structure
- Generalizes

**Cons:**
- Not sample efficient. Requires lots of data

**Improve the model**

### 3) Learn a Policy
Standard model-free RL approach: Optimize a function approximation of the controller

**Pros:**
- Makes few assumptions

**Cons:**
- Doesn't generalize
- Not sample efficient. Requires lots of "rollouts"

### 4) Transfer
Assume we have a reference trajectory computed with a nominal model. Improve it with data from the real system.

**Pros:**
- Makes few assumptions
- Very sample efficient

**Cons:**
- Doesn't generalize (task specific)
- Assumes a decent prior model

**Improve the controller**

---

## Iterative Learning Control (ILC)

- Can think of this as a very specialized policy-gradient method on the policy class:

$$u_n(x_n) = \bar{u}_n - K_n(x_n - \bar{x}_n)$$

where:
- $\bar{u}_n$ = reference inputs
- $\bar{x}_n$ = reference states
- $K_n$ = any tracking controller

- We only update $\bar{u}_n$
- Can think of this as SQP where we get the RHS vector from a rollout on the real system.

### Problem Formulation

Assume we have a reference trajectory $\bar{x}, \bar{u}$ that we want to track:

$$\min_{x,u} J = \sum_{n=1}^{N-1} \frac{1}{2}(x_n - \bar{x}_n)^T Q(x_n - \bar{x}_n) + \frac{1}{2}(u_n - \bar{u}_n)^T R(u_n - \bar{u}_n) + \frac{1}{2}(x_N - \bar{x}_N)^T Q(x_N - \bar{x}_N)$$

$$\text{s.t.} \quad x_{n+1} = f(x_n, u_n)$$
$$\text{(other constraints)}$$

### KKT System

The KKT system for this problem looks like:

$$\begin{bmatrix} H & C^T \\ C & 0 \end{bmatrix} \begin{bmatrix} \delta z \\ \lambda \end{bmatrix} = \begin{bmatrix} -\nabla J \\ -C(z) \end{bmatrix}$$

where:

$$\delta z = \begin{bmatrix} \delta x_1 \\ \delta u_1 \\ \vdots \\ \delta x_0 \end{bmatrix}, \quad C(z) = \begin{bmatrix} \vdots \\ f(x_n,u_n) - x_{n+1} \\ \vdots \end{bmatrix}, \quad C = \frac{\partial c}{\partial z}$$

Note: $C(z)$ uses the nominal dynamics model

$$H \approx \begin{bmatrix} Q & & \\ & R & \\ & & Q_N \end{bmatrix} \leftarrow \text{Gauss-Newton Hessian}$$

### Two Important Observations:

1) If we do a rollout on the real system, $C(z) = 0$ always (for the true dynamics)

2) Since we know $J$, given $x_n, u_n$ from a rollout, we can compute $\nabla J$

### Solution Strategy

- Now we have RHS vector from the KKT system
- We also know $H$ from the cost
- We can compute $C$ using $x_n, u_n$ and the nominal model
- However, since our nominal model is approximate and assuming $x_n, u_n$ is already close to $\bar{x}, \bar{u}$, we can just use $C = \frac{\partial f}{\partial z}\bigg|_{\bar{x},\bar{u}}$, which can be computed offline
- Now just solve KKT system for $\delta z$, update: $z \leftarrow z + \delta u$, and repeat
- Can easily add inequality constraints (e.g. torque limits) and solve a QP

---

## ILC Algorithm:

Given nominal $\bar{x}, \bar{u}$

**do:**
1. $x_{1:N}, u_{1:N-1} \leftarrow \text{rollout}(\bar{x}_0, \bar{u})$ on real system
2. $\delta x, \delta u \leftarrow \arg\min J(\delta x, \delta u)$

   This is a QP:
   $$\text{s.t.} \quad \delta x_{n+1} = A_n \delta x_n + B_n \delta u_n$$
   $$u_{\min} \leq u_n \leq u_{\max}$$

3. $\bar{u} \leftarrow \bar{u} + \delta u$
4. **while** $\|x_N - \bar{x}_N\| \geq \text{tol}$ (whatever you care about)

---

## Why Should ILC Work?

- We've already seen approximations of Newton's method (e.g. Gauss-Newton)
- In general, these are called "quasi-Newton" or "inexact Newton" methods. Many variants e.g. BFGS/Newton-CG. Well developed theory.

### For a generic root-finding problem:

$$f(x + \delta x) \approx f(x) + \frac{\partial f}{\partial x} \delta x = 0$$

The term $f(x) + \frac{\partial f}{\partial x} \delta x$ represents the exact Newton step.

- As long as $\delta x$ satisfies:
  $$\|f(x) + \tilde{J} \delta x\| \leq \eta \|f(x)\| \quad \text{for some } \eta < 1$$
  an inexact Newton method will converge

- Convergence is slower than exact Newton
- This means we can use $\tilde{J} \approx \frac{\partial f}{\partial x} + \frac{\partial^2 f}{\partial x^2}$ to compute $\delta x$
