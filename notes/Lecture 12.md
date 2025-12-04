# Lecture 12

## Last Time:
- Convex MPC Examples
- Nonlinear Traj Opt
- DDP/iLQR

## Today:
- DDP details + extensions
- Constraints

---

## DDP Recap:

**Solve the unconstrained traj Opt problem:**

$$
\min_{x_{1:N}, u_{1:N-1}} J = \sum_{n=1}^{N-1} \ell(x_n, u_n) + \ell_N(x_N)
$$

$$
\text{s.t. } x_{n+1} = f(x_n, u_n)
$$

**Backward Pass:**

$$
V_n(x+\delta x) \approx V(x) + p_n^T\delta x + \frac{1}{2}\delta x^T P_n \delta x
$$

$$
P_N = \nabla^2 \ell_N(x), \quad p_N = \nabla \ell_N(x)
$$

$$
V_{n-1}(x+\delta x) = \min_{\delta u} S(x+\delta x, u+\delta u)
$$

$$
\Rightarrow \begin{cases}
\delta u_{n-1} = -d_{n-1} - K_{n-1}\delta x_{n-1} \\
P_{n-1} = G_{xx} + K^T G_{uu}K - G_{xu}K - K^T G_{ux} \\
p_{n-1} = g_x - K^T g_u + K^T G_{uu}d - G_{xu}d
\end{cases}
$$

---

## Forward Rollout

$$
\Delta J = 0
$$
$$
x_1' = x_1
$$

**for** $k = 1:N-1$

$$
u_n' = u - \alpha d_n - K_n(x_n' - x_n)
$$
$$
x_{n+1}' = f(x_n', u_n')
$$
$$
\Delta J \leftarrow \Delta J + \alpha g_u^T d_n
$$

**end**

**Line Search:**

```
α = 1

do:
    x', u', ΔJ = rollout(x, u, d, K, α)
    α ← cα

while J(x', u') ≤ J(x, u) - bΔJ

x, u ← x', u'
```

- **repeat until** $|d|_\infty < \text{tol}$
- **Armijo parameters:** $c \sim \frac{1}{2}$, $b \sim 10^{-4} - 0.01$

---

## Examples:

- Cartpole + acrobot swing up
- DDP can converge in fewer iterations but iLQR often wins in wall-clock time
- Problems are nonconvex $\Rightarrow$ can land in different local optima depending on initial guess

## Regularization

- Just like standard Newton, $V(x)$ and/or $S(x,u)$ Hessians can become indefinite in backward pass
- Regularization is definitely necessary for DDP, often a good idea with iLQR as well.
- Many options for regularizing:
  - Add a multiple of identity to $\nabla^2 S(x,u)$
  - Regularize $P_n$ or $G_{xx}$ as needed in the backward pass
- **Regularize just** $G_{uu} = \nabla_u^2 S(x,u)$ **(this is the only matrix you have to invert):**

  $$
  d = G_{uu}^{-1}g_u, \quad K = G_{uu}^{-1}G_{ux}
  $$

- This last one is good for iLQR but not DDP
- Regularization should not be required for iLQR but can be necessary due to floating-point error.

---

## DDP Notes:

**Positives:**
+ Can be very fast (iterations + wall-clock)
+ One of the most efficient methods due to exploitation of DP structure
+ Always dynamically feasible due to forward rollout $\Rightarrow$ can always execute on robot
+ Comes with TVLQR tracking controller for free $\Rightarrow$ can be very effective for online use

**Negatives:**
- Does not naturally handle constraints
- Does not support infeasible initial guess for state trajectory due to forward rollout. Bad for "maze" or "bug trap" problems.
- Can suffer from numerical ill-conditioning on long trajectories.

---

## Handling Constraints in DDP

- Many options depending on type of constraint
- Torque limits can be handled with a "squashing function" e.g. $\tanh(u)$:

```
      u̅
  umax ┼───────────
       │      ╱
───────┼────╱──────────> u
       │  ╱
 umin  ┼╱

  u̅ = umax tanh(u/umax)
```

- Effective, but adds nonlinearity and may need more iterations
- Better option: Solve box-constrained QP in the backward pass!

  $$
  \delta u = \arg\min_{\delta u} S(x+\delta x, u+\delta u)
  $$
  $$
  \text{s.t. } u_{\min} \leq u+\delta u \leq u_{\max}
  $$

- State constraints are harder. Often penalties are added to cost function. Can cause ill-conditioning.
- Better option: Wrap entire DDP algorithm in an Augmented Lagrangian method.
- AL method adds linear (multiplier) and quadratic (penalty) terms to the cost $\Rightarrow$ Fits into DDP nicely.
