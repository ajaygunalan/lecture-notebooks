# Lecture 13

## Last Time:
- DDP details
- Constraints

## Today:
- Minimum/free-time problems
- Direct Trajectory Optimization
- Direct Collocation
- Sequential Quadratic Programming

---

## Handling Free/Minimum-Time Problems

**min time**

$$
\begin{align}
\min_{x(t), u(t), T_f} \quad & T = \int_0^{T_f} 1 \, dt \\
\text{s.t.} \quad & \dot{x} = f(x, u) \\
& x(T_f) = x_{goal} \\
& u_{min} \leq u \leq u_{max}
\end{align}
$$

- We don't want to change the number of knot points
- Make $h$ (time step) from RK a "control input"
  - $x_{n+1} = f_{RK4}(x_n, u_n, h)$
  - $U = [u_n, h_n]$

- Also want to scale the cost by $h$ e.g.
  $$J(X, U) = \sum_{n} h_n \ell(x_n, u_n) + \ell_f(x_N)$$

- Requires constraints on $h$, otherwise solver can "cheat physics" by making $h$ very large or negative.

- Always nonlinear/nonconvex even if dynamics are linear

---

## Direct Traj Opt

**Basic strategy:** Discretize/"transcribe" continuous-time problem into a standard nonlinear program (NLP):

**"Standard" NLP:**
$$
\begin{align}
\min_x \quad & f(x) \quad \leftarrow \text{ cost function} \\
\text{s.t.} \quad & c(x) = 0 \quad \leftarrow \text{ "dynamics" constraints} \\
& d(x) \leq 0 \quad \leftarrow \text{ other constraints}
\end{align}
$$

- All functions assumed $C^2$ smooth
- Lots of off-the-shelf solvers for large-scale NLP
- Most common: **IPOPT** (free), **SNOPT** (commercial), **KNITRO** (commercial)
- Common solution: **Sequential Quadratic Programming (SQP)**

---

## SQP

**Strategy:** Use 2nd order Taylor expansion of the Lagrangian and linearize $c(x)$, $d(x)$ to approximate the NLP as a QP:

$$
\begin{align}
\min_{\delta x} \quad & f(x) + g^T \delta x + \frac{1}{2} \delta x^T H \delta x \\
\text{s.t.} \quad & c(x) + C \delta x = 0 \\
& d(x) + D \delta x \leq 0
\end{align}
$$

Where:
- $H = \frac{\partial^2 \mathcal{L}}{\partial x^2}$
- $g = \frac{\partial f}{\partial x}$
- $C = \frac{\partial c}{\partial x}$
- $D = \frac{\partial d}{\partial x}$
- $\mathcal{L}(x, \lambda, \mu) = f(x) + \lambda^T c(x) + \mu^T d(x)$

---

## SQP (continued)

- Solve QP to compute primal-dual search direction:
  $$\delta z = [\delta x, \delta \lambda, \delta \mu]$$

- Perform line search with merit function

- With only equality constraints, reduces to Newton method

- Think of SQP as a generalization of Newton's method to handle inequalities

- Can use any QP solver for sub-problems but good implementations typically warm start using previous QP iteration

- For good performance on TrajOpt problems, taking advantage of sparsity in KKT systems is critical.

- If inequalities are convex (e.g. conic) can generalize SQP to **SCP** (sequential convex programming) where inequalities passed directly to the sub-problem solver.

---

## Direct Collocation

- So far we've used explicit RK methods:
  $$\dot{x} = f(x, u) \rightarrow x_{n+1} = f_{RK}(x_n, u_n)$$

- This makes sense if you're doing a rollout

- However in a direct method we're just enforcing dynamics as equality constraints between knot points:
  $$c_n(x_n, u_n, x_{n+1}, u_{n+1}) = 0$$
  $\Rightarrow$ implicit integration is "free"

- Collocation methods use polynomial splines to represent trajectories and enforce dynamics as constraints on spline derivatives.

- Classic DIRCOL algorithm uses cubic splines for states and piecewise linear interpolation for $u(t)$

---

## DIRCOL Spline Approximations

**"Collocation point"**

```
State trajectory (cubic spline):
x(t)
  |     •           •
  |    / \         /
  |   /   \       /
  |  •     •     •
  |__|_____|_____|____> t
     t_n  t_n+½  t_n+1
     "Knot points"

Control trajectory (piecewise linear):
u(t)
  |  •     •     •
  | /|    /|    /|
  |/ |   / |   / |
  |__|___|__|___|___> t
     t_n t_n+1 t_n+2
```

Cubic spline for state:
$$
\begin{align}
x(t) &= c_0 + c_1 t + c_2 t^2 + c_3 t^3 \\
\dot{x}(t) &= c_1 + 2c_2 t + 3c_3 t^2
\end{align}
$$

Linear system relating coefficients to states:
$$
\begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & 1 & 0 & 0 \\
1 & h & h^2 & h^3 \\
0 & 1 & 2h & 3h^2
\end{bmatrix}
\begin{bmatrix}
c_0 \\ c_1 \\ c_2 \\ c_3
\end{bmatrix}
=
\begin{bmatrix}
x_n \\ \dot{x}_n \\ x_{n+1} \\ \dot{x}_{n+1}
\end{bmatrix}
$$

---

## DIRCOL Integration

Evaluate at $t_{n+\frac{1}{2}}$:

$$
\begin{align}
x_{n+\frac{1}{2}} &= x(t_n + h/2) = \frac{1}{2}(x_n + x_{n+1}) + \frac{h}{8}(\dot{x}_n - \dot{x}_{n+1}) \\
&= \frac{1}{2}(x_n + x_{n+1}) + \frac{h}{8}(\underbrace{f(x_n, u_n) - f(x_{n+1}, u_{n+1})}_{\text{continuous-time dynamics}})
\end{align}
$$

$$
\begin{align}
\dot{x}_{n+\frac{1}{2}} &= \dot{x}(t_n + h/2) = -\frac{3}{2h}(x_n - x_{n+1}) - \frac{1}{4}(\dot{x}_n + \dot{x}_{n+1}) \\
&= -\frac{3}{2h}(x_n - x_{n+1}) - \frac{1}{4}(f(x_n, u_n) + f(x_{n+1}, u_{n+1}))
\end{align}
$$

$$u_{n+\frac{1}{2}} = u(t_n + h/2) = \frac{1}{2}(u_n + u_{n+1})$$

We can enforce the dynamics constraints:

$$
\begin{align}
c_n(x_n, u_n, x_{n+1}, u_{n+1}) &= \\
\underbrace{f(x_{n+\frac{1}{2}}, u_{n+\frac{1}{2}})}_{\text{continuous dynamics}} &+ \frac{3}{2h}(x_n - x_{n+1}) + \frac{1}{4}(f(x_n, u_n) + f(x_{n+1}, u_{n+1})) = 0
\end{align}
$$

- Note that only $x_n$, $u_n$ are decision variables (not $x_{n+\frac{1}{2}}$, $u_{n+\frac{1}{2}}$)
- Called **"Hermite-Simpson"** integration
- Achieves 3rd order integration accuracy like RK3
- Requires fewer dynamics calls than explicit RK3!

---

## Comparison: RK3 vs Hermite-Simpson

**Explicit RK3:**
$$
\begin{align}
f_1 &= f(x_n, u_n) \\
f_2 &= f(x_n + \frac{1}{2}hf_1, u_n) \\
f_3 &= f(x_n + 2hf_1 - hf_2, u_n) \\
x_{n+1} &= x_n + \frac{h}{6}(f_1 + 4f_2 + f_3)
\end{align}
$$
$\Rightarrow$ **3 dynamics calls per time step**

**Hermite-Simpson:**
$$
f(x_{n+\frac{1}{2}}, u_{n+\frac{1}{2}}) + \frac{3}{2h}(x_n - x_{n+1}) - \frac{1}{4}(\underbrace{f(x_n, u_n) + f(x_{n+1}, u_{n+1})}_{\text{these get re-used at adjacent steps!}}) = 0
$$
$\Rightarrow$ **Only 2 dynamics calls per time step!**

- Since dynamics calls often dominate total compute cost, this is ~30-50% savings!

---

## Example:
- Acrobot w/DIRCOL
- Warm starting with dynamically infeasible guess can help a lot!
