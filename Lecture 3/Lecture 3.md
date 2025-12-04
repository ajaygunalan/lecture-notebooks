# Lecture 3

## Last Time:
- Stability
- Discrete-time Simulation
- Forward/Backward Euler
- RK4

## Today:
- Notation
- Root Finding
- Minimization

---

## Some Notation:

Given $f(x): \mathbb{R}^n \to \mathbb{R}$

$\frac{\partial f}{\partial x} \in \mathbb{R}^{1 \times n}$ is a row vector

- This is because $\frac{\partial f}{\partial x}$ is the linear operator mapping $\partial x$ into $\partial f$:

$$f(x+\partial x) \approx f(x) + \frac{\partial f}{\partial x}\partial x$$

Similarly $g(y): \mathbb{R}^m \to \mathbb{R}^n$ because:

$$g(y+\partial y) \approx g(y) + \frac{\partial g}{\partial y}\partial y$$

These conventions make the chain rule work:

$$f(g(y+\partial y)) \approx f(g(y)) + \left.\frac{\partial f}{\partial x}\right|_{g(y)} \left.\frac{\partial g}{\partial y}\right|_y \partial y$$

For convenience, we will define:

- $\nabla f(x) = \left(\frac{\partial f}{\partial x}\right)^T \in \mathbb{R}^{n \times 1}$ (column vector)
- $\nabla^2 f(x) = \frac{\partial}{\partial x}(\nabla f(x)) \approx \frac{\partial^2 f}{\partial x^2} \in \mathbb{R}^{n \times n}$

$$f(x+\partial x) \approx f(x) + \frac{\partial f}{\partial x}\partial x + \frac{1}{2}\partial x^T \frac{\partial^2 f}{\partial x^2}\partial x$$

---

## Root Finding:

Given $f(x)$, find $x^*$ such that $f(x^*) = 0$

**Example:** equilibrium of a continuous-time dynamics

**Closely related:** fixed point: $f(x^*) = x^*$
(equilibrium of discrete-time dynamics)

### Fixed-Point Iteration

- Simplest solution method
- If fixed point is stable, just "iterate the dynamics" until it converges
- Only works if $x^*$ is a stable equilibrium point and if initial guess is in the basin of attraction
- Can converge slowly (depends on $f$)

### Newton's Method:

- Fit a linear approximation to $f(x)$:

$$f(x+\partial x) \approx f(x) + \left.\frac{\partial f}{\partial x}\right|_x \partial x$$

- Set approximation to zero and solve for $\partial x$:

$$f(x) + \frac{\partial f}{\partial x}\partial x = 0 \implies \partial x = -\left(\frac{\partial f}{\partial x}\right)^{-1} f(x)$$

- Apply correction:

$$x \leftarrow x + \partial x$$

- Repeat until convergence

#### Example: Backward Euler

$$f(x_{n+1}, x_n, u_n) = 0$$

$$x_{n+1} = x_n + hf(x_{n+1})$$
$\leftarrow$ evaluate $f$ at future time

$$\implies f(x_{n+1}, x_n, u_n) = x_{n+1} - x_n - hf(x_{n+1}) = 0$$

- Very fast convergence with Newton (Quadratic)
- Can get machine precision
- Most expensive part is solving a linear system $O(n^3)$
- Can improve complexity by taking advantage of problem structure/sparsity (more later)

---

## Minimization:

$$\min_x f(x), \quad f(x): \mathbb{R}^n \to \mathbb{R}$$

- If $f$ is smooth, $\left.\frac{\partial f}{\partial x}\right|_{x^*} = 0$ at a local minimum
- Now we have a root-finding problem: $\nabla f(x) = 0$

$$\implies \text{Apply Newton!}$$

$$\nabla f(x+\partial x) \approx \nabla f(x) + \underbrace{\frac{\partial}{\partial x}(\nabla f(x))}_{\nabla^2 f} \partial x = 0$$

$$\implies \partial x = -(\nabla^2 f(x))^{-1} \nabla f(x)$$

$$x \leftarrow x + \partial x$$

repeat until convergence

### Intuition:
- Fit a quadratic approximation to $f(x)$
- Exactly minimize approximation

#### Example:

$$\min_x f(x) = x^4 + x^3 - x^2 - x$$

- Start at: $(1.0, -1.5, 0)$ **$\leftarrow$ maximizes!**

---

## Take-away Messages:

- Newton is a **local** root-finding method. Will converge to the closest fixed point to the initial guess (min, max, saddle)

### Sufficient Conditions

- $\nabla f = 0$ is a "first-order necessary condition" for a minimum. Not a sufficient condition.

Let's look at scalar case:

$$\partial x = -\underbrace{\frac{1}{\sigma}\underbrace{(\nabla^2 f)^{-1}}_{\text{descent}}\underbrace{\nabla f}_{\text{gradient}}}_{\text{learning rate}/\text{step size}}$$

- $\nabla^2 f > 0 \implies$ descent (minimization)
- $\nabla^2 f < 0 \implies$ ascent (maximization)

In $\mathbb{R}^n$, $\nabla^2 f > 0$, $\nabla^2 f \in S_n^+$ (positive definite) $\implies$ descent

- If $\nabla^2 f > 0$ everywhere $(\forall x) \implies f(x)$ is strongly convex

$$\implies \text{Can always solve with Newton}$$

- Usually not the case for hard/nonlinear problems

---

## Regularization:

Practical solution to make sure we always minimize:

```
H ← ∇²f  ← "not pos. def."

while H ⋨ 0
    H ← H + βI  (β > 0)  ← scalar hyperparameter
end

∂x = -H⁻¹∇f

x ← x + ∂x
```

- Also called "damped Newton" (shrinks steps)
- Guarantees descent

#### Example:
- Regularization makes sure we minimize
- What about overshoot?
