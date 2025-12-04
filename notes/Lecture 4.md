# Lecture 4

## Last Time:
- Root Finding
- Newton's Method
- Minimization
- Regularization

## Today:
- Line Search
- Constrained Minimization

---

## Line Search

### Overview
- Often $\delta x$ step from Newton overshoots the minimum
- To fix this, check $f(x + \delta x)$ and "back track" until we get a "good" reduction

### Many Strategies

#### Armijo Rule
A simple and effective strategy:

```
α = 1
while f(x + α·δx) > f(x) + δα∇f(x)ᵀδx
    α ← c·α
end
```

Where:
- $\delta$: tolerance
- Expected reduction from linearization
- $c$: scalar $< 1$

### Intuition
- Make sure step agrees with linearization within some tolerance $\delta$

### Typical Values
- $c = \frac{1}{2}$
- $\delta = 10^{-4} \sim 0.1$

### Take Away
Newton with simple and cheap modifications ("globalization strategies") is extremely effective at finding local optima.

---

## Equality Constraints

### Problem Formulation
$$\min_x f(x) \quad \text{where } f(x): \mathbb{R}^n \to \mathbb{R}$$
$$\text{s.t. } c(x) = 0 \quad \text{where } c(x): \mathbb{R}^n \to \mathbb{R}^m$$

### First-Order Necessary Conditions
1. Need $\nabla f(x) = 0$ in free directions
2. Need $c(x) = 0$

**Key Insight**: Any non-zero component of $\nabla f$ must be normal to the constraint at an optimum. Equivalently, $\nabla f$ must be parallel to $\nabla c$.

This gives us:
$$\nabla f + \lambda \nabla c = 0$$

Where $\lambda$ is the **Lagrange multiplier** (also called "dual variable")

### In General:
$$\frac{\partial f}{\partial x} + \lambda^T \left(\frac{\partial c}{\partial x}\right) = 0, \quad \lambda \in \mathbb{R}^m$$

### Lagrangian
Based on this gradient condition, we define:

$$L(x, \lambda) = f(x) + \lambda^T c(x)$$

Such that:
- $\nabla_x L(x, \lambda) = \nabla f + \left(\frac{\partial c}{\partial x}\right)^T \lambda = 0$
- $\nabla_\lambda L(x, \lambda) = c(x) = 0$

### Newton's Method for Constrained Optimization

We can solve this with Newton:

$$\nabla_x L(x + \delta x, \lambda + \delta\lambda) \approx \nabla_x L(x, \lambda) + \frac{\partial^2 L}{\partial x^2}\delta x + \frac{\partial^2 L}{\partial x \partial\lambda}\delta\lambda = 0$$

$$\nabla_\lambda L(x + \delta x, \lambda + \delta\lambda) \approx c(x) + \frac{\partial c}{\partial x}\delta x = 0$$

This leads to the **KKT System**:

$$\begin{bmatrix}
\frac{\partial^2 L}{\partial x^2} & \left(\frac{\partial c}{\partial x}\right)^T \\
\frac{\partial c}{\partial x} & 0
\end{bmatrix}
\begin{bmatrix}
\delta x \\
\delta\lambda
\end{bmatrix}
=
\begin{bmatrix}
-\nabla_x L(x, \lambda) \\
-c(x)
\end{bmatrix}$$

---

## Gauss-Newton Method

### Formula
$$\frac{\partial^2 L}{\partial x^2} = \nabla^2 f + \frac{\partial}{\partial x}\left[\left(\frac{\partial c}{\partial x}\right)^T \lambda\right]$$

where the second term (constraint curvature) is expensive to compute.

### Simplification
- We often drop the 2nd "constraint curvature" term
- Called "Gauss-Newton"
- Slightly slower convergence than full Newton (more iterations) but iterations are cheaper
- Often wins in wall-clock time

### Example
Starting point: $(-1, -1)$, target: $[-3, 2]$
- Full Newton gets stuck
- Gauss-Newton doesn't

### Take Aways:
- May still need regularization even if $\nabla^2 f > 0$
- Gauss-Newton is often used in practice

---

## Inequality Constraints

### Problem Formulation
$$\min_x f(x)$$
$$\text{s.t. } c(x) \geq 0$$

Note: We'll just look at inequalities for now. Just combine with previous methods to handle both kinds of constraints.

### First-Order Necessary Conditions (KKT Conditions):

1. **Stationarity**: $\nabla f - \left(\frac{\partial c}{\partial x}\right)^T \lambda = 0$

   $\nabla f = 0$ in the free directions

2. **Primal Feasibility**: $c(x) \geq 0$

3. **Dual Feasibility**: $\lambda \geq 0$

4. **Complementarity**: $\lambda \circ c(x) = \lambda^T c(x) = 0$

The complementarity condition means that for each constraint:
- Either the constraint is inactive ($c(x) > 0$) and $\lambda = 0$, OR
- The constraint is active ($c(x) = 0$) and $\lambda \geq 0$

These conditions are collectively known as the **Karush-Kuhn-Tucker (KKT) conditions**.
