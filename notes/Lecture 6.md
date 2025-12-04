# Lecture 6

## Last Time:
- Inequality Constraints
- Interior-Point Methods

## Today:
- Revisit regularization + line search
- Control History

---

## Duality + Regularization

### Given:
$$\begin{align}
\min_x \quad & f(x) \\
\text{s.t.} \quad & c(x) = 0
\end{align}$$

We can theoretically turn this into:
$$\min_x f(x) + P_\infty(c(x))$$

where
$$P_\infty(x) = \begin{cases}
0, & c(x) = 0 \\
+\infty, & c(x) \neq 0
\end{cases}$$

Practically terrible, but we can get the same effect by solving:
$$\min_x \max_\lambda f(x) + \lambda^T c(x)$$

- Whenever $c(x) \neq 0$, inner problem blows up

---

## Similar for inequalities:

$$\begin{align}
\min_x \quad & f(x) \\
\text{s.t.} \quad & c(x) \geq 0
\end{align}$$

$$\Rightarrow \quad \min_x f(x) + P_\infty^-(c(x))$$

where
$$P_\infty^- = \begin{cases}
0, & c(x) \geq 0 \\
+\infty, & c(x) < 0
\end{cases}$$

$$= \min_x \max_{\lambda \geq 0} f(x) - \lambda^T c(x), \quad \lambda \geq 0$$

- Whenever $c(x) < 0$, inner problem blows up.

### Interpretation:
KKT conditions define a saddle point in $(x, \lambda)$

- KKT system should have $\dim(x)$ positive eigenvalues and $\dim(\lambda)$ negative eigenvalues at an optimum.
  - (Called "quasi-definite")

**Matrix form:**
$$\begin{bmatrix}
H + \beta I & C^T \\
C & -\beta I
\end{bmatrix}
\begin{bmatrix}
\delta x \\
\delta \lambda
\end{bmatrix}
=
\begin{bmatrix}
-\nabla f \\
-c(x)
\end{bmatrix}, \quad \beta > 0$$

### Example:
- Still overshoot $\Rightarrow$ need line search

---

## Merit Function

How do we do a line search on a root-finding problem?
$$\text{find } x^* \text{ s.t. } c(x^*) = 0$$

Define scalar "merit function" $P(x)$ that measures distance to solution

### Standard Choice:
$$P(x) = \frac{1}{2}c(x)^T c(x) = \frac{1}{2}\|c(x)\|_2^2$$

$$P(x) = \|c(x)\|_1 \quad \text{(any norm works)}$$

Now just do Armijo on $P(x)$:
```
α = 1                                    ← tolerance
while P(x + α·δx) > P(x) + b·α·∇P(x)ᵀ·δx
                                         ← step length
    α ← α × α
end
x ← x + α·δx
```

---

## How about constrained optimization?

$$\begin{align}
\min_x \quad & f(x) \\
\text{s.t.} \quad & c(x) \geq 0 \\
& d(x) = 0
\end{align}$$

**Lagrangian:**
$$L(x, \lambda, \mu) = f(x) - \lambda^T c(x) + \mu^T d(x)$$

### Lots of options for merit functions:

**Option 1: KKT Residual**
$$P(x, \lambda, \mu) = \frac{1}{2}\|r_{KKT}(x, \lambda, \mu)\|_2^2$$

where KKT Residual includes:
- $\nabla_x L$
- $\min(0, c(x))$
- $d(x)$

**Option 2: L1 penalty function**
$$P(x, \lambda, \mu) = f(x) + \rho \left\| \begin{bmatrix} \min(0, c(x)) \\ d(x) \end{bmatrix} \right\|_1$$

where:
- $\rho$ = scalar weight
- works with any norm
