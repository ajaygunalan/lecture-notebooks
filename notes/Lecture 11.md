# Lecture 11

## Last Time:
- Convex Optimization Overview
- Convex MPC

## Today:
- Nonlinear Trajectory Optimization
- Differential Dynamic Programming

## What About Nonlinear Dynamics?

- Linear stuff often works well, so use it if you can
- Nonlinear dynamics makes MPC problem non-convex
  - $\Rightarrow$ no convergence/optimality guarantees
- Can work well in practice with effort

## Nonlinear Traj Opt Problem:

$$
\begin{align}
\min_{x_k, u_k} \quad & J = \sum_{k=0}^{N-1} \ell_k(x_k, u_k) + \ell_N(x_N) \\
& \qquad \underbrace{\qquad\qquad\qquad}_{\text{running cost}} \\
\text{s.t.} \quad & x_{k+1} = f(x_k, u_k) \quad \leftarrow \text{ nonlinear} \\
& x_k \in \mathcal{X}_k \quad \left\} \text{ non-convex constraints} \right. \\
& u_k \in \mathcal{U}_k \quad \left. \phantom{\}} \right\}
\end{align}
$$

- Usually assume cost + constraints are $C^2$ (continuous 2nd derivatives)

---

## Differential Dynamic Programming (DDP)

- Nonlinear trajopt method based on approximate DP
- Use a 2nd-order Taylor expansion of cost-to-go in DP to compute Newton steps
- Very fast convergence is possible
- Can stop early in real-time application

### Cost-to-go Expansion:

$$
V_n(x + \delta x) \approx V_n(x) + p_n^T \delta x + \frac{1}{2} \delta x^T P_n \delta x
$$

$$
p_n = \nabla_x \ell_N(x) \qquad P_n = \nabla_{xx}^2 \ell_N(x)
$$

### Action-Value Function Expansion:

$$
S_n(x+\delta x, u+\delta u) \approx S_n(x,u) + \begin{bmatrix} g_x \\ g_u \end{bmatrix}^T \begin{bmatrix} \delta x \\ \delta u \end{bmatrix} + \frac{1}{2} \begin{bmatrix} \delta x \\ \delta u \end{bmatrix}^T \begin{bmatrix} G_{xx} & G_{xu} \\ G_{ux} & G_{uu} \end{bmatrix} \begin{bmatrix} \delta x \\ \delta u \end{bmatrix}
$$

$(G_{ux} = G_{xu}^T)$

$$
\begin{align}
V_{n-1}(x) = \min_{\delta u} \bigg[ & S_{n-1}(x,u) + g_x^T \delta x + g_u^T \delta u \\
& + \frac{1}{2} \delta x^T G_{xx} \delta x + \frac{1}{2} \delta u^T G_{uu} \delta u \\
& + \frac{1}{2} \delta x^T G_{xu} \delta u + \frac{1}{2} \delta u^T G_{ux} \delta x \bigg]
\end{align}
$$

$$
\nabla_{\delta u} [\cdot] = g_u + G_{uu} \delta u + G_{ux} \delta x = 0
$$

---

## Optimal Control Solution

$$
\begin{align}
\Rightarrow \delta u_{n-1} &= -G_{uu}^{-1} g_u - G_{uu}^{-1} G_{ux} \delta x \\
& \quad \underbrace{\qquad}_{d_{n-1}} \quad \underbrace{\qquad\quad}_{K_{n-1}} \\
&= -d_{n-1} - K_{n-1} \delta x
\end{align}
$$

$$
\underbrace{-d_{n-1}}_{\text{"Feed-forward"}} \quad \underbrace{-K_{n-1} \delta x}_{\text{"Feedback"}}
$$

### Plug back into $S_{n-1}$ to get $V_{n-1}(x+\delta x)$:

$$
\begin{align}
\Rightarrow V_{n-1}(x+\delta x) = & V_{n-1}(x) + g_x^T \delta x + g_u^T(-d_{n-1}-K_{n-1}\delta x) \\
& + \frac{1}{2} \delta x^T G_{xx} \delta x + \frac{1}{2}(-d_{n-1}-K_{n-1}\delta x)^T G_{uu}(\cdots) \\
& - \frac{1}{2} \delta x^T G_{xu}(\cdots) - \frac{1}{2}(\cdots)^T G_{ux} \delta x
\end{align}
$$

$$
\Downarrow
$$

$$
\boxed{
\begin{align}
P_{n-1} &= G_{xx} + K_{n-1}^T G_{uu} K_{n-1} - G_{xu} K_{n-1} - K_{n-1}^T G_{ux} \\
p_{n-1} &= g_x - K_{n-1}^T g_u + K_{n-1}^T G_{uu} d_{n-1} - G_{xu} d_{n-1}
\end{align}
}
$$

---

## Matrix Calculus

- given $f(x): \mathbb{R}^n \to \mathbb{R}^m$, look at 2nd-order Taylor expansions
- if $m = 1$:

$$
f(x+\delta x) \approx f(x) + \underbrace{\frac{\partial f}{\partial x}}_{\mathbb{R}^{1 \times n}} \delta x + \frac{1}{2} \delta x^T \underbrace{\frac{\partial^2 f}{\partial x^2}}_{\mathbb{R}^{n \times n}} \delta x
$$

- for $m > 1$: $\frac{\partial^2 f}{\partial x^2}$ is a 3rd-rank tensor. Think of "3D matrix"
  We need some tricks to deal with these

### Kronecker Product:

$$
\underbrace{A}_{\ell \times m} \otimes \underbrace{B}_{m \times p} = \begin{bmatrix} a_{11}B & a_{12}B & \cdots \\ a_{21}B & a_{22}B & \cdots \\ \vdots & \vdots & \end{bmatrix}_{\ell n \times mp}
$$

### Vectorization Operator

$$
\underbrace{A}_{\ell \times m} = \underbrace{[A_1 \; A_2 \; A_3 \; \cdots \; A_m]}_{\text{column vectors}}
$$

$$
\text{vec}(A) = \begin{bmatrix} A_1 \\ A_2 \\ A_3 \\ \vdots \\ A_m \end{bmatrix}_{\ell m \times 1}
$$

### The "vec trick"

$$
\text{vec}(ABC) = (C^T \otimes A) \, \text{vec}(B)
$$

$$
\Rightarrow \text{vec}(AB) = (B^T \otimes I) \, \text{vec}(A) = (I \otimes A) \, \text{vec}(B)
$$

---

## Differentiating Matrices

- If I want to diff a matrix wrt a vector, vectorize the matrix:

$$
\frac{\partial A(x)}{\partial x} = \underbrace{\frac{\partial \, \text{vec}(A)}{\partial x}}_{\ell m \times n} \quad \text{(implied whenever we diff a matrix)}
$$

### Taylor expansion of $f(x)$:

$$
\begin{align}
f(x+\delta x) &\approx f(x) + \underbrace{\frac{\partial f}{\partial x}}_{A} \delta x + \frac{1}{2} (\delta x^T \otimes I) \underbrace{\frac{\partial^2 f}{\partial x^2}}_{\frac{\partial A}{\partial x} = \frac{\partial \, \text{vec}(A)}{\partial x}} \delta x \\
&= \frac{\partial}{\partial x}[\text{vec}(I \, A \, x \, \delta x)]
\end{align}
$$

### Sometimes we need to diff through a transpose:

$$
\frac{\partial}{\partial x}(A(x)^T B) = \underbrace{(B^T \otimes I) T}_{\text{"commutator matrix"}} \frac{\partial A}{\partial x}
$$

$$
T \, \text{vec}(A) = \text{vec}(A^T)
$$

---

## Action-Value Function Derivatives:

$$
S_k(x,u) = \ell_k(x,u) + V_{k+1}(f(x,u))
$$

$$
\Rightarrow \frac{\partial S}{\partial x} = \frac{\partial \ell}{\partial x} + \underbrace{\frac{\partial V}{\partial f}}_{A} \frac{\partial f}{\partial x} \quad \Rightarrow \quad \boxed{g_x = \nabla_x \ell_k + A_k^T p_{k+1}}
$$

$$
\frac{\partial S}{\partial u} = \frac{\partial \ell}{\partial u} + \underbrace{\frac{\partial V}{\partial f}}_{B} \frac{\partial f}{\partial u} \quad \Rightarrow \quad \boxed{g_u = \nabla_u \ell_k + B_k^T p_{k+1}}
$$

---

## Second-Order Derivatives:

$$
\boxed{
\begin{align}
G_{xx} &= \frac{\partial^2 S}{\partial x^2} = \nabla_{xx}^2 \ell_k + A_k^T P_{k+1} A_k + (p_{k+1} \otimes I)^T T \frac{\partial^2 A}{\partial x^2} \\
G_{uu} &= \frac{\partial^2 S}{\partial u^2} = \nabla_{uu}^2 \ell_k + B_k^T P_{k+1} B_k + (p_{k+1} \otimes I)^T T \frac{\partial^2 B}{\partial u^2} \\
G_{xu} &= \frac{\partial^2 S}{\partial x \partial u} = \nabla_{xu}^2 \ell_k + A_k^T P_{k+1} B_k + (p_{k+1} \otimes I)^T T \frac{\partial^2 A}{\partial u}
\end{align}
}
$$
