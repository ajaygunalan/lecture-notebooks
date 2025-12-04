# Lecture 19: Stochastic Optimal Control

## Last Time:
- Iterative Learning Control

## Today:
- **Stochastic Optimal Control**

---

## Stochastic Control

### Introduction

- So far we have assumed we know the system's state perfectly.
- What happens when all we have are noisy measurements of quantities related to the state?

**Measurement Model:**
$$y = g(x, v)$$

where:
- $v$ = noise
- $y$ = measurements
- $g$ = measurement model

**From Deterministic to Stochastic:**
- Deterministic: $x$
- Stochastic: $p(x | y)$ - PDF of the state conditioned on the measurements

### Stochastic Optimal Control Problem

$$\min_{u} \mathbb{E}[J(x, u) | y]$$

- In principle, we can solve with DP
- In general, very hard

---

## LQG (Linear Quadratic Gaussian)

**Special case we can solve in closed form:**
1. **Linear** Dynamics
2. **Quadratic** costs
3. **Gaussian** Noise

### Dynamics

$$x_{n+1} = Ax_n + Bu_n + w_n \quad \text{(Process noise)}$$
$$y_n = Cx_n + v_n \quad \text{(Measurement noise)}$$

**Noise distributions:**
$$w_n \sim \mathcal{N}(0, W) \quad \quad v_n \sim \mathcal{N}(0, V)$$

where:
- $\sim$ means "drawn from"
- $\mathcal{N}(0, W)$ = Normal Distribution (Gaussian)
- $0$ = mean
- $W, V$ = Covariance matrices

---

## Multivariate Gaussian

$$p(x) = \frac{1}{\sqrt{(2\pi)^n \det(S)}} \exp\left(-\frac{1}{2} (x-m)^T S^{-1}(x-m)\right)$$

**Parameters:**
- **mean:** $m = \hat{x} = \mathbb{E}[x] \in \mathbb{R}^n$
- **covariance:** $S = \mathbb{E}[(x-m)(x-m)^T] \in \mathbb{S}_{++}^n$

**Expectation:**
$$\mathbb{E}[f(x)] = \int_{\text{all space}} f(x) \, p(x) \, dx$$

---

## Cost Function

$$J = \mathbb{E}\left[\frac{1}{2} x_N^T Q_N x_N + \frac{1}{2} \sum_{k=0}^{N-1} \left(x_k^T Q x_k + u_k^T R u_k\right)\right]$$

---

## D.P. Recursion

$$V_N(x) = \frac{1}{2} \mathbb{E}[x_N^T Q_N x_N] = \frac{1}{2} \mathbb{E}[x_N^T P_N x_N]$$

$$V_{n-1}(x) = \min_{u} \mathbb{E}\left[\frac{1}{2} x_n^T Q x_n + \frac{1}{2} u^T R u + \frac{1}{2} (Ax_n + Bu + w_n)^T P_n (Ax_n + Bu + w_n)\right]$$

**Expanding:**
$$= \min_{u} \mathbb{E}\left[\frac{1}{2} x_n^T Q x_n + \frac{1}{2} u^T R u + (Ax_n + Bu)^T P_n (Ax_n + Bu)\right]$$
$$\quad \quad \quad \quad \quad \underbrace{\qquad \qquad \qquad \qquad \qquad \qquad \qquad \qquad \qquad}_{\text{Standard LQR}}$$

$$+ \mathbb{E}\left[(Ax_n + Bu)^T P_n w_{n-1} + w_n^T P_n (Ax_n + Bu) + w_n^T P_n w_n\right]$$
$$\quad \quad \underbrace{\qquad \qquad}_{= 0} \quad \underbrace{\qquad \qquad \qquad \qquad}_{\text{Noise Terms}} \quad \underbrace{\qquad \qquad}_{\text{Constant!}}$$

**Key observation:** Noise sample drawn at time $k$ has nothing to do with the state (or control) at time $k$. $x_k$ depends on $w_{k-1}$ (and all previous $w$) but not on $w_k$ or future $w$.

---

## Important Principles

$\Rightarrow$ **Uncorrelated** $\Rightarrow$ Cross-correlation is zero

$\Rightarrow$ Noise term has no impact on control design! (You just get a higher cost)

### "Certainty Equivalence Principle"
- The optimal LQG controller is just LQR with $x$ replaced by $\mathbb{E}[x]$

### "Separation Principle"
- For LQG we can design an optimal feedback controller and an optimal estimator separately and then hook them together. The resulting feedback policy is optimal.

**Note:** Neither of these holds in general, but are still frequently used in practice to design sub-optimal policies.

---

## Optimal State Estimator

**What should I try to optimize?**

### Two approaches:

**1. Maximum a-posteriori (MAP):**
$$\arg\max_{x} \, p(x|y)$$

**2. Minimum mean-squared error (MMSE):** ("Least Squares")
$$\arg\min_{\hat{x}} \mathbb{E}[(x-\hat{x})(x-\hat{x})^T]$$

**Important:** These are the same for a Gaussian!
