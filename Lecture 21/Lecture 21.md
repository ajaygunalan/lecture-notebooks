# Lecture 21

## Last Time:
- Stochastic Optimal Control
- LQG

## Today:
- Optimal Estimation
- Finish LQG
- Duality

---

## From Last Time:

### Key Principles
- **"Certainty Equivalence"**
- **"Separation Principle"**
- Frequently applied to nonlinear systems in practice

### Optimal State Estimation

**What should I optimize?**

- **MMSE** (Minimum Mean Squared Error)
- **MAP** (Maximum a-posteriori)
  - $\arg\max p(x|y)$
  - probability of state given measurements

---

## Minimum Mean Squared Error (MMSE)

**"least squares"** / **"minimum variance"**

$$\arg\min_{\hat{x}} E[(x-\hat{x})^T(x-\hat{x})]$$

$$E[\text{tr}((x-\hat{x})^T(x-\hat{x}))] = E[\text{tr}((x-\hat{x})(x-\hat{x})^T)]$$

$$= \text{tr}(E[(x-\hat{x})(x-\hat{x})^T]) = \text{tr}(\Sigma)$$

**Note:** These are the same for a Gaussian!

---

## Kalman Filter

### Overview
- Recursive linear MMSE estimator

### Assumptions
- Assume an estimate of the state that includes all measurements up to the current time:
  - $\hat{x}_{k|k} = E[x_n | y_{1:k}]$

- Assume we also know the error covariance:
  - $\Sigma_{k|k} = E[(x_n - \hat{x}_{n|n})(x_n - \hat{x}_{n|n})^T]$

- We want to update $\hat{x}$ and $\Sigma$ to include a new measurement at $k+1$

### Process
The KF can be broken into 2 steps

---

## Kalman Filter: Prediction Step

### Prediction:

$$\hat{x}_{n+1|n} = E[Ax_n + Bu_n + w_n | y_{1:k}]$$
$$= A\hat{x}_{n|n} + Bu_n$$

$$\Sigma_{n+1|n} = E[(x_{n+1} - \hat{x}_{n+1|n})(\cdots)^T]$$
$$= E[(Ax_n + Bu_n + w_n - A\hat{x}_{n|n} - Bu_n)(\cdots)^T]$$
$$= A E[(x_n - \hat{x}_{n|n})(\cdots)^T]A^T + E[w_n w_n^T]$$
$$= A\Sigma_{n|n}A^T + W$$

($x_n$ and $w_n$ are uncorrelated)

---

## Kalman Filter: Measurement Update

### Define "innovation":

$$z_{n+1} = y_{n+1} - C\hat{x}_{n+1|n}$$
$$= Cx_{n+1} + v_{n+1} - C\hat{x}_{n+1|n}$$

### Innovation Covariance:

$$S_{n+1} = E[z_{n+1}z_{n+1}^T]$$
$$= E[(Cx_{n+1} + v_{n+1} - C\hat{x}_{n+1|n})(\cdots)^T]$$

**Note:** $v_{n+1}$ and $x_{n+1}$ are uncorrelated

$$\Rightarrow S_{n+1} = CE[(x_{n+1} - \hat{x}_{n+1|n})(\cdots)^T]C^T + E[v_{n+1}v_{n+1}^T]$$
$$= C\Sigma_{n+1|n}C^T + V$$

- Innovation is the error signal we feed back into the estimator.

---

## Kalman Filter: State and Covariance Update

### State Update:

$$\hat{x}_{n+1|n+1} = \hat{x}_{n+1|n} + L_{n+1}z_{n+1}$$

($L_{n+1}$ is the "Kalman Gain")

### Covariance Update:

$$\Sigma_{n+1|n+1} = E[(x_{n+1} - \hat{x}_{n+1})(\cdots)^T]$$
$$= E[(x_{n+1} - \hat{x}_{n+1|n} - L_{n+1}(Cx_{n+1} + v_{n+1} - C\hat{x}_{n+1|n}))(\cdots)^T]$$

**Note:** $v_{n+1}$ and $x_{n+1}$ are uncorrelated

$$= (I - L_{n+1}C)\Sigma_{n+1|n}(I - L_{n+1}C)^T + L_{n+1}VL_{n+1}^T$$

**"Joseph Form"**

### Kalman Gain

MMSE $\Rightarrow$ minimize $E[(x_{n+1} - \hat{x}_{n+1|n+1})^T(\cdots)]$ = $\text{tr}(\Sigma_{n+1|n+1})$

$$\Rightarrow \text{set } \frac{\partial \text{tr}(\Sigma_{n+1|n+1})}{\partial L_{n+1}} = 0 \text{ (and solve for } L_{n+1}\text{)}$$

$$\Rightarrow \boxed{L_{n+1} = \Sigma_{n+1|n}C^T S_{n+1}^{-1}}$$

---

## KF Algorithm Summary:

**1) Start with** $\hat{x}_{0|0}$, $\Sigma_{0|0}$, $W$, $V$

**2) Predict:**
   - $\hat{x}_{n+1|n} = A\hat{x}_{n|n} + Bu_n$
   - $\Sigma_{n+1|n} = A\Sigma_{n|n}A^T + W$

**3) Calculate Innovation + Covariance:**
   - $z_{n+1} = y_{n+1} - C\hat{x}_{n+1|n}$
   - $S_{n+1} = C\Sigma_{n+1|n}C^T + V$

**4) Calculate Kalman Gain:**
   - $L_{n+1} = \Sigma_{n+1|n}C^T S_{n+1}^{-1}$

**5) Update:**
   - $\hat{x}_{n+1|n+1} = \hat{x}_{n+1|n} + L_{n+1}z_{n+1}$
   - $\Sigma_{n+1|n+1} = (I - L_{n+1}C)\Sigma_{n+1|n}(I - L_{n+1}C)^T + L_{n+1}VL_{n+1}^T$

**6) Goto 2**

---

## Extensions to Nonlinear Systems

### How do we apply this to nonlinear systems?

- **Extended KF:** Linearize about $\hat{x}$ and proceed as in standard KF
- Many other generalizations

---

## Duality + Trajectory Optimization

### Duality Concept

The MMSE estimation problem is equivalent to the following optimal control problem:

**Optimization Problem:**

$$\min_{\substack{x_n \\ w_{n|n=1}^N}} \sum_{n=1}^{N} \left[\frac{1}{2}(y_n - g(x_n))^T V^{-1}(y_n - g(x_n)) + \frac{1}{2}w_n^T W^{-1} w_n\right]$$

(measurement model)

"State cost" $\qquad\qquad\qquad$ "Control cost"

**Subject to:**
$$x_{n+1} = f(x_n) + w_n$$

($w_n$ are the "Controls")

- **If** $f(x) = Ax$ and $g(x) = Cx$, this is an LQR problem
