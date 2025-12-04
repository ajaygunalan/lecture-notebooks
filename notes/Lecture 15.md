# Lecture 15: Optimization with Quaternions

## Last Time:
- Deterministic OC summary
- LQR v. MPC
- DDP vs. DIRCOL
- Quaternion

## Today:
- Optimization with Quaternions

---

## Quaternion Recap

### 4D Unit vectors
- Multiplication rule:

$$q_1 * q_2 = \begin{bmatrix} s_1 \\ v_1 \end{bmatrix} * \begin{bmatrix} s_2 \\ v_2 \end{bmatrix} = \begin{bmatrix} s_1 s_2 - v_1^T v_2 \\ s_1 v_2 + s_2 v_1 + v_1 \times v_2 \end{bmatrix}$$

$$L(q_1) = \begin{bmatrix} s_1 & -v_1^T \\ v_1 & s_1 I + \hat{v}_1 \end{bmatrix} \implies q_1 * q_2 = \begin{cases} L(q_1)q_2 \\ R(q_2)q_1 \end{cases}$$

### Conjugate

$$q^+ = \begin{bmatrix} s \\ -v \end{bmatrix} = Tq, \quad T = \begin{bmatrix} 1 & 0 \\ 0 & -I \end{bmatrix}$$

### Identity

$$q_I = \begin{bmatrix} 1 \\ 0 \end{bmatrix}$$

---

## "Hat map" for Quaternions

$$\hat{\omega} = \begin{bmatrix} 0 \\ \omega \end{bmatrix} = H\omega, \quad H = \begin{bmatrix} 0_- \\ I \end{bmatrix}$$

---

## Geometry of Quaternions

- $q$ lives on a sphere in $\mathbb{R}^4$
- $\dot{q}$ lives in the tangent plane at $q$

### Kinematics

$$q \in \mathbb{R}^4, \quad \omega \in \mathbb{R}^3, \quad \dot{q} = \frac{1}{2} q * \hat{\omega} = \frac{1}{2} L(q) H \omega$$

**Note:** $\omega$ is always written in the tangent plane at the identity, then Kinematics rotates to tangent plane at $q$.

### Analogy with unit complex numbers in 1D

$$v = \cos(\theta) + i \sin(\theta)$$

$$v = \begin{bmatrix} \cos(\theta) \\ \sin(\theta) \end{bmatrix} \implies v^T v = 1$$

$$\dot{v} = \frac{\partial v}{\partial \theta} \dot{\theta} = \begin{bmatrix} -\sin(\theta) \\ \cos(\theta) \end{bmatrix} \dot{\theta} = \begin{bmatrix} \cos(\theta) & -\sin(\theta) \\ \sin(\theta) & \cos(\theta) \end{bmatrix} \begin{bmatrix} 0 \\ \dot{\theta} \end{bmatrix}$$

$$\underbrace{\begin{bmatrix} \cos(\theta) & -\sin(\theta) \\ \sin(\theta) & \cos(\theta) \end{bmatrix}}_{\text{rotation matrix}} \underbrace{\begin{bmatrix} 0 \\ \dot{\theta} \end{bmatrix}}_{\text{2D "hat map"}}$$

Kinematics rotates $\dot{\theta}$ from tangent plane at $\theta=0$ to tangent at current $v$.

---

## Differentiating Quaternions

### Two key facts:
1. Derivatives are really 3D tangent vectors
2. Rotations compose by multiplication, not addition

### Infinitesimal Rotation

$$\delta q = \begin{bmatrix} \cos(\theta/2) \\ a\sin(\theta/2) \end{bmatrix} \approx \begin{bmatrix} 1 \\ \frac{1}{2}a\theta \end{bmatrix} \approx \begin{bmatrix} 1 \\ \frac{1}{2}\phi \end{bmatrix} \quad \text{(small axis-angle rotation)}$$

$$= \begin{bmatrix} 1 \\ 0 \end{bmatrix} + \frac{1}{2}\begin{bmatrix} 0 \\ \phi \end{bmatrix} = \begin{bmatrix} 1 \\ 0 \end{bmatrix} + \frac{1}{2}H\phi$$

### Compose with $q$:

$$q' = \delta q \otimes q = L(q)\left(\begin{bmatrix} 1 \\ 0 \end{bmatrix} + \frac{1}{2}H\phi\right)$$

$$= q + \frac{1}{2}\underbrace{L(q)H}_{G(q) \in \mathbb{R}^{4\times 3}} \phi$$

where $G(q)$ is the **"Attitude Jacobian"**

**Note:** We can use any 3-parameter rotation representation we want for $\phi$. They all linearize the same (up to a permutation/scaling)

$$q = \begin{bmatrix} \cos(\|\phi\|/2) \\ \frac{\phi}{\|\phi\|} \sin(\|\phi\|/2) \end{bmatrix} = \begin{bmatrix} \sqrt{1-\phi^T\phi} \\ \phi \end{bmatrix} \approx \frac{1}{\sqrt{1-\phi^T\phi}} \begin{bmatrix} 1 \\ \phi \end{bmatrix}$$

$$\underbrace{\quad\quad\quad\quad\quad\quad\quad}_{\text{axis-angle}} \underbrace{\quad\quad\quad\quad}_{\substack{\text{vector part} \\ \text{of } q}} \underbrace{\quad\quad\quad\quad\quad}_{\text{Gibbs/Rodrigues}}$$

---

## Using the vector part of $q$ in class

This lets us differentiate with quaternions by inserting $G(q)$ in the right places:

### $f(q): \mathbb{H} \to \mathbb{R}$ (gradient of a scalar-valued function)

$$\nabla f = \frac{\partial f}{\partial \phi} = \underbrace{\frac{\partial f}{\partial q}}_{\substack{\text{quaternions} \\ \text{("Hamilton")}}} G(q)$$

### $f(q): \mathbb{H} \to \mathbb{H}$ (Jacobian of a quaternion-valued function)

$$\phi' = \left[\underbrace{G(f(q))^T}_{\substack{\text{transform} \\ \text{output}}} \underbrace{\frac{\partial f}{\partial q}}_{\nabla f \in \mathbb{R}^{3\times 3}} \underbrace{G(q)}_{\substack{\text{transform} \\ \text{input}}}\right] \phi$$

### Hessian of $f(q): \mathbb{H} \to \mathbb{R}$

$$\nabla^2 f = G(q)^T \frac{\partial^2 f}{\partial q^2} G(q) + \underbrace{\mathcal{I}\left(\frac{\partial f}{\partial q} q\right)}_{\substack{\text{Comes from } \frac{\partial G}{\partial q} \\ \text{(scalar) } \frac{\partial G}{\partial q}}}$$

**Now we can do Newton's method and DDP and SQP with quaternions**

---

## Example: Pose Estimation

Given a bunch of vectors to known landmarks in the environment, determine robot's attitude.

Called **"Wahba's Problem"**

$$\min_q J(q) = \sum_{k=1}^M \|{}^M X_k - Q(q) {}^B X_k\|^2 = \|r(q)\|^2$$

$$\underbrace{{}^M X_k}_{\substack{\text{Known vectors} \\ \text{in World frame} \\ \text{(from map)}}} \quad \underbrace{{}^B X_k}_{\substack{\text{Observed vectors} \\ \text{in Body frame} \\ \text{(from Camera)}}} \quad \underbrace{\|r(q)\|^2}_{\substack{r(q)^T r(q) \\ \text{"residual"} \\ \text{vector}}}$$

**${}^M X_k$ and ${}^B X_k$ are unit vectors ("directions")**

$$r(q) = \begin{bmatrix} {}^M X_1 - Q(q) {}^B X_1 \\ {}^M X_2 - Q(q) {}^B X_2 \\ \vdots \\ {}^M X_m - Q(q) {}^B X_m \end{bmatrix} \implies Dr(q) = \underbrace{\frac{\partial r}{\partial q}}_{3m \times 4} \underbrace{G(q)}_{4 \times 3}$$

$$\underbrace{Dr(q)}_{3m \times 3}$$

### Background: Gauss-Newton for Least-squares

$$\min_x J(x) = \frac{1}{2}\|r(x)\|^2 = \frac{1}{2}r(x)^T r(x)$$

$$\frac{\partial J}{\partial x} = r(x)^T \frac{\partial r}{\partial x}$$

$$\frac{\partial^2 J}{\partial x^2} = \left(\frac{\partial r}{\partial x}\right)^T \left(\frac{\partial r}{\partial x}\right) + \underbrace{(I \otimes r(x)) \frac{\partial^2 \text{vec}(r)}{\partial x^2}}_{\text{throw this out}}$$

$$\implies \left(\frac{\partial^2 J}{\partial x^2}\right)^{-1} \nabla J \approx \left[\left(\frac{\partial r}{\partial x}\right)^T \left(\frac{\partial r}{\partial x}\right)\right]^{-1} \frac{\partial r}{\partial x}^T r(x)$$

---

## Gauss-Newton for Wahba's Problem

```
q ← q₀  (initial guess)
do:
    Dr(q) = ∂r/∂q G(q)

    ϕ = -[(Drᵀ Dr)⁻¹ Drᵀ] r(q)

    q ← q * [√(1-ϕᵀϕ)] = L(q) [√(1-ϕᵀϕ)]
            [    ϕ    ]         [    ϕ    ]

    (multiplicative update)

    (in general, do line search)

while ‖r(q)‖ > tol
```
