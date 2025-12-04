# Lecture 16

## Last Time:
- Optimization with Quaternions

## Today:
- LQR with Quaternions
- Quadrotor Control

---

## LQR with Quaternions

- Naively linearizing a system with a quaternion state results in an uncontrollable linear system
- We'll apply our quaternion differentiation tricks to LQR to make this work.

### Given a reference $\bar{x}_n$, $\bar{u}_n$ for a discrete-time system:

$$
\bar{x}_{n+1} + \delta x_{n+1} = f(\bar{x}_n + \delta x_n, \bar{u}_n + \delta u_n)
$$

$$
\approx f(\bar{x}_n, \bar{u}_n) + A_n \delta x_n + B_n \delta u_n
$$

where $A_n = \frac{\partial f}{\partial x}$ and $B_n = \frac{\partial f}{\partial u}$

- For the quaternion part of the state, we apply the attitude Jacobian to convert $\delta q \to \phi \in \mathbb{R}^3$:

$$
x = \begin{bmatrix} r \\ q \\ \sigma \\ \dot{r} \\ \omega \\ \dot{\theta} \end{bmatrix} \quad \begin{matrix} x[1:3] \\ x[4:7] \\ x[8:n] \\ \vdots \\ \vdots \\ \vdots \end{matrix}
$$

### Linearization with Attitude Jacobian:

$$
\begin{bmatrix} \delta x_n[1:3] \\ \phi_{n+1} \\ \delta x[8:n] \end{bmatrix} \approx \begin{bmatrix} I & & O \\ & G(\bar{q}_{n+1}) & \\ O & & I \end{bmatrix}^T A_n \begin{bmatrix} I & & O \\ & G(\bar{q}_n) & \\ O & & I \end{bmatrix} \begin{bmatrix} \delta x_n[1:3] \\ \phi_n \\ \delta x[8:n] \end{bmatrix}
$$

$$
+ E(\bar{x}_{n+1})^T B_n \delta u_n
$$

where:
- $\delta \tilde{x}_{n+1}$ is the reduced state
- $E(\bar{x}_{n+1})$ is the expansion matrix
- $E(\bar{x}_n)$ is the expansion matrix at time $n$

- Once we have these "reduced" Jacobians $\tilde{A}_n$, $\tilde{B}_n$:

$$
\tilde{A}_n = E(\bar{x}_{n+1})^T A_n E(\bar{x}_n), \quad \tilde{B}_n = E(\bar{x}_{n+1})^T B_n
$$

we compute the LQR controller as usual.

- When we run the controller, we calculate $\delta \tilde{x}$ before multiplying by $K$:

$$
\text{given } x_n, \quad \delta \tilde{x}_n = \begin{bmatrix} x_n[1:3] - \bar{x}_n[1:3] \\ \phi(L(\bar{q}_n)^T q_n) \\ x_n[8:n] - \bar{x}_n[8:n] \end{bmatrix}
$$

where $\phi$ is whatever 3-parameter representation you like.

$$
u_n = \bar{u}_n - K_n \delta \tilde{x}_n
$$

---

## Computing error/delta rotations:

- Many possible conventions
- We will write it as rotation from body frame $B$ to the reference/desired body frame $R$:

$$
{}^N Q^B = {}^N Q^R \, {}^R Q^B \quad \Rightarrow \quad ({}^N Q^R)^{-1} {}^N Q = {}^R Q^B
$$

$$
\hat{Q} = \bar{Q} \, \delta Q \quad \Rightarrow \quad \delta Q = \bar{Q}^T \hat{Q}
$$

- Using quaternions:

$$
\delta q = \bar{q}^+ * \hat{q} = L(\bar{q})^T \hat{q}
$$

---

## 3D Quadrotor

### Diagram:
```
        F₁
        ↑
    ω^B ↑  ← F₂
    ─●──┼──
        ↑
        ↓ u₂
```

Where:
- $F_i = K_F u_i$, $u \in \mathbb{R}^4$
- $\tau_i = K_\tau u_i$

### State:

$$
x = \begin{bmatrix} {}^N r \in \mathbb{R}^3 \\ {}^N q^B \in \mathbb{H} \\ {}^B v \in \mathbb{R}^3 \\ {}^B \omega \in \mathbb{R}^3 \end{bmatrix} \quad \begin{matrix} \text{position} \\ \text{attitude} \\ \text{linear velocity in } B \text{ frame} \\ \text{angular velocity} \end{matrix}
$$

### Kinematics:

$$
{}^N \dot{r} = {}^N v = Q \, {}^B v
$$

$$
\dot{q} = \frac{1}{2} \bar{q} * \hat{\omega} = \frac{1}{2} L(q) H \hat{\omega} = \frac{1}{2} G(q) \omega
$$

### Translation Dynamics:

$$
m {}^N \dot{v} = {}^N F \quad \leftarrow \text{ total force}
$$

Need to rotate into $B$ frame:

$$
{}^N v = Q \, {}^B v \quad \Rightarrow \quad {}^N \dot{v} = \dot{Q} \, {}^B v + Q \, {}^B \dot{v} = Q \hat{\omega} \, {}^B v + Q \, {}^B \dot{v}
$$

where $\dot{Q} = Q \hat{\omega}$

$$
\Rightarrow \quad {}^B \dot{v} = Q^T {}^N F - \omega \times {}^B v
$$

where:
- $Q^T {}^N F$ rotates force into $B$ frame
- $-\omega \times {}^B v$ is the extra rotating-frame term

### Force equation:

$$
\Rightarrow \quad {}^B \dot{v} = \frac{1}{m} {}^B F - \omega \times {}^B v
$$

$$
{}^B F = Q^T \begin{bmatrix} 0 \\ 0 \\ mg \end{bmatrix} + \begin{bmatrix} 0 & 0 & 0 & 0 \\ 0 & 0 & 0 & 0 \\ K_F & K_F & K_F & K_F \end{bmatrix} \begin{bmatrix} u_1 \\ u_2 \\ u_3 \\ u_4 \end{bmatrix}
$$

### Rotation Dynamics:

$$
J \dot{\omega} + \omega \times J \omega = {}^B \tau \quad \leftarrow \text{ total torque}
$$

where the left side is Euler's Equation ($J$ is the inertia matrix)

$$
{}^B \tau = \begin{bmatrix} \ell K_F (u_2 - u_4) \\ \ell K_F (u_3 - u_1) \\ K_\tau (u_1 - u_2 + u_3 - u_4) \end{bmatrix}
$$

---
