# Lecture 14

## Last time:
- SQP
- Direct Collocation

## Today:
- Algorithm Recap
- Dealing With Attitude (3D rotational algorithms)
- Linear/"Local" Control
  - "Linearization Works"
- Constraints?

---

## Recap: Deterministic Optimal Control Algorithms

### Linear/"Local" Control
- "Linearization Works"

### Constraints?

```
                Constraints?
                /          \
              no            yes
             /                \
          [LQR]              [MPC]
           / \                / \
       leading efficient   control-limited
      /       \            /          \
   [TVLQR]  [iLQR]      [QP]        [SQP]
```

---

## Nonlinear Trajectory Optimization

### DIRCOL vs DDP Comparison

| DIRCOL | DDP |
|--------|-----|
| Only respects dynamics at convergence | Always dynamically feasible |
| Can use any infeasible guess | Can only guess controls |
| Can handle arbitrary constraints | Hard to state constraints |
| Tracking controller must be designed separately | TVLQR tracking controller is free |
| Very fast (local) convergence | |
| Typically not as fast | |
| Difficult to implement large-scale SQP solver | Easy to implement on embedded systems |
| Numerically robust | Has issues with ill-conditioning |

---

## Attitude

### Why Attitude Matters
- Many robotic systems undergo large rotations
  - quadrotors, airplanes, spacecraft, underwater vehicles, legged robots

- Naive angle parameterizations (Euler angles) have singularities that cause failures and/or require hacks

- Rotation matrices and quaternions are singularity-free but optimizing over them requires some extra tricks

### What is Attitude?

```
     z
     ↑
     |    ^V_b
     |   /
     |  /
     | /
     |/
     +------→ x
```

$$^w V = Q^b V$$

- Rotation from body frame to world frame
- 3DOF, but there is no globally nonsingular 3-parameter attitude representation

### Rotation / "Direction-Cosine" Matrix

$$\begin{bmatrix} ^w x_1 \\ ^w x_2 \\ ^w x_3 \end{bmatrix} = \begin{bmatrix} ^b n_1^T \\ ^b n_2^T \\ ^b n_3^T \end{bmatrix} \begin{bmatrix} ^b x_1 \\ ^b x_2 \\ ^b x_3 \end{bmatrix}$$

$$= \begin{bmatrix} ^w b_1 & ^w b_2 & ^w b_3 \end{bmatrix} \begin{bmatrix} ^b x_1 \\ ^b x_2 \\ ^b x_3 \end{bmatrix}$$

where the middle matrix is $Q^b$.

$$Q^T Q = I$$ → "Orthogonal"
- $\Rightarrow Q^{-1} = Q^T$
- $\Rightarrow \det(Q) = 1$ → "Special"

$\Rightarrow$ **$Q \in SO(3)$** "Special Orthogonal group in 3D"

### Kinematics (how do I integrate a gyro)

```
     ω
     ↑
     |
  r ⟋ |
   (  |
   (  Bx )
   ⟍__⟋
```

$$^w X = Q(t)^b X, \quad ^w \dot{X} = ^w \omega \times ^w X$$

$$= Q(^b \omega \times ^b X)$$

- Apply product rule:

  $$^w \dot{X} = Q \omega ^b \dot{X}$$

  $$\Rightarrow ^w \dot{X} = \dot{Q}^b X + Q^b \dot{x}$$

  $$\Rightarrow \dot{Q}^b X = Q(^b \omega \times ^b X)$$

The cross product can be written as a skew-symmetric matrix:

$$\omega \times X = \begin{bmatrix} 0 & -\omega_3 & \omega_2 \\ \omega_3 & 0 & -\omega_1 \\ -\omega_2 & \omega_1 & 0 \end{bmatrix} \begin{bmatrix} x_1 \\ x_2 \\ x_3 \end{bmatrix} = \hat{\omega} X$$

where $\hat{\omega}$ is the skew-symmetric matrix.

$$\Rightarrow \dot{Q}^b X = Q\hat{\omega}^b X \Rightarrow \dot{Q} = Q^b\hat{\omega}$$

- We can do dynamics with rotation matrices in our state, but has a lot of redundancy

- Quaternions are more compact/efficient

---

## Quaternions

- Define axis of rotation (unit vector) $\mathbf{a}$
- Define angle of rotation (scalar, radians) $\theta$

$$\phi = a\theta$$
- "axis-angle" vector $\in \mathbb{R}^3$

$$\|\phi\| = \theta, \quad \phi/\|\phi\| = a$$

- For constant $\omega$ (for short $h$) can think of $\phi$ as integral of $\omega$:

  $$\phi \approx \int_0^h \omega(t) \, dt$$ (not true in general!)

- In terms of $\phi$, we can define quaternion:

$$q = \begin{bmatrix} \cos(\theta/2) \\ a \sin(\theta/2) \end{bmatrix}$$

where $\cos(\theta/2)$ is the "scalar part" and $a \sin(\theta/2)$ is the "vector part".

$$q^T q = 1 \Rightarrow \text{valid rotations correspond to unit quaternions}$$
- Easy to normalize

- $q$ and $-q$ correspond to the same rotation (add $2\pi$ to $\theta$)
  - Called a "double cover"

- Operations on quaternions are analogous to rotation matrices

### Quaternion Multiplication

$$q_1 * q_2 = \begin{bmatrix} s_1 \\ v_1 \end{bmatrix} * \begin{bmatrix} s_2 \\ v_2 \end{bmatrix} = \begin{bmatrix} s_1 s_2 - v_1^T v_2 \\ s_1 v_2 + s_2 v_1 + v_1 \times v_2 \end{bmatrix}$$

$$= \begin{bmatrix} s_1 I & -v_1^T \\ v_1 I & s_1 I + \hat{v}_1 \end{bmatrix} \begin{bmatrix} s_2 \\ v_2 \end{bmatrix} = \begin{bmatrix} s_2 I & -v_2^T \\ v_2 I & s_2 I - \hat{v}_2 \end{bmatrix} \begin{bmatrix} s_1 \\ v_1 \end{bmatrix}$$

where $L(q_1)$ and $R(q_2)$ denote the left and right quaternion multiplication matrices respectively.

$\rightarrow$ $Q(q_2 q_1)$

### Quaternion Conjugate

$$q^* = \begin{bmatrix} s \\ -v \end{bmatrix} = \bar{T} q = \begin{bmatrix} 1 & 0 \\ 0 & -I \end{bmatrix} \begin{bmatrix} s \\ v \end{bmatrix}$$

### Rotate a Vector:

$$\begin{bmatrix} 0 \\ X \end{bmatrix} = q * \begin{bmatrix} 0 \\ X \end{bmatrix} * q^+ = H^T L(q) R(q) H \, ^b X = H^T R(q) L(q) H \, ^b X$$

where the rotation matrix is defined as:

$$Q(q) = H^T R(q) L(q) H$$

$$H x = \begin{bmatrix} 0 \\ x \end{bmatrix} \Rightarrow H = \begin{bmatrix} 0 \\ I \end{bmatrix}$$

$$^w x = Q(q) ^b X$$

### Quaternion Kinematics:

$$\dot{q} = \frac{1}{2} q * \begin{bmatrix} 0 \\ \omega \end{bmatrix} = \frac{1}{2} L(q) H \omega$$

where $L(q) H$ is a $4 \times 3$ matrix.

- Now we can simulate dynamics with rotations
