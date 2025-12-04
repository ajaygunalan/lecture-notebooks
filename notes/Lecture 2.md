# Lecture 2

## Last Time:
- Continuous time Dynamics
- Manipulator Dynamics
- Linear Systems

## Today:
- Equilibria
- Stability
- Discrete-time Dynamics
- Simulation

---

## Equilibria

A point where the system will "remain at rest"

$$\implies \dot{x} = f(x, u) = 0$$

- Algebraically, roots of the dynamics

### Pendulum:

$$\dot{x} = \begin{bmatrix} \dot{\theta} \\ -\frac{g}{l} \sin(\theta) \end{bmatrix} = \begin{bmatrix} 0 \\ 0 \end{bmatrix}$$

$$\implies \dot{\theta} = 0, \quad \theta = 0, \pi$$

```
      θ = π
        |
    ----+----
        |
        O
      θ = 0
```

---

## First Control Problem

Can I move the equilibria?

$$\theta = \frac{\pi}{2}$$

$$\dot{x} = \begin{bmatrix} \dot{\theta} \\ -\frac{g}{l} \sin(\frac{\pi}{2}) + \frac{1}{ml^2} u \end{bmatrix} = \begin{bmatrix} 0 \\ 0 \end{bmatrix}$$

$$\implies \frac{1}{ml^2} u = \frac{g}{l} \sin(\frac{\pi}{2}) \implies u = mgl$$

In general, we get a root-finding problem in $u$:

$$f(x, u) = 0$$

---

## Stability of Equilibria

When will we stay "near" an equilibrium point under perturbations?

### Look at a 1D system ($x \in \mathbb{R}$):

```
         unstable    stable      unstable
          ←---→       ←---→       ←---→
    ------o-----------X-----------o------→ x
          |           |           |
          |    basin of attraction
```

- $\frac{\partial f}{\partial x} < 0 \implies$ stable
- $\frac{\partial f}{\partial x} > 0 \implies$ unstable

---

## In higher dimensions:

$\frac{\partial f}{\partial x}$ is a Jacobian matrix

- Take an eigendecomposition $\implies$ decouple into $n$ 1D systems

$$\text{Re}\left[\text{eigvals}\left(\frac{\partial f}{\partial x}\right)\right] < 0 \implies \text{stable}$$

otherwise $\implies$ unstable

### Pendulum:

$$f(x) = \begin{bmatrix} \dot{\theta} \\ -\frac{g}{l} \sin(\theta) \end{bmatrix}$$

$$\implies \frac{\partial f}{\partial x} = \begin{bmatrix} 0 & 1 \\ -\frac{g}{l} \cos(\theta) & 0 \end{bmatrix}$$

At $\theta = \pi$:

$$\left.\frac{\partial f}{\partial x}\right|_{\theta=\pi} = \begin{bmatrix} 0 & 1 \\ \frac{g}{l} & 0 \end{bmatrix}$$

$$\implies \text{eigvals}\left(\frac{\partial f}{\partial x}\right) = \pm \sqrt{\frac{g}{l}}$$

$\implies$ Unstable

At $\theta = 0$:

$$\left.\frac{\partial f}{\partial x}\right|_{\theta=0} = \begin{bmatrix} 0 & 1 \\ -\frac{g}{l} & 0 \end{bmatrix}$$

$$\implies \text{eigvals}\left(\frac{\partial f}{\partial x}\right) = \pm i\sqrt{\frac{g}{l}}$$

$\implies$ undamped oscillation

- Add damping (e.g. $u = -k_d \dot{\theta}$) results in strictly negative real part.

---

## Discrete-Time Dynamics

### Motivation:

- In general, we can't solve $\dot{x}=f(x)$ for $x(t)$
- Computationally, need to represent $x(t)$ with discrete $x_n$
- Discrete-time models can capture some effects that continuous ODEs can't

### "Explicit Form":

$$x_{n+1} = f_d(x_n, u_n)$$

$\leftarrow$ "discrete"

### Simplest discretization:

$$x_{n+1} = x_n + h f(x_n, u_n)$$

$\leftarrow$ "Forward Euler Integration"

$\leftarrow$ $h$ "timestep"

### Pendulum Sim:

$l = m = 1, \quad h = 0.1, 0.01$

**blows up!**

---

## Stability of Discrete-Time Systems

In discrete time, dynamics is an iterated map:

$$x_n = f_d(f_d(f_d(\cdots f_d(x_0))))$$

### Linearize & apply chain rule:

$$\frac{\partial x_n}{\partial x_0} = \left.\frac{\partial f_d}{\partial x}\right|_{x_0} \left.\frac{\partial f_d}{\partial x}\right|_{x_1} \cdots \left.\frac{\partial f_d}{\partial x}\right|_{x_n} = A_d^N$$

Assume $x^* = 0$ is an equilibrium

$$\text{Stable} \implies \lim_{k \to \infty} A_d^k x_0 = 0 \quad \forall x_0$$

$$\implies \lim_{n \to \infty} A_d^n = 0$$

$$\implies |\text{eigvals}(A_d)| < 1$$

(inside unit circle)

### Pendulum with Forward Euler:

$$x_{n+1} = x_n + h f(x_n)$$

$\leftarrow$ $f_d(x_n)$

$$A_d = \frac{\partial f_d}{\partial x} = I + hA = I + h\begin{bmatrix} 0 & 1 \\ -\frac{g}{l} \cos(\theta) & 0 \end{bmatrix}$$

$$\text{eigvals}(A_d|_{\theta=0}) = 1 \pm 0.313i$$

---

## Intuition:

```
     θ
     ^
     |     /\    Linear approximation
     |    /  \   always overshoots
     |   /    \
     |  /      \/
     +----------------→ t
```

**linear approximation always overshoots**

### Take aways:

- Be careful
- Always sanity check e.g. energy, momentum behavior
- Never use forward Euler!

---

## A better explicit integrator:

### 4th-order Runge-Kutta Method

- RK4 fits a cubic polynomial to $x(t)$ rather than a line
  $\implies$ much better accuracy!

### Pseudo-code:

$$x_{n+1} = f_d(x_n)$$

$$k_1 = f(x_n)$$

$$k_2 = f(x_n + \frac{h}{2} k_1)$$

$$k_3 = f(x_n + \frac{h}{2} k_2)$$

$$k_4 = f(x_n + h k_3)$$

$$x_{n+1} = x_n + \frac{h}{6} (k_1 + 2k_2 + 2k_3 + k_4)$$

---

## Take Away:

- Accuracy $\gg$ additional compute cost
- Even "good" integrators have issues
  $\implies$ always sanity check
