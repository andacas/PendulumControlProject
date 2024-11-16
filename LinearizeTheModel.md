# Phase 1.2: Linearize the Model

In this phase, we will linearize the nonlinear equations of motion derived in Phase 1.1 around the equilibrium points. Linearization simplifies the equations, making them more tractable for analysis and control design.

---

## Table of Contents

- [Single Pendulum Linearization](#single-pendulum-linearization)
  - [Equilibrium Points](#equilibrium-points-single)
  - [Linearization around \(\theta = 0\)](#linearization-theta0)
  - [Linearization around \(\theta = \pi\)](#linearization-thetapi)
- [Double Pendulum Linearization](#double-pendulum-linearization)
  - [Equilibrium Points](#equilibrium-points-double)
  - [Linearization around \(\theta_1 = 0, \theta_2 = 0\)](#linearization-theta0-double)
  - [Linearization around \(\theta_1 = \pi, \theta_2 = \pi\)](#linearization-thetapi-double)
- [Next Steps](#next-steps)

---

## Single Pendulum Linearization<a name="single-pendulum-linearization"></a>

### Equilibrium Points<a name="equilibrium-points-single"></a>

The single pendulum has two equilibrium points:

1. **Stable Equilibrium:** $\theta = 0$ (pendulum hanging downward)
2. **Unstable Equilibrium:** $\theta = \pi$ (pendulum inverted upward)

---

### Equation of Motion

Recall the nonlinear equation of motion for the single pendulum:

$$
\ddot{\theta} + \frac{g}{L} \sin\theta = 0
$$

---

### Linearization around $\theta = 0$<a name="linearization-theta0"></a>

#### Step 1: Taylor Series Expansion of $\sin\theta$

Around $\theta = 0$, the Taylor series of $\sin\theta$ is:

$$
\sin\theta = \theta - \frac{\theta^3}{6} + \frac{\theta^5}{120} - \cdots
$$

For small angles $(\theta$ near zero), higher-order terms can be neglected:

$$
\sin\theta \approx \theta
$$

#### Step 2: Substitute Linear Approximation into the Equation

Substitute $\sin\theta \approx \theta$ into the equation of motion:

$$
\ddot{\theta} + \frac{g}{L} \theta = 0
$$

#### Step 3: Obtain the Linearized Equation

The linearized equation around $\theta = 0$ is:

$$
\ddot{\theta} + \frac{g}{L} \theta = 0
$$

This is a linear second-order differential equation representing simple harmonic motion.

#### Step 4: State-Space Representation

Define state variables:

$$
x_1 = \theta, \quad x_2 = \dot{\theta}
$$

State-space equations:

$$
\begin{cases}
\dot{x}_1 = x_2 \\
\dot{x}_2 = -\dfrac{g}{L} x_1
\end{cases}
$$

#### Step 5: Matrix Form

Express the system in matrix form:

$$
\dot{\mathbf{x}} = \mathbf{A} \mathbf{x}
$$

where:

$$
\mathbf{x} = \begin{bmatrix} 
x_1 \\
x_2 \\
\end{bmatrix},
\quad
\mathbf{A} = \begin{bmatrix}
0 & 1 \\
-\dfrac{g}{L} & 0 \\
\end{bmatrix}
$$


---

### Linearization around $\theta = \pi$<a name="linearization-thetapi"></a>

#### Step 1: Coordinate Transformation

Let $\phi = \theta - \pi$. Then, when $\theta = \pi$, $\phi = 0$.

#### Step 2: Taylor Series Expansion of $\sin\theta$

Since $\theta = \phi + \pi$:

$$
\sin\theta = \sin(\phi + \pi) = -\sin\phi
$$

For small $\phi$:

$$
\sin\phi \approx \phi
$$

Thus:

$$
\sin\theta \approx -\phi
$$

#### Step 3: Substitute into the Equation of Motion

Substitute $\theta = \phi + \pi$ and $\sin\theta \approx -\phi$:

$$
\ddot{\phi} + \frac{g}{L} (-\phi) = 0
$$

Simplify:

$$
\ddot{\phi} - \frac{g}{L} \phi = 0
$$

#### Step 4: Obtain the Linearized Equation

The linearized equation around $\theta = \pi$ is:

$$
\ddot{\phi} - \frac{g}{L} \phi = 0
$$

This represents an inverted pendulum, where the equilibrium is unstable.

#### Step 5: State-Space Representation

Define state variables:

$$
x_1 = \phi, \quad x_2 = \dot{\phi}
$$

State-space equations:

$$
\begin{cases}
\dot{x}_1 = x_2 \\
\dot{x}_2 = \dfrac{g}{L} x_1
\end{cases}
$$

Matrix form:

$$
\dot{\mathbf{x}} = \mathbf{A} \mathbf{x}
$$

where:

$$
\mathbf{x} = 
\begin{bmatrix} x_1 \\
x_2 \\
\end{bmatrix},
\quad 
\mathbf{A} = 
\begin{bmatrix} 0 & 1 \\
\dfrac{g}{L} & 0 \\
\end{bmatrix}
$$

---

## Double Pendulum Linearization<a name="double-pendulum-linearization"></a>

### Equilibrium Points<a name="equilibrium-points-double"></a>

For the double pendulum, we consider two equilibrium positions:

1. **Both Pendulums Downward (Stable Equilibrium):**

   $$(\theta_1 = 0\), \(\theta_2 = 0)$$

2. **Both Pendulums Upward (Unstable Equilibrium):**

   $$(\theta_1 = \pi\), \(\theta_2 = \pi)$$

---

### Equations of Motion

Recall the nonlinear equations derived using Lagrangian mechanics:

$$(m_1 + m_2) L_1 \ddot{\theta}_1 + m_2 L_2 \ddot{\theta}_2 \cos(\theta_1 - \theta_2) + m_2 L_2 \dot{\theta}_2^2 \sin(\theta_1 - \theta_2) + (m_1 + m_2) g \sin\theta_1 = 0$$

$$m_2 L_2 \ddot{\theta}_2 + m_2 L_1 \ddot{\theta}_1 \cos(\theta_1 - \theta_2) - m_2 L_1 \dot{\theta}_1^2 \sin(\theta_1 - \theta_2) + m_2 g \sin\theta_2 = 0$$

---

### Linearization around $\theta_1 = 0, \theta_2 = 0$<a name="linearization-theta0-double"></a>

#### Step 1: Small Angle Approximations

For small $\theta_1$ and $\theta_2$:

- $\sin\theta_i \approx \theta_i$
- $\cos\theta_i \approx 1$
- $\sin(\theta_1 - \theta_2) \approx \theta_1 - \theta_2$
- $\cos(\theta_1 - \theta_2) \approx 1$

#### Step 2: Simplify the Equations

Simplify the equations using the approximations:

1. **First Equation:**

   $$(m_1 + m_2) L_1 \ddot{\theta}_1 + m_2 L_2 \ddot{\theta}_2 + m_2 L_2 \dot{\theta}_2^2 (\theta_1 - \theta_2) + (m_1 + m_2) g \theta_1 = 0$$

   Since $\dot{\theta}_2^2 (\theta_1 - \theta_2)$ is a product of small quantities, we neglect it:

   $$(m_1 + m_2) L_1 \ddot{\theta}_1 + m_2 L_2 \ddot{\theta}_2 + (m_1 + m_2) g \theta_1 = 0$$

2. **Second Equation:**

   $$m_2 L_2 \ddot{\theta}_2 + m_2 L_1 \ddot{\theta}_1 - m_2 L_1 \dot{\theta}_1^2 (\theta_1 - \theta_2) + m_2 g \theta_2 = 0$$

   Again, neglect $\dot{\theta}_1^2 (\theta_1 - \theta_2)$:

   $$m_2 L_2 \ddot{\theta}_2 + m_2 L_1 \ddot{\theta}_1 + m_2 g \theta_2 = 0$$

#### Step 3: Write in Matrix Form

Combine the equations:

$$
\begin{bmatrix}
(m_1 + m_2) L_1 & m_2 L_2 \\
m_2 L_1 & m_2 L_2 \\
\end{bmatrix}
\begin{bmatrix}
\ddot{\theta}_1 \\
\ddot{\theta}_2 \\
\end{bmatrix}
+
\begin{bmatrix}
(m_1 + m_2) g & 0 \\
0 & m_2 g \\
\end{bmatrix}
\begin{bmatrix}
\theta_1 \\
\theta_2 \\
\end{bmatrix}
= \begin{bmatrix}
0 \\
0 \\
\end{bmatrix}
$$

Let:

$$
M = 
\begin{bmatrix}
(m_1 + m_2) L_1 & m_2 L_2 \\
m_2 L_1 & m_2 L_2 \\
\end{bmatrix}
$$

$$
K = 
\begin{bmatrix} 
(m_1 + m_2) g & 0 \\
0 & m_2 g \\ 
\end{bmatrix}
$$

$$
\Theta = 
\begin{bmatrix} 
\theta_1 \\
\theta_2 \\
\end{bmatrix}
$$

$$
\ddot{\Theta} =
\begin{bmatrix} 
\ddot{\theta}_1 \\
\ddot{\theta}_2 \\
\end{bmatrix}
$$

Then:

$$M \ddot{\Theta} + K \Theta = 0$$

#### Step 4: State-Space Representation

Let the state vector be:

$$
\mathbf{x} =
\begin{bmatrix} 
\Theta \\
\dot{\Theta} \\
\end{bmatrix}
$$

The state-space equations are:

$$
\dot{\mathbf{x}} =
\begin{bmatrix}
\dot{\Theta} \\
-M^{-1} K \Theta \\
\end{bmatrix}
$$

This representation allows us to analyze and simulate the system using linear control techniques.

---

### Linearization around $\theta_1 = \pi, \theta_2 = \pi$<a name="linearization-thetapi-double"></a>

#### Step 1: Coordinate Transformation

Let:

$$
\phi_1 = \theta_1 - \pi, \quad \phi_2 = \theta_2 - \pi
$$

Then, when $\theta_1 = \pi$ and $\theta_2 = \pi$, $\phi_1 = 0$ and $\phi_2 = 0$.

#### Step 2: Small Angle Approximations

For small $\phi_1$ and $\phi_2$:

- $\sin\theta_i = \sin(\phi_i + \pi) = -\sin\phi_i \approx -\phi_i$
- $\cos\theta_i = \cos(\phi_i + \pi) = -\cos\phi_i \approx -1$
- $\sin(\theta_1 - \theta_2) = \sin(\phi_1 - \phi_2) \approx \phi_1 - \phi_2$
- $\cos(\theta_1 - \theta_2) = \cos(\phi_1 - \phi_2) \approx 1$

#### Step 3: Simplify the Equations

1. **First Equation:**

   $$(m_1 + m_2) L_1 \ddot{\phi}_1 + m_2 L_2 \ddot{\phi}_2 + m_2 L_2 \dot{\phi}_2^2 (\phi_1 - \phi_2) - (m_1 + m_2) g \phi_1 = 0$$

   Neglecting the product of small quantities:

   $$(m_1 + m_2) L_1 \ddot{\phi}_1 + m_2 L_2 \ddot{\phi}_2 - (m_1 + m_2) g \phi_1 = 0$$

2. **Second Equation:**

   $$m_2 L_2 \ddot{\phi}_2 + m_2 L_1 \ddot{\phi}_1 - m_2 L_1 \dot{\phi}_1^2 (\phi_1 - \phi_2) - m_2 g \phi_2 = 0$$

   Neglecting the product of small quantities:

   $$m_2 L_2 \ddot{\phi}_2 + m_2 L_1 \ddot{\phi}_1 - m_2 g \phi_2 = 0$$

#### Step 4: Write in Matrix Form

Let:

$$
M =
\begin{bmatrix}
(m_1 + m_2) L_1 & m_2 L_2 \\
m_2 L_1 & m_2 L_2 \\
\end{bmatrix}
$$

$$
K =
\begin{bmatrix}
(m_1 + m_2) g & 0 \\
0 & -m_2 g \\
\end{bmatrix}
$$

$$
\Phi =
\begin{bmatrix}
\phi_1 \\
\phi_2 \\
\end{bmatrix}
$$

Then:

$$
M \ddot{\Phi} + K \Phi = 0
$$

#### Step 5: State-Space Representation

Define the state vector:

$$
\mathbf{x} = 
\begin{bmatrix} 
\Phi \\ 
\dot{\Phi} \\
\end{bmatrix}
$$

The state-space equations are:

$$
\dot{\mathbf{x}} = 
\begin{bmatrix} 
\dot{\Phi} \\
-M^{-1} K \Phi \\
\end{bmatrix}
$$

This linear representation is suitable for designing controllers for the inverted double pendulum.

---


[Back to Phase 1.1: Develop Dynamic Model](#)
