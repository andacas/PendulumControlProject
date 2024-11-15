# Pendulum Control Project

This project aims to explore different control techniques for pendulum systems, including PID, Model Predictive Control (MPC), and Deep Learning methods. The project will involve simulations using Python and Gazebo, as well as real-world implementation using a Raspberry Pi and ODrive 3.6 motor controller.

---

# Phase 1.1: Develop Dynamic Model

In this phase, we will derive the dynamic equations of motion for both single and double pendulum systems using Newtonian and Lagrangian mechanics. This detailed derivation will serve as the foundation for our simulations and control system designs.

---

## Table of Contents

- [Single Pendulum](#single-pendulum)
  - [Newtonian Mechanics](#newtonian-mechanics-single)
  - [Lagrangian Mechanics](#lagrangian-mechanics-single)
- [Double Pendulum](#double-pendulum)
  - [Newtonian Mechanics](#newtonian-mechanics-double)
  - [Lagrangian Mechanics](#lagrangian-mechanics-double)


---

## Single Pendulum<a name="single-pendulum"></a>

### System Description

A single pendulum consists of a mass \( m \) attached to the end of a rigid, massless rod of length \( L \), which is pivoted at the other end. The pendulum swings under the influence of gravity.

---

### Newtonian Mechanics<a name="newtonian-mechanics-single"></a>

#### Step 1: Define Coordinate System

- **Figure 1:** Imagine the pendulum swinging in a plane. Let \( \theta \) be the angle between the rod and the vertical downward direction.
  
#### Step 2: Draw Free-Body Diagram

- **Figure 2:** At any angle \( \theta \), the forces acting on the mass \( m \) are:
  - Gravitational force \( \mathbf{F}_g = m\mathbf{g} \) downward.
  - Tension \( \mathbf{T} \) along the rod.

#### Step 3: Apply Newton's Second Law

- The motion is constrained to a circular arc, so we consider tangential and radial components.
  
##### Radial Direction:
 $$ T - mg\cos\theta = m \left( -L\dot{\theta}^2 \right) $$

##### Tangential Direction:
$$ -mg\sin\theta = m \left( L\ddot{\theta} \right) $$

#### Step 4: Simplify the Equations

- The tension $T$ can be eliminated if focusing on the tangential equation.
  
##### The equation of motion is:
$$\ddot{\theta} + \frac{g}{L} \sin\theta = 0$$

#### Summary

##### The nonlinear differential equation governing the single pendulum is:
$$\ddot{\theta} + \frac{g}{L} \sin\theta = 0$$

---

### Lagrangian Mechanics<a name="lagrangian-mechanics-single"></a>

#### Step 1: Define Generalized Coordinates

- Use $\theta$ as the generalized coordinate.

#### Step 2: Write Expressions for Kinetic and Potential Energy

- **Kinetic Energy (T):**
$$T = \frac{1}{2} m v^2 = \frac{1}{2} m (L\dot{\theta})^2 = \frac{1}{2} m L^2 \dot{\theta}^2$$

- **Potential Energy (V):**
$$V = m g h = m g L (1 - \cos\theta)$$

#### Step 3: Write the Lagrangian (L)

$$L = T - V$$

$$L = \frac{1}{2} m L^2 \dot{\theta}^2 - m g L (1 - \cos\theta)$$

#### Step 4: Apply Euler-Lagrange Equation

- The Euler-Lagrange equation is:
  $$\frac{d}{dt} \left( \frac{\partial L}{\partial \dot{\theta}} \right) - \frac{\partial L}{\partial \theta} = 0$$

- Compute $\frac{\partial L}{\partial \dot{\theta}}$ and $\frac{\partial L}{\partial \theta}$:

  
  $$\frac{\partial L}{\partial \dot{\theta}} = m L^2 \dot{\theta}$$
  
  
  $$\frac{\partial L}{\partial \theta} = -m g L \sin\theta$$

- Compute the time derivative:

  $$\frac{d}{dt} \left( \frac{\partial L}{\partial \dot{\theta}} \right) = m L^2 \ddot{\theta}$$

#### Step 5: Derive Equation of Motion

- Substitute into Euler-Lagrange equation:

  
  $$m L^2 \ddot{\theta} + m g L \sin\theta = 0$$

- Simplify:

  $$\ddot{\theta} + \frac{g}{L} \sin\theta = 0$$

#### Summary

- The Lagrangian approach yields the same equation of motion as the Newtonian method.

---

## Double Pendulum<a name="double-pendulum"></a>

### System Description

A double pendulum consists of two masses $m_1$  and  $m_2$ attached to two rigid, massless rods of lengths  $L_1$ and $L_2$ . The first mass is attached to a pivot, and the second mass hangs from the first mass.

---

### Newtonian Mechanics<a name="newtonian-mechanics-double"></a>

Deriving the equations of motion using Newtonian mechanics for a double pendulum is complex due to the coupling between the two pendulums. Here, we will outline the steps.

#### Step 1: Define Coordinate System

- **Figure 3:** Let $\theta_1$ be the angle of the first rod with the vertical, and $\theta_2$ be the angle of the second rod relative to the vertical.

#### Step 2: Determine Positions

- Position of $m_1$ :

  $$x_1 = L_1 \sin\theta_1, \quad y_1 = -L_1 \cos\theta_1$$

- Position of $m_2$ :

  $$x_2 = x_1 + L_2 \sin\theta_2, \quad y_2 = y_1 - L_2 \cos\theta_2$$

#### Step 3: Compute Velocities

- Velocity of $m_1$ :

  $$\dot{x}_1 = L_1 \cos\theta_1 \dot{\theta}_1, \quad \dot{y}_1 = L_1 \sin\theta_1 \dot{\theta}_1$$

- Velocity of $m_2$:

  $$\dot{x}_2 = \dot{x}_1 + L_2 \cos\theta_2 \dot{\theta}_2, \quad \dot{y}_2 = \dot{y}_1 + L_2 \sin\theta_2 \dot{\theta}_2$$

#### Step 4: Apply Newton's Second Law

- Write equations for each mass considering the forces and accelerations.
- This involves setting up equations in the tangential and radial directions for each mass.

#### Step 5: Derive Equations of Motion

- After some algebraic manipulation, the equations of motion are:

  
  $$(m_1 + m_2) L_1 \ddot{\theta}_1 + m_2 L_2 \ddot{\theta}_2 \cos(\theta_1 - \theta_2) + m_2 L_2 \dot{\theta}_2^2 \sin(\theta_1 - \theta_2) + (m_1 + m_2) g \sin\theta_1 = 0$$
  
  
  $$m_2 L_2 \ddot{\theta}_2 + m_2 L_1 \ddot{\theta}_1 \cos(\theta_1 - \theta_2) - m_2 L_1 \dot{\theta}_1^2 \sin(\theta_1 - \theta_2) + m_2 g \sin\theta_2 = 0$$

#### Summary

- The equations are coupled second-order nonlinear differential equations.

---

### Lagrangian Mechanics<a name="lagrangian-mechanics-double"></a>

The Lagrangian method simplifies the derivation for complex systems like the double pendulum.

#### Step 1: Define Generalized Coordinates

- Use $\theta_1$  and $\theta_2$  as the generalized coordinates.

#### Step 2: Write Expressions for Kinetic and Potential Energy

##### Kinetic Energy (T):

- For $m_1$ :

  
  $$T_1 = \frac{1}{2} m_1 \left( \dot{x}_1^2 + \dot{y}_1^2 \right)$$
  
- For $m_2$:

  $$T_2 = \frac{1}{2} m_2 \left( \dot{x}_2^2 + \dot{y}_2^2 \right)$$

- Total kinetic energy:

  
  $$T = T_1 + T_2$$

##### Potential Energy (V):

- For $m_1$ :

  
  $$V_1 = m_1 g y_1$$

- For $m_2$ :

  
  $$V_2 = m_2 g y_2$$

- Total potential energy:

  
  $$V = V_1 + V_2$$

#### Step 3: Compute the Lagrangian (L)

- $$L = T - V$$ 

#### Step 4: Derive Equations of Motion Using Euler-Lagrange Equations

- Compute $\frac{\partial L}{\partial \theta_i}$ and  $\frac{\partial L}{\partial \dot{\theta}_i}$ for  $i = 1, 2$ .
- Apply the Euler-Lagrange equation for each coordinate:

  
  $$\frac{d}{dt} \left( \frac{\partial L}{\partial \dot{\theta}_i} \right) - \frac{\partial L}{\partial \theta_i} = 0$$

#### Step 5: Final Equations of Motion

- The resulting equations are:

  
  $$(m_1 + m_2) L_1 \ddot{\theta}_1 + m_2 L_2 \ddot{\theta}_2 \cos(\theta_1 - \theta_2) + m_2 L_2 \dot{\theta}_2^2 \sin(\theta_1 - \theta_2) + (m_1 + m_2) g \sin\theta_1 = 0$$
  
  
  $$m_2 L_2 \ddot{\theta}_2 + m_2 L_1 \ddot{\theta}_1 \cos(\theta_1 - \theta_2) - m_2 L_1 \dot{\theta}_1^2 \sin(\theta_1 - \theta_2) + m_2 g \sin\theta_2 = 0$$



---

## Roadmap

### Phase 1: Mathematical Modeling

- [ ] **1.1 Develop Dynamic Model**
  - Derive the mathematical equations governing the pendulum's motion using Newtonian or Lagrangian mechanics.
- [ ] **1.2 Linearize the Model**
  - Linearize the nonlinear dynamic model around the equilibrium point to simplify control design.
- [ ] **1.3 Validate the Model**
  - Use analytical or numerical methods to ensure the model accurately represents the physical system.
- [ ] **1.4 Document the Model**
  - Write detailed documentation explaining the derivation and assumptions of the model.

### Phase 2: Simulation in Python

- [ ] **2.1 Implement Model in Python**
  - Translate the mathematical model into Python code using libraries like NumPy and SciPy.
- [ ] **2.2 Create Simulation Environment**
  - Develop a simulation environment to test the pendulum's behavior over time.
- [ ] **2.3 Visualize Results**
  - Use Matplotlib or similar libraries to plot the pendulum's angle, angular velocity, etc.
- [ ] **2.4 Add User Inputs**
  - Allow users to input different parameters and initial conditions.

### Phase 3: 3D Modeling and Gazebo Setup

- [ ] **3.1 Design 3D Parts in Inventor**
  - Create detailed 3D models of the pendulum and its components using Autodesk Inventor.
- [ ] **3.2 Export Models to URDF**
  - Convert the 3D models into URDF (Unified Robot Description Format) files compatible with Gazebo.
- [ ] **3.3 Set Up Gazebo Simulation**
  - Import URDF models into Gazebo and configure the simulation environment.
- [ ] **3.4 Integrate with ROS**
  - If necessary, set up ROS (Robot Operating System) for better simulation control.

### Phase 4: Implementing Control Techniques

#### 4.1 PID Control

- [ ] **4.1.1 Design PID Controller**
  - Develop a PID control algorithm based on the linearized model.
- [ ] **4.1.2 Simulate PID Control in Python**
  - Test the PID controller within the Python simulation environment.
- [ ] **4.1.3 Tune PID Parameters**
  - Adjust the proportional, integral, and derivative gains for optimal performance.
- [ ] **4.1.4 Apply PID Control in Gazebo**
  - Implement the PID controller in the Gazebo simulation.

#### 4.2 Model Predictive Control (MPC)

- [ ] **4.2.1 Formulate MPC Problem**
  - Define the cost function, constraints, and prediction horizon for the MPC.
- [ ] **4.2.2 Implement MPC in Python**
  - Use libraries like `cvxpy` to solve the MPC optimization problem.
- [ ] **4.2.3 Simulate MPC Control**
  - Test the MPC controller in the Python simulation.
- [ ] **4.2.4 Validate MPC in Gazebo**
  - Apply the MPC controller in the Gazebo simulation.

#### 4.3 Deep Learning Control

- [ ] **4.3.1 Explore Deep Learning Techniques**
  - Research methods like reinforcement learning for control applications.
- [ ] **4.3.2 Implement Deep Learning Model**
  - Use TensorFlow or PyTorch to develop a neural network controller.
- [ ] **4.3.3 Train the Model**
  - Train the model using simulation data.
- [ ] **4.3.4 Test Deep Learning Control**
  - Evaluate the performance of the deep learning controller in simulation.

### Phase 5: Hardware Implementation

- [ ] **5.1 Assemble Physical System**
  - Build the pendulum setup using the designed 3D parts and hardware components.
- [ ] **5.2 Set Up Raspberry Pi**
  - Install the necessary software and libraries on the Raspberry Pi.
- [ ] **5.3 Configure ODrive 3.6**
  - Set up the ODrive motor controller to interface with the Raspberry Pi.
- [ ] **5.4 Develop Control Software**
  - Write code to implement control algorithms on the Raspberry Pi.
- [ ] **5.5 Test PID Control on Hardware**
  - Apply the PID controller to the physical system and observe performance.
- [ ] **5.6 Implement MPC on Hardware**
  - Adapt the MPC algorithm for real-time control on the Raspberry Pi.
- [ ] **5.7 Deploy Deep Learning Model**
  - Run the trained deep learning controller on the physical system.

### Phase 6: Analysis and Optimization

- [ ] **6.1 Collect Data**
  - Record system responses, errors, and performance metrics during tests.
- [ ] **6.2 Compare Control Techniques**
  - Analyze the effectiveness of each control method based on collected data.
- [ ] **6.3 Optimize Controllers**
  - Refine control algorithms and parameters to improve performance.
- [ ] **6.4 Document Findings**
  - Summarize the results, challenges, and insights gained from the project.

### Phase 7: Documentation and Sharing

- [ ] **7.1 Update README**
  - Provide clear instructions and information in the README file.
- [ ] **7.2 Create Tutorials**
  - Develop step-by-step guides for replicating simulations and experiments.
- [ ] **7.3 Record Demonstrations**
  - Capture videos of simulations and hardware tests to showcase results.
- [ ] **7.4 Share Code and Models**
  - Ensure all code and models are pushed to the GitHub repository.


---

## Tools and Technologies

- **Programming Languages**
  - Python
- **Simulation Software**
  - Gazebo
- **3D Modeling Software**
  - Autodesk Inventor
- **Hardware Components**
  - Raspberry Pi
  - ODrive 3.6 motor controller
  - Motors, sensors, and mechanical parts
- **Python Libraries**
  - NumPy, SciPy, Matplotlib
  - Control systems libraries (`python-control`)
  - Optimization libraries (`cvxpy`)
  - Deep learning frameworks (TensorFlow, PyTorch)
