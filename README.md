# Pendulum Control Project

This project aims to explore different control techniques for pendulum systems, including PID, Model Predictive Control (MPC), and Deep Learning methods. The project will involve simulations using Python and Gazebo, as well as real-world implementation using a Raspberry Pi and ODrive 3.6 motor controller.

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
