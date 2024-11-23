# main.py

import numpy as np
import matplotlib.pyplot as plt
from pendulum_model import (
    NonlinearPendulum, LinearizedPendulum,
    NonlinearDoublePendulum, LinearizedDoublePendulum
)

def main():
    # User inputs
    pendulum_type = input("Select pendulum type (single/double): ").strip().lower()
    model_type = input("Select model (nonlinear/linearized): ").strip().lower()
    damping = float(input("Enter damping coefficient (e.g., 0.1): "))
    initial_angle = float(input("Enter initial angle (degrees): "))
    initial_velocity = float(input("Enter initial angular velocity (deg/s): "))
    simulation_time = float(input("Enter simulation time (s): "))

    # Convert degrees to radians
    initial_angle_rad = np.radians(initial_angle)
    initial_velocity_rad = np.radians(initial_velocity)

    # Simulation parameters
    gravity = 9.81
    t_span = (0, simulation_time)
    t_eval = np.linspace(t_span[0], t_span[1], 1000)

    if pendulum_type == 'single':
        length = 1.0  # Length in meters
        mass = 1.0    # Mass in kg
        y0 = [initial_angle_rad, initial_velocity_rad]

        # Instantiate model
        if model_type == 'nonlinear':
            pendulum = NonlinearPendulum(length, mass, gravity, damping)
        elif model_type == 'linearized':
            pendulum = LinearizedPendulum(length, mass, gravity, damping)
        else:
            print("Invalid model type selected.")
            return

        # Run simulation
        sol = pendulum.simulate(y0, t_span, t_eval)

        # Extract results
        theta = sol.y[0]
        omega = sol.y[1]

        # Compute angular acceleration
        if model_type == 'nonlinear':
            alpha = - (pendulum.gravity / pendulum.length) * np.sin(theta) - \
                    (pendulum.damping / (pendulum.mass * pendulum.length**2)) * omega
        elif model_type == 'linearized':
            alpha = - (pendulum.gravity / pendulum.length) * theta - \
                    (pendulum.damping / (pendulum.mass * pendulum.length**2)) * omega

        # Convert back to degrees
        theta_deg = np.degrees(theta)
        omega_deg = np.degrees(omega)
        alpha_deg = np.degrees(alpha)

        # Plotting
        plt.figure(figsize=(12, 8))

        plt.subplot(3, 1, 1)
        plt.plot(sol.t, theta_deg)
        plt.title('Single Pendulum Simulation')
        plt.ylabel('Angle (degrees)')
        plt.grid(True)

        plt.subplot(3, 1, 2)
        plt.plot(sol.t, omega_deg)
        plt.ylabel('Angular Velocity (deg/s)')
        plt.grid(True)

        plt.subplot(3, 1, 3)
        plt.plot(sol.t, alpha_deg)
        plt.xlabel('Time (s)')
        plt.ylabel('Angular Acceleration (deg/s²)')
        plt.grid(True)

    elif pendulum_type == 'double':
        lengths = (1.0, 1.0)
        masses = (1.0, 1.0)
        damping1 = damping
        damping2 = damping  # For simplicity, use the same damping for both pendulums

        initial_angle2 = float(input("Enter initial angle for second pendulum (degrees): "))
        initial_velocity2 = float(input("Enter initial angular velocity for second pendulum (deg/s): "))

        initial_angle2_rad = np.radians(initial_angle2)
        initial_velocity2_rad = np.radians(initial_velocity2)

        y0 = [initial_angle_rad, initial_velocity_rad, initial_angle2_rad, initial_velocity2_rad]

        # Instantiate model
        if model_type == 'nonlinear':
            pendulum = NonlinearDoublePendulum(lengths, masses, gravity, (damping1, damping2))
        elif model_type == 'linearized':
            pendulum = LinearizedDoublePendulum(lengths, masses, gravity, (damping1, damping2))
        else:
            print("Invalid model type selected.")
            return

        # Run simulation
        sol = pendulum.simulate(y0, t_span, t_eval)

        # Extract results
        theta1 = sol.y[0]
        omega1 = sol.y[1]
        theta2 = sol.y[2]
        omega2 = sol.y[3]

        # Convert back to degrees
        theta1_deg = np.degrees(theta1)
        omega1_deg = np.degrees(omega1)
        theta2_deg = np.degrees(theta2)
        omega2_deg = np.degrees(omega2)

        # Plotting
        plt.figure(figsize=(12, 8))

        plt.subplot(2, 2, 1)
        plt.plot(sol.t, theta1_deg)
        plt.title('Double Pendulum Simulation')
        plt.ylabel('Theta1 (degrees)')
        plt.grid(True)

        plt.subplot(2, 2, 2)
        plt.plot(sol.t, theta2_deg)
        plt.title('Double Pendulum Simulation')
        plt.ylabel('Theta2 (degrees)')
        plt.grid(True)

        plt.subplot(2, 2, 3)
        plt.plot(sol.t, omega1_deg)
        plt.xlabel('Time (s)')
        plt.ylabel('Omega1 (deg/s)')
        plt.grid(True)

        plt.subplot(2, 2, 4)
        plt.plot(sol.t, omega2_deg)
        plt.xlabel('Time (s)')
        plt.ylabel('Omega2 (deg/s)')
        plt.grid(True)

    else:
        print("Invalid pendulum type selected.")
        return

    plt.tight_layout()
    plt.show()

    # Logging data
    if pendulum_type == 'single':
        data = np.vstack((sol.t, theta_deg, omega_deg, alpha_deg)).T
        np.savetxt('single_pendulum_data.csv', data, delimiter=',', header='Time,Angle,Angular Velocity,Angular Acceleration', comments='')
    else:
        data = np.vstack((sol.t, theta1_deg, omega1_deg, theta2_deg, omega2_deg)).T
        np.savetxt('double_pendulum_data.csv', data, delimiter=',', header='Time,Theta1,Omega1,Theta2,Omega2', comments='')

if __name__ == "__main__":
    main()
