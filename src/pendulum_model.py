# pendulum_model.py

import numpy as np
from scipy.integrate import solve_ivp

class NonlinearPendulum:
    def __init__(self, length, mass, gravity=9.81, damping=0.0):
        self.length = length
        self.mass = mass
        self.gravity = gravity
        self.damping = damping

    def equations_of_motion(self, t, y):
        theta, omega = y
        theta_dot = omega
        omega_dot = - (self.gravity / self.length) * np.sin(theta) - \
                    (self.damping / (self.mass * self.length**2)) * omega
        return [theta_dot, omega_dot]

    def integrate_step(self, y_prev, t_span):
        sol = solve_ivp(self.equations_of_motion, t_span, y_prev, t_eval=[t_span[1]])
        return sol

    def simulate(self, y0, t_span, t_eval):
        sol = solve_ivp(self.equations_of_motion, t_span, y0, t_eval=t_eval)
        return sol

class LinearizedPendulum:
    def __init__(self, length, mass, gravity=9.81, damping=0.0):
        self.length = length
        self.mass = mass
        self.gravity = gravity
        self.damping = damping

    def equations_of_motion(self, t, y):
        theta, omega = y
        theta_dot = omega
        omega_dot = - (self.gravity / self.length) * theta - \
                    (self.damping / (self.mass * self.length**2)) * omega
        return [theta_dot, omega_dot]

    def integrate_step(self, y_prev, t_span):
        sol = solve_ivp(self.equations_of_motion, t_span, y_prev, t_eval=[t_span[1]])
        return sol

    def simulate(self, y0, t_span, t_eval):
        sol = solve_ivp(self.equations_of_motion, t_span, y0, t_eval=t_eval)
        return sol

class NonlinearDoublePendulum:
    def __init__(self, lengths, masses, gravity=9.81, damping=(0.0, 0.0)):
        self.L1, self.L2 = lengths
        self.m1, self.m2 = masses
        self.gravity = gravity
        self.damping1, self.damping2 = damping

    def equations_of_motion(self, t, y):
        theta1, omega1, theta2, omega2 = y

        m1 = self.m1
        m2 = self.m2
        L1 = self.L1
        L2 = self.L2
        g = self.gravity
        b1 = self.damping1
        b2 = self.damping2

        delta = theta2 - theta1

        denom1 = (m1 + m2) * L1 - m2 * L1 * np.cos(delta) ** 2
        denom2 = (L2 / L1) * denom1

        theta1_dot = omega1
        theta2_dot = omega2

        omega1_num = m2 * L1 * omega1 ** 2 * np.sin(delta) * np.cos(delta) + \
                     m2 * g * np.sin(theta2) * np.cos(delta) + \
                     m2 * L2 * omega2 ** 2 * np.sin(delta) - \
                     (m1 + m2) * g * np.sin(theta1) - b1 * omega1

        omega1_dot = omega1_num / denom1

        omega2_num = -m2 * L2 * omega2 ** 2 * np.sin(delta) * np.cos(delta) + \
                     (m1 + m2) * (g * np.sin(theta1) * np.cos(delta) - \
                     L1 * omega1 ** 2 * np.sin(delta) - g * np.sin(theta2)) - b2 * omega2

        omega2_dot = omega2_num / denom2

        return [theta1_dot, omega1_dot, theta2_dot, omega2_dot]

    def integrate_step(self, y_prev, t_span):
        sol = solve_ivp(self.equations_of_motion, t_span, y_prev, t_eval=[t_span[1]], max_step=0.01)
        return sol

    def simulate(self, y0, t_span, t_eval):
        sol = solve_ivp(self.equations_of_motion, t_span, y0, t_eval=t_eval, max_step=0.01)
        return sol

class LinearizedDoublePendulum:
    def __init__(self, lengths, masses, gravity=9.81, damping=(0.0, 0.0)):
        self.L1, self.L2 = lengths
        self.m1, self.m2 = masses
        self.gravity = gravity
        self.damping1, self.damping2 = damping

    def equations_of_motion(self, t, y):
        theta1, omega1, theta2, omega2 = y

        m1 = self.m1
        m2 = self.m2
        L1 = self.L1
        L2 = self.L2
        g = self.gravity
        b1 = self.damping1
        b2 = self.damping2

        theta1_dot = omega1
        theta2_dot = omega2

        delta = theta2 - theta1

        omega1_dot = (- (m1 + m2) * g * theta1 + m2 * g * theta2 - b1 * omega1) / ((m1 + m2) * L1)
        omega2_dot = (- m2 * g * theta2 + m2 * g * theta1 - b2 * omega2) / (m2 * L2)

        return [theta1_dot, omega1_dot, theta2_dot, omega2_dot]

    def integrate_step(self, y_prev, t_span):
        sol = solve_ivp(self.equations_of_motion, t_span, y_prev, t_eval=[t_span[1]])
        return sol

    def simulate(self, y0, t_span, t_eval):
        sol = solve_ivp(self.equations_of_motion, t_span, y0, t_eval=t_eval)
        return sol
