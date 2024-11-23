# gui.py

import sys
import numpy as np
import matplotlib
matplotlib.use('Qt5Agg')
from PyQt5 import QtWidgets, QtCore, QtGui
from pendulum_model import (
    NonlinearPendulum, LinearizedPendulum,
    NonlinearDoublePendulum, LinearizedDoublePendulum
)
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from matplotlib.animation import FuncAnimation

class TitleBar(QtWidgets.QWidget):
    def __init__(self, parent):
        super().__init__(parent)
        self.init_ui()
        self.parent = parent

    def init_ui(self):
        self.setAutoFillBackground(True)
        self.setBackgroundRole(QtGui.QPalette.Highlight)

        self.layout = QtWidgets.QHBoxLayout(self)
        self.layout.setContentsMargins(0, 0, 0, 0)  # Removed extra spacing
        self.layout.setSpacing(0)

        # Title label
        self.title = QtWidgets.QLabel("Pendulum Simulator")
        self.title.setStyleSheet("color: white; font-size: 14px; font-weight: bold; font-family: 'Segoe UI';")
        self.title.setAlignment(QtCore.Qt.AlignCenter)

        # Spacer
        self.spacer = QtWidgets.QWidget()
        self.spacer.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Preferred)

        # Minimize, Maximize, Close buttons using Unicode characters
        self.min_btn = QtWidgets.QPushButton('\u2212')  # Unicode minus sign
        self.max_btn = QtWidgets.QPushButton('\u25A1')  # Unicode square
        self.close_btn = QtWidgets.QPushButton('\u2715')  # Unicode cross

        # Style the buttons
        button_style = """
            QPushButton {
                background-color: transparent;
                border: none;
                color: white;
                padding: 5px;
                border-radius: 0px;
                font-size: 12px;
                font-family: 'Segoe UI';
            }
            QPushButton:hover {
                background-color: #505357;
            }
        """
        self.min_btn.setStyleSheet(button_style)
        self.max_btn.setStyleSheet(button_style)
        self.close_btn.setStyleSheet(button_style.replace("#505357", "#D32F2F"))

        # Set button sizes
        self.min_btn.setFixedSize(30, 30)
        self.max_btn.setFixedSize(30, 30)
        self.close_btn.setFixedSize(30, 30)

        # Connect buttons
        self.min_btn.clicked.connect(self.minimize)
        self.max_btn.clicked.connect(self.maximize)
        self.close_btn.clicked.connect(self.close)

        # Add widgets to layout
        self.layout.addWidget(self.title)
        self.layout.addWidget(self.spacer)
        self.layout.addWidget(self.min_btn)
        self.layout.addWidget(self.max_btn)
        self.layout.addWidget(self.close_btn)

        # Enable window dragging
        self.is_moving = False

    def minimize(self):
        self.parent.showMinimized()

    def maximize(self):
        if self.parent.isMaximized():
            self.parent.showNormal()
            self.max_btn.setText('\u25A1')  # Square symbol
        else:
            self.parent.showMaximized()
            self.max_btn.setText('\u25A0')  # Filled square symbol

    def close(self):
        self.parent.close()

    def mousePressEvent(self, event):
        if event.button() == QtCore.Qt.LeftButton:
            self.is_moving = True
            self.offset = event.pos()

    def mouseMoveEvent(self, event):
        if self.is_moving:
            self.parent.move(event.globalPos() - self.offset)

    def mouseReleaseEvent(self, event):
        self.is_moving = False

class PendulumSimulator(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowFlags(QtCore.Qt.FramelessWindowHint)
        self.init_ui()
        self.animation = None

    def init_ui(self):
        # Main layout
        self.main_layout = QtWidgets.QVBoxLayout(self)
        self.main_layout.setContentsMargins(0, 0, 0, 0)
        self.main_layout.setSpacing(0)

        # Title bar
        self.title_bar = TitleBar(self)
        self.main_layout.addWidget(self.title_bar)

        # Content area
        content_widget = QtWidgets.QWidget()
        self.content_layout = QtWidgets.QHBoxLayout(content_widget)
        self.content_layout.setContentsMargins(0, 0, 0, 0)
        self.content_layout.setSpacing(0)

        # Left side: Animation and plots
        self.figure = Figure(figsize=(8, 6), facecolor='#2b2b2b')
        self.canvas = FigureCanvas(self.figure)

        # Right side: Controls
        controls_layout = QtWidgets.QVBoxLayout()
        controls_layout.setAlignment(QtCore.Qt.AlignTop)
        controls_layout.setContentsMargins(10, 10, 10, 10)

        # Pendulum Type
        self.pendulum_label = QtWidgets.QLabel('Select Pendulum Type:')
        self.pendulum_combo = QtWidgets.QComboBox()
        self.pendulum_combo.addItems(['Single', 'Double'])
        self.pendulum_combo.currentIndexChanged.connect(self.update_pendulum_type)

        # Model Type
        self.model_label = QtWidgets.QLabel('Select Model:')
        self.model_combo = QtWidgets.QComboBox()
        self.model_combo.addItems(['Nonlinear', 'Linearized'])

        # Damping Coefficient
        self.damping_label = QtWidgets.QLabel('Damping Coefficient:')
        self.damping_input = QtWidgets.QLineEdit('0.0')

        # Initial Conditions
        self.angle_label = QtWidgets.QLabel('Initial Angle (degrees):')
        self.angle_input = QtWidgets.QLineEdit('10')

        self.velocity_label = QtWidgets.QLabel('Initial Angular Velocity (deg/s):')
        self.velocity_input = QtWidgets.QLineEdit('0')

        # For double pendulum
        self.angle2_label = QtWidgets.QLabel('Initial Angle 2 (degrees):')
        self.angle2_input = QtWidgets.QLineEdit('10')
        self.velocity2_label = QtWidgets.QLabel('Initial Angular Velocity 2 (deg/s):')
        self.velocity2_input = QtWidgets.QLineEdit('0')

        # Simulation Mode
        self.time_mode_label = QtWidgets.QLabel('Simulation Mode:')
        self.time_mode_combo = QtWidgets.QComboBox()
        self.time_mode_combo.addItems(['Fixed Time Interval', 'Endless Simulation'])
        self.time_mode_combo.currentIndexChanged.connect(self.update_time_mode)

        self.simulation_time_label = QtWidgets.QLabel('Simulation Time (s):')
        self.simulation_time_input = QtWidgets.QLineEdit('10')

        # Simulation Options
        self.animation_checkbox = QtWidgets.QCheckBox('Show Animation')
        self.animation_checkbox.setChecked(True)
        self.plots_checkbox = QtWidgets.QCheckBox('Show Plots')
        self.plots_checkbox.setChecked(True)

        # Run Button
        self.run_button = QtWidgets.QPushButton('Run Simulation')
        self.run_button.clicked.connect(self.run_simulation)

        # Stop Button
        self.stop_button = QtWidgets.QPushButton('Stop Simulation')
        self.stop_button.clicked.connect(self.stop_simulation)
        self.stop_button.setEnabled(False)

        # Organize controls
        controls_layout.addWidget(self.pendulum_label)
        controls_layout.addWidget(self.pendulum_combo)
        controls_layout.addWidget(self.model_label)
        controls_layout.addWidget(self.model_combo)
        controls_layout.addWidget(self.damping_label)
        controls_layout.addWidget(self.damping_input)
        controls_layout.addWidget(self.angle_label)
        controls_layout.addWidget(self.angle_input)
        controls_layout.addWidget(self.velocity_label)
        controls_layout.addWidget(self.velocity_input)
        controls_layout.addWidget(self.angle2_label)
        controls_layout.addWidget(self.angle2_input)
        controls_layout.addWidget(self.velocity2_label)
        controls_layout.addWidget(self.velocity2_input)
        controls_layout.addWidget(self.time_mode_label)
        controls_layout.addWidget(self.time_mode_combo)
        controls_layout.addWidget(self.simulation_time_label)
        controls_layout.addWidget(self.simulation_time_input)
        controls_layout.addWidget(self.animation_checkbox)
        controls_layout.addWidget(self.plots_checkbox)
        controls_layout.addWidget(self.run_button)
        controls_layout.addWidget(self.stop_button)

        # Add widgets to content layout
        self.content_layout.addWidget(self.canvas)
        self.content_layout.addLayout(controls_layout)

        self.main_layout.addWidget(content_widget)

        self.setGeometry(100, 100, 1200, 700)

        self.update_pendulum_type()  # Initialize visibility
        self.update_time_mode()

        self.apply_stylesheet()  # Apply custom styles

    def apply_stylesheet(self):
        # Set dark theme with professional font
        self.setStyleSheet("""
            QWidget {
                background-color: #2b2b2b;
                color: #ffffff;
                font-family: 'Segoe UI';
                font-size: 11px;
            }
            QLineEdit, QComboBox {
                background-color: #3c3f41;
                border: 1px solid #5c5c5c;
                padding: 5px;
                border-radius: 5px;
                color: #ffffff;
                font-family: 'Segoe UI';
            }
            QPushButton {
                background-color: #3c3f41;
                border: 1px solid #5c5c5c;
                padding: 5px;
                border-radius: 5px;
                color: #ffffff;
                font-family: 'Segoe UI';
            }
            QPushButton:hover {
                background-color: #505357;
            }
            QCheckBox {
                padding: 5px;
                font-family: 'Segoe UI';
            }
            QLabel {
                padding: 2px;
                font-family: 'Segoe UI';
            }
        """)

    def update_pendulum_type(self):
        pendulum_type = self.pendulum_combo.currentText().lower()
        if pendulum_type == 'single':
            self.angle2_label.hide()
            self.angle2_input.hide()
            self.velocity2_label.hide()
            self.velocity2_input.hide()
        else:
            self.angle2_label.show()
            self.angle2_input.show()
            self.velocity2_label.show()
            self.velocity2_input.show()

    def update_time_mode(self):
        time_mode = self.time_mode_combo.currentText()
        if time_mode == 'Fixed Time Interval':
            self.simulation_time_label.show()
            self.simulation_time_input.show()
        else:
            self.simulation_time_label.hide()
            self.simulation_time_input.hide()

    def run_simulation(self):
        # Disable run button and enable stop button
        self.run_button.setEnabled(False)
        self.stop_button.setEnabled(True)

        pendulum_type = self.pendulum_combo.currentText().lower()
        model_type = self.model_combo.currentText().lower()
        show_animation = self.animation_checkbox.isChecked()
        show_plots = self.plots_checkbox.isChecked()
        time_mode = self.time_mode_combo.currentText()

        try:
            damping = float(self.damping_input.text())
            initial_angle = float(self.angle_input.text())
            initial_velocity = float(self.velocity_input.text())
            if time_mode == 'Fixed Time Interval':
                simulation_time = float(self.simulation_time_input.text())
            else:
                simulation_time = None  # Endless simulation
        except ValueError:
            QtWidgets.QMessageBox.warning(self, 'Input Error', 'Please enter valid numerical values.')
            self.run_button.setEnabled(True)
            self.stop_button.setEnabled(False)
            return

        # Convert degrees to radians
        initial_angle_rad = np.radians(initial_angle)
        initial_velocity_rad = np.radians(initial_velocity)

        gravity = 9.81

        if pendulum_type == 'single':
            length = 1.0  # Length in meters
            mass = 1.0    # Mass in kg
            y0 = [initial_angle_rad, initial_velocity_rad]
            self.y0 = y0  # Store initial conditions

            # Instantiate model
            if model_type == 'nonlinear':
                self.pendulum = NonlinearPendulum(length, mass, gravity, damping)
            elif model_type == 'linearized':
                self.pendulum = LinearizedPendulum(length, mass, gravity, damping)
            else:
                QtWidgets.QMessageBox.warning(self, 'Model Error', 'Invalid model selected.')
                self.run_button.setEnabled(True)
                self.stop_button.setEnabled(False)
                return

            # Prepare for animation or plots
            if show_animation or show_plots:
                self.prepare_single_pendulum_animation(show_animation, show_plots, simulation_time)
            else:
                QtWidgets.QMessageBox.warning(self, 'Selection Error', 'Please select at least one of "Show Animation" or "Show Plots".')
                self.run_button.setEnabled(True)
                self.stop_button.setEnabled(False)

        elif pendulum_type == 'double':
            try:
                initial_angle2 = float(self.angle2_input.text())
                initial_velocity2 = float(self.velocity2_input.text())
            except ValueError:
                QtWidgets.QMessageBox.warning(self, 'Input Error', 'Please enter valid numerical values for the second pendulum.')
                self.run_button.setEnabled(True)
                self.stop_button.setEnabled(False)
                return

            initial_angle2_rad = np.radians(initial_angle2)
            initial_velocity2_rad = np.radians(initial_velocity2)

            lengths = (1.0, 1.0)
            masses = (1.0, 1.0)
            damping1 = damping
            damping2 = damping  # For simplicity, use the same damping for both pendulums

            y0 = [initial_angle_rad, initial_velocity_rad, initial_angle2_rad, initial_velocity2_rad]
            self.y0 = y0  # Store initial conditions

            # Instantiate model
            if model_type == 'nonlinear':
                self.pendulum = NonlinearDoublePendulum(lengths, masses, gravity, (damping1, damping2))
            elif model_type == 'linearized':
                self.pendulum = LinearizedDoublePendulum(lengths, masses, gravity, (damping1, damping2))
            else:
                QtWidgets.QMessageBox.warning(self, 'Model Error', 'Invalid model selected.')
                self.run_button.setEnabled(True)
                self.stop_button.setEnabled(False)
                return

            # Prepare for animation or plots
            if show_animation or show_plots:
                self.prepare_double_pendulum_animation(show_animation, show_plots, simulation_time)
            else:
                QtWidgets.QMessageBox.warning(self, 'Selection Error', 'Please select at least one of "Show Animation" or "Show Plots".')
                self.run_button.setEnabled(True)
                self.stop_button.setEnabled(False)
        else:
            QtWidgets.QMessageBox.warning(self, 'Pendulum Error', 'Invalid pendulum type selected.')
            self.run_button.setEnabled(True)
            self.stop_button.setEnabled(False)
            return

    def stop_simulation(self):
        if self.animation:
            self.animation.event_source.stop()
            self.animation = None
        self.run_button.setEnabled(True)
        self.stop_button.setEnabled(False)

    def prepare_single_pendulum_animation(self, show_animation, show_plots, simulation_time):
        self.figure.clear()

        # Data buffers for endless simulation
        self.t_data = []
        self.theta_data = []
        self.omega_data = []
        self.alpha_data = []

        if show_animation and show_plots:
            grid_spec = self.figure.add_gridspec(2, 1)
            ax_anim = self.figure.add_subplot(grid_spec[0, 0])
            ax_plot = self.figure.add_subplot(grid_spec[1, 0])
        elif show_animation:
            ax_anim = self.figure.add_subplot(1, 1, 1)
            ax_plot = None
        elif show_plots:
            ax_anim = None
            ax_plot = self.figure.add_subplot(1, 1, 1)
        else:
            return

        # Animation axis
        if ax_anim:
            ax_anim.set_aspect('equal')
            ax_anim.grid(True)
            ax_anim.set_xlim(-1.2, 1.2)
            ax_anim.set_ylim(-1.2, 1.2)
            ax_anim.set_title('Single Pendulum Animation', color='white')
            ax_anim.set_facecolor('#2b2b2b')
            ax_anim.tick_params(axis='x', colors='white')
            ax_anim.tick_params(axis='y', colors='white')
            ax_anim.title.set_color('white')

            line_anim, = ax_anim.plot([], [], 'o-', lw=2, color='cyan')
            time_text = ax_anim.text(0.05, 0.9, '', transform=ax_anim.transAxes, color='white')
        else:
            line_anim = None
            time_text = None

        # Plot axis
        if ax_plot:
            ax_plot.set_title('Angle, Angular Velocity, and Acceleration', color='white')
            ax_plot.set_xlabel('Time (s)', color='white')
            ax_plot.grid(True)
            ax_plot.set_facecolor('#2b2b2b')
            ax_plot.tick_params(axis='x', colors='white')
            ax_plot.tick_params(axis='y', colors='white')

            angle_line, = ax_plot.plot([], [], label='Angle (deg)', color='yellow')
            omega_line, = ax_plot.plot([], [], label='Angular Velocity (deg/s)', color='magenta')
            alpha_line, = ax_plot.plot([], [], label='Angular Acceleration (deg/s²)', color='cyan')
            ax_plot.legend(facecolor='#2b2b2b', edgecolor='#2b2b2b', labelcolor='white')
        else:
            angle_line = omega_line = alpha_line = None

        def init():
            if line_anim:
                line_anim.set_data([], [])
                time_text.set_text('')
            if angle_line:
                angle_line.set_data([], [])
                omega_line.set_data([], [])
                alpha_line.set_data([], [])
            return []

        def update(frame):
            if self.animation is None:
                return []

            dt = 0.02  # Time step for simulation

            if simulation_time:
                if len(self.t_data) * dt >= simulation_time:
                    self.stop_simulation()
                    return []
            else:
                pass  # Endless simulation

            # Simulate one time step
            if len(self.t_data) == 0:
                t_prev = 0
                y_prev = self.y0  # Use stored initial conditions
            else:
                t_prev = self.t_data[-1]
                y_prev = [self.theta_data[-1], self.omega_data[-1]]

            t_span = [t_prev, t_prev + dt]
            sol = self.pendulum.integrate_step(y_prev, t_span)

            theta = sol.y[0][-1]
            omega = sol.y[1][-1]
            alpha = self.pendulum.equations_of_motion(0, [theta, omega])[1]

            self.t_data.append(sol.t[-1])
            self.theta_data.append(theta)
            self.omega_data.append(omega)
            self.alpha_data.append(alpha)

            artists = []
            if line_anim:
                x = self.pendulum.length * np.sin(theta)
                y = -self.pendulum.length * np.cos(theta)
                x1 = [0, x]
                y1 = [0, y]
                line_anim.set_data(x1, y1)
                time_text.set_text(f'Time = {sol.t[-1]:.2f}s')
                artists.extend([line_anim, time_text])

            if angle_line:
                t = np.array(self.t_data)
                theta_deg = np.degrees(self.theta_data)
                omega_deg = np.degrees(self.omega_data)
                alpha_deg = np.degrees(self.alpha_data)
                angle_line.set_data(t, theta_deg)
                omega_line.set_data(t, omega_deg)
                alpha_line.set_data(t, alpha_deg)
                ax_plot.relim()
                ax_plot.autoscale_view()
                artists.extend([angle_line, omega_line, alpha_line])

            return artists

        self.animation = FuncAnimation(self.figure, update, init_func=init,
                                       blit=False, interval=20, cache_frame_data=False)
        self.canvas.draw()

    def prepare_double_pendulum_animation(self, show_animation, show_plots, simulation_time):
        self.figure.clear()

        # Data buffers for endless simulation
        self.t_data = []
        self.theta1_data = []
        self.theta2_data = []
        self.omega1_data = []
        self.omega2_data = []

        if show_animation and show_plots:
            grid_spec = self.figure.add_gridspec(2, 1)
            ax_anim = self.figure.add_subplot(grid_spec[0, 0])
            ax_plot = self.figure.add_subplot(grid_spec[1, 0])
        elif show_animation:
            ax_anim = self.figure.add_subplot(1, 1, 1)
            ax_plot = None
        elif show_plots:
            ax_anim = None
            ax_plot = self.figure.add_subplot(1, 1, 1)
        else:
            return

        # Animation axis
        if ax_anim:
            ax_anim.set_aspect('equal')
            ax_anim.grid(True)
            ax_anim.set_xlim(-2.2, 2.2)
            ax_anim.set_ylim(-2.2, 2.2)
            ax_anim.set_title('Double Pendulum Animation', color='white')
            ax_anim.set_facecolor('#2b2b2b')
            ax_anim.tick_params(axis='x', colors='white')
            ax_anim.tick_params(axis='y', colors='white')
            ax_anim.title.set_color('white')

            line_anim, = ax_anim.plot([], [], 'o-', lw=2, color='cyan')
            time_text = ax_anim.text(0.05, 0.9, '', transform=ax_anim.transAxes, color='white')
        else:
            line_anim = None
            time_text = None

        # Plot axis
        if ax_plot:
            ax_plot.set_title('Angles and Angular Velocities', color='white')
            ax_plot.set_xlabel('Time (s)', color='white')
            ax_plot.grid(True)
            ax_plot.set_facecolor('#2b2b2b')
            ax_plot.tick_params(axis='x', colors='white')
            ax_plot.tick_params(axis='y', colors='white')

            angle1_line, = ax_plot.plot([], [], label='Theta1 (deg)', color='yellow')
            angle2_line, = ax_plot.plot([], [], label='Theta2 (deg)', color='magenta')
            omega1_line, = ax_plot.plot([], [], label='Omega1 (deg/s)', color='cyan')
            omega2_line, = ax_plot.plot([], [], label='Omega2 (deg/s)', color='green')
            ax_plot.legend(facecolor='#2b2b2b', edgecolor='#2b2b2b', labelcolor='white')
        else:
            angle1_line = angle2_line = omega1_line = omega2_line = None

        def init():
            if line_anim:
                line_anim.set_data([], [])
                time_text.set_text('')
            if angle1_line:
                angle1_line.set_data([], [])
                angle2_line.set_data([], [])
                omega1_line.set_data([], [])
                omega2_line.set_data([], [])
            return []

        def update(frame):
            if self.animation is None:
                return []

            dt = 0.02  # Time step for simulation

            if simulation_time:
                if len(self.t_data) * dt >= simulation_time:
                    self.stop_simulation()
                    return []
            else:
                pass  # Endless simulation

            # Simulate one time step
            if len(self.t_data) == 0:
                t_prev = 0
                y_prev = self.y0  # Use stored initial conditions
            else:
                t_prev = self.t_data[-1]
                y_prev = [self.theta1_data[-1], self.omega1_data[-1],
                          self.theta2_data[-1], self.omega2_data[-1]]

            t_span = [t_prev, t_prev + dt]
            sol = self.pendulum.integrate_step(y_prev, t_span)

            theta1 = sol.y[0][-1]
            omega1 = sol.y[1][-1]
            theta2 = sol.y[2][-1]
            omega2 = sol.y[3][-1]

            self.t_data.append(sol.t[-1])
            self.theta1_data.append(theta1)
            self.omega1_data.append(omega1)
            self.theta2_data.append(theta2)
            self.omega2_data.append(omega2)

            artists = []
            if line_anim:
                x1 = self.pendulum.L1 * np.sin(theta1)
                y1 = -self.pendulum.L1 * np.cos(theta1)
                x2 = x1 + self.pendulum.L2 * np.sin(theta2)
                y2 = y1 - self.pendulum.L2 * np.cos(theta2)

                x_vals = [0, x1, x2]
                y_vals = [0, y1, y2]
                line_anim.set_data(x_vals, y_vals)
                time_text.set_text(f'Time = {sol.t[-1]:.2f}s')
                artists.extend([line_anim, time_text])

            if angle1_line:
                t = np.array(self.t_data)
                theta1_deg = np.degrees(self.theta1_data)
                theta2_deg = np.degrees(self.theta2_data)
                omega1_deg = np.degrees(self.omega1_data)
                omega2_deg = np.degrees(self.omega2_data)

                angle1_line.set_data(t, theta1_deg)
                angle2_line.set_data(t, theta2_deg)
                omega1_line.set_data(t, omega1_deg)
                omega2_line.set_data(t, omega2_deg)
                ax_plot.relim()
                ax_plot.autoscale_view()
                artists.extend([angle1_line, angle2_line, omega1_line, omega2_line])

            return artists

        self.animation = FuncAnimation(self.figure, update, init_func=init,
                                       blit=False, interval=20, cache_frame_data=False)
        self.canvas.draw()

    @staticmethod
    def main():
        app = QtWidgets.QApplication(sys.argv)
        simulator = PendulumSimulator()
        simulator.show()
        sys.exit(app.exec_())

if __name__ == '__main__':
    PendulumSimulator.main()
