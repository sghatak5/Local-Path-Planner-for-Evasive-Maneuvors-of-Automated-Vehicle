import numpy as np
import matplotlib.pyplot as plt
import time as tm
import math
import pidcontroller as pid

class Vehicle:
    def __init__(self, x, y, delta, vel, dt):
        self.vel = vel  # Vehicle velocity
        self.x = x  # X position
        self.y = y  # Y position
        self.delta = delta  # Steering angle
        self.dt = dt  # Time step
        self.beta = 0  # Slip angle
        self.r = 0  # Yaw rate
        self.yaw = 0  # Yaw angle

        # Vehicle parameters
        self.length = 2.338
        self.width = 1.381
        self.rear_to_wheel = 0.339
        self.wheel_length = 0.531
        self.wheel_width = 0.125
        self.track = 1.094
        self.wheel_base = 1.686
        self.Caf = 2 * 32857.5  # Cornering stiffness front
        self.Car = 2 * 32857.5  # Cornering stiffness rear
        self.mass = 633
        self.lf = 0.9442  # Distance from CG to front axle
        self.lr = 0.7417  # Distance from CG to rear axle
        self.Iz = 430.166  # Yaw moment of inertia

    def deg_to_rad(self, deg):
        """Convert degrees to radians."""
        return deg * math.pi / 180
    
    def Motion_model(self):
        """Update the vehicle state based on the motion model."""
        # System dynamics matrices
        coefficient_A = np.array([
            [-1 * (self.Caf + self.Car) / (self.mass * self.vel), ((self.Car * self.lr - self.Caf * self.lf) / (self.mass * self.vel**2)) - 1],
            [(self.Car * self.lr - self.Caf * self.lf) / self.Iz, -(self.Caf * self.lf**2 + self.Car * self.lr**2) / (self.Iz * self.vel)]
        ])
        
        state_vector = np.array([[self.beta], [self.r]])

        coefficient_B = np.array([
            [self.Caf / (self.mass * self.vel)],
            [self.Caf * self.lf / self.Iz]
        ])

        control_matrix = np.array([[self.delta]])
        
        output_matrix = np.dot(coefficient_A, state_vector) + np.dot(coefficient_B, control_matrix)

        beta_dot = output_matrix[0][0]
        r_dot = output_matrix[1][0]

        self.beta += beta_dot * self.dt
        self.r += r_dot * self.dt
        self.yaw += self.r * self.dt

        # Ensure yaw is within -180 to 180 degrees
        if self.yaw > 180:
            self.yaw -= 360
        elif self.yaw < -180: 
            self.yaw += 360

        # Update position
        x_dot = self.vel * math.cos(self.deg_to_rad(self.yaw + self.beta))
        y_dot = self.vel * math.sin(self.deg_to_rad(self.yaw + self.beta))
        self.x += x_dot * self.dt
        self.y += y_dot * self.dt

    
def simulate_vehicle_with_pid(pid, trajectories, vehicle, dt, sim_time):
    # Combine all trajectory points
    trajectory_x = []
    trajectory_y = []
    for traj in trajectories:
        trajectory_x.extend(traj[0])
        trajectory_y.extend(traj[1])

    x_positions = []
    y_positions = []

    for t in np.arange(0, sim_time, dt):
        if len(trajectory_x) == 0:
            break

        closest_index = np.argmin(np.hypot(np.array(trajectory_x) - vehicle.x, np.array(trajectory_y) - vehicle.y))
        #print(closest_index)
        #target_x = trajectory_x[closest_index]
        target_y = trajectory_y[closest_index]

        # Compute the steering angle using PID controller
        #print(target_y, vehicle.y)
        steering_angle = pid.compute(setpoint=target_y, current_value=vehicle.y)
        vehicle.delta = steering_angle

        # Update the vehicle state
        vehicle.Motion_model()
        x_positions.append(vehicle.x)
        y_positions.append(vehicle.y)

    return x_positions, y_positions, trajectory_x, trajectory_y