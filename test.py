import numpy as np
import matplotlib.pyplot as plt
import time as tm
import math

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

class PIDController:
    def __init__(self, Kp, Ki, Kd, dt, windup_guard=30):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.dt = dt
        self.integral = 0
        self.previous_error = 0
        self.windup_guard = windup_guard

    def compute(self, setpoint, current_value):
        error = setpoint - current_value
        
        # Update integral term with anti-windup mechanism
        self.integral += error * self.dt
        if self.integral > self.windup_guard:
            self.integral = self.windup_guard
        elif self.integral < -self.windup_guard:
            self.integral = -self.windup_guard
        
        # Calculate derivative term
        derivative = (error - self.previous_error) / self.dt
        
        # Compute PID output
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        
        # Update previous error for next iteration
        self.previous_error = error
        
        return output

class TrajectoryPlanner:
    def __init__(self, s0, v0, a0, sf, vf, af, ttc, ego_v):
        self.s0 = s0
        self.v0 = v0
        self.a0 = a0 / 2
        self.t0 = 0
        self.sf = sf
        self.vf = vf
        self.af = af
        self.tf = ttc
        self.ego_v = ego_v

        T = np.array([
            [1, self.t0, self.t0**2, self.t0**3, self.t0**4, self.t0**5],
            [0, 1, 2*self.t0, 3*self.t0**2, 4*self.t0**3, 5*self.t0**4],
            [0, 0, 2, 6*self.t0, 12*self.t0**2, 20*self.t0**3],
            [1, self.tf, self.tf**2, self.tf**3, self.tf**4, self.tf**5],
            [0, 1, 2*self.tf, 3*self.tf**2, 4*self.tf**3, 5*self.tf**4],
            [0, 0, 2, 6*self.tf, 12*self.tf**2, 20*self.tf**3]
        ])

        Q = [self.s0, self.v0, self.a0, self.sf, self.vf, self.af]
        self.c0, self.c1, self.c2, self.c3, self.c4, self.c5 = np.linalg.solve(T, Q)

    def calc_x_coord(self, t):
        return self.ego_v * t

    def calc_y_coord(self, t):
        return self.c0 + self.c1 * t + self.c2 * t**2 + self.c3 * t**3 + self.c4 * t**4 + self.c5 * t**5

def calculate_trajectory(trajectory_type, s0, v0, a0, sf, vf, af, ego_v, ped_v, pedestrian_x_start, pedestrian_y_start, time):
    """
    Calculate the trajectory of the vehicle.

    Parameters:
    trajectory_type (int): Type of the trajectory.
    s0 (float): Initial position.
    v0 (float): Initial velocity.
    a0 (float): Initial acceleration.
    sf (float): Desired final position.
    vf (float): Desired final velocity.
    af (float): Desired final acceleration.
    ego_v (float): Ego vehicle velocity.
    ped_v (float): Pedestrian velocity.
    pedestrian_x_start (float): Pedestrian starting x-coordinate.
    pedestrian_y_start (float): Pedestrian starting y-coordinate.
    time (float): Current time.

    Returns:
    tuple: Depending on the trajectory type, returns a tuple of vehicle and possibly pedestrian trajectory coordinates and labels.
    """
    if trajectory_type == 1:
        pedestrian_x_start = pedestrian_x_start
        pedestrian_y_start = pedestrian_y_start
        ttc = (pedestrian_x_start - (ego_v * time) )/ ego_v
        print(f"Time to collision: {ttc}")
        label = "Trajectory 1"
    elif trajectory_type == 2:
        ttc = 0.5
        label = "Trajectory 2"
    elif trajectory_type == 3:
        ttc = 3
        label = "Trajectory 3"
    else:
        raise ValueError("Invalid trajectory type")

    trajectory = TrajectoryPlanner(s0, v0, a0, sf, vf, af, ttc, ego_v)
    x_t_list = np.arange(time, time + ttc, 0.02)
    y_t_list = np.arange(0, ttc, 0.02)
    vehicle_x_list = [trajectory.calc_x_coord(t) for t in x_t_list]
    vehicle_y_list = [trajectory.calc_y_coord(t) for t in y_t_list]

    if trajectory_type == 1:
        ped_t = time + ttc
        t_list_ped = np.arange(0, ped_t, 0.2)
        pedestrian_x_list = [pedestrian_x_start] * len(t_list_ped)
        pedestrian_y_list = [pedestrian_y_start + ped_v * t for t in t_list_ped]
        return vehicle_x_list, vehicle_y_list, label, x_t_list, pedestrian_x_list, pedestrian_y_list
    else:
        return vehicle_x_list, vehicle_y_list, label, x_t_list

def plot_trajectories(trajectories, pedestrian_trajectory=None, ax=None):
    """
    Plot the trajectories of the vehicle and optionally the pedestrian trajectory.

    Parameters:
    trajectories (list): List of trajectories to plot.
    pedestrian_trajectory (tuple, optional): Tuple of x and y coordinates for the pedestrian trajectory. Defaults to None.
    """
    if ax is None:
        ax = plt.gca()
    ax.set_title("Trajectory Planner")
    
    for traj in trajectories:
        ax.plot(traj[0], traj[1], label=traj[2])
    
    if pedestrian_trajectory:
        ax.scatter(pedestrian_trajectory[0], pedestrian_trajectory[1], label="Pedestrian Trajectory", color='red', s=10)
    
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_ylim(-10, 10)
    ax.set_xlim(left=0)
    ax.grid()
    ax.legend()

def simulate_vehicle_with_pid(trajectories, pedestrian_trajectory=None, ax=None):
    dt = 0.03
    total_time = 23.49  # Simulate for 30 seconds
    vehicle = Vehicle(92.5, 0, 0, 5, dt)
    #pid = PIDController(Kp=2.0, Ki=0.3, Kd=0.2, dt=dt)
    pid = PIDController(Kp=1.3, Ki=0.003, Kd=3.8, dt=dt)

    # Combine all trajectory points
    trajectory_x = []
    trajectory_y = []
    for traj in trajectories:
        trajectory_x.extend(traj[0])
        trajectory_y.extend(traj[1])

    x_positions = []
    y_positions = []

    for t in np.arange(0, total_time, dt):
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

    #print(len(trajectory_x), len(trajectory_y))
    #print(len(x_positions), len(x_positions))

    if ax is None:
        ax = plt.gca()
    ax.set_title("Vehicle Trajectory with PID Control")
    ax.plot(x_positions, y_positions, label="Vehicle Trajectory")
    ax.plot(trajectory_x, trajectory_y, label="Desired Trajectory", linestyle='dashed')
    if pedestrian_trajectory:
        ax.scatter(pedestrian_trajectory[0], pedestrian_trajectory[1], label="Pedestrian Trajectory", color='green', s=10)
    ax.scatter(x_positions[0], y_positions[0], color='red', label="Start Position")
    ax.set_xlabel("X Position (m)")
    ax.set_ylabel("Y Position (m)")
    ax.legend()
    ax.grid(True)
    #ax.axis('equal')
    ax.set_xlim(left=0)
    ax.set_ylim(-10, 10)

def main():
    #Initial Conditions
    s0 = 0 #Initial postion at t = 0
    v0 = 0 #Initial velocity at t = 0
    a0 = 0 #Initial acceleration at t = 0

    #Final Conditions

    sf = 3.5 #Desired final position at some final time T
    vf = 0 #Desired final velocity a t = T
    af = 0 #Desired final acceleration at t = T

    #Other Parameters 
    ego_v = 5 #(m/s) Ego vehicle velocity
    ped_v = 0.15 #(m/s) Pedestrian velocity,
    trigger_t = 18.5 #A triggering time for initiating the maneuvor
    pedestrian_x_start = 100 # Pedestrian starting x co-ordinate
    pedestrian_y_start = -2 # Pedestrian starting y co-ordinate
    
    trajectory1 = calculate_trajectory(1, s0, v0, a0, sf, vf, af, ego_v, ped_v, pedestrian_x_start, pedestrian_y_start, time=trigger_t)
    s0 = trajectory1[1][-1]

    trajectory2 = calculate_trajectory(2, s0, v0, a0, sf, vf, af, ego_v, ped_v, pedestrian_x_start, pedestrian_y_start, time=trajectory1[3][-1])
    s0 = trajectory2[1][-1]
    sf = 0

    trajectory3 = calculate_trajectory(3, s0, v0, a0, sf, vf, af, ego_v, ped_v, pedestrian_x_start, pedestrian_y_start, time=trajectory2[3][-1])
    #print(len(trajectory3[0]), len(trajectory3[1]))
    fig, axs = plt.subplots(1, 1, figsize=(20, 10))

    #plot_trajectories([trajectory1, trajectory2, trajectory3], pedestrian_trajectory=(trajectory1[4], trajectory1[5]), ax=axs[0])
    simulate_vehicle_with_pid([trajectory1, trajectory2, trajectory3], pedestrian_trajectory=(trajectory1[4], trajectory1[5]), ax=None)

    plt.show()

if __name__ == "__main__":
    main()
