import numpy as np 
import matplotlib.pyplot as plt

class TrajectoryPlanner:
    """
    A class to represent a trajectory planner for a vehicle.

    Attributes:
    s0 (float): Initial position.
    v0 (float): Initial velocity.
    a0 (float): Initial acceleration.
    sf (float): Desired final position.
    vf (float): Desired final velocity.
    af (float): Desired final acceleration.
    tf (float): Time to collision or final time.
    ego_v (float): Ego vehicle velocity.
    c0, c1, c2, c3, c4, c5 (float): Coefficients of the quintic polynomial.
    """
    def __init__(self, s0, v0, a0, sf, vf, af, ttc, ego_v):
        """
        Initialize the TrajectoryPlanner with initial and final conditions..
        """
        self.s0 = s0
        self.v0 = v0
        self.a0 = a0/2 
        self.t0 = 0 
        self.sf = sf
        self.vf = vf
        self.af = af
        self.tf = ttc
        self.ego_v = ego_v
        
        T = np.array([[1, self.t0, self.t0**2, self.t0**3, self.t0**4, self.t0**5],
                      [0, 1, 2*self.t0, 3*self.t0**2, 4*self.t0**3, 5*self.t0**4],
                      [0, 0, 2, 6*self.t0, 12*self.t0**2, 20*self.t0**3],
                      [1, self.tf, self.tf**2, self.tf**3, self.tf**4, self.tf**5],
                      [0, 1, 2*self.tf, 3*self.tf**2, 4*self.tf**3, 5*self.tf**4],
                      [0, 0, 2, 6*self.tf, 12*self.tf**2, 20*self.tf**3]])
        
        Q = [self.s0, self.v0, self.a0, self.sf, self.vf, self.af]
        self.c0, self.c1, self.c2, self.c3, self.c4, self.c5 = np.linalg.solve(T, Q)

    def calc_x_coord(self, t):
        """
        Calculate the x-coordinate of the vehicle at time t.

        Parameters:
        t (float): Time.

        Returns:
        float: x-coordinate.
        """
        return self.ego_v * t

    def calc_y_coord(self, t):
        """
        Calculate the y-coordinate of the vehicle at time t.

        Parameters:
        t (float): Time.

        Returns:
        float: y-coordinate.
        """
        return self.c0 + self.c1 * t + self.c2 * t **2 + self.c3 * t**3 + self.c4 * t**4 + self.c5 * t**5

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