# Importing libraries
import numpy as np 
import matplotlib.pyplot as plt


class TrajectoryPlanner:
    def __init__(self, s0, v0, a0, sf, vf, af, ttc, ego_v):
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
        return self.ego_v * t

    def calc_y_coord(self, t):
        return self.c0 + self.c1 * t + self.c2 * t **2 + self.c3 * t**3 + self.c4 * t**4 + self.c5 * t**5


def plot_trajectories(trajectories, pedestrian_trajectory=None):
    plt.figure(figsize=(10, 6))
    plt.title("Coordinate of the center of rear axle")
    
    for traj in trajectories:
        plt.plot(traj[0], traj[1], label=traj[2])
    
    if pedestrian_trajectory:
        plt.scatter(pedestrian_trajectory[0], pedestrian_trajectory[1], label="Pedestrian Trajectory", color='red', s=10)
    
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.axis('equal')
    plt.grid()
    plt.legend()
    plt.show()


def calculate_trajectory1(s0, v0, a0, sf, vf, af, ego_v):
    ttc = 0.5
    trajectory = TrajectoryPlanner(s0, v0, a0, sf, vf, af, ttc, ego_v)
    t_list = np.arange(0, ttc, 0.002)
    vehicle_x_list = [trajectory.calc_x_coord(t) for t in t_list]
    vehicle_y_list = [trajectory.calc_y_coord(t) for t in t_list]
    return vehicle_x_list, vehicle_y_list, "Trajectory 1"


def calculate_trajectory2(s0, v0, a0, sf, vf, af, ego_v, ped_v, time):
    pedestrian_x_start = 6
    pedestrian_y_start = -2
    ttc = pedestrian_x_start / ego_v
    trajectory = TrajectoryPlanner(s0, v0, a0, sf, vf, af, ttc, ego_v)
    t_list = np.arange(0, ttc, 0.002)
    vehicle_x_list = [time + trajectory.calc_x_coord(t) for t in t_list]
    vehicle_y_list = [trajectory.calc_y_coord(t) for t in t_list]
    pedestrian_x_list = [time + pedestrian_x_start] * len(t_list)
    pedestrian_y_list = [pedestrian_y_start + ped_v * t for t in t_list]
    index_to_trim = None
    for i in range(len(pedestrian_y_list)):
        if pedestrian_y_list[i] >= 0:
            index_to_trim = i
            break
    f_pedestrian_x_list = pedestrian_x_list[:index_to_trim]
    f_pedestrian_y_list = pedestrian_y_list[:index_to_trim]
    return vehicle_x_list, vehicle_y_list, "Trajectory 2", f_pedestrian_x_list, f_pedestrian_y_list


def calculate_trajectory3(s0, v0, a0, sf, vf, af, ego_v, time):
    ttc = 0.1
    trajectory = TrajectoryPlanner(s0, v0, a0, sf, vf, af, ttc, ego_v)
    t_list = np.arange(0, ttc, 0.002)
    vehicle_x_list = [time + trajectory.calc_x_coord(t) for t in t_list]
    vehicle_y_list = [trajectory.calc_y_coord(t) for t in t_list]
    return vehicle_x_list, vehicle_y_list, "Trajectory 3"


def calculate_trajectory4(s0, v0, a0, sf, vf, af, ego_v, time):
    ttc = 0.8
    trajectory = TrajectoryPlanner(s0, v0, a0, sf, vf, af, ttc, ego_v)
    t_list = np.arange(0, ttc, 0.002)
    vehicle_x_list = [time + trajectory.calc_x_coord(t) for t in t_list]
    vehicle_y_list = [trajectory.calc_y_coord(t) for t in t_list]
    return vehicle_x_list, vehicle_y_list, "Trajectory 4"


def main():
    s0, v0, a0, sf, vf, af, ego_v, ped_v = 0, 0, 0, 0, 0, 0, 8.3, 3
    
    traj1 = calculate_trajectory1(s0, v0, a0, sf, vf, af, ego_v)
    
    s0 = traj1[1][-1]
    sf = 3.5
    time = traj1[0][-1]
    
    traj2 = calculate_trajectory2(s0, v0, a0, sf, vf, af, ego_v, ped_v, time)
    
    s0 = traj2[1][-1]
    sf = 3.5
    time = traj2[0][-1]
    
    traj3 = calculate_trajectory3(s0, v0, a0, sf, vf, af, ego_v, time)
    
    s0 = traj3[1][-1]
    sf = 0
    time = traj3[0][-1]
    
    traj4 = calculate_trajectory4(s0, v0, a0, sf, vf, af, ego_v, time)
    
    plot_trajectories([traj1, traj2, traj3, traj4], pedestrian_trajectory=(traj2[3], traj2[4]))


if __name__ == "__main__":
    main()
