import numpy as np 
import matplotlib.pyplot as plt
import time as tm

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
        #print(f"self.c0:{self.c0}, self.c1 * t:{self.c1 * t}, self.c2 * t **2:{self.c2 * t **2}, self.c3 * t**3:{self.c3 * t**3}, self.c4 * t**4:{self.c4 * t**4}, self.c5 * t**5:{self.c5 * t**5}")
        return self.c0 + self.c1 * t + self.c2 * t **2 + self.c3 * t**3 + self.c4 * t**4 + self.c5 * t**5


def plot_trajectories(trajectories, pedestrian_trajectory=None):
    plt.figure(figsize=(12, 8))
    #print(len(trajectories[0][2]))
    plt.title("Coordinate of the center of rear axle")
    
    for traj in trajectories:
        plt.plot(traj[0], traj[1], label=traj[2])
    
    if pedestrian_trajectory:
        plt.scatter(pedestrian_trajectory[0], pedestrian_trajectory[1], label="Pedestrian Trajectory", color='red', s=10)
    
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.ylim(-10, 10)
    plt.xlim(left=0)
    plt.axis('equal')
    plt.grid()
    plt.legend()
    plt.show()


def calculate_trajectory(trajectory_type, s0, v0, a0, sf, vf, af, ego_v, ped_v, trigger_t, time):
    # if trajectory_type == 1:
    #     ttc = trigger_t
    #     label = "Trajectory 1"
    if trajectory_type == 2:
        pedestrian_x_start = 100
        pedestrian_y_start = -2
        ttc = (pedestrian_x_start - (ego_v * time) )/ ego_v
        print(f"Time to collision: {ttc}")
        label = "Trajectory 2"
    elif trajectory_type == 3:
        ttc = 0.5
        label = "Trajectory 3"
    elif trajectory_type == 4:
        ttc = 3
        label = "Trajectory 4"
    else:
        raise ValueError("Invalid trajectory type")

    
    if trajectory_type ==2:
        trajectory = TrajectoryPlanner(s0, v0, a0, sf, vf, af, ttc, ego_v)
        x_t_list = np.arange(time, time + ttc, 0.02)
        y_t_list = np.arange(0, ttc, 0.02)
        vehicle_x_list = [trajectory.calc_x_coord(t) for t in x_t_list]
        #print(f"Trajectory{trajectory_type}: t_list: {t_list}, length of t_list: {len(t_list)}")
        #print(f"vehicle_x_list: {vehicle_x_list}, length of vehicle_x_list: {len(vehicle_x_list)}")
        vehicle_y_list = [trajectory.calc_y_coord(t) for t in y_t_list]
        #print(f"vehicle_y_list: {vehicle_y_list}, length of vehicle_y_list: {len(vehicle_y_list)}")
    else: 
        trajectory = TrajectoryPlanner(s0, v0, a0, sf, vf, af, ttc, ego_v)
        x_t_list = np.arange(time, time + ttc, 0.02)
        y_t_list = np.arange(0, ttc, 0.02)
        #print(t_list)
        vehicle_x_list = [trajectory.calc_x_coord(t) for t in x_t_list]
        #print(f"Trajectory{trajectory_type}: t_list: {t_list}, length of t_list: {len(t_list)}")
       # print(f"vehicle_x_list: {vehicle_x_list}, length of vehicle_x_list: {len(vehicle_x_list)}")
        vehicle_y_list = [trajectory.calc_y_coord(t) for t in y_t_list]

    if trajectory_type == 2:
        ped_t = time + ttc
        t_list_ped = np.arange(0, ped_t, 0.2)
        pedestrian_x_list = [pedestrian_x_start] * len(t_list_ped)
        #print(pedestrian_x_list)
        pedestrian_y_list = [pedestrian_y_start + ped_v * t for t in t_list_ped]
       # print(pedestrian_y_list)
        return vehicle_x_list, vehicle_y_list, label, x_t_list, pedestrian_x_list, pedestrian_y_list
    else:
        return vehicle_x_list, vehicle_y_list, label, x_t_list

def main():
    s0, v0, a0, sf, vf, af, ego_v, ped_v, trigger_t = 0, 0, 0, 3.5, 0, 0, 5, 0.15, 18.5 


    
    #traj1 = calculate_trajectory(1, s0, v0, a0, sf, vf, af, ego_v, ped_v, trigger_t,time=0)
    
    # s0 = traj1[1][-1]
    # sf = 3.5
    # time = traj1[0][-1]

    #tm.sleep(trigger_t)
    
    traj2 = calculate_trajectory(2, s0, v0, a0, sf, vf, af, ego_v, ped_v, trigger_t, time=trigger_t )
    
    s0 = traj2[1][-1]
    sf = 3.5
    time = traj2[3][-1]
    
    traj3 = calculate_trajectory(3, s0, v0, a0, sf, vf, af, ego_v, ped_v, trigger_t, time=time )
    s0 = traj3[1][-1]
    sf = 0
    time = traj3[3][-1]
    
    traj4 = calculate_trajectory(4, s0, v0, a0, sf, vf, af, ego_v, ped_v, trigger_t, time=time)
    
    plot_trajectories([traj2, traj3, traj4], pedestrian_trajectory=(traj2[4], traj2[5]))


if __name__ == "__main__":
    main()