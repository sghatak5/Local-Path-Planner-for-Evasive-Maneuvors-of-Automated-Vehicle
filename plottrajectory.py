import matplotlib.pyplot as plt

def plot_plannar_trajectories(trajectories, pedestrian_trajectory=None):
    """
    Plot the trajectories of the vehicle and optionally the pedestrian trajectory.

    Parameters:
    trajectories (list): List of trajectories to plot.
    pedestrian_trajectory (tuple, optional): Tuple of x and y coordinates for the pedestrian trajectory. Defaults to None.
    """
    plt.figure(figsize=(12, 8))
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

def plot_vehicle_trajectories(vehicle_trajectory, pedestrian_trajectory, ax):
    if ax is None:
        ax = plt.gca()
    ax.set_title("Vehicle Trajectory with PID Control")
    ax.plot(vehicle_trajectory[0], vehicle_trajectory[1], label="Vehicle Trajectory")
    ax.plot(vehicle_trajectory[2], vehicle_trajectory[3], label="Desired Trajectory", linestyle='dashed')
    if pedestrian_trajectory:
        ax.scatter(pedestrian_trajectory[0], pedestrian_trajectory[1], label="Pedestrian Trajectory", color='green', s=10)
    ax.scatter(vehicle_trajectory[0][0], vehicle_trajectory[1][0], color='red', label="Start Position")
    ax.set_xlabel("X Position (m)")
    ax.set_ylabel("Y Position (m)")
    ax.legend()
    ax.grid(True)
    #ax.axis('equal')
    #ax.set_xlim(left=0)
    ax.set_ylim(-10, 10)

    plt.show()
    pass
