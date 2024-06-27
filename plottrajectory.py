import matplotlib.pyplot as plt

def plot_plannar_trajectories(trajectories, pedestrian_trajectory=None):
    """
    Plot the planned trajectories of the vehicle and optionally the pedestrian trajectory.

    Args:
        trajectories (list): List of trajectories to plot. Each trajectory should be a tuple where the first element 
                             is the x coordinates, the second element is the y coordinates, and the third element is the label.
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

def plot_vehicle_trajectories(vehicle_trajectory, pedestrian_trajectory, ax):#
    """
    Plot the vehicle's trajectory with PID control and optionally the pedestrian trajectory.

    Args:
        vehicle_trajectory (tuple): Tuple containing the vehicle's actual x and y positions, and the desired x and y positions.
                                    The tuple format should be (vehicle_x, vehicle_y, desired_x, desired_y).
        pedestrian_trajectory (tuple, optional): Tuple of x and y coordinates for the pedestrian trajectory. Defaults to None.
        ax (matplotlib.axes._axes.Axes, optional): Matplotlib axes object to plot on. Defaults to None.
    """
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
    ax.set_ylim(-10, 10)

    plt.show()
    
