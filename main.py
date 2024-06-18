import trajectoryplannar as tp
import vehiclemodel as vm
import pidcontroller as pid
import plottrajectory as plot

def main():
    """
    Main function to calculate and plot the trajectory of a vehicle 
    and a pedestrian, and simulate the vehicle motion using a PID controller.

    Trajectory Plannar:
    - Initial Conditions:
        s0: Initial position at t = 0 (meters)
        v0: Initial velocity at t = 0 (m/s)
        a0: Initial acceleration at t = 0 (m/s^2)
    - Final Conditions:
        sf: Desired final position at some final time T (meters)
        vf: Desired final velocity at t = T (m/s)
        af: Desired final acceleration at t = T (m/s^2)
    - Other Parameters:
        ego_v: Ego vehicle velocity (m/s)
        ped_v: Pedestrian velocity (m/s)
        trigger_t: A triggering time for initiating the maneuver (seconds)
        pedestrian_x_start: Pedestrian starting x-coordinate (meters)
        pedestrian_y_start: Pedestrian starting y-coordinate (meters)

    Vehicle Model:
    - Initial conditions:
        x: Vehicle x position (meters)
        y: Vehicle y position (meters)
        steering_angle: Vehicle steering angle (radians)

    PID Controller:
    - Parameters:
        Kp: Proportional gain
        Ki: Integral gain
        Kd: Derivative gain
        dt: Time step for simulation (seconds)
        sim_time: Total simulation time (seconds)

    Flags:
    - calculate_trajectory: Whether to calculate the trajectory
    - plot_plannar_trajectory: Whether to plot the planned trajectories
    - simulate_vehicle_with_pid: Whether to simulate the vehicle with PID control
    - plot_vehicle_trajectory: Whether to plot the vehicle trajectory
    """

    #Initial Conditions
    s0 = 0 
    v0 = 0 
    a0 = 0 

    #Final Conditions
    sf = 3.5 
    vf = 0 
    af = 0 

    #Other Parameters 
    ego_v = 10 
    ped_v = 0.17 
    trigger_t = 7 
    pedestrian_x_start = 100 
    pedestrian_y_start = -2 

    #Vehicle Model
    x = 0 
    y = 0 
    steering_angle = 0

    #PID Controller
    Kp=150
    Ki=10
    Kd=3.8
    dt=0.03
    sim_time = 30

    calculate_trajectory = True
    plot_plannar_trajectory = True
    simulate_vehicle_with_pid = True
    plot_vehicle_trajectory = True

    if calculate_trajectory:
        trajectory1 = tp.calculate_trajectory(1, s0, v0, a0, sf, vf, af, ego_v, ped_v, 
                                            pedestrian_x_start, 
                                            pedestrian_y_start, 
                                            time=trigger_t)
        s0 = trajectory1[1][-1]

        trajectory2 = tp.calculate_trajectory(2, s0, v0, a0, sf, vf, af, ego_v, ped_v, 
                                            pedestrian_x_start, 
                                            pedestrian_y_start, 
                                            time=trajectory1[3][-1])
        s0 = trajectory2[1][-1]
        sf = 0

        trajectory3 = tp.calculate_trajectory(3, s0, v0, a0, sf, vf, af, ego_v, ped_v, 
                                            pedestrian_x_start, 
                                            pedestrian_y_start, 
                                            time=trajectory2[3][-1])



    if simulate_vehicle_with_pid:
        vehicle = vm.Vehicle(x=x, 
                            y=y, 
                            delta=steering_angle, 
                            vel=ego_v, 
                            dt=dt)

        pid_controller = pid.PIDController(Kp=150, 
                                Ki=10, 
                                Kd=3.8, 
                                dt=dt)
        
        vehicle_trajectory = vm.simulate_vehicle_with_pid(pid_controller,
                                                        [trajectory1, trajectory2, trajectory3],
                                                        vehicle, 
                                                        dt=dt, 
                                                        sim_time=sim_time)
    if plot_plannar_trajectory:  
        plot.plot_plannar_trajectories([trajectory1, trajectory2, trajectory3], 
                                    pedestrian_trajectory=(trajectory1[4], trajectory1[5]))
               
    if plot_vehicle_trajectory:
        plot.plot_vehicle_trajectories(vehicle_trajectory,
                                    pedestrian_trajectory=(trajectory1[4], trajectory1[5]), 
                                    ax=None)
        
if __name__ == "__main__":
    main()