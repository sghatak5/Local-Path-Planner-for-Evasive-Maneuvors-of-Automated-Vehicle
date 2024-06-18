import trajectoryplannar as tp
import vehiclemodel as vm
import pidcontroller as pid
import plottrajectory as plot

#Trajectory Plannar
#Initial Conditions
s0 = 0 #Initial postion at t = 0
v0 = 0 #Initial velocity at t = 0
a0 = 0 #Initial acceleration at t = 0

#Final Conditions
sf = 3.5 #Desired final position at some final time T
vf = 0 #Desired final velocity a t = T
af = 0 #Desired final acceleration at t = T

#Other Parameters 
ego_v = 10 #(m/s) Ego vehicle velocity
ped_v = 0.17 #(m/s) Pedestrian velocity,
trigger_t = 7 #A triggering time for initiating the maneuvor
pedestrian_x_start = 100 # Pedestrian starting x co-ordinate
pedestrian_y_start = -2 # Pedestrian starting y co-ordinate

#Vehicle Model
x = 0 #Vehicle x position
y = 0 # Vehicle y position
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

if plot_plannar_trajectory:  
    plot.plot_plannar_trajectories([trajectory1, trajectory2, trajectory3], 
                                   pedestrian_trajectory=(trajectory1[4], trajectory1[5]))

if simulate_vehicle_with_pid:
    vehicle = vm.Vehicle(x=x, 
                         y=y, 
                         delta=steering_angle, 
                         vel=ego_v, 
                         dt=dt)

    pid = pid.PIDController(Kp=150, 
                            Ki=10, 
                            Kd=3.8, 
                            dt=dt)
    
    vehicle_trajectory = vm.simulate_vehicle_with_pid(pid,
                                                      [trajectory1, trajectory2, trajectory3],
                                                      vehicle, 
                                                      dt=dt, 
                                                      sim_time=sim_time)
    
if plot_vehicle_trajectory:
    plot.plot_vehicle_trajectories(vehicle_trajectory,
                                   pedestrian_trajectory=(trajectory1[4], trajectory1[5]), 
                                   ax=None)