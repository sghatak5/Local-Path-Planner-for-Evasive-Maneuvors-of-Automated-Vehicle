import trajectoryplannar as tp


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

calculate_trajectory = True
plot_trajectory = True

if calculate_trajectory:
    trajectory1 = tp.calculate_trajectory(1, s0, v0, a0, sf, vf, af, ego_v, ped_v, pedestrian_x_start, pedestrian_y_start, time=trigger_t)
    s0 = trajectory1[1][-1]

    trajectory2 = tp.calculate_trajectory(2, s0, v0, a0, sf, vf, af, ego_v, ped_v, pedestrian_x_start, pedestrian_y_start, time=trajectory1[3][-1])
    s0 = trajectory2[1][-1]
    sf = 0

    trajectory3 = tp.calculate_trajectory(3, s0, v0, a0, sf, vf, af, ego_v, ped_v, pedestrian_x_start, pedestrian_y_start, time=trajectory2[3][-1])
    #print(trajectory3[3][-1])
    #print(len(trajectory3[0]), len(trajectory3[1]))

if plot_trajectory:  
    tp.plot_trajectories([trajectory1, trajectory2, trajectory3], pedestrian_trajectory=(trajectory1[4], trajectory1[5]))