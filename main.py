import trajectoryplannar as tp

def main():
    s0, v0, a0, sf, vf, af, ego_v, ped_v, trigger_t = 0, 0, 0, 3.5, 0, 0, 5, 0.15, 18.5 
    
    traj2 = tp.calculate_trajectory(2, s0, v0, a0, sf, vf, af, ego_v, ped_v, trigger_t, time=trigger_t )
    
    s0 = traj2[1][-1]
    sf = 3.5
    time = traj2[3][-1]
    
    traj3 = tp.calculate_trajectory(3, s0, v0, a0, sf, vf, af, ego_v, ped_v, trigger_t, time=time )
    s0 = traj3[1][-1]
    sf = 0
    time = traj3[3][-1]
    
    traj4 = tp.calculate_trajectory(4, s0, v0, a0, sf, vf, af, ego_v, ped_v, trigger_t, time=time)
    
    tp.plot_trajectories([traj2, traj3, traj4], pedestrian_trajectory=(traj2[4], traj2[5]))


if __name__ == "__main__":
    main()