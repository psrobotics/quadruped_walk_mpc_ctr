function [state_traj] = body_traj_planner(x_init ,vel_tar_local, horizon_steps_num, dt)
% body traj planner
% calcuate a mpc horizon's global body traj with desired velocity input

% r p y x y z, and their velocity, 6+6
state_traj = zeros(12,horizon_steps_num); 
% copy input init state and target velocity
state_traj(1:6,1) = x_init;

for k=1:horizon_steps_num-1
    x_t = state_traj(1:6,k);
    % input velocity is in local coordinate, while state traj is under global coordinate
    % turn it into global coordinate
    xyz_vel_local = vel_tar_local(4:6);
    % get current body rotation mat
    rot_mat_t = rot_zyx(x_t(1:3));
    xyz_vel_global = rot_mat_t*xyz_vel_local;
    % copy the rpy velocity, and xyz velocity under world coord
    vel_tar_global = vel_tar_local;
    vel_tar_global(4:6) = xyz_vel_global;
    % desired body state at next time step
    x_n = x_t + vel_tar_global*dt;
    state_traj(1:6,k+1) = x_n;
    state_traj(7:12,k) = vel_tar_global;
end

% copy final step velocity
state_traj(7:12,end) = state_traj(7:12,end-1);

end

