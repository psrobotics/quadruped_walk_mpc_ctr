function [ref_traj_v, ctr_p] =...
    ref_traj_planner(x_init, vel_tar_local, world_p, body_p, ctr_p, path)
% generate ref and initial guess traj for 1 mpc cycle with target velocity
% x init - init pose, 6*1
% vel_tar_local - target input velocity in body coordinate, dot rpy, xyz, 6*1

addpath(path.casadi);
import casadi.*;

horizon_step_num = ctr_p.mpc_horizon_steps;
dt_mpc = ctr_p.dt_mpc;

% first get ref body traj
[state_traj] = body_traj_planner(x_init, vel_tar_local, horizon_step_num, dt_mpc);
% get fpp and contact event
[ref_fpp_traj, ref_contact_event, ctr_p]...
    = stance_fpp_planner_n(state_traj, vel_tar_local, world_p, body_p, ctr_p);

ref_traj_v.x_ref_val = state_traj;
% just set all initial GRForces to be zero 
ref_traj_v.f_ref_val = zeros(12,horizon_step_num);
ref_traj_v.fp_ref_val = ref_fpp_traj;
ref_traj_v.contact_event_val = ref_contact_event;

% combine into 1d to feedinto casadi
ref_traj_v.p = [reshape(ref_traj_v.x_ref_val, body_p.state_dim*(horizon_step_num+1), 1);...
                reshape(ref_traj_v.f_ref_val, body_p.f_dim*horizon_step_num, 1);...
                reshape(ref_traj_v.fp_ref_val, body_p.fp_dim*horizon_step_num, 1);...
                reshape(ref_traj_v.contact_event_val, 4*horizon_step_num,1)];
% initial states, for all states in each timestep
ref_traj_v.x0 = [reshape(ref_traj_v.x_ref_val, body_p.state_dim*(horizon_step_num+1), 1);...
                 reshape(ref_traj_v.f_ref_val, body_p.f_dim*horizon_step_num, 1);...
                 reshape(ref_traj_v.fp_ref_val, body_p.fp_dim*horizon_step_num, 1)]; 

end

