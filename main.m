%% Add casadi path

clc; clear;
close all; warning off;

%% Get hardware params & set package path
[world_p, body_p, ctr_p, path] = hardware_params();

addpath(path.casadi);
import casadi.*;

addpath('math\');
addpath('srb_dynamics\');
addpath('mpc_controller\');
addpath('fpp_planner\');
addpath('visualization\');

%% Get dynamics
[dyn_f] = get_srb_dynamics(world_p, body_p, path);

%% Form the mpc problem
[mpc_v, mpc_c, mpc_p] = form_mpc_prob(world_p, body_p, ctr_p, dyn_f, path);

[boundray_v] = add_state_boundaries(mpc_v, mpc_c, world_p, body_p, ctr_p, path);

%% Paras for mpc
x_arr = zeros(12,ctr_p.sim_N - ctr_p.N+1);
x_arr(:,1) = [ctr_p.x_init_tar_val; ctr_p.dx_init_tar_val];
f_arr = zeros(12,ctr_p.sim_N - ctr_p.N+1);
fp_g_arr = zeros(12,ctr_p.sim_N - ctr_p.N+1);

x_ref_arr = zeros(12,ctr_p.sim_N - ctr_p.N+1);
fp_g_ref_arr = zeros(12,ctr_p.sim_N - ctr_p.N+1);

dt_t = ctr_p.dt_val(1);

%% MPC loop
for k = 1:ctr_p.sim_N - ctr_p.N
    
    fprintf('sim %d of %d steps',k,ctr_p.sim_N - ctr_p.N);
    
    %current state
    x_t = x_arr(:,k);
    
    % init params for this mpc horizon
    ctr_p.x_init_tar_val = x_arr(1:6,k);
    ctr_p.dx_init_tar_val = x_arr(7:12,k);
    % control target for this mpc horizon
    ctr_p.x_final_tar_val = ctr_p.x_init_tar_val + [ctr_p.tar_body_angular_vel*ctr_p.T; ctr_p.tar_body_vel*ctr_p.T + [0;0;0.3]];
    ctr_p.dx_final_tar_val = [ctr_p.tar_body_angular_vel; ctr_p.tar_body_vel];
    % get current contact events
    ctr_p.contact_state_val = ctr_p.contact_state_val_all(:, k:k+ctr_p.N-1);
    
    % plan the local reference x,dx,fpp traj based on updated contact event, desired states
    [ref_traj_v] = fpp_planner(world_p, body_p, ctr_p, path);
    
    % Slove the NLP prob for this mpc horizon
    sol = mpc_p.solver('x0',ref_traj_v.x0,...
                       'lbx',boundray_v.lbx,...
                       'ubx',boundray_v.ubx,...
                       'lbg',boundray_v.lbg,...
                       'ubg',boundray_v.ubg,...
                       'p',ref_traj_v.p);
    % Unpack data
    [x_sol, f_sol, fp_l_sol, fp_g_sol] = unpacks_sol(sol, body_p, ctr_p, path);
    
    f_optimal = f_sol(:,1); % only use first optimal ctr from this mpc horizon
    fp_optimal = fp_l_sol(:,1); % the local fpp
    fp_g_optimal = fp_g_sol(:,1); % the global fpp
    
    % sim next state
    x_next = x_t + dyn_f(x_t,f_optimal,fp_optimal)*dt_t;
    % convert state from casadi variable to double
    x_next_v = full(x_next);
    
    x_arr(:,k+1) = x_next_v;
    f_arr(:,k+1) = f_optimal;
    fp_g_arr(:,k+1) = fp_g_optimal; % stored global foot pos
    
    x_ref_arr(:,k+1) = ref_traj_v.x_ref_val(:,1);
    fp_g_ref_arr(:,k+1) = ref_traj_v.fp_ref_val(:,1);
 
end
               
%% Unpack data
%[x_sol, f_sol, fp_l_sol, fp_g_sol] = unpacks_sol(sol, body_p, ctr_p, path);
  
%   rbt_anime(x_sol,f_sol,fp_g_sol,...
%             ref_traj_v.x_ref_val,ref_traj_v.fp_ref_val,...
%             ctr_p.T,ctr_p.N);
  
  rbt_anime(x_arr,f_arr,fp_g_arr,...
            x_ref_arr,fp_g_ref_arr,...
            ctr_p.T,ctr_p.N);
 
