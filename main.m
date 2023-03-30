%% Add casadi path

clc; clear;
close all; warning off;

%% Get hardware params & set package path
[world_params, body_params, ctr_params, path] = hardware_params();

addpath(path.casadi);
import casadi.*;

addpath('math\');
addpath('srb_dynamics\');
addpath('mpc_controller\');
addpath('fpp_planner\');
addpath('visualization\');

%% Get dynamics
[dyn_f] = get_srb_dynamics(world_params, body_params, path);

%% Form the mpc problem
[mpc_v, mpc_c, mpc_p] = form_mpc_prob(world_params, body_params, ctr_params, dyn_f, path);

[boundray_v] = add_state_boundaries(mpc_v, mpc_c, world_params, body_params, ctr_params, path);

%%
[ref_traj_v] = fpp_planner(world_params, body_params, ctr_params, path);

%% Slove the NLP prob
sol = mpc_p.solver('x0',ref_traj_v.x0,...
                   'lbx',boundray_v.lbx,...
                   'ubx',boundray_v.ubx,...
                   'lbg',boundray_v.lbg,...
                   'ubg',boundray_v.ubg,...
                   'p',ref_traj_v.p);
               
  %% Unpack data
  [x_sol, f_sol, fp_l_sol, fp_g_sol] = unpacks_sol(sol, body_params, ctr_params, path);
  
  rbt_anime(x_sol,f_sol,fp_g_sol,...
            ref_traj_v.x_ref_val,ref_traj_v.fp_ref_val,...
            ctr_params.T,ctr_params.N);