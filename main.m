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

%% Get dynamics
[dyn_f] = get_srb_dynamics(world_params, body_params, path);

[mpc_v, mpc_c, mpc_p] = form_mpc_prob(path, world_params, body_params, ctr_params, dyn_f)
