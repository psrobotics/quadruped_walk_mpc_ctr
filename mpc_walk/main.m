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

%% init x and init target velocity
x_init = [0;0;0; 0;0;0.3; 0;0;0.3; 0.35;0;0];
x_arr = x_init;
f_arr = [];
fpp_g_arr = [];
vel_tar = [0;0;0.3; 0.35;0;0];
%% 
% get com to hip vec for 4 legs
hip_vec_s = [body_params.length/2; body_params.width/2; 0];
hip_dir_mat = [1 1 -1 -1; 1 -1 1 -1; 0 0 0 0];
% each col is a hip vec
fpp_mat_now = hip_dir_mat .* repmat(hip_vec_s,1,4);
fpp_mat_next = fpp_mat_now;
fpp_u_flag = 0;

%% traj generator test demo
state_ref_arr = [];
fpp_ref_arr = [];
contact_mat_arr = [];

%%
% sim_k = 1;
% for sim_t = 0:ctr_params.dt_mpc:2
% 
%     %fprintf('sim time %f of %f \n', sim_t, 6);
% 
%     % fpp_mat_now
%     % fpp_mat_next
% 
%     % update global clock
%     ctr_params.t_gloal_n = sim_t;
%     % current state
%     x_t = x_arr(:,sim_k);
%     % get ref traj
%     [ref_traj_v, fpp_mat_now, fpp_mat_next, fpp_u_flag] =...
%         ref_traj_planner(x_t, vel_tar, fpp_mat_now, fpp_mat_next, fpp_u_flag,...
%         world_params, body_params, ctr_params, path);
% 
%     state_ref_arr = [state_ref_arr, ref_traj_v.x_ref_val(:,1)];
%     fpp_ref_arr = [fpp_ref_arr, ref_traj_v.fp_ref_val(:,1)];
%     contact_mat_arr = [contact_mat_arr, ref_traj_v.contact_event_val(:,1)];
%     x_next = x_t;
%     % just update the velocity-based model
%     x_next(1:6) = x_next(1:6) + ctr_params.dt_mpc*vel_tar;
%     x_arr = [x_arr, x_next];
% 
%     sim_k = sim_k+1;
% end

%%
% clf;
% plot(fpp_ref_arr(1,:),'linewidth',1.5);
% hold on
% %plot(fpp_ref_arr(4,:),'linewidth',1.5)
% hold on
% plot(fpp_ref_arr(2,:),'linewidth',1.5)
% hold on
% %plot(fpp_ref_arr(5,:),'linewidth',1.5)
% hold on
% plot(fpp_ref_arr(3,:),'linewidth',1.5)
% hold on
% 
% %plot(state_ref_arr(4,:),'linewidth',1.5)
% hold on
% %plot(state_ref_arr(5,:),'linewidth',1.5)
% hold on

%% mpc cycle
sim_k = 1;
for sim_t = 0:ctr_params.dt_mpc:4
    
    fprintf('sim time %f of %f \n', sim_t, 6);
    % update global clock
    ctr_params.t_gloal_n = sim_t;
    % current state
    x_t = x_arr(:,sim_k);
    % get ref traj
    [ref_traj_v, fpp_mat_now, fpp_mat_next, fpp_u_flag] =...
        ref_traj_planner(x_t, vel_tar, fpp_mat_now, fpp_mat_next, fpp_u_flag,...
        world_params, body_params, ctr_params, path);
    % solve nlp prob
    sol = mpc_p.solver('x0',ref_traj_v.x0,...
                       'lbx',boundray_v.lbx,...
                       'ubx',boundray_v.ubx,...
                       'lbg',boundray_v.lbg,...
                       'ubg',boundray_v.ubg,...
                       'p',ref_traj_v.p);
    % unpack data
    [x_sol, f_sol, fp_l_sol, fp_g_sol] =...
        unpacks_sol(sol, body_params, ctr_params, path);
    % pick optimal control, only use first optimal ctr from this mpc horizon
    f_optimal = f_sol(:,1); 
    fp_optimal = fp_l_sol(:,1); % the local fpp, we use local coord for dynamic model
    fp_g_optimal = fp_g_sol(:,1); % the global fpp
    % sim next timestep
    tspan = [sim_t, sim_t+ctr_params.dt_mpc];
    %[t,x_out] = ode45(@quad_dyn, tspan, x_t, [], x_t, f_optimal, fp_optimal, dyn_f);
    %x_out = x_out'; % for raw output from ode45, each row is a timestep
    %x_next = x_out(:,end);
    x_next = x_t + dyn_f(x_t,f_optimal,fp_optimal)*ctr_params.dt_mpc;
    x_next = full(x_next);

    x_arr = [x_arr, x_next];
    f_arr = [f_arr, f_optimal];
    fpp_g_arr = [fpp_g_arr, fp_g_optimal];

    sim_k = sim_k+1;
end

%%
%rbt_anime(x_sol,f_sol,fp_g_sol,[], ctr_params.t_mpc_horizon, ctr_params.mpc_horizon_steps);
rbt_anime(x_arr,f_arr,fpp_g_arr,[], ctr_params.t_mpc_horizon, ctr_params.mpc_horizon_steps);
%%
clf;
plot(fpp_g_arr(4,:),'linewidth',1.5);
%%
clf;
plot(ref_traj_v.fp_ref_val(1,:),'linewidth',1.5);
hold on
plot(ref_traj_v.fp_ref_val(4,:),'linewidth',1.5)
hold on
plot(ref_traj_v.fp_ref_val(2,:),'linewidth',1.5)
hold on
plot(ref_traj_v.fp_ref_val(5,:),'linewidth',1.5)
hold on

plot(ref_traj_v.x_ref_val(4,:),'linewidth',1.5)
hold on
plot(ref_traj_v.x_ref_val(5,:),'linewidth',1.5)
hold on
%% dyn model for ode simulation
function dy = quad_dyn(t,y,x_t,f_k,fp_k,dyn_f)
% we pass this freaking casadi fcn inside
dy_dm = dyn_f(x_t,f_k,fp_k);
dy = full(dy_dm);
end

%%
% %% Slove the NLP prob
% sol = mpc_p.solver('x0',ref_traj_v.x0,...
%                    'lbx',boundray_v.lbx,...
%                    'ubx',boundray_v.ubx,...
%                    'lbg',boundray_v.lbg,...
%                    'ubg',boundray_v.ubg,...
%                    'p',ref_traj_v.p);
% 
% %% Unpack data
% [x_sol, f_sol, fp_l_sol, fp_g_sol] = unpacks_sol(sol, body_params, ctr_params, path);
% 
% %%
% %rbt_anime(x_sol,f_sol,fp_g_sol,[],ctr_params.T,ctr_params.N);

  
