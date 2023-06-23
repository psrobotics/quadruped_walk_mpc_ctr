function [world_p, body_p, ctr_p, path] = hardware_params()
%% Casadi path
% Change to your casadi path
path.casadi = 'D:\matlab_lib\casadi-windows-matlabR2016a-v3.5.5';

%% Simulation params
world_p.fk = 0.6; % friction
world_p.g = 9.81; % gravity constant

world_p.friction_cone = [1/world_p.fk, 0 -1;...
                      -1/world_p.fk, 0 -1;...
                      0, 1/world_p.fk, -1;...
                      0, -1/world_p.fk, -1];
%% Controller boundries

ctr_p.max_jump_z = 0.55; % max jumping height, constraints
ctr_p.min_dump_z = 0.15; % min standing height
ctr_p.max_lift_vel_z = 6.5; % max jumping velocity
ctr_p.init_z = 0.15;

%% Robot hardware params

body_p.state_dim = 12; % number of dim of state, rpy xyz dot_rpy dot_xyz
body_p.f_dim = 12; % number of dim of leg force, 3*4
body_p.fp_dim = 12; % number of dim of leg pos, 3*4


body_p.m = 5;
body_p.i_vec = [0.06 0.1 0.05]*2;
body_p.i_mat = [body_p.i_vec(1) 0 0;... % roll
              0 body_p.i_vec(2) 0;... % pitch
              0 0 body_p.i_vec(3)]; % yaw
       
body_p.length = 0.34;
body_p.width = 0.26;

% foot motion range, in m
body_p.foot_x_range = 0.15;
body_p.foot_y_range = 0.15;
body_p.foot_z_range = 0.3;

% output force range
body_p.max_zforce = 1000;

% calaute hip position
body_p.size_vec = [body_p.length/2; body_p.width/2; 0];
body_p.hip_dir_mat = [1 1 -1 -1; 1 -1 1 -1; 0 0 0 0];
body_p.hip_vec = body_p.hip_dir_mat .* repmat(body_p.size_vec,1,4);
body_p.foot_pos = repmat([0; 0; -0.6*ctr_p.init_z],1,4); % init foot pos

body_p.phip_swing_ref = body_p.hip_vec + body_p.foot_pos;
% ref foot pos at swing phase
body_p.phip_swing_ref_vec = reshape(body_p.phip_swing_ref,[],1);

% the range foot can move within
body_p.foot_convex_hull = [1 0 0 -body_p.foot_x_range;
                        -1 0 0 -body_p.foot_x_range;
                        0 1 0 -body_p.foot_y_range;
                        0 -1 0 -body_p.foot_y_range;
                        0 0 1 -ctr_p.min_dump_z;
                        0 0 -1 -body_p.foot_z_range];

%% new controller params
ctr_p.t_gloal_n = 0; % global clock

ctr_p.t_gait = 1.0; % time for each gait cycle

% gait counter
ctr_p.counter_gait = 0;
ctr_p.counter_gait_cycle = 1000; % 1000 cycle a gait cycle
ctr_p.dt_gait = ctr_p.t_gait/ctr_p.counter_gait_cycle;

% steps in one mpc horizon window
ctr_p.mpc_horizon_steps = 10; 
% mpc simulation timestp
ctr_p.dt_mpc = 0.0025;
% time for one mpc horizon
ctr_p.t_mpc_horizon = ctr_p.dt_mpc*ctr_p.mpc_horizon_steps;

% gait params
ctr_p.gait_tar = 1; % 1 trot, 2 pace, 3 bounding, 4 gallop
% gait height
ctr_p.gait_h = 0.12;
% each gait have 4 phase, 
% each col is a contact event for 4 legs at a time step
ctr_p.contact_trot = [[1;0;0;1],[1;0;0;1],[0;1;1;0],[0;1;1;0]];
ctr_p.contact_pace = [[1;0;1;0],[1;0;1;0],[0;1;0;1],[0;1;0;1]];
ctr_p.contact_bound = [[1;1;0;0],[1;1;0;0],[0;0;1;1],[0;0;1;1]];
ctr_p.contact_gallop = [[1;1;1;1],[1;1;1;1],[0;0;0;0],[0;0;0;0]];

% kp for gait p controller
ctr_p.gait_k_p = 0.00;

% fpp params
% each col is a leg pos
ctr_p.fpp_mat_now = body_p.hip_vec;
ctr_p.fpp_mat_next = body_p.hip_vec;
ctr_p.phase_lock = 0;
 
%% mpc gains
% cost gains
ctr_p.weight.QX = [10 10 10, 10 10 10, 10 10 10, 10 10 10 ]'; % state error
ctr_p.weight.QN = [10 10 10, 10 10 10, 10 10 10, 10 10 10 ]'; % state error, final
ctr_p.weight.Qc = 200*[5 5 5]'; % foot placement error
ctr_p.weight.Qf = [0.0001 0.0001 0.001]'; % force error, min energy

%% casadi optimal setting
% casadi optimal settings
ctr_p.opt_setting.expand =true;
ctr_p.opt_setting.ipopt.max_iter=800; % 1500
ctr_p.opt_setting.ipopt.print_level=0;
ctr_p.opt_setting.ipopt.acceptable_tol=1e-4 * 1;
ctr_p.opt_setting.ipopt.acceptable_obj_change_tol=1e-6 * 1;
ctr_p.opt_setting.ipopt.tol=1e-4 * 1;
ctr_p.opt_setting.ipopt.nlp_scaling_method='gradient-based';
ctr_p.opt_setting.ipopt.constr_viol_tol=1e-3 * 1;
ctr_p.opt_setting.ipopt.fixed_variable_treatment='relax_bounds';
end