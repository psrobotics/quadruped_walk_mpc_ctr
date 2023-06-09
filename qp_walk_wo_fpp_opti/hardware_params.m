function [world, body, ctr, path] = hardware_params()
%% Casadi path
% Change to your casadi path
path.casadi = 'D:\matlab_lib\casadi-windows-matlabR2016a-v3.5.5';

%% Simulation params
world.fk = 0.6; % friction
world.g = 9.81; % gravity constant

world.friction_cone = [1/world.fk, 0 -1;...
                      -1/world.fk, 0 -1;...
                      0, 1/world.fk, -1;...
                      0, -1/world.fk, -1];

%% Controller params
ctr.phase_num = 4;
ctr.N = 80*3; % mpc period window
ctr.T = 3*3; % mpc period time
ctr.dt_val = (ctr.T/ctr.N) .* ones(ctr.N,1); % dt vector

ctr.max_jump_z = 0.55; % max jumping height, constraints
ctr.min_dump_z = 0.15; % min standing height
ctr.max_lift_vel_z = 6.5; % max jumping velocity
ctr.init_z = 0.15;

ctr.x_init_tar_val =   [pi/6*0;     pi/6*0;     pi/6*0;      0;             0;          ctr.init_z]; % init state
ctr.dx_init_tar_val =  [pi/6/9*0;   pi/6/9*0;   pi/6/9*0;    0.21*1;        0;          0]; % init d state
ctr.x_final_tar_val =  [pi/6*0;     pi/6*0;     pi/6*0;      0.21*9*1;      0.0005*9;   ctr.init_z]; % final target state r p y; x y z
ctr.dx_final_tar_val = [pi/6/9*0;   pi/6/9*0;   pi/6/9*0;    0.21*1;        0.0005;     0];

%ctr.contact_state_ratio = ctr.N.*[0.35 0.15 0.475 0.025]; % pull, jump, flight, impact
ctr.contact_state_ratio = ctr.N.*[1/12 1/12 1/12 1/12 1/12 1/12 1/12 1/12 1/12 1/12 1/12 1/12]; % pull, jump, flight, impact
% ctr.contact_state_val = [ones(4, ctr.contact_state_ratio(1)),...
%                              0 * ones(4, ctr.contact_state_ratio(2)),...
%                              ones(4, ctr.contact_state_ratio(3)),...
%                              0 * ones(4, ctr.contact_state_ratio(4)),...
%                              ones(4, ctr.contact_state_ratio(1)),...
%                              0 * ones(4, ctr.contact_state_ratio(2)),...
%                              ones(4, ctr.contact_state_ratio(3)),...
%                              0 * ones(4, ctr.contact_state_ratio(4)),...
%                              ones(4, ctr.contact_state_ratio(1)),...
%                              0 * ones(4, ctr.contact_state_ratio(2)),...
%                              ones(4, ctr.contact_state_ratio(3)),...
%                              0 * ones(4, ctr.contact_state_ratio(4))]; % no foot contact during last 2 phases

ctr.contact_state_val = [repmat([1;0;0;1], 1, ctr.contact_state_ratio(1)),...
                         repmat([0;1;1;0], 1, ctr.contact_state_ratio(1)),...
                         repmat([1;0;0;1], 1, ctr.contact_state_ratio(1)),...
                         repmat([0;1;1;0], 1, ctr.contact_state_ratio(1)),...
                         repmat([1;0;0;1], 1, ctr.contact_state_ratio(1)),...
                         repmat([0;1;1;0], 1, ctr.contact_state_ratio(1)),...
                         repmat([1;0;0;1], 1, ctr.contact_state_ratio(1)),...
                         repmat([0;1;1;0], 1, ctr.contact_state_ratio(1)),...
                         repmat([1;0;0;1], 1, ctr.contact_state_ratio(1)),...
                         repmat([0;1;1;0], 1, ctr.contact_state_ratio(1)),...
                         repmat([1;0;0;1], 1, ctr.contact_state_ratio(1)),...
                         repmat([0;1;1;0], 1, ctr.contact_state_ratio(1))]; % no foot contact during last 2 phases

% ctr.contact_state_val = [repmat([1;1;0;0], 1, ctr.contact_state_ratio(1)),...
%                          repmat([0;0;1;1], 1, ctr.contact_state_ratio(1)),...
%                          repmat([1;1;0;0], 1, ctr.contact_state_ratio(1)),...
%                          repmat([0;0;1;1], 1, ctr.contact_state_ratio(1)),...
%                          repmat([1;1;0;0], 1, ctr.contact_state_ratio(1)),...
%                          repmat([0;0;1;1], 1, ctr.contact_state_ratio(1)),...
%                          repmat([1;1;0;0], 1, ctr.contact_state_ratio(1)),...
%                          repmat([0;0;1;1], 1, ctr.contact_state_ratio(1)),...
%                          repmat([1;1;0;0], 1, ctr.contact_state_ratio(1)),...
%                          repmat([0;0;1;1], 1, ctr.contact_state_ratio(1)),...
%                          repmat([1;1;0;0], 1, ctr.contact_state_ratio(1)),...
%                          repmat([0;0;1;1], 1, ctr.contact_state_ratio(1))]; % no foot contact during last 2 phases

ctr.gait_num = 12;

%% new controller params
ctr.t_gloal_n = 0; % global clock

ctr.t_gait = 1.0; % time for each gait cycle
ctr.dt = 0.001; % simulation timestep

% steps in one mpc horizon window
ctr.mpc_horizon_steps = 400; 
% mpc simulation timestep
ctr.dt_mpc = 0.03;
% time for one mpc horizon
ctr.t_mpc_horizon = ctr.dt_mpc*ctr.mpc_horizon_steps;

% gait params
ctr.gait_tar = 1; % 1 trot, 2 pace, 3 bounding, 4 gallop
% gait height
ctr.gait_h = 0.12;
% each gait have 4 phase, 
% each col is a contact event for 4 legs at a time step
ctr.contact_trot = [[1;0;0;1],[1;0;0;1],[0;1;1;0],[0;1;1;0]];
ctr.contact_pace = [[1;0;1;0],[1;0;1;0],[0;1;0;1],[0;1;0;1]];
ctr.contact_bound = [[1;1;0;0],[1;1;0;0],[0;0;1;1],[0;0;1;1]];
ctr.contact_gallop = [[1;1;1;1],[1;1;1;1],[0;0;0;0],[0;0;0;0]];

% kp for gait p controller
ctr.gait_k_p = 0.03;
 
%% mpc gains
% cost gains
ctr.weight.QX = [10 10 10, 10 10 10, 10 10 10, 10 10 10 ]'; % state error
ctr.weight.QN = [10 10 10, 10 10 10, 10 10 10, 10 10 10 ]'; % state error, final
ctr.weight.Qc = 200*[5 5 5]'; % foot placement error
ctr.weight.Qf = [0.0001 0.0001 0.001]'; % force error, min energy

%% casadi optimal setting
% casadi optimal settings
ctr.opt_setting.expand =true;
ctr.opt_setting.ipopt.max_iter=800; % 1500
ctr.opt_setting.ipopt.print_level=0;
ctr.opt_setting.ipopt.acceptable_tol=1e-4 * 1;
ctr.opt_setting.ipopt.acceptable_obj_change_tol=1e-6 * 1;
ctr.opt_setting.ipopt.tol=1e-4 * 1;
ctr.opt_setting.ipopt.nlp_scaling_method='gradient-based';
ctr.opt_setting.ipopt.constr_viol_tol=1e-3 * 1;
ctr.opt_setting.ipopt.fixed_variable_treatment='relax_bounds';

%% Robot hardware params

body.state_dim = 12; % number of dim of state, rpy xyz dot_rpy dot_xyz
body.f_dim = 12; % number of dim of leg force, 3*4
body.fp_dim = 12; % number of dim of leg pos, 3*4


body.m = 5;
body.i_vec = [0.06 0.1 0.05]*2;
body.i_mat = [body.i_vec(1) 0 0;... % roll
              0 body.i_vec(2) 0;... % pitch
              0 0 body.i_vec(3)]; % yaw
       
body.length = 0.34;
body.width = 0.26;

% foot motion range, in m
body.foot_x_range = 0.15;
body.foot_y_range = 0.15;
body.foot_z_range = 0.3;

% output force range
body.max_zforce = 1000;

% calaute hip position
body.hip_vec = [body.length/2; body.width/2; 0];
body.hip_dir_mat = [1 1 -1 -1; 1 -1 1 -1; 0 0 0 0];
body.hip_pos = body.hip_dir_mat .* repmat(body.hip_vec,1,4);
body.foot_pos = repmat([0; 0; -0.6*ctr.init_z],1,4); % init foot pos

body.phip_swing_ref = body.hip_pos + body.foot_pos;
% ref foot pos at swing phase

body.phip_swing_ref_vec = reshape(body.phip_swing_ref,[],1);

% the range foot can move within
body.foot_convex_hull = [1 0 0 -body.foot_x_range;
                        -1 0 0 -body.foot_x_range;
                        0 1 0 -body.foot_y_range;
                        0 -1 0 -body.foot_y_range;
                        0 0 1 -ctr.min_dump_z;
                        0 0 -1 -body.foot_z_range];
                

end