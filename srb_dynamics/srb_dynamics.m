%% SRB dynamic model of quadruped in 3d space
% Shuang Peng 02/2023 

clc; 
clear;
close all;
warning off;

% Change to your casadi path - https://web.casadi.org/
addpath('D:\matlab_lib\casadi-windows-matlabR2016a-v3.5.5');
import casadi.*;
                 
%% SRB dynamic model
world.fk = 0.5; %friction coefficient
world.g = 9.81;

body.m = 5*2; % Body Mass

init_z = 0.3;

i_vec = [0.059150 0.101150 0.046240]*2; 
body.i_mat = [i_vec(1) 0 0;... % roll
           0 i_vec(2) 0;... % pitch
           0 0 i_vec(3)]; % yaw
body.length = 0.34;
body.width = 0.26;

% foot motion range, in m
body.foot_x_range = 0.15;
body.foot_y_range = 0.15;
body.foot_z_range = 0.3;

body.max_zforce = 1000;

hip_vec = [body.length/2; body.width/2; 0];
hip_dir_mat = [1 1 -1 -1; 1 -1 1 -1; 0 0 0 0];
body.hip_pos = hip_dir_mat .* repmat(hip_vec,1,4);
body.foot_pos = repmat([0; 0; -0.6*init_z],1,4); % init foot pos

body.phip_swing_ref = body.hip_pos + body.foot_pos;
body.phip_swing_ref_vec = reshape(body.phip_swing_ref,[],1); % ref foot pos at swing phase

% build the dynamic equation
state_dim = 12; % number of dim of state, rpy xyz dot_rpy dot_xyz
f_dim = 12; % number of dim of leg force, 3*4
fp_dim = 12; % number of dim of leg pos, 3*4

% use casadi variables for optimization
x_k = SX.sym('x_k', state_dim, 1); % state
f_k = SX.sym('f_k', f_dim, 1); % foot force
fp_k = SX.sym('fp_k', fp_dim, 1); % foot position

% z-psi-yaw
% y-theta-pitch
% x-phi-roll

rot_mat_zyx = rot_zyx(x_k(1:3));

s_yaw = sin(x_k(3));
c_yaw = cos(x_k(3));
t_yaw = tan(x_k(3));

s_pitch = sin(x_k(2));
c_pitch = cos(x_k(2));
t_pitch = tan(x_k(2));

inv_rot_linear = [c_yaw s_yaw 0; 
                  -1*s_yaw c_yaw 0; 
                  0 0 1];
              
inv_rot_nonlinear = [c_yaw/c_pitch s_yaw/c_pitch 0;
                     -1*s_yaw c_yaw 0;
                     c_yaw*t_pitch s_yaw*t_pitch 1];
                
% convert the intertia tensor from local cod to world cod
i_mat_w = rot_mat_zyx*body.i_mat*rot_mat_zyx'; %i_mat in world
i_mat_w_inv = eye(3)/i_mat_w;

% A, B, G mat
A = [zeros(3) zeros(3) inv_rot_linear zeros(3);...
     zeros(3) zeros(3) zeros(3) eye(3);...
     zeros(3) zeros(3) zeros(3) zeros(3);...
     zeros(3) zeros(3) zeros(3) zeros(3)];
% A = [z3 z3 inv_rot z3;
%      z3 z3 z3      I3
%      z3 z3 z3      z3
%      z3 z3 z3      z3];

B = [zeros(3) zeros(3) zeros(3) zeros(3);...
     zeros(3) zeros(3) zeros(3) zeros(3);...
     i_mat_w_inv*skew_mat(fp_k(1:3)), i_mat_w_inv*skew_mat(fp_k(4:6)), i_mat_w_inv*skew_mat(fp_k(7:9)), i_mat_w_inv*skew_mat(fp_k(10:12));...
     eye(3)/body.m, eye(3)/body.m, eye(3)/body.m, eye(3)/body.m];
% B = [z3 z3 z3 z3
%      z3 z3 z3 z3
%      (I_m)^-1*[f_pos]x
%      I3/m I3/m I3/m I3/m];

G = zeros(12,1);
G(12) = -1*world.g;

d_x = A*x_k + B*f_k + G;

% map the dynamic function
dyn_f = Function('dyn_f',{x_k;f_k;fp_k},{d_x},{'state','leg_force','foot_pos'},{'d_state'});

body.foot_pos

% simple test
x_init = [0;0.0;0; 0.0;0.0;0.5 ;0;0;0; 0;0;0];
% output is dot_state
dyn_f(x_init,zeros(12,1),zeros(12,1))