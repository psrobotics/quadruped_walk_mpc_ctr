%% get SRB dynamic model of quadruped in 3d space
% Shuang Peng 02/2023 

function [dyn_f] = get_srb_dynamics(world_p, body_p, path)

addpath(path.casadi);
import casadi.*;

% build the dynamic equation
% use casadi variables for optimization
x_k = SX.sym('x_k', body_p.state_dim, 1); % state
f_k = SX.sym('f_k', body_p.f_dim, 1); % foot force
fp_k = SX.sym('fp_k', body_p.fp_dim, 1); % foot position

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
i_mat_w = rot_mat_zyx*body_p.i_mat*rot_mat_zyx'; %i_mat in world
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
     eye(3)/body_p.m, eye(3)/body_p.m, eye(3)/body_p.m, eye(3)/body_p.m];
% B = [z3 z3 z3 z3
%      z3 z3 z3 z3
%      (I_m)^-1*[f_pos]x
%      I3/m I3/m I3/m I3/m];

G = zeros(12,1);
G(12) = -1*world_p.g;

d_x = A*x_k + B*f_k + G;

% map the dynamic function
dyn_f = Function('dyn_f',{x_k;f_k;fp_k},{d_x},{'state','leg_force','foot_pos'},{'d_state'});

end