%% Get all constraints in a mpc cycle

function [mpc_v] = form_mpc_prob(path, world_p, body_p, ctr_p)

addpath(path.casadi);
import casadi.*;

%% Casadi variables array in one prediction widow
mpc_v.x_arr = SX.sym('x_arr', state_dim, N+1); % state
mpc_v.f_arr = SX.sym('f_arr', f_dim, N); % foot force / input
mpc_v.fp_arr = SX.sym('fp_arr', fp_dim, N); % foot position

%% Reference traj
mpc_v.x_ref_arr = SX.sym('x_ref_arr', state_dim, N+1); % state
mpc_v.f_ref_arr = SX.sym('f_ref_arr', f_dim, N); % foot force
mpc_v.fp_ref_arr = SX.sym('fp_ref_arr', fp_dim, N); % foot position

end

