function [ref_traj_v] = fpp_planner(world_p, body_p, ctr_p, path)

addpath(path.casadi);
import casadi.*;

%% Generate reference trajectory
%fp_ref_i = diag([1 1 1, 1 -1 1, -1 1 1, -1 -1 1])*repmat([0.4 0.2 -1*ctr_p.init_z],1,4)'; % foot pos ref

ref_traj_v.x_ref_val = zeros(12,ctr_p.N+1);
ref_traj_v.f_ref_val = zeros(12,ctr_p.N);
ref_traj_v.fp_ref_val = zeros(12,ctr_p.N);

for i = 1:6
    ref_traj_v.x_ref_val(i,:) = linspace(ctr_p.x_init_tar_val(i),ctr_p.x_final_tar_val(i),ctr_p.N+1); %rpy xyz
    ref_traj_v.x_ref_val(i+6,:) = linspace(ctr_p.dx_init_tar_val(i),ctr_p.dx_final_tar_val(i),ctr_p.N+1); %velocity
end

% spine on the z axis
s_a = [ref_traj_v.x_ref_val(4,1),ref_traj_v.x_ref_val(4,ctr_p.N/2),ref_traj_v.x_ref_val(4,ctr_p.N)]; % x axis
s_b = [ctr_p.x_init_tar_val(6),ctr_p.x_final_tar_val(6),ctr_p.x_init_tar_val(6)+0]; % z axis
ref_traj_v.x_ref_val(6,:) = interp1(s_a,s_b,ref_traj_v.x_ref_val(4,:),'spline');


% calcuate foot placement points, openloop one
t_stance = ctr_p.T/12;
ww = body_p.width;
ll = body_p.length;

for k=1:ctr_p.N
    fp_symmetry = t_stance/2*ctr_p.dx_final_tar_val(4:6);
    fp_centrifugal = cross((1/2*sqrt(ctr_p.init_z/world_p.g)*ctr_p.dx_final_tar_val(4:6)), ctr_p.dx_final_tar_val(1:3));
    rot_mat_z_arr = rotz(ref_traj_v.x_ref_val(3,k));
    hip_com_g_vec_fr = ref_traj_v.x_ref_val(4:6,k) + rot_mat_z_arr* [1*ll/2; 1*ww/2; 0] - [0;0;ctr_p.init_z];
    hip_com_g_vec_fl = ref_traj_v.x_ref_val(4:6,k) + rot_mat_z_arr* [1*ll/2; -1*ww/2; 0] - [0;0;ctr_p.init_z];
    hip_com_g_vec_br = ref_traj_v.x_ref_val(4:6,k) + rot_mat_z_arr* [-1*ll/2; 1*ww/2; 0] - [0;0;ctr_p.init_z];
    hip_com_g_vec_bl = ref_traj_v.x_ref_val(4:6,k) + rot_mat_z_arr* [-1*ll/2; -1*ww/2; 0] - [0;0;ctr_p.init_z];
    
    ref_traj_v.fp_ref_val(:,k) = [hip_com_g_vec_fr;hip_com_g_vec_fl;hip_com_g_vec_br;hip_com_g_vec_bl]+...
        repmat(fp_symmetry,4,1)+...
        repmat(fp_centrifugal,4,1);
   
    contact_v = ctr_p.contact_state_val(:,k);
    
    % calcuate ref pos for swing foot
    period_now = mod(k,ctr_p.N/ctr_p.gait_num);
    period_now_rad = pi*(period_now/(ctr_p.N/ctr_p.gait_num));
    for leg_k = 1:4
        if(contact_v(leg_k) == 0)
            ref_traj_v.fp_ref_val(leg_k*3,k) = 0.15*sin(period_now_rad); % sine foot swing height
        end
    end
    
end

% combine all ref traj
ref_traj_v.p = [reshape(ref_traj_v.x_ref_val,body_p.state_dim*(ctr_p.N+1),1);...
                reshape(ref_traj_v.f_ref_val,body_p.f_dim*ctr_p.N,1);...
                reshape(ref_traj_v.fp_ref_val,body_p.fp_dim*ctr_p.N,1);...
                reshape(ctr_p.contact_state_val,4*ctr_p.N,1)];
      
ref_traj_v.x0 = [reshape(ref_traj_v.x_ref_val,body_p.state_dim*(ctr_p.N+1),1);...
                 reshape(ref_traj_v.f_ref_val,body_p.f_dim*ctr_p.N,1);...
                 reshape(ref_traj_v.fp_ref_val,body_p.fp_dim*ctr_p.N,1)]; % initial states
      
end

