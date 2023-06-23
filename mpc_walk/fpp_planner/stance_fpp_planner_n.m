function [ref_fpp_traj, ref_contact_event, ctr_p]...
    = stance_fpp_planner_n(state_traj, vel_tar_local, world_p, body_p, ctr_p)
% plan and combine stance swing fpps, and contact matrix from target
% velocity and gait clocks

% update local clock based on current time
% add a tiny time offset avoid t=0 situation
counter_g = ctr_p.counter_gait;
c_num_cyc = ctr_p.counter_gait_cycle;
c_num_half_cyc = ctr_p.counter_gait_cycle/2;

t_gait = ctr_p.t_gait;
dt_mpc = ctr_p.dt_mpc;
dt_gait = ctr_p.dt_gait;

% pick up local contact event mat
contact_mat = ones(4,4);
if ctr_p.gait_tar == 1 % trot
    contact_mat = ctr_p.contact_trot;
elseif ctr_p.gait_tar == 2 % pace
    contact_mat = ctr_p.contact_pace;
elseif ctr_p.gait_tar == 3 % bounding
    contact_mat = ctr_p.contact_bound;
elseif ctr_p.gait_tar == 4 % gallop
    contact_mat = ctr_p.contact_gallop;
else
    error('unknown gait defined');
end

% init ref contact event, 4*N
ref_contact_event = ones(4, ctr_p.mpc_horizon_steps);
% init ref fpp, 12*n
ref_fpp_traj = zeros(12, ctr_p.mpc_horizon_steps);

% plan leg traj for next mpc horizion
for k=1:ctr_p.mpc_horizon_steps

    if (counter_g>0 && counter_g<c_num_half_cyc)
        ref_contact_event(:,k) = contact_mat(:,1);
    elseif (counter_g>=c_num_half_cyc && counter_g<c_num_cyc)
        ref_contact_event(:,k) = contact_mat(:,3);
    end

    if (counter_g>=0 && counter_g<100 && ~ctr_p.phase_lock) || ...
       (counter_g>=c_num_half_cyc && counter_g<c_num_half_cyc+100 && ~ctr_p.phase_lock)
        % stance/swing switch situation
        ctr_p.phase_lock = 1;
        % calc next fpp
        fpp_mat_next_tmp = calc_next_fpp(state_traj, vel_tar_local, world_p, body_p, ctr_p, k);
        % local fpp state update, 
        % since old swing foot already touches ground
        ctr_p.fpp_mat_now = ctr_p.fpp_mat_next;
        % update next fpp target
        ctr_p.fpp_mat_next = fpp_mat_next_tmp;
        fprintf('fpp swap\n');
    elseif (counter_g>=250 && counter_g<350 && ctr_p.phase_lock) || ...
           (counter_g>=750 && counter_g<450 && ctr_p.phase_lock)
        ctr_p.phase_lock = 0;
    end

    for leg_k=1:4
        if ref_contact_event(leg_k,k) == 1
            % stance leg
            ref_fpp_traj((leg_k-1)*3+1:(leg_k-1)*3+3,k) = ctr_p.fpp_mat_now(:,leg_k);
        elseif ref_contact_event(leg_k,k) == 0
            % swing leg
            ref_fpp_traj((leg_k-1)*3+1:(leg_k-1)*3+3,k) = -1;
        end
    end

    counter_g = counter_g + dt_mpc/dt_gait
end

end

%% calc next foot placement point mat based on state traj, target velocity
function fpp_mat_next = calc_next_fpp(state_traj, vel_tar_local, world_p, body_p, ctr_p, k)
% k is current time step
fpp_mat_next = zeros(3,4);
% calc next stance fpp
rot_mat_t = rot_zyx(state_traj(1:3,k));
% get global target linear velocity (v cmd in global coord)
xyz_vel_global = rot_mat_t*vel_tar_local(4:6);

t_stance = 0.5*ctr_p.t_gait;
p_symmetry = t_stance/2*state_traj(10:12,k) + ctr_p.gait_k_p*(state_traj(10:12,k)-xyz_vel_global);

com_z = state_traj(6,k); 
p_cent = 0.5*sqrt(com_z/world_p.g) * cross(state_traj(10:12,k),vel_tar_local(1:3));

for leg_k = 1:4
    p_shoulder = state_traj(4:6,k) + rot_mat_t*body_p.hip_vec(:,leg_k);
    p_next = p_shoulder + p_symmetry + p_cent;
    % set locol fpp z height (=0 if no terrain sensor)
    p_next(3) = 0;
    fpp_mat_next(:,leg_k) = p_next;
end

end

