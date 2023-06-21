function [ref_fpp_traj, ref_contact_event]...
    = stance_fpp_planner(x_init ,vel_tar_local, world_p, body_p, ctr_p)
% plan and combine stance swing fpps, and contact matrix from target
% velocity and gait clocks

% update local clock based on current time
% add a tiny time offset avoid t=0 situation
t_global_n = ctr_p.t_gloal_n + 1e-8;
t_gait = ctr_p.t_gait;
dt_mpc = ctr_p.dt_mpc;

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
ref_fpp_traj = 0;

for k=1:ctr_p.mpc_horizon_steps
    % local gait process, 0~1
    t_local_gait_n = (mod(t_global_n, t_gait)) / t_gait;
    % which phase current gait in, 4 phases in total, 1~4
    phase_local_gait_n = ceil(t_local_gait_n/0.25);
    % fill current contact event
    ref_contact_event(:,k) = contact_mat(:,phase_local_gait_n);

    t_global_n = t_global_n + dt_mpc;
end

end

