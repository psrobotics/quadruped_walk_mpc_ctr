%% Simulation params

world.fk = 0.5; % friction
world.g = 9.81; % gravity constant

%% Robot params
body.m = 5;
body.i_vec = [0.059150 0.101150 0.046240]*2;
body.i_mat = [i_vec(1) 0 0;... % roll
              0 i_vec(2) 0;... % pitch
              0 0 i_vec(3)]; % yaw
       
body.init_z = 0.3;

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
body.foot_pos = repmat([0; 0; -0.6*body.init_z],1,4); % init foot pos

body.phip_swing_ref = body.hip_pos + body.foot_pos;
body.phip_swing_ref_vec = reshape(body.phip_swing_ref,[],1); % ref foot pos at swing phase