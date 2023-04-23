function params = get_params(brt_params)
    %% Pack all params

    % Dynamics parameters
    params.nX = 3; % States - [p_x, p_y, theta]
    params.nU = 1; % Controls - [w]
    params.dt = 0.05; % Discretization step
    params.speed = 1; % Maximum speed - 1 m/s
    params.wMax = 1.1; % Maximum control - 1.1 rad/s
    params.xinit = [0; 0.35/2; 0]; % Initial state % -4 -4

    % Setup environment parameters
    % goal params  
    params.goalX = 0.7;
    params.goalY = 0.35/2;
    params.goalR = 0.25;

    % obstacle 1 params
    params.obsX1 = brt_params.obst_pos_1(1);
    params.obsY1 = brt_params.obst_pos_1(2);
    params.obswidth1 = brt_params.obst_l;
    params.obsheight1 = brt_params.obst_l;

    % obstacle 2 params
    params.obsX2 = brt_params.obst_pos_2(1);
    params.obsY2 = brt_params.obst_pos_2(2);
    params.obswidth2 = brt_params.obst_l;
    params.obsheight2 = brt_params.obst_l;

    % grid size
    params.grid_size = brt_params.grid_size;

    % disturbance params 
    params.dMax = 0.05;

    % bonus: disturbance range
    params.dist_min = 0.1;
    params.dist_max = 0.8;

    % mpc params
    params.H = 10; % receeding control horizon
    
    % set controller choice: to be set by user
    params.controller_choice = 2; % choice 0 -> nominal, 1 -> least restrictive, 2 -> qp
    
    % test case: to be set by user
    params.test_choice = 1;
    
    
    % preload the optDist file for test case 4,5
    switch params.test_choice
        case 4
             [params.precom_optDist_g, params.precom_optDist_values] = ...
                 get_optDst_precom("optimal_disturbance/optDst_0_1.mat");
        case 5
             [params.precom_optDist_g, params.precom_optDist_values] = ...
                 get_optDst_precom("optimal_disturbance/optDst_0_8.mat");
    end    
end