clear all; close all; clc;

%% Add path
addpath(genpath('..\tool_box\helperOC'))  
addpath(genpath('..\tool_box\ToolboxLS'))

%% Get the params
params = get_params();

%% TODO: Reachability analysis: Please complete the template in the BRT_computation.m file for this section to get the params
disp('Pre-computing the safety controller with the BRT.........................')
[params.safety_controller, params.worst_dist, params.data, params.tau, params.g, params.derivatives, params.grid_info] = BRT_computation(params); % the BRT gives us the safety controller for free