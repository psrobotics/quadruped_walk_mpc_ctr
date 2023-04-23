function [safety_controller, worst_dist, data, tau, g, derivatives, grid_info] = BRT_computation(params)

addpath(genpath('..\tool_box\helperOC'))  
addpath(genpath('..\tool_box\ToolboxLS'))

% unpack params
obsX1 = params.obsX1;
obsY1 = params.obsY1;
obsX2 = params.obsX2;
obsY2 = params.obsY2;
speed = params.speed;
wMax = params.wMax;
dMax = params.dMax;
width1 = params.obswidth1;
height1 = params.obsheight1;
width2 = params.obswidth2;
height2 = params.obsheight2;

%% TODO
% Define the grid for the computation: g
% g =...
% 3d grid
grid_min = [0; 0; -pi]; % Lower corner of computation domain
grid_max = [params.grid_size(1); params.grid_size(2); pi];    % Upper corner of computation domain
N = [40; 40; 20];         % Number of grid points per dimension %180 180 80
g = createGrid(grid_min, grid_max, N);

grid_info.min = grid_min;
grid_info.max = grid_max;
grid_info.N = N;

%% TODO
% Define the failure set: data0
g_tmp_min = grid_min(1:2);
g_tmp_max = grid_max(1:2);
N_tmp = [N(1); N(2)];
g_tmp = createGrid(g_tmp_min, g_tmp_max, N_tmp);

%ToolboxLS\Kernel\InitialConditions\BasicShapes
data_obs_1 = shapeRectangleByCorners(g_tmp, [obsX1; obsY1], [obsX1+width1; obsY1+height1]);
data_obs_2 = shapeRectangleByCorners(g_tmp, [obsX2; obsY2], [obsX2+width2; obsY2+height2]);

data0 = zeros(N(1),N(2),N(3));
for i=1:N(3)
    %ToolboxLS\Kernel\InitialConditions\SetOperations, Get combined shape
    data0(:,:,i) = shapeUnion(data_obs_1,data_obs_2); 
end

% time
t0 = 0;
tMax = 2;
dt = 0.05;
tau = t0:dt:tMax;

% control trying to min or max value function?
uMode = 'max';
dMode = 'min';

% Define dynamic system
dCar = DubinsCar([0, 0, 0], wMax, speed, dMax);

% Put grid and dynamic systems into schemeData
schemeData.grid = g;
schemeData.dynSys = dCar;
schemeData.accuracy = 'high'; %set accuracy
schemeData.uMode = uMode;
schemeData.dMode = dMode;
% HJIextraArgs.obstacles = obs;

%% Compute value function
% HJIextraArgs.visualize = true; %show plot
HJIextraArgs.visualize.valueSet = 1;
HJIextraArgs.visualize.initialValueSet = 1;
HJIextraArgs.visualize.figNum = 1; %set figure number
HJIextraArgs.visualize.deleteLastPlot = false; %delete previous plot as you update

% uncomment if you want to see a 2D slice
xlim([0, 0.7]);
ylim([0, 0.35]);
axis equal;
HJIextraArgs.visualize.plotData.plotDims = [1 1 0]; %plot x, y
HJIextraArgs.visualize.plotData.projpt = [pi/2]; %project at theta = 0
HJIextraArgs.visualize.viewAngle = [0,90]; % view 2D
%axis equal;
%[data, tau, extraOuts] = ...
% HJIPDE_solve(data0, tau, schemeData, minWith, extraArgs)
[data, tau2, ~] = ...
  HJIPDE_solve(data0, tau, schemeData, 'zero', HJIextraArgs);

derivatives = computeGradients(g, data(:,:,:,end));
safety_controller =  dCar.optCtrl([], [], derivatives, 'max');
worst_dist =  dCar.optDstb([], [], derivatives, 'min');
tau = tau2;

