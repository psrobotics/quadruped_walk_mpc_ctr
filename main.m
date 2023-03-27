%% Add casadi path

clc; clear;
close all; warning off;

addpath('D:\matlab_lib\casadi-windows-matlabR2016a-v3.5.5');
import casadi.*;

addpath('math\');
addpath('srb_dynamics');

%% Get hardware params

%% Get dynamics