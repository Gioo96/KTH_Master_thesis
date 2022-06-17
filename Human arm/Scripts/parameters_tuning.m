% Clear variables
clear;
close all;
clc;

% Run human_arm_parameters
run('human_arm_parameters.m');

%% Simulink

% q0 model
q0_model = zeros(7, 1);

% Comment System
set_param('master_thesis_simulink/System', 'commented', 'on');

% Uncomment Ros2Matlab
set_param('master_thesis_simulink/Ros2Matlab', 'commented', 'off');

% Uncomment Experiments
set_param('master_thesis_simulink/Experiments', 'commented', 'off');
% Uncomment Human Arm
set_param('master_thesis_simulink/Experiments/Human arm', 'commented', 'off');
% Comment LS
set_param('master_thesis_simulink/Experiments/LS', 'commented', 'on');
% Comment KF
set_param('master_thesis_simulink/Experiments/KF', 'commented', 'on');
% Comment EKF
set_param('master_thesis_simulink/Experiments/EKF', 'commented', 'on');

% Set markers in Simulink
set_markers_simulink_Experiments(shoulder_num, forearm_num, hand_num);

% Simulation of 10s
simulink.time = 10;

%% Initialize ROS

rosinit;

%% Callback --> start simulation when M0_W(1) > 0

marker = 0;
tuning = callback_class(marker);
tuning.simulation_tuningParameters(marker);

% Shutdown ROS
rosshutdown;

% Run SImulation
remove_blocks_simulink_Experiments(shoulder_num, forearm_num, hand_num);

