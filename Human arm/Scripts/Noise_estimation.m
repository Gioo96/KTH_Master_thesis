% Clear variables, figures
clear;
clear global;
close all;
clc;

% Initialize ROS
rosinit;

%% Noise (TO BE ESTIMATED)

% Sampling time
sample_Time = 0.01;

% Simulation of 10s
simulink.time = 10;

m = callback_class(0);
ros_sub = m.ros_subscribe();
%m.simulation(1);

% Shutdown ROS
rosshutdown;