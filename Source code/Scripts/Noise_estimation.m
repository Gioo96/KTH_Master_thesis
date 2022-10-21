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

marker = 0;
m = callback_class(marker);
%ros_sub = m.ros_subscribe();
m.simulation(marker, 1, 0);

% Shutdown ROS
rosshutdown;