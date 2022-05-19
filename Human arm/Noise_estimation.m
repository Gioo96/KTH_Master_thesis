% Human arm parameters
run('human_arm_param.m');

% Comment System
set_param('master_thesis_simulink/System', 'commented', 'on');

%% Noise (TO BE ESTIMATED)
% Sampling time
sample_Time = 0.01;
% Simulation of 10s
simulink.time = 10;

% MARKER M1 POSITION

% Uncomment Pose block
set_param('master_thesis_simulink/Ros2Matlab/Pose', 'commented', 'off');
% Comment Twist block
set_param('master_thesis_simulink/Ros2Matlab/Twist', 'commented', 'on');
% Simulation
m1_pos = sim("master_thesis_simulink.slx");
% Sample mean
[marker.m1.pos.sample_mean, marker.m1.pos.delay] = super_marker_mean(m1_pos.super_m1_pos);
% Sample variance
marker.m1.pos.sample_variance = super_marker_variance(m1_pos.super_m1_pos, marker.m1.pos.sample_mean, marker.m1.pos.delay);

% MARKER M1 VELOCITY

% Uncomment Twist block
set_param('master_thesis_simulink/Ros2Matlab/Twist', 'commented', 'off');
% Comment Pose block
set_param('master_thesis_simulink/Ros2Matlab/Pose', 'commented', 'on');
global meas;
global m1_vel;
sub = rossubscriber("/qualisys/Super_marker_1/odom", @Callback, 'DataFormat', 'struct');
% % Sample mean
% [marker.m1.pos.sample_mean, marker.m1.pos.delay] = super_marker_mean(output_est);
% % Sample variance
% marker.m1.pos.sample_variance = super_marker_variance(output_est, marker.m1.pos.sample_mean, marker.m1.pos.delay);