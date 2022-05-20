% Clear variables, figures
clear;
clear global;
close all;
clc;
rosinit;

% Human arm parameters
run('human_arm_param.m');

% Comment System
set_param('master_thesis_simulink/System', 'commented', 'on');

%% Noise (TO BE ESTIMATED)
% Sampling time
sample_Time = 0.01;
% Simulation of 10s
simulink.time = 10;

global est;
global marker;

global meas_pos;
global meas_vel;
global current_time;
global flag;

vel = rossubscriber("/qualisys/Super_marker_1/odom", @Callback_velocity, 'DataFormat', 'struct');
pause(1);

fig_103 = figure(103);
set(fig_103, 'position', [10, 10, 1500, 500]);
sgtitle('Marker Position');
subp_x = subplot(1,3,1);
title(subp_x, '$x$', 'Interpreter', 'latex');
plotp_x = animatedline(subp_x, 'Color', 'r', 'LineWidth', 1);
subp_y = subplot(1,3,2);
title(subp_y, '$y$', 'Interpreter', 'latex');
plotp_y = animatedline(subp_y, 'Color', 'b', 'LineWidth', 1);
subp_z = subplot(1,3,3);
title(subp_z, '$z$', 'Interpreter', 'latex');
plotp_z = animatedline(subp_z, 'Color', 'g', 'LineWidth', 1);

fig_101 = figure(101);
set(fig_101, 'position', [10, 10, 1500, 500]);
sgtitle('Marker Velocity');
subv_x = subplot(1,3,1);
title(subv_x, '$\dot{x}$', 'Interpreter', 'latex');
plotv_x = animatedline(subv_x, 'Color', 'r', 'LineWidth', 1);
subv_y = subplot(1,3,2);
title(subv_y, '$\dot{y}$', 'Interpreter', 'latex')
plotv_y = animatedline(subv_y, 'Color', 'b', 'LineWidth', 1);
subv_z = subplot(1,3,3);
title(subv_z, '$\dot{z}$', 'Interpreter', 'latex');
plotv_z = animatedline(subv_z, 'Color', 'g', 'LineWidth', 1);


% Continue only if the conditions in the Callback C1 are met
while ~flag

    pause(0.00001);
end

real_time = false;
% Get data for 7s
while ~real_time

    if (current_time > 7)

        real_time = true;
    end

    pause(0.1);
    
    addpoints(plotp_x, current_time, meas_pos(1));
    addpoints(plotp_y, current_time, meas_pos(2));
    addpoints(plotp_z, current_time, meas_pos(3));

    addpoints(plotv_x, current_time, meas_vel(1));
    addpoints(plotv_y, current_time, meas_vel(2));
    addpoints(plotv_z, current_time, meas_vel(3));
    
end

% Ros shutdown
rosshutdown