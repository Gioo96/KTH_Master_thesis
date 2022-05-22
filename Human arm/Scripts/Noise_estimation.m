% Clear variables, figures
clear;
clear global;
close all;
clc;

% Initialize ROS
rosinit;

% Comment System
set_param('master_thesis_simulink/System', 'commented', 'on');

% Uncomment Ros2Matlab
set_param('master_thesis_simulink/Ros2Matlab', 'commented', 'off');

%% Noise (TO BE ESTIMATED)

% Sampling time
sample_Time = 0.01;

% Simulation of 10s
simulink.time = 10;

% Global variables
global meas_pos;
global meas_vel;
global current_time;
global start;

vel = rossubscriber("/qualisys/Super_marker_1/odom", @Callback, 'DataFormat', 'struct');
pause(1);

figure.fig_1 = figure(1);
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

figure.fig_2 = figure(2);
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


% Continue only if C1 is met
while ~start

    pause(0.00001);
end

% Get data for 7s
stop = false;
while ~stop

    if (current_time > 7)

        stop = true;
    end

    pause(0.1);
    
    addpoints(plotp_x, current_time, meas_pos(1));
    addpoints(plotp_y, current_time, meas_pos(2));
    addpoints(plotp_z, current_time, meas_pos(3));

    addpoints(plotv_x, current_time, meas_vel(1));
    addpoints(plotv_y, current_time, meas_vel(2));
    addpoints(plotv_z, current_time, meas_vel(3));
    
end

% Shutdown ROS
rosshutdown;