% Clear variables, figures
clear;
clear global;
close all;
clc;

% Initialize ROS
rosinit;

% Open Simulink
open_system('master_thesis_simulink.slx');

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
global covariance_pos_current;
global covariance_vel_current;
global old_estimate_pos;
global old_estimate_vel;

vel = rossubscriber("/qualisys/Super_marker_1/odom", @Callback, 'DataFormat', 'struct');
pause(1);

% % Set marker position figure
% figures.fig_1 = figure(1);
% set(figures.fig_1, 'position', [10, 10, 1500, 500]);
% sgtitle('Marker Position');
% subp_x = subplot(1,3,1);
% title(subp_x, '$x$', 'Interpreter', 'latex');
% plotp_x = animatedline(subp_x, 'Color', 'r', 'LineWidth', 1);
% subp_y = subplot(1,3,2);
% title(subp_y, '$y$', 'Interpreter', 'latex');
% plotp_y = animatedline(subp_y, 'Color', 'b', 'LineWidth', 1);
% subp_z = subplot(1,3,3);
% title(subp_z, '$z$', 'Interpreter', 'latex');
% plotp_z = animatedline(subp_z, 'Color', 'g', 'LineWidth', 1);
% 
% % Set marker velocity figure
% figures.fig_2 = figure(2);
% set(figures.fig_2, 'position', [10, 10, 1500, 500]);
% sgtitle('Marker Velocity');
% subv_x = subplot(1,3,1);
% title(subv_x, '$\dot{x}$', 'Interpreter', 'latex');
% plotv_x = animatedline(subv_x, 'Color', 'r', 'LineWidth', 1);
% subv_y = subplot(1,3,2);
% title(subv_y, '$\dot{y}$', 'Interpreter', 'latex')
% plotv_y = animatedline(subv_y, 'Color', 'b', 'LineWidth', 1);
% subv_z = subplot(1,3,3);
% title(subv_z, '$\dot{z}$', 'Interpreter', 'latex');
% plotv_z = animatedline(subv_z, 'Color', 'g', 'LineWidth', 1);
% 
% % Set noise position covariance figure
% figures.fig_3 = figure(3);
% set(figures.fig_3, 'position', [10, 10, 1500, 500]);
% sgtitle('Marker POSITION noise Covariance');
% subcp_x = subplot(1,3,1);
% title(subcp_x, '$Var[v_x]$', 'Interpreter', 'latex');
% plotcp_x = animatedline(subcp_x, 'Color', 'r', 'LineWidth', 1);
% subcp_y = subplot(1,3,2);
% title(subcp_y, '$Var[v_y]$', 'Interpreter', 'latex')
% plotcp_y = animatedline(subcp_y, 'Color', 'b', 'LineWidth', 1);
% subcp_z = subplot(1,3,3);
% title(subcp_z, '$Var[v_z]$', 'Interpreter', 'latex');
% plotcp_z = animatedline(subcp_z, 'Color', 'g', 'LineWidth', 1);
% 
% % Set noise velocity covariance figure
% figures.fig_4 = figure(4);
% set(figures.fig_4, 'position', [10, 10, 1500, 500]);
% sgtitle('Marker VELOCITY noise Covariance');
% subcv_x = subplot(1,3,1);
% title(subcv_x, '$Var[w_{u_x}]$', 'Interpreter', 'latex');
% plotcv_x = animatedline(subcv_x, 'Color', 'r', 'LineWidth', 1);
% subcv_y = subplot(1,3,2);
% title(subcv_y, '$Var[w_{u_y}]$', 'Interpreter', 'latex')
% plotcv_y = animatedline(subcv_y, 'Color', 'b', 'LineWidth', 1);
% subcv_z = subplot(1,3,3);
% title(subcv_z, '$Var[w_{u_z}]$', 'Interpreter', 'latex');
% plotcv_z = animatedline(subcv_z, 'Color', 'g', 'LineWidth', 1);

% figures.fig_5 = figure(5);
% set(figures.fig_5, 'position', [10, 10, 1000, 200]);
% sub_sim1 = subplot(1,2,1);
% title(sub_sim1, '$Var[w_{u_x}]$', 'Interpreter', 'latex');
% sub_sim2 = subplot(1,2,2);
% title(sub_sim2, '$Var[w_{u_x}]$', 'Interpreter', 'latex');

% fig_print{1} = 'Covariance of POSITION noise:  ';
% fig_print{3} = 'Covariance of VELOCITY noise:  ';

% i = 1;
% while i <= 2

    % Continue only if C1 is met
    while ~start
    
        pause(0.00001);
    end
    
    % Get Data
    stop = false;
    count = 1;
    %fig_set1 = uicontrol('Parent', figures.fig_5, 'Style', 'text', 'max', 2, 'Units', 'norm', 'Position', [0 0 1 1]);
    %fig_set2 = uicontrol('Parent', sub_sim2, 'Style', 'text', 'max', 2, 'Units', 'norm', 'Position', [0 0 1 1]);
%     while ~stop
%     
%         if (current_time > 1)
%     
%             stop = true;
%         end
%     
%         pause(0.1);
%     
%         addpoints(plotp_x, current_time, meas_pos(1));
%         addpoints(plotp_y, current_time, meas_pos(2));
%         addpoints(plotp_z, current_time, meas_pos(3)); 
%     
%         addpoints(plotv_x, current_time, meas_vel(1));
%         addpoints(plotv_y, current_time, meas_vel(2));
%         addpoints(plotv_z, current_time, meas_vel(3));
%         
%         addpoints(plotcp_x, current_time, covariance_pos_current(1,1));
%         addpoints(plotcp_y, current_time, covariance_pos_current(2,2));
%         addpoints(plotcp_z, current_time, covariance_pos_current(3,3));
%     
%         addpoints(plotcv_x, current_time, covariance_vel_current(1,1));
%         addpoints(plotcv_y, current_time, covariance_vel_current(2,2));
%         addpoints(plotcv_z, current_time, covariance_vel_current(3,3));
%     
%         fig_print{2} = sprintf('%d %d %d\n', old_estimate_pos);
%         fig_print{4} = sprintf('%d %d %d\n', old_estimate_vel);
% %         set(fig_set1, 'String', fig_print);
% %         set(fig_set2, 'String', fig_print);
%         %fprintf('Covariance of POSITION noise /n'); 
%         %fprintf('%d %d %d\n', old_estimate);
%     
%         count = count + 1;
%     end


   fignum = figure('Units', 'normal', 'Position', [0.1 0.1 .8 .8]);  %not quite full screen  
   subgroup1 = uipanel('Parent', fignum, 'Units', 'normal', 'Position', [0 2/3 1 1/3])  %top third
   subgroup1_plotbox = uipanel('Parent', subgroup1, 'Units', 'normal', 'Position', [0 .1 1 .9])  %plot in top 9/10 of the group
   subgroup1_controls = uipanel('Parent', subgroup1, 'Units', 'normal', 'Position', [0 0 1 .1]); %control area in bottom 1/10 of the group
   subgroup1_axes = axes('Parent', subgroup1_plotbox);
   
  fig_print{1} = 'Covariance of POSITION noise:  ';
  fig_print{3} = 'Covariance of VELOCITY noise:  ';
  fig_print{2} = sprintf('%d %d %d\n', old_estimate_pos);
  fig_print{4} = sprintf('%d %d %d\n', old_estimate_vel);
   subgroup1_slider = uicontrol('style', 'text', 'Parent', sugroup1_controls);
   set(subgroup1_slider, fig_print)
    
% figures.fig_5 = figure(5);
% figures.fig_6 = figure(6);
% set(figures.fig_5, 'position', [10, 10, 200, 200]);
% set(figures.fig_6, 'position', [10, 10, 200, 200]);
% hslide = uicontrol('Parent', figures.fig_5, 'style', 'text');
% hslide1 = uicontrol('Parent', figures.fig_6, 'style', 'text');
% slideoff = [0.7 0 0 0];
% for ind = 1:2
%       %plot(old_estimate_pos);
%             fig_print{1} = 'Covariance of POSITION noise:  ';
%       fig_print{3} = 'Covariance of VELOCITY noise:  ';
%       fig_print{2} = sprintf('%d %d %d\n', old_estimate_pos);
%       fig_print{4} = sprintf('%d %d %d\n', old_estimate_vel);
%       if (ind == 1)
% 
%           subpos = [-0.4 0 1 1];
%           set(hslide, 'String', fig_print);
%       else
% 
%           subpos = subpos + slideoff;
%           set(hslide1, 'String', fig_print);
%       end
%       set(hslide, 'units', 'norm', 'position', subpos);
% %       slideoff = [0 0 1];
%      % slidepos = get(hslide(ind), 'position');
% %       slidepos = [0.20 0.9 0 0]
% 
% end


%     i = i + 1;
% end

% Shutdown ROS
rosshutdown;