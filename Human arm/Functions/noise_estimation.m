function [Nu_mi, R_mi] = noise_estimation(number_simulations)

% Initialize ROS
rosinit;

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
global count;
global old_estimate_pos;
global old_estimate_vel;

vel = rossubscriber("/qualisys/Super_marker_1/odom", @Callback, 'DataFormat', 'struct');
pause(0.5);

% Offset for the covariances figure
offset_x = [0.2 0 0 0];
offset_y = [0 -0.5 0 0];

Nu_mi = zeros(3, 3);
R_mi = zeros(3, 3);
i = 1;
while i <= number_simulations

    % Markers' POSITION
   figures.fig_1 = figure(1);
   set(figures.fig_1, 'position', [10, 10, 1500, 500]);
   sgtitle('Marker Position');

   subp.(sprintf('x%d', i))= subplot(3,5,i);
   subp.(sprintf('y%d', i))= subplot(3,5,i+5);
   subp.(sprintf('z%d', i))= subplot(3,5,i+10);
   title(subp.(sprintf('x%d', i)), '$x$', 'Interpreter', 'latex');
   title(subp.(sprintf('y%d', i)), '$y$', 'Interpreter', 'latex');
   title(subp.(sprintf('z%d', i)), '$z$', 'Interpreter', 'latex');
   plotp.(sprintf('x%d', i)) = animatedline(subp.(sprintf('x%d', i)), 'Color', 'r', 'LineWidth', 1);
   plotp.(sprintf('y%d', i)) = animatedline(subp.(sprintf('y%d', i)), 'Color', 'b', 'LineWidth', 1);
   plotp.(sprintf('z%d', i)) = animatedline(subp.(sprintf('z%d', i)), 'Color', 'g', 'LineWidth', 1);

   % Markers' VELOCITY
   figures.fig_2 = figure(2);
   set(figures.fig_2, 'position', [10, 10, 1500, 500]);
   sgtitle('Marker Velocity');

   subv.(sprintf('x%d', i))= subplot(3,5,i);
   subv.(sprintf('y%d', i))= subplot(3,5,i+5);
   subv.(sprintf('z%d', i))= subplot(3,5,i+10);
   title(subv.(sprintf('x%d', i)), '$\dot{x}$', 'Interpreter', 'latex');
   title(subv.(sprintf('y%d', i)), '$\dot{y}$', 'Interpreter', 'latex');
   title(subv.(sprintf('z%d', i)), '$\dot{z}$', 'Interpreter', 'latex');
   plotv.(sprintf('x%d', i)) = animatedline(subv.(sprintf('x%d', i)), 'Color', 'r', 'LineWidth', 1);
   plotv.(sprintf('y%d', i)) = animatedline(subv.(sprintf('y%d', i)), 'Color', 'b', 'LineWidth', 1);
   plotv.(sprintf('z%d', i)) = animatedline(subv.(sprintf('z%d', i)), 'Color', 'g', 'LineWidth', 1);

   figures.fig_5 = figure(5);
   set(figures.fig_5, 'position', [10, 10, 1500, 500]);
   position_top_left = [];

   if (i == 1)

       current_position = [0, 0.5, 0.2, 0.5];
       position_top_left = current_position;
            
   elseif (i == 6)
       
       current_position = position_top_left + offset_y;

   else 
        
       current_position = current_position + offset_x;
   end
    

   ax = axes('Parent', figures.fig_5, ...
        'Position', current_position, ...
        'Box', 'on');
   text(0.35, 0.95, sprintf('\\bf Sim n. %d', i));
   text(0.15, 0.85, 'Covariance of POSITION noise');
   text(0.15, 0.35, 'Covariance of VELOCITY noise');
   set(ax, 'XTick', [], 'YTick', []);

    % Continue only if C1 is met
    while ~start
    
        pause(0.00001);
    end
    
    % Get Data
    stop = false;
    
    % Stop simulation when meas_pos(1) < 0
    n_samples = 0;
    while ~stop
    
        if (meas_pos(1) < 0)
    
            stop = true;
        end
    
        pause(0.1);
    
        addpoints(plotp.(sprintf('x%d', i)), current_time, meas_pos(1));
        addpoints(plotp.(sprintf('y%d', i)), current_time, meas_pos(2));
        addpoints(plotp.(sprintf('z%d', i)), current_time, meas_pos(3)); 
    
        addpoints(plotv.(sprintf('x%d', i)), current_time, meas_vel(1));
        addpoints(plotv.(sprintf('y%d', i)), current_time, meas_vel(2));
        addpoints(plotv.(sprintf('z%d', i)), current_time, meas_vel(3));
        
        if (n_samples == 0)

            t1 = text(0.25, 0.65, sprintf('%d %d %d\n', old_estimate_pos));
            t2 = text(0.25, 0.2, sprintf('%d %d %d\n', old_estimate_vel));

        else

            delete(t1);
            t1 = text(0.25, 0.65, sprintf('%d %d %d\n', old_estimate_pos));
           
            delete(t2);
            t2 = text(0.25, 0.2, sprintf('%d %d %d\n', old_estimate_vel));
            
        end
    
        n_samples = n_samples + 1;
    end

    % Noise covariances related to the position and velocity of the marker
    Nu_mi = Nu_mi + (old_estimate_vel - Nu_mi) / i;
    R_mi = R_mi + (old_estimate_pos - R_mi) / i;

    % Initialization
    start = false;
    count = 0;
    current_time = 0;
    old_estimate_pos = zeros(3, 3);
    old_estimate_vel = zeros(3, 3);
    i = i + 1;
    position_top_left = [];

end

% Shutdown ROS
rosshutdown;
end