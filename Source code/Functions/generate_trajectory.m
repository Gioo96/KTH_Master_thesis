function q_trajectory = generate_trajectory(q_ic, num_points, is_lkf)

%% Description

% generate_reference generate the trajectory of sample time = 0.002 (sum of sinusoids of
% different amplitude and frequency) such that the overall signal goes
% through q_paints

% Inputs:
% -- q_ic             : initial condition
% -- num_points       : number of points foreach qi
% -- is_lkf           : true --> genertate trajectory for lkf
%                       false --> generate regular trajectory

% Outputs:
% -- q_trajectory     : computed trajectory

%% Function

% Upper saturation     : upper-limit of the joint variables
upper_saturation = [pi/2; pi/10; 2/3*pi; pi/2; pi/4; pi/10; pi/4];
% Lower saturation     : lower-limit of the joint variables
lower_saturation = [-pi/2; -pi/10; -pi/6; 0; -pi/2;  -pi/2; -pi/4];

% Sample Time
Ts = 0.002;

% Stop time
StopTime = 10;

% time
t = 0:StopTime/(num_points-1):StopTime;
time = linspace(0, StopTime, StopTime/Ts);
time

% Generate q_points
q_points = zeros(7, num_points);
q_points(:, 1) = q_ic;
q_trajectory = [time' zeros(length(time), 7)];

switch is_lkf

    % LKF trajectory
    case true

        for i = 1 : 7
        
            % Check if max and min of joint_i are within [lower_saturation(i), upper_saturation(i)]
            is_jointi_ok = false;

            while (~is_jointi_ok)

                % First set of points
                for j = 2 : floor(num_points/2)+2
                   
                    % jth point of the ith joint
                    q_ij = unifrnd(q_ic(i)-deg2rad(5), q_ic(i)+deg2rad(5), 1);
                    q_points(i, j) = q_ij;
                end
            
                % Second set of points
                for j = floor(num_points/2)+3 : num_points
                   
                    % jth point of the ith joint
                    q_ij = unifrnd(0.9*lower_saturation(i), 0.9*upper_saturation(i), 1);
                    q_points(i, j) = q_ij;
                end

                % Interpolation
                cs = spline(t,[0 q_points(i, :) 0]);
                q = ppval(cs, time);

                if (max(q) < upper_saturation(i) && min(q) > lower_saturation(i))

                    is_jointi_ok = true;
                end

            end
            
        
            figure();
            plot(time, rad2deg(q), '-', 'LineWidth', 1.5);
            hold on;
            plot(t, rad2deg(q_points(i, :)), 'o');
            hold on;
            plot(time, rad2deg(upper_saturation(i)*ones(size(time, 2), 1)), 'LineWidth', 1.5);
            hold on;
            plot(time, rad2deg(lower_saturation(i)*ones(size(time, 2), 1)), 'LineWidth', 1.5);
            xlabel('Time [s]');
            ylabel('Joint variable [°]');
            if (upper_saturation(i) == 0)
            
                ylim([rad2deg(1.1*lower_saturation(i)), 10]);

            elseif (lower_saturation(i) == 0)

                ylim([-10, rad2deg(1.1*upper_saturation(i))]);
              
            else

                ylim([rad2deg(1.1*lower_saturation(i)), rad2deg(1.1*upper_saturation(i))]);
            end
            legend_name{1} = strcat('$\eta_', num2str(i), '$');
            legend_name{2} = strcat('markers');
            legend_name{3} = strcat('$\eta_', num2str(i), '^{upper}$');
            legend_name{4} = strcat('$\eta_', num2str(i), '^{lower}$');
            legend(legend_name, 'Interpreter', 'latex');
            grid on;
        
            % Stack q in the trakectory
            q_trajectory(:, i+1) = q;
        end

    % REGULAR case
    case false

        for i = 1 : 7

            % Check if max and min of joint_i are within [lower_saturation(i), upper_saturation(i)]
            is_jointi_ok = false;
    
            while (~is_jointi_ok)
    
                for j = 2 : num_points
                   
                    % jth point of the ith joint
                    q_ij = unifrnd(0.8*lower_saturation(i), 0.8*upper_saturation(i), 1);
                    q_points(i, j) = q_ij;
                end
            
                % Interpolation
                cs = spline(t,[0 q_points(i, :) 0]);
                q = ppval(cs, time);
    
                if (max(q) < upper_saturation(i) && min(q) > lower_saturation(i))
    
                    is_jointi_ok = true;
                end
    
            end
        
            figure();
            plot(time, rad2deg(q), '-', 'LineWidth', 1.5);
            hold on;
            plot(t, rad2deg(q_points(i, :)), 'o');
            hold on;
            plot(time, rad2deg(upper_saturation(i)*ones(size(time, 2), 1)), 'LineWidth', 1.5);
            hold on;
            plot(time, rad2deg(lower_saturation(i)*ones(size(time, 2), 1)), 'LineWidth', 1.5);
            xlabel('Time [s]');
            ylabel('Joint variable [°]');
            if (upper_saturation(i) == 0)
            
                ylim([rad2deg(1.1*lower_saturation(i)), 10]);

            elseif (lower_saturation(i) == 0)

                ylim([-10, rad2deg(1.1*upper_saturation(i))]);
              
            else

                ylim([rad2deg(1.1*lower_saturation(i)), rad2deg(1.1*upper_saturation(i))]);
            end
            legend_name{1} = strcat('$\eta_', num2str(i), '$');
            legend_name{2} = strcat('markers');
            legend_name{3} = strcat('$\eta_', num2str(i), '^{upper}$');
            legend_name{4} = strcat('$\eta_', num2str(i), '^{lower}$');
            legend(legend_name, 'Interpreter', 'latex');
            grid on;
        
            % Stack q in the trakectory
            q_trajectory(:, i+1) = q;
        end
end

end