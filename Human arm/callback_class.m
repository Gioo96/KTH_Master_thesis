classdef callback_class < handle


    properties (Access = public)
   
        % Topic
        topic string;
        marker_number;
        meas_pos;
        meas_vel;
        count;
        current_time;
        old_estimate_pos;
        old_estimate_vel;
        covariance_pos_current;
        covariance_vel_current;
        start;
        rospub;
    end

    methods 
        function obj = callback_class(marker_number)
            obj.topic = strcat('/qualisys/Super_marker_', num2str(marker_number), '/odom');
            obj.marker_number = marker_number;
            obj.meas_pos = zeros(1, 3);
            obj.meas_vel = zeros(1, 3);
            obj.count = 0;
            obj.current_time = 0;
            obj.old_estimate_pos = zeros(3, 3);
            obj.old_estimate_vel = zeros(3, 3);
            obj.start = false;

        end


        function aa = ros_subscribe(obj)

            obj.rospub = rossubscriber(obj.topic, @obj.callback, 'DataFormat', 'struct');
            aa = obj.rospub;
        end

    end

    %% Static methods
    methods (Static)

        function obj = access_properties(marker)

            persistent obj_callback;
            if isempty(obj_callback)

                obj_callback = callback_class(marker);               
            end
            obj = obj_callback;
        end

        function callback(~, msg)

            % # marker = {1,2,3,..}
            persistent marker;
            if isempty(marker)

                marker = msg.ChildFrameId(14);
            end

            % Access properties
            obj = callback_class.access_properties(marker);

            obj.meas_pos = [msg.Pose.Pose.Position.X msg.Pose.Pose.Position.Y msg.Pose.Pose.Position.Z];
            obj.meas_vel = [msg.Twist.Twist.Linear.X msg.Twist.Twist.Linear.Y msg.Twist.Twist.Linear.Z];

            % C1 --> Condition satisfied at max once           
            if (obj.meas_pos(1) > 0 && obj.count == 0)

                obj.start = true;
            end
            
            % Condition always satisfied from condition C1 onward
            if obj.start
            
                % Position Covariance 
                covariance_pos_matrix = reshape(msg.Pose.Covariance, sqrt(length(msg.Pose.Covariance)), sqrt(length(msg.Pose.Covariance)));
                obj.covariance_pos_current = covariance_pos_matrix(1:3, 1:3);
            
                % Velocity Covariance
                covariance_vel_matrix = reshape(msg.Twist.Covariance, sqrt(length(msg.Twist.Covariance)), sqrt(length(msg.Twist.Covariance)));
                obj.covariance_vel_current = covariance_vel_matrix(1:3, 1:3);
            
                obj.old_estimate_pos = obj.old_estimate_pos + (obj.covariance_pos_current - obj.old_estimate_pos) / (obj.count + 1);
                obj.old_estimate_vel = obj.old_estimate_vel + (obj.covariance_vel_current - obj.old_estimate_vel) / (obj.count + 1);
            
                obj.count = obj.count + 1;
                obj.current_time = obj.current_time + 0.01;
            end
        end

        function simulation(number_simulations)

            % Access properties
            obj = callback_class.access_properties(1);
            offset_x = [0.2 0 0 0];
            offset_y = [0 -0.5 0 0];

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
                while ~obj.start
                
                    pause(0.00001);
                end
                
                % Get Data
                stop = false;
                
                % Stop simulation when meas_pos(1) < 0
                n_samples = 0;
                %time_shifted = current_time

                while ~stop
            
                    if (obj.meas_pos(1) < 0)
                
                        stop = true;
                    end
                
                    pause(0.1);
                
                    addpoints(plotp.(sprintf('x%d', i)), obj.current_time, obj.meas_pos(1));
                    addpoints(plotp.(sprintf('y%d', i)), obj.current_time, obj.meas_pos(2));
                    addpoints(plotp.(sprintf('z%d', i)), obj.current_time, obj.meas_pos(3)); 

                    addpoints(plotv.(sprintf('x%d', i)), obj.current_time, obj.meas_vel(1));
                    addpoints(plotv.(sprintf('y%d', i)), obj.current_time, obj.meas_vel(2));
                    addpoints(plotv.(sprintf('z%d', i)), obj.current_time, obj.meas_vel(3));
                    
                    if (n_samples == 0)
            
                        text(0.25, 0.65, sprintf('%d %d %d\n', obj.old_estimate_pos));
                        text(0.25, 0.2, sprintf('%d %d %d\n', obj.old_estimate_vel));
            
                    else
            
                        t1 = text(0.25, 0.65, sprintf('%d %d %d\n', obj.old_estimate_pos));
                        delete(t1);
                        t2 = text(0.25, 0.2, sprintf('%d %d %d\n', obj.old_estimate_vel));
                        delete(t2);
                    end
                
                    n_samples = n_samples + 1;
                end
            
                % Initialization
                obj.start = false;
                obj.count = 0;
                obj.current_time = 0;
                obj.old_estimate_pos = zeros(3, 3);
                obj.old_estimate_vel = zeros(3, 3);
                i = i + 1;
                position_top_left = [];
            
            end
        end
    end
end