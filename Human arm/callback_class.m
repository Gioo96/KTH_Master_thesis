classdef callback_class


    properties
   
        message;
        meas_pos;
    end

    methods
        function obj = callback_class(topic, m_pos)
        
            if (strcmp('/qualisys/Super_marker_1/odom', topic) == 1)
                obj.message = rostopic("echo", "/qualisys/Super_marker_1/odom").MessageType;
                obj.meas_pos = m_pos;
            
            elseif (strcmp('/qualisys/Super_marker_2/odom', topic) == 1)
                
                obj.message = rostopic("echo", "/qualisys/Super_marker_2/odom").MessageType;

            end
        end

        function callback(~, msg)
        

            %% Defining GLOBAL variables
            % Count every time Callback is called since C1 is met
            disp('aaaaaaaa')
            global count;
            
            % Measurement position and velocity
            global meas_pos;
            global meas_vel;
            
            % Current time of simulation
            global current_time;
            
            % True when exp starts
            global start;
            
            % % Simulation output
            % global noise_estimation;
            
            global old_estimate_pos;
            global old_estimate_vel;
            
            % Covariance of the marker position noise
            global covariance_pos_current;
            
            
            % Covariance of the marker velocity noise
            global covariance_vel_current;
            
            
            % Initialization
            if isempty(count)
            
                current_time = 0;
                count = 0;
                start = false;
                old_estimate_pos = zeros(3, 3);
                old_estimate_vel = zeros(3, 3);
            end
            
            % Extract data from the measurement
            % Set threshold to 0.4
            
            meas_pos = [msg.Pose.Pose.Position.X msg.Pose.Pose.Position.Y msg.Pose.Pose.Position.Z];
            meas_vel = [msg.Twist.Twist.Linear.X msg.Twist.Twist.Linear.Y msg.Twist.Twist.Linear.Z];
            
            % C1 --> Condition satisfied at max once
            if (obj.meas_pos(1) > 0 && count == 0)
            
                start = true;
            end
            
            % Condition always satisfied from condition C1 onward
            if start
            
                % Position Covariance 
                covariance_pos_matrix = reshape(msg.Pose.Covariance, sqrt(length(msg.Pose.Covariance)), sqrt(length(msg.Pose.Covariance)));
                covariance_pos_current = covariance_pos_matrix(1:3, 1:3);
            
                % Velocity Covariance
                covariance_vel_matrix = reshape(msg.Twist.Covariance, sqrt(length(msg.Twist.Covariance)), sqrt(length(msg.Twist.Covariance)));
                covariance_vel_current = covariance_vel_matrix(1:3, 1:3);
            
                old_estimate_pos = old_estimate_pos + (covariance_pos_current - old_estimate_pos) / (count + 1);
                old_estimate_vel = old_estimate_vel + (covariance_vel_current - old_estimate_vel) / (count + 1);
            
                count = count + 1;
                current_time = current_time + 0.01;
            end
        end
        function ros_subscribe(obj)

            %disp(obj.message);
            rossubscriber('/qualisys/Super_marker_1/odom', @obj.callback, 'DataFormat', 'struct');
        end

        function dispp(a)

            disp(a);
        end

        function call()

            
        end
    end
end