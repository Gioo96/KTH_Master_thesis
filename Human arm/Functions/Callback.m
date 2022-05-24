function Callback(~, message)

%% Defining GLOBAL variables
% Count every time Callback is called since C1 is met
global count;

% Measurement position and velocity
global meas_pos;
global meas_vel;

% Current time of simulation
global current_time;

% True when exp starts
global start;

% Simulation output
global noise_estimation;

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
meas_pos = [message.Pose.Pose.Position.X message.Pose.Pose.Position.Y message.Pose.Pose.Position.Z];
meas_vel = [message.Twist.Twist.Linear.X message.Twist.Twist.Linear.Y message.Twist.Twist.Linear.Z];

% C1 --> Condition satisfied at max once
if (meas_pos(1) < 0 && count == 0)

    start = true;
    count = 1;
end

% Condition always satisfied from condition C1 onward
if start

    % Position Covariance 
    covariance_pos_matrix = reshape(message.Pose.Covariance, sqrt(length(message.Pose.Covariance)), sqrt(length(message.Pose.Covariance)));
    covariance_pos_current = covariance_pos_matrix(1:3, 1:3);

    % Velocity Covariance
    covariance_vel_matrix = reshape(message.Twist.Covariance, sqrt(length(message.Twist.Covariance)), sqrt(length(message.Twist.Covariance)));
    covariance_vel_current = covariance_vel_matrix(1:3, 1:3);

    pos = old_estimate_pos + (covariance_pos_current - old_estimate_pos) / count;
    old_estimate_pos = pos;
    vel = old_estimate_vel + (covariance_vel_current - old_estimate_vel) / count;
    old_estimate_vel = vel;

    count = count + 1;
    current_time = current_time + 0.01;
end

% Simulation
% if (count == 1) 
% 
%     noise_estimation = sim("master_thesis_simulink.slx");
%    
%     %% Position
%     % Sample mean
%     [marker.m1.pos.sample_mean, marker.m1.pos.delay] = super_marker_mean(noise_estimation.super_m1_pos);
%     % Sample variance
%     marker.m1.pos.sample_variance = super_marker_variance(noise_estimation.super_m1_pos, marker.m1.pos.sample_mean, marker.m1.pos.delay);
% 
%     %% Velocity
%     % Sample mean
%     [marker.m1.vel.sample_mean, marker.m1.vel.delay] = super_marker_mean(noise_estimation.super_m1_vel);
%     % Sample variance
%     marker.m1.vel.sample_variance = super_marker_variance(noise_estimation.super_m1_vel, marker.m1.vel.sample_mean, marker.m1.vel.delay);
% end

end