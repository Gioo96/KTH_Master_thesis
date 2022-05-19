function Callback_velocity(~, message)

% count = 0 --> Simulation starts if the condition is reached
persistent count;

% flag = true if simulation is done
persistent flag;

% Simulation output
global m1_vel;

% Saving marker sample mean, sample variance
global marker;

% Initialization
if isempty(count)

    count = 0;
    flag = false;

end

% Extract data from the measurement when the marker is moving
% Set threshold to 0.4
meas = [message.Twist.Twist.Linear.X message.Twist.Twist.Linear.Y message.Twist.Twist.Linear.Z];

% Condition satisfied at max once (C1)
if (meas(3) > 1 && count == 0)

    flag = true;
end

% Condition always satisfied from condition C1 onward
if flag

    count = count + 1;
end

% Simulation
if (count == 1) 


    m1_vel = sim("master_thesis_simulink.slx");

    % Sample mean
    [marker.m1.vel.sample_mean, marker.m1.vel.delay] = super_marker_mean(m1_vel.super_m1_vel);
    % Sample variance
    marker.m1.vel.sample_variance = super_marker_variance(m1_vel.super_m1_vel, marker.m1.vel.sample_mean, marker.m1.vel.delay);
end

end