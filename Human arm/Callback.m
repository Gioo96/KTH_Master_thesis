function Callback_velocity(~, message)

% % Current measurement
% global meas;
% 
% % Simulink output
% global m1_vel;

% count = 0 --> Simulation starts if the condition is reached
persistent count;
if isempty(count)

    count = 0;
    clear global;
    global meas;
    global m1_vel;

end
% Extract data from the measurement when the marker is moving
% Set threshold to 0.1
meas = [message.Twist.Twist.Linear.X message.Twist.Twist.Linear.Y message.Twist.Twist.Linear.Z];
if (norm(meas' - zeros(3,1)) > 0.2 && count == 0)

    m1_vel = sim("master_thesis_simulink.slx");
    count = 1;
end

end