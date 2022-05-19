function [sample_mean, delay] = super_marker_mean(struct)

% super_marker_mean computes the sample mean given a set of measurements

% Input
% -- out           : Simulation output
 
% Output
% -- sample_mean   : Sample mean

% Measurments can be affected by delay
delay = 0; % No delay
for i = 1 : length(struct.time)

    if (struct.signals.values(delay+1, :) == zeros(1, 3))
    
        delay = delay + 1; % Delay
    
    else

        break;
    end
end

% Number of samples
n = length(struct.time) - delay;

% Sample mean computation
sum = zeros(3, 1);
for i = delay+1 : length(struct.time)

    sum = sum + struct.signals.values(i, :)';
end
sample_mean = sum / n;

end