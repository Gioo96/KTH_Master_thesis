function [sample_mean, delay] = super_marker_mean(out)

% super_marker_mean computes the sample mean given a set of measurements

% Input
% -- out           : Simulation output
 
% Output
% -- sample_mean   : Sample mean

% Measurments can be affected by delay
delay = 0; % No delay
for i = 1 : length(out.super_marker_1.time)

    if (out.super_marker_1.signals.values(delay+1, :) == zeros(1, 3))
    
        delay = delay + 1; % Delay
    
    else

        break;
    end
end

% Number of samples
n = length(out.super_marker_1.time) - delay;

% Sample mean computation
sum = zeros(3, 1);
for i = delay+1 : length(out.super_marker_1.time)

    sum = sum + out.super_marker_1.signals.values(i, :)';
end
sample_mean = sum / n;

end