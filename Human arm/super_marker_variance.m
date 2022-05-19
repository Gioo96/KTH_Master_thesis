function sample_variance = super_marker_variance(struct, sample_mean, delay)

% super_marker_var computes the sample variance given a set of measurements

% Input
% -- out           : Simulation output
% -- sample_mean   : Sample mean
% -- delay         : Delay of the measurements --> delay is the # steps I have to wait until measurement != 0
 
% Output
% -- sample_variance   : Sample variance

% Number of samples
n = length(struct.time) - delay;

% Sample variance computation --> Unbiased estimate
sum = zeros(3, 3);
for i = delay+1 : length(struct.time)

    sum = sum + (struct.signals.values(i, :)' - sample_mean) * (struct.signals.values(i, :)' - sample_mean)';
end
sample_variance = sum / (n - 1);

end