%% Run Human arm aparameters
run('human_arm_parameters_exp.m')

%% Simulink time
simulink.time = 20;

%% Comment / Uncomment blocks in Simulink
% Uncomment Ros2Matlab, Experiments
set_param(strcat('master_thesis_simulink/Ros2Matlab/', directory), 'commented', 'off');
set_param(strcat('master_thesis_simulink/Experiments/', directory), 'commented', 'off');
set_param(strcat('master_thesis_simulink/Experiments/', directory, '/Experiments'), 'commented', 'on');
set_param(strcat('master_thesis_simulink/Experiments/', directory, '/Model validation'), 'commented', 'on');
set_param(strcat('master_thesis_simulink/Experiments/', directory, '/Noise estimation'), 'commented', 'off');
% Comment System
set_param('master_thesis_simulink/System', 'commented', 'on');

%% Simulate Model
out = sim('master_thesis_simulink.slx');

%% Get Rpi_0 and Rvi_0 estimates and check if all covariances are Symmetric Positive Definite

is_covariance_p = true;
is_covariance_v = true;
noise.Rp = zeros(3*m, 3*m);
noise.Rv = zeros(3*m, 3*m);
count_areCovariances_p = 0;
count_areCovariances_v = 0;
for i = 1 : m

    % Get Rpi_0 and Rvi_0 estimates
    Rpi_0_name = strcat('Rp', num2str(i), '_0');
    Rpi_0 = out.(Rpi_0_name).signals.values(:, :, end);
    Rvi_0_name = strcat('Rv', num2str(i), '_0');
    Rvi_0 = out.(Rvi_0_name).signals.values(:, :, end);

    % Check if all covariances (POSITION of markers) are Symmetric Positive Definite
    is_symmetric_p = issymmetric(Rpi_0);
    if ~is_symmetric_p

        is_covariance_p = false;
        fprintf('Matrix %s is NOT Symmetric', Rpi_0_name);

    % Rpi_0 is symmetric
    else

        % Eigenvalues
        eigs = eig(Rpi_0);
        is_PosDef_p = all(eigs > 0);
        if ~is_PosDef_p

            is_covariance_p = false;
            fprintf('Matrix %s is NOT positive definite', Rpi_0_name);
        end
    end

    if is_covariance_p

        count_areCovariances_p = count_areCovariances_p + 1;
        noise.Rp(3*(i-1)+1 : 3*(i-1)+3, 3*(i-1)+1 : 3*(i-1)+3) = Rpi_0;
    end

    is_covariance_p = true;

    % Check if all covariances (VELOCITY of markers) are Symmetric Positive Definite
    is_symmetric_v = issymmetric(Rvi_0);
    if ~is_symmetric_v

        is_covariance_v = false;
        fprintf('Matrix %s is NOT Symmetric', Rvi_0_name);

    % Rvi_0 is symmetric
    else

        % Eigenvalues
        eigs = eig(Rvi_0);
        is_PosDef_v = all(eigs > 0);
        if ~is_PosDef_v

            is_covariance_v = false;
            fprintf('Matrix %s is NOT positive definite', Rvi_0_name);
        end
    end

    if is_covariance_v

        count_areCovariances_v = count_areCovariances_v + 1;
        noise.Rv(3*(i-1)+1 : 3*(i-1)+3, 3*(i-1)+1 : 3*(i-1)+3) = Rvi_0;
    end

    is_covariance_v = true;

    % Clear variables
    clear Rpi_0_name Rvi_0_name;
    clear Rpi_0 Rvi_0;
end

if (count_areCovariances_p ~= m)

    fprintf('Not possible to compute Measurement Covariance %s ', 'Rp');
end

if (count_areCovariances_v ~= m)

    fprintf('Not possible to compute Measurement Covariance %s ', 'Rv');
end

% Clear variables
clear eigs is_symmetric_p is_symmetric_v is_PosDef_p is_PosDef_v is_covariance_p is_covariance_v;

%% Measuremnet Noise
noise.R = noise.Rp;

%% Process noise
noise.Q = zeros(n, n);

%% Save data
save(strcat('noise_', directory, '.mat'), 'noise');