function [Q_best, R_best] = Q_tuning(directory_list, markers_num_list, sim_num_list, Q_list, R_list)

 %% Run Human arm aparameters
run('human_ar_p.m');

global ekf noise;
global p_0p v_0p;
global markers_shoulder markers_forearm markers_hand;

%% Comment/Uncomment blocks

% Uncomment System and Show Result block
set_param('master_thesis_simulink/System', 'commented', 'off');
set_param('master_thesis_simulink/System/Show Results', 'commented', 'off');
set_param('master_thesis_simulink/System/LS', 'commented', 'on');
set_param('master_thesis_simulink/System/LKF', 'commented', 'on');
set_param('master_thesis_simulink/System/EKF', 'commented', 'on');
set_param('master_thesis_simulink/System/EKF1', 'commented', 'on');
set_param('master_thesis_simulink/System/Human arm', 'commented', 'on');
set_param('master_thesis_simulink/System/Ref', 'commented', 'on');
set_param('master_thesis_simulink/System/Noisy p', 'commented', 'on');
set_param('master_thesis_simulink/System/Noisy pdot', 'commented', 'on');
set_param('master_thesis_simulink/System/Show Results/LS', 'commented', 'on');

% Comment Ros2Matlab and Experiments block
set_param('master_thesis_simulink/Ros2Matlab', 'commented', 'on ');
set_param('master_thesis_simulink/Experiments', 'commented', 'on ');

%% Loop foreach directory
Q_best = zeros(size(directory_list, 1), 1);
R_best = zeros(size(directory_list, 1), 1);
for dir = 1:size(directory_list, 1)

    % Directory
    directory = directory_list(dir, :);

    % Set mex
    set_Mex(directory);

    % Number of markers
    markers_shoulder_num = str2double(directory(2));
    markers_forearm_num = str2double(directory(5));
    markers_hand_num = str2double(directory(8));
    fprintf('Set of markers: SHOULDER: %d, FOREARM: %d, HAND: %d\n', markers_shoulder_num, markers_forearm_num, markers_hand_num);

    % Load noise
    noise = load(strcat('Simulations/', directory, '/noise.mat'));
    noise = noise.noise;
    
    % Initialization
    Q_size = size(Q_list, 1);
    R_size = size(R_list, 1);
 
    %% Markers
    % Load corresponding set of markers
    markers_num = markers_num_list(dir);
    markers_shoulder = load(strcat('Simulations/', directory, '/Markers_Pos_', num2str(markers_num), '/markers_shoulder.mat'));
    markers_shoulder = markers_shoulder.markers_shoulder;
    markers_forearm = load(strcat('Simulations/', directory, '/Markers_Pos_', num2str(markers_num), '/markers_forearm.mat'));
    markers_forearm = markers_forearm.markers_forearm;
    markers_hand = load(strcat('Simulations/', directory, '/Markers_Pos_', num2str(markers_num), '/markers_hand.mat'));
    markers_hand = markers_hand.markers_hand;
    
    % Number of markers
    m = size(markers_shoulder, 2) + size(markers_forearm, 2) + size(markers_hand, 2);

    % Load corresponding measurements
    sim_num = sim_num_list(dir);
    p_0p = load(strcat('Simulations/', directory, '/Markers_Pos_', num2str(markers_num), '/Sim_', num2str(sim_num), '/p_0p.mat'));
    p_0p = p_0p.p_0p;
    v_0p = load(strcat('Simulations/', directory, '/Markers_Pos_', num2str(markers_num), '/Sim_', num2str(sim_num), '/v_0p.mat'));
    v_0p = v_0p.v_0p;

    RMSE_dir = zeros(Q_size, R_size);
    %% Loop foreach Q
    for q = 1:Q_size
    
        %% Loop foreach R
        for r = 1:R_size
    
            % Comment/Uncomment/Uncomment KF-EKF-LS
            set_param('master_thesis_simulink/System/Show Results/KF', 'commented', 'on');
            set_param('master_thesis_simulink/System/Show Results/EKF', 'commented', 'off');
    
            % Noise R
            ekf.R = noise.Rp*R_list(r);
            ekf.Q = Q_list(q)*eye(n);
    
            %% Run Simulation 
            disp(['Q: ', num2str(q), ', R: ', num2str(r)]);
    
            try
                set_param('master_thesis_simulink', 'StopTime', 'p_0p.time(end)');
                output = sim('master_thesis_simulink.slx');
    
                % PLOT RMSE_t
                % ||e(1)||, ||e(2)||, ..., ||e(N)||
                norm_error_EKF = vecnorm(output.error_FK__EKF.signals.values);
    
                % ||e(1)||^2, ||e(2)||^2, ..., ||e(N)||^2
                norm2_error_EKF = norm_error_EKF.^2;
    
                % 1/(3m)*||e(1)||^2, 1/(3m)*||e(2)||^2, ..., 1/(3*m)*||e(N)||^2
                mean_norm2_error_EKF = norm2_error_EKF./(3*m);
    
                % sqrt(1/(3m)*||e(1)||^2),  sqrt(1/(3m)*||e(2)||^2), ...,  sqrt(1/(3m)*||e(N)||^2)
                RMSE_t = sqrt(mean_norm2_error_EKF);
    
                % DATA RMSE
                RMSE_dir(q, r) = sqrt(1/(3*m) * mean(norm2_error_EKF));
    
            catch 
    
                % DATA RMSE
                RMSE_dir(q, r) = 1000;
    
            end
    
        end

    end

    %% Find best (Q, R)
    disp(['RMSE of DIR: ', directory]);
    disp(RMSE_dir);
    [RMSE_dir_min, index_min] = min(RMSE_dir(:));
    [row, col] = ind2sub(size(RMSE_dir), index_min);
    Q = Q_list(row);
    R = R_list(col);
    Q_best(dir) = Q;
    R_best(dir) = R;

end

end