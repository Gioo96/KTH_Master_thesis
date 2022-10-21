function LS(initial_estimate, N_pdot, scenario_num, C_code_folder)

%% Description
% LS computes the Least squares estimate of q(t); it compares the true
% quantity and the estimated one; it also provides the corrresponding RMSE.

% Inputs
% -- initial_estimate    : Initial estimate of q(0)
% -- N_pdot              : 2 situations can occur: 
%                          -- pdot = J*qdot --> covariance_n_pdot = 0 (DETERMINISTIC pdot)
%                          -- pdot = J*qdot+n_pdot, n_pdot is N(0, covariance_n_pdot) --> covariance_n_pdot != 0 (NOISY pdot)                         
% -- Scenario_num        : Scenario number : 1, 2, 3 or 4
% -- C_code_folder       : Folder containing the mex functions needed, depending on the location of the markers

%% Function

% Number of DoF
n = 7;

% Number of markers
m = str2num(C_code_folder(2)) + str2num(C_code_folder(5))  + str2num(C_code_folder(8));

% Global variables
global q0_LS; % Initial estimate
global noise; % N_pdot

% Add path 'C code/C_code_folder' <-- target_folder
set_Mex(C_code_folder);


%% Comment blocks and set parameters 

% Uncomment System
set_param('master_thesis_simulink/System', 'commented', 'off');
% Comment Ros2Matlab, Experiments
set_param('master_thesis_simulink/Ros2Matlab', 'commented', 'on');
set_param('master_thesis_simulink/Experiments', 'commented', 'on');

% Uncomment LS block 
set_param('master_thesis_simulink/System/LS', 'commented', 'off');
% Comment EKF block
set_param('master_thesis_simulink/System/EKF', 'commented', 'on');
% Comment LKF block
set_param('master_thesis_simulink/System/LKF', 'commented', 'on');
% Comment Noisy p block
set_param('master_thesis_simulink/System/Noisy p', 'commented', 'on');

% Comment ZOH blocks
set_param('master_thesis_simulink/System/ZOH', 'commented', 'on');
set_param('master_thesis_simulink/System/ZOH1', 'commented', 'on');

% No not set sample time for q
set_param('master_thesis_simulink/System/ZOH_q', 'SampleTime', '-1');

%% Least Squares IC
q0_LS = initial_estimate;
set_param('master_thesis_simulink/System/LS/Integrator', 'InitialCondition', 'q0_LS');

%% Noise covariance
% Deterministic pdot
if (isequal(N_pdot, zeros(m*3, m*3)))

    % Comment Noisy pdot block
    set_param('master_thesis_simulink/System/Noisy pdot', 'commented', 'on');

    % pdot_true
    set_param('master_thesis_simulink/System/pdot', 'GotoTag', 'pdot_true');

% Noisy pdot
else

    % Uncomment Noisy pdot block
    set_param('master_thesis_simulink/System/Noisy pdot', 'commented', 'off');
    set_param('master_thesis_simulink/System/Noisy pdot/n_pdot', 'Variance', 'diag(noise.N_pdot)');
    set_param('master_thesis_simulink/System/Noisy pdot/n_pdot', 'SampleTime', '-1');
    % Assign N_pdot to noise
    noise.N_pdot = N_pdot;

    % pdot
    set_param('master_thesis_simulink/System/pdot', 'GotoTag', 'pdot');
end

%% Simulation

output = sim('master_thesis_simulink.slx');

%% RMSE & Results

% q - q_LS of size:LXn
difference = output.q.signals.values(:, :)-output.q_LS_MinimumNorm.signals.values(:, :)';

% norm(q-q_LS) of size L
norm_difference = zeros(length(output.q.time), 1);
for i = 1 : length(output.q.time)

    norm_difference(i) = norm(difference(i, :));
end

% Mean Squared Error
MSE = mean(norm_difference);

% Root Mean Squared Error
RMSE = sqrt(MSE);

% Plot results
fig = figure();
for i = 1 : n

    legend_name{1} = strcat('$\eta_', num2str(i), '$');
    legend_name{2} = strcat('$\hat{\eta_', num2str(i), '}^{LS}$');
    set(fig, 'position', [10, 10, 1300, 900]);
    subplot(3,3,i);
    plot(output.q.time, rad2deg(output.q.signals.values(:, i)));
    hold on;
    plot(output.q_LS_MinimumNorm.time, rad2deg(squeeze(output.q_LS_MinimumNorm.signals.values(i, :))));
    xlabel('Time [s]');
    ylabel('Joint variable [°]');
    legend(legend_name, 'Interpreter', 'latex', 'FontSize', 18);
    grid on;
end

% Disp RMSE
disp(['The RMSE of scenario ',num2str(scenario_num), ' is ', num2str(RMSE)]);

%% Save EPS figures
switch scenario_num

    case 1

        for i = 1 : n
        
            fig = figure('Visible', 'off');
            legend_name{1} = strcat('$\eta_', num2str(i), '$');
            legend_name{2} = strcat('$\hat{\eta_', num2str(i), '}^{LS}$');
            plot(output.q.time, rad2deg(output.q.signals.values(:, i)), 'LineWidth', 1.5);
            hold on;
            plot(output.q.time, rad2deg(squeeze(output.q_LS_MinimumNorm.signals.values(i, :))'), 'LineWidth', 1.5);
            xlabel('Time [s]');
            ylabel('Joint variable [°]');
            legend(legend_name, 'Location', 'NorthOutside', 'Orientation', 'horizontal', 'Box', 'off', 'Interpreter', 'latex', 'FontSize', 26);
            set(gca, 'FontSize', 18);
            ylim('padded');
            grid on;
            eps_name = strcat('Scenario1_eta', num2str(i));
            saveas(gcf, eps_name, 'epsc');
        end

    case 2

        for i = 1 : n
        
            fig = figure('Visible', 'off');
            legend_name{1} = strcat('$\eta_', num2str(i), '$');
            legend_name{2} = strcat('$\hat{\eta_', num2str(i), '}^{LS}$');
            plot(output.q.time, rad2deg(output.q.signals.values(:, i)), 'LineWidth', 1.5);
            hold on;
            plot(output.q.time, rad2deg(squeeze(output.q_LS_MinimumNorm.signals.values(i, :))'), 'LineWidth', 1.5);
            xlabel('Time [s]');
            ylabel('Joint variable [°]');
            legend(legend_name, 'Location', 'NorthOutside', 'Orientation', 'horizontal', 'Box', 'off', 'Interpreter', 'latex', 'FontSize', 26);
            set(gca, 'FontSize', 18);
            ylim('padded');
            grid on;
            eps_name = strcat('Scenario2_eta', num2str(i));
            saveas(gcf, eps_name, 'epsc');
        end

    case 3

        for i = 1 : n
        
            fig = figure('Visible', 'off');
            legend_name{1} = strcat('$\eta_', num2str(i), '$');
            legend_name{2} = strcat('$\hat{\eta_', num2str(i), '}^{LS}$');
            plot(output.q.time, rad2deg(output.q.signals.values(:, i)), 'LineWidth', 1.5);
            hold on;
            plot(output.q.time, rad2deg(squeeze(output.q_LS_MinimumNorm.signals.values(i, :))'), 'LineWidth', 1.5);
            xlabel('Time [s]');
            ylabel('Joint variable [°]');
            legend(legend_name, 'Location', 'NorthOutside', 'Orientation', 'horizontal', 'Box', 'off', 'Interpreter', 'latex', 'FontSize', 26);
            set(gca, 'FontSize', 18);
            ylim('padded');
            grid on;
            eps_name = strcat('Scenario3_eta', num2str(i));
            saveas(gcf, eps_name, 'epsc');
        end

    case 4

        for i = 1 : n
        
            fig = figure('Visible', 'off');
            legend_name{1} = strcat('$\eta_', num2str(i), '$');
            legend_name{2} = strcat('$\hat{\eta_', num2str(i), '}^{LS}$');
            plot(output.q.time, rad2deg(output.q.signals.values(:, i)), 'LineWidth', 1.5);
            hold on;
            plot(output.q.time, rad2deg(squeeze(output.q_LS_MinimumNorm.signals.values(i, :))'), 'LineWidth', 1.5);
            xlabel('Time [s]');
            ylabel('Joint variable [°]');
            legend(legend_name, 'Location', 'NorthOutside', 'Orientation', 'horizontal', 'Box', 'off', 'Interpreter', 'latex', 'FontSize', 26);
            set(gca, 'FontSize', 18);
            ylim('padded');
            grid on;
            eps_name = strcat('Scenario4_eta', num2str(i));
            saveas(gcf, eps_name, 'epsc');
        end

    case 5

        for i = 1 : n
        
            fig = figure('Visible', 'off');
            legend_name{1} = strcat('$\eta_', num2str(i), '$');
            legend_name{2} = strcat('$\hat{\eta_', num2str(i), '}^{LS}$');
            plot(output.q.time, rad2deg(output.q.signals.values(:, i)), 'LineWidth', 1.5);
            hold on;
            plot(output.q.time, rad2deg(squeeze(output.q_LS_MinimumNorm.signals.values(i, :))'), 'LineWidth', 1.5);
            xlabel('Time [s]');
            ylabel('Joint variable [°]');
            legend(legend_name, 'Location', 'NorthOutside', 'Orientation', 'horizontal', 'Box', 'off', 'Interpreter', 'latex', 'FontSize', 26);
            set(gca, 'FontSize', 18);
            ylim('padded');
            grid on;
            eps_name = strcat('Scenario5_eta', num2str(i));
            saveas(gcf, eps_name, 'epsc');
        end

end

% Remove blocks
global markers_shoulder markers_forearm markers_hand;
remove_blocks_simulink(markers_shoulder, markers_forearm, markers_hand);

end