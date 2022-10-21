function EKF(q0, P0, Qlist, Rlist, N_pdot, C_code_folder, save)

%% Description
% EKF computes the best Extended Kalman Filter estimate of q(t) among Qlist and Rlist; it compares the true
% quantity and the estimated one; it also provides the corrresponding RMSE.

% Inputs
% -- q0                 : initial estimate of q
% -- P0                 : initial estimate of the covariance
% -- Q                  : list of process covariances
% -- R                  : list of measurement covariances                    
% -- N_pdot             : pdot covariance
% -- C_code_folder      : Folder containing the mex functions needed, depending on the location of the markers

%% Function

% Number of DoF
n = 7;

% Number of markers
m = str2num(C_code_folder(2)) + str2num(C_code_folder(5))  + str2num(C_code_folder(8));

% Global variables
global ekf; 
global noise; 

% Add path 'C code/C_code_folder' <-- target_folder
set_Mex(C_code_folder);

%% Comment blocks and set parameters 

% Uncomment System
set_param('master_thesis_simulink/System', 'commented', 'off');
% Comment Ros2Matlab, Experiments
set_param('master_thesis_simulink/Ros2Matlab', 'commented', 'on');
set_param('master_thesis_simulink/Experiments', 'commented', 'on');

% Uncomment EKF block
set_param('master_thesis_simulink/System/EKF', 'commented', 'off');
% Uncomment Ref
set_param('master_thesis_simulink/System/Ref', 'commented', 'off');
% Uncomment Human arm
set_param('master_thesis_simulink/System/Human arm', 'commented', 'off');
% Comment LKF block
set_param('master_thesis_simulink/System/LKF', 'commented', 'on');
% Comment LS block 
set_param('master_thesis_simulink/System/LS', 'commented', 'on');
% Comment Show reults
set_param('master_thesis_simulink/System/Show Results', 'commented', 'on');

% Uncomment ZOH blocks
set_param('master_thesis_simulink/System/ZOH', 'commented', 'off');
set_param('master_thesis_simulink/System/ZOH1', 'commented', 'off');

% Uncomment Noisy pdot block
set_param('master_thesis_simulink/System/Noisy pdot', 'commented', 'off');
set_param('master_thesis_simulink/System/Noisy pdot/n_pdot', 'Variance', 'diag(noise.N_pdot)');
set_param('master_thesis_simulink/System/Noisy pdot/n_pdot', 'SampleTime', 'sample_Time');
% Uncomment Noisy p block
set_param('master_thesis_simulink/System/Noisy p', 'commented', 'off');
set_param('master_thesis_simulink/System/Noisy p/v', 'Variance', 'diag(noise.R)');
%set_param('master_thesis_simulink/System/Noisy p/v', 'Variance', 'diag(diag(0.000000001*ones(3*m, 1)))');
set_param('master_thesis_simulink/System/Noisy p/v', 'SampleTime', 'sample_Time');
% Set sample time for q
set_param('master_thesis_simulink/System/ZOH_q', 'SampleTime', 'sample_Time');

% pdot
set_param('master_thesis_simulink/System/pdot', 'GotoTag', 'pdot');
% p
set_param('master_thesis_simulink/System/p', 'GotoTag', 'p');

%% N_pdot

noise.N_pdot = N_pdot;

%% Initial condition

ekf.q0 = q0;
ekf.P0 = P0;

%% EKF
% Computes the RMSE for each couple (Qi, Rj) and save the best one
num = 0;
best_Q = zeros(n, n);
best_R = zeros(3*m, 3*m);
min_RMSE = 100;

for i = 1:size(Qlist, 1)/n

    for j = 1:size(Rlist, 1)/(3*m)

        num = num + 1;
        noise.Q = Qlist(n*(i-1)+1:n*(i-1)+n, :);
        noise.R = Rlist(3*m*(j-1)+1:3*m*(j-1)+3*m, :);

        %% Simulation
        set_param('master_thesis_simulink', 'StopTime', '10');
        output = sim('master_thesis_simulink.slx');

        q = output.q;
        q_ekf = output.q_EKF;

        % q - q_ekf of size:LXn
        difference = q.signals.values(:, :)-q_ekf.signals.values(:, :);
        
        % norm(q-q_LS) of size L
        norm_difference = zeros(length(q.time), 1);
        for k = 1 : length(q.time)
        
            norm_difference(k) = norm(difference(k, :));
        end
        
        % Mean Squared Error
        MSE = mean(norm_difference);
        
        % Root Mean Squared Error
        RMSE = sqrt(MSE);
        if (RMSE < min_RMSE)

            min_RMSE = RMSE;
            best_Q = noise.Q;
            best_R = noise.R;
            best_q = q;
            best_q_EKF = q_ekf;
            best_num = num;
        end
        if (num == 1)

            q1 = q_ekf;
        elseif (num == 2)

            q2 = q_ekf;
        end

        disp(['CASE ', num2str(num), ' with Q = Q', num2str(i), ' and R = R', num2str(j)]);
        disp(['RMSE = ', num2str(RMSE)]);

    end
end

% Plot results
fig = figure();
for f = 1 : n

    legend_name{1} = strcat('$\eta_', num2str(f), '$');
    legend_name{2} = strcat('$\hat{\eta_', num2str(f), '}^{EKF}_{R_1}$');
    %legend_name{3} = strcat('$\hat{\eta_', num2str(f), '}^{EKF}_{R_2}$');
    set(fig, 'position', [10, 10, 1300, 900]);
    subplot(3,3,f);
    plot(best_q.time, rad2deg(best_q.signals.values(:, f)), 'LineWidth', 1.5);
    hold on;
    plot(q1.time, rad2deg(squeeze(q1.signals.values(:, f))), 'LineWidth', 1.5);
    %hold on;
    %plot(q2.time, rad2deg(squeeze(q2.signals.values(:, f))), 'LineStyle', '--', 'LineWidth', 0.5);
    xlabel('Time [s]');
    ylabel('Joint variable [°]');
    ylim('padded');
    legend(legend_name, 'Interpreter', 'latex', 'FontSize', 18);
    grid on;
end

%% Save EPS figures

if (save)

    for i = 1 : n
    
        fig = figure('Visible', 'off');
        legend_name{1} = strcat('$\eta_', num2str(i), '$');
        legend_name{2} = strcat('$\hat{\eta_', num2str(i), '}^{EKF}_{R_1}$');
        %legend_name{3} = strcat('$\hat{\eta_', num2str(i), '}^{EKF}_{R_2}$');
        plot(q.time, rad2deg(q.signals.values(:, i)), 'LineWidth', 1.5);
        hold on;
        plot(q1.time, rad2deg(squeeze(q1.signals.values(:, i))), 'LineWidth', 1.5);
        %hold on;
        %plot(q2.time, rad2deg(squeeze(q2.signals.values(:, i))), 'LineStyle', '--', 'LineWidth', 0.5);
        xlabel('Time [s]');
        ylabel('Joint variable [°]');
        legend(legend_name, 'Location', 'NorthOutside', 'Orientation', 'horizontal', 'Box', 'off', 'Interpreter', 'latex', 'FontSize', 26);
        set(gca, 'FontSize', 18);
        grid on;
        ylim('padded');
        eps_name = strcat('Scenario2_eta', num2str(i));
        saveas(gcf, eps_name, 'epsc');
    end
end

% Remove blocks
global markers_shoulder markers_forearm markers_hand;
remove_blocks_simulink(markers_shoulder, markers_forearm, markers_hand);

end