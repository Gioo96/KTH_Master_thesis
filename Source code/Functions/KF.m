function KF(Delta_q0, P0, q_eq, pdot_eq, Q, R, N_pdot, C_code_folder)

%% Description
% KF computes the Kalman Filter estimate of q(t); it compares the true
% quantity and the estimated one; it also provides the corrresponding RMSE.

% Inputs
% -- Delta_q0           : initial estimate of Delta_q
% -- P0                 : initial estimate of the covariance
% -- q_eq               : equilibrium point
% -- pdot_eq            : equilibrium point
% -- Q                  : process covariance
% -- R                  : measurement covariance                       
% -- N_pdot             : pdot covariance
% -- C_code_folder      : Folder containing the mex functions needed, depending on the location of the markers

%% Function

% Number of DoF
n = 7;

% Number of markers
m = str2num(C_code_folder(2)) + str2num(C_code_folder(5))  + str2num(C_code_folder(8));

% Global variables
global kf; 
global noise; 
global markers_shoulder markers_forearm markers_hand;

% Add path 'C code/C_code_folder' <-- target_folder
set_Mex(C_code_folder);

%% Comment blocks and set parameters 

% Uncomment System
set_param('master_thesis_simulink/System', 'commented', 'off');
% Comment Ros2Matlab, Experiments
set_param('master_thesis_simulink/Ros2Matlab', 'commented', 'on');
set_param('master_thesis_simulink/Experiments', 'commented', 'on');

% Uncomment LKF block
set_param('master_thesis_simulink/System/LKF', 'commented', 'off');
% Comment LS block 
set_param('master_thesis_simulink/System/LS', 'commented', 'on');
% Comment EKF block
set_param('master_thesis_simulink/System/EKF', 'commented', 'on');

% Uncomment ZOH blocks
set_param('master_thesis_simulink/System/ZOH', 'commented', 'off');
set_param('master_thesis_simulink/System/ZOH1', 'commented', 'off');

%% Equilibrium point
kf.q_eq = q_eq;
kf.pdot_eq = pdot_eq;

%% N_pdot
noise.N_pdot = N_pdot;

%% Initial condition

kf.Delta_q0 = Delta_q0;

%% Set noise covariances

% Uncomment Noisy pdot block
set_param('master_thesis_simulink/System/Noisy pdot', 'commented', 'off');
set_param('master_thesis_simulink/System/Noisy pdot/n_pdot', 'Variance', 'diag(noise.N_pdot)');
set_param('master_thesis_simulink/System/Noisy pdot/n_pdot', 'SampleTime', 'sample_Time');
% Uncomment Noisy p block
set_param('master_thesis_simulink/System/Noisy p', 'commented', 'off');
set_param('master_thesis_simulink/System/Noisy p/v', 'Variance', 'diag(noise.R)');
set_param('master_thesis_simulink/System/Noisy p/v', 'SampleTime', 'sample_Time');
% Set sample time for q
set_param('master_thesis_simulink/System/ZOH_q', 'SampleTime', 'sample_Time');

% pdot
set_param('master_thesis_simulink/System/pdot', 'GotoTag', 'pdot');
% p
set_param('master_thesis_simulink/System/p', 'GotoTag', 'p');

%% Matrices computation

% F matrix --> df/dq evaluated at the equilibrium point
kf.F = full(f_Fekf_mex(kf.q_eq, markers_shoulder, markers_forearm, markers_hand, kf.pdot_eq));
% G matrix --> df/du evaluated at the equilibrium point
kf.G = full(f_Gekf_mex(kf.q_eq, markers_shoulder, markers_forearm, markers_hand, kf.pdot_eq));

% J matrix --> dPhi/dq (Jacobian J) evaluated at the equilibrium point
kf.J = full(f_J_mex(kf.q_eq, markers_shoulder, markers_forearm, markers_hand));
% I matrix --> dPhi/du = 0 
kf.I = zeros(3*m, 3*m);

% p_eq
kf.p_eq = full(f_Phi_mex(kf.q_eq, markers_shoulder, markers_forearm, markers_hand));

%% Determine scenario

scenario_num = -1;
% Scenario 1
if (size(Q, 1) > n)

    scenario_num = 1;
    % Qi
    for i = 1 : size(Q, 1)/n

        % Process noise
        Qi = Q(7*(i-1)+1 : 7*(i-1)+n, :);
        noise.Q = Qi;

        % Measurement noise
        noise.R = R;

        % Initial covariance
        kf.P0 = P0;

        % Check DETECTABILITY of (F, J) and STABILIZABILITY of (F, Q)
        [is_stabilizable, is_detectable] = is_stabilizable_detectable(kf.F, noise.Q, kf.J);

        if (is_stabilizable && is_detectable)

            %% Compute Pk, Pk_, Kk offline
        
            % List of Pk for k = 1, .., N
            kf.Pk = [kf.P0];
            % List of Pk_ for k = 1, .., N
            kf.Pk_ = [];
            % List of Kk for k = 1, .., N
            kf.Kk = [];
            
            % Compute Pk, Pk_, Kk offline
            N_samples = 10 / 0.01;
            for k = 1 : N_samples
            
                Pk_ = kf.F * kf.Pk(1:n, n*(k-1)+1:n*(k-1)+n) * kf.F' + noise.Q;
                Kk = Pk_ * kf.J' / (kf.J * Pk_ * kf.J' + noise.R);
                Pk = (eye(n) - Kk * kf.J) * Pk_ * (eye(size(kf.F, 1)) - Kk * kf.J)' + Kk * noise.R * Kk';
                kf.Pk_ = [kf.Pk_, Pk_];
                kf.Kk = [kf.Kk, Kk];
                kf.Pk = [kf.Pk, Pk];
            end
        
            %% Simulation
            output = sim('master_thesis_simulink.slx');

            q = output.q;
            qi = output.q_KF;
            if (i == 1)

                q1 = qi;

            elseif (i == 2)

                q2 = qi;

            end

            [Pbar, K, L] = idare(kf.F', kf.J', noise.Q, noise.R, [], []);
            disp('P_{k|k-1} tends to');
            disp(Pbar)

            % q - q_LS of size:LXn
            difference = q.signals.values(:, :)-qi.signals.values(:, :)';
            
            % norm(q-q_LS) of size L
            norm_difference = zeros(length(q.time), 1);
            for j = 1 : length(q.time)
            
                norm_difference(j) = norm(difference(j, :));
            end
            
            % Mean Squared Error
            MSE = mean(norm_difference);
            
            % Root Mean Squared Error
            RMSE = sqrt(MSE);
            disp('RMSE is');
            disp(RMSE);
           
        end

    end

% Scenario 2
elseif(size(R, 1) > 3*m)

    scenario_num = 2;
    % Ri
    for i = 1 : size(R, 1)/(3*m)

        % Process noise
        noise.Q = Q;

        % Measurement noise
        Ri = R(3*m*(i-1)+1 : 3*m*(i-1)+3*m, :);
        noise.R = Ri;

        % Initial covariance
        kf.P0 = P0;

        % Check DETECTABILITY of (F, J) and STABILIZABILITY of (F, Q)
        [is_stabilizable, is_detectable] = is_stabilizable_detectable(kf.F, noise.Q, kf.J);

        if (is_stabilizable && is_detectable)

            %% Compute Pk, Pk_, Kk offline
        
            % List of Pk for k = 1, .., N
            kf.Pk = [kf.P0];
            % List of Pk_ for k = 1, .., N
            kf.Pk_ = [];
            % List of Kk for k = 1, .., N
            kf.Kk = [];
            
            % Compute Pk, Pk_, Kk offline
            N_samples = 10 / 0.01;
            for k = 1 : N_samples
            
                Pk_ = kf.F * kf.Pk(1:n, n*(k-1)+1:n*(k-1)+n) * kf.F' + noise.Q;
                Kk = Pk_ * kf.J' / (kf.J * Pk_ * kf.J' + noise.R);
                Pk = (eye(n) - Kk * kf.J) * Pk_ * (eye(size(kf.F, 1)) - Kk * kf.J)' + Kk * noise.R * Kk';
                kf.Pk_ = [kf.Pk_, Pk_];
                kf.Kk = [kf.Kk, Kk];
                kf.Pk = [kf.Pk, Pk];
            end
        
            %% Simulation
            output = sim('master_thesis_simulink.slx');

            q = output.q;
            qi = output.q_KF;
            if (i == 1)

                q1 = qi;

            elseif (i == 2)

                q2 = qi;

            end

            [Pbar, K, L] = idare(kf.F', kf.J', noise.Q, noise.R, [], []);
            disp('P_{k|k-1} tends to');
            disp(Pbar)

            % q - q_LS of size:LXn
            difference = q.signals.values(:, :)-qi.signals.values(:, :)';
            
            % norm(q-q_LS) of size L
            norm_difference = zeros(length(q.time), 1);
            for j = 1 : length(q.time)
            
                norm_difference(j) = norm(difference(j, :));
            end
            
            % Mean Squared Error
            MSE = mean(norm_difference);
            
            % Root Mean Squared Error
            RMSE = sqrt(MSE);
            disp('RMSE is');
            disp(RMSE);
           
        end

    end
    
% Scenario 3
else

    scenario_num = 3;

    % Process noise
    noise.Q = Q;

    % Measurement noise
    noise.R = R;

    % Initial covariance
    kf.P0 = P0;

    % Check DETECTABILITY of (F, J) and STABILIZABILITY of (F, Q)
    [is_stabilizable, is_detectable] = is_stabilizable_detectable(kf.F, noise.Q, kf.J);
    if is_stabilizable
    
        disp('STABILIZABLE');
    else

        disp('NOT STABILIZABLE');
        set_param('master_thesis_simulink/System/LKF/Kalman Filter1', 'commented', 'on');
    end

    if is_detectable
        
        disp('DETECTABLE');
    else

        disp('NOT DETECTABLE');
        set_param('master_thesis_simulink/System/LKF/Kalman Filter1', 'commented', 'on');
    end
    
    %% Compute Pk, Pk_, Kk offline

    % List of Pk for k = 1, .., N
    kf.Pk = [kf.P0];
    % List of Pk_ for k = 1, .., N
    kf.Pk_ = [];
    % List of Kk for k = 1, .., N
    kf.Kk = [];
    
    % Compute Pk, Pk_, Kk offline
    N_samples = 10 / 0.01;
    for k = 1 : N_samples
    
        Pk_ = kf.F * kf.Pk(1:n, n*(k-1)+1:n*(k-1)+n) * kf.F' + noise.Q;
        Kk = Pk_ * kf.J' / (kf.J * Pk_ * kf.J' + noise.R);
        Pk = (eye(n) - Kk * kf.J) * Pk_ * (eye(size(kf.F, 1)) - Kk * kf.J)' + Kk * noise.R * Kk';
        kf.Pk_ = [kf.Pk_, Pk_];
        kf.Kk = [kf.Kk, Kk];
        kf.Pk = [kf.Pk, Pk];
    end
    
    %% Simulation
    output = sim('master_thesis_simulink.slx');
  
    q = output.q;
    q1 = output.q_KF;

    [Pbar, K, L] = idare(kf.F', kf.J', noise.Q, noise.R, [], []);
    disp('P_{k|k-1} tends to');
    disp(kf.Pk(:, size(kf.Pk, 2)-6:size(kf.Pk, 2)));
    disp('eigenvalues of F(I-KJ):');
    K = kf.Kk(:, size(kf.Kk, 2)-11:size(kf.Kk, 2));
    disp(eigs(kf.F-kf.F*K*kf.J));

    % q - q_LS of size:LXn
    difference = q.signals.values(:, :)-q1.signals.values(:, :)';
    
    % norm(q-q_LS) of size L
    norm_difference = zeros(length(q.time), 1);
    for j = 1 : length(q.time)
    
        norm_difference(j) = norm(difference(j, :));
    end
    
    % Mean Squared Error
    MSE = mean(norm_difference);
    
    % Root Mean Squared Error
    RMSE = sqrt(MSE);
    disp('RMSE is');
    disp(RMSE);

end



switch scenario_num 

    case 1

        % Plot results
        fig = figure();
        for i = 1 : n
        
            legend_name{1} = strcat('$\eta_', num2str(i), '$');
            legend_name{2} = strcat('$\hat{\eta_', num2str(i), '}^{KF}_{Q_1}$');
            legend_name{3} = strcat('$\hat{\eta_', num2str(i), '}^{KF}_{Q_2}$');
            set(fig, 'position', [10, 10, 1300, 900]);
            subplot(3,3,i);
            plot(q.time, rad2deg(q.signals.values(:, i)), 'LineWidth', 1.5);
            hold on;
            plot(q1.time, rad2deg(squeeze(q1.signals.values(i, :))'), 'LineWidth', 1.5);
            hold on;
            plot(q2.time, rad2deg(squeeze(q2.signals.values(i, :))'), 'LineStyle', '-', 'LineWidth', 0.5);
            xlabel('Time [s]');
            ylabel('Joint variable [°]');
            ylim('padded');
            legend(legend_name, 'Interpreter', 'latex', 'FontSize', 18);
            grid on;
        end

    case 2

        % Plot results
        fig = figure();
        for i = 1 : n
        
            legend_name{1} = strcat('$\eta_', num2str(i), '$');
            legend_name{2} = strcat('$\hat{\eta_', num2str(i), '}^{KF}_{R_1}$');
            legend_name{3} = strcat('$\hat{\eta_', num2str(i), '}^{KF}_{R_2}$');
            set(fig, 'position', [10, 10, 1300, 900]);
            subplot(3,3,i);
            plot(q.time, rad2deg(q.signals.values(:, i)), 'LineWidth', 1.5);
            hold on;
            plot(q1.time, rad2deg(squeeze(q1.signals.values(i, :))'), 'LineWidth', 1.5);
            hold on;
            plot(q2.time, rad2deg(squeeze(q2.signals.values(i, :))'), 'LineStyle', '-', 'LineWidth', 0.5);
            xlabel('Time [s]');
            ylabel('Joint variable [°]');
            ylim('padded');
            legend(legend_name, 'Interpreter', 'latex', 'FontSize', 18);
            grid on;
        end


    case 3
        
        % Plot results
        fig = figure();
        for i = 1 : n
        
            legend_name{1} = strcat('$\eta_', num2str(i), '$');
            legend_name{2} = strcat('$\hat{\eta_', num2str(i), '}^{KF}$');
            set(fig, 'position', [10, 10, 1300, 900]);
            subplot(3,3,i);
            plot(q.time, rad2deg(q.signals.values(:, i)), 'LineWidth', 1.5);
            hold on;
            plot(q1.time, rad2deg(squeeze(q1.signals.values(i, :))'), 'Color', [0.9290 0.6940 0.1250], 'LineWidth', 1.5);
            xlabel('Time [s]');
            ylabel('Joint variable [°]');
            ylim('padded');
            legend(legend_name, 'Interpreter', 'latex', 'FontSize', 18);
            grid on;
        end

end

%% Save EPS figures
switch scenario_num

    case 1

        for i = 1 : n
        
            fig = figure('Visible', 'off');
            legend_name{1} = strcat('$\eta_', num2str(i), '$');
            legend_name{2} = strcat('$\hat{\eta_', num2str(i), '}^{KF}_{Q_1}$');
            legend_name{3} = strcat('$\hat{\eta_', num2str(i), '}^{KF}_{Q_2}$');
            plot(q.time, rad2deg(q.signals.values(:, i)), 'LineWidth', 1.5);
            hold on;
            plot(q1.time, rad2deg(squeeze(q1.signals.values(i, :))'), 'LineWidth', 1.5);
            hold on;
            plot(q2.time, rad2deg(squeeze(q2.signals.values(i, :))'), 'LineStyle', '-', 'LineWidth', 0.5);
            xlabel('Time [s]');
            ylabel('Joint variable [°]');
            legend(legend_name, 'Location', 'NorthOutside', 'Orientation', 'horizontal', 'Box', 'off', 'Interpreter', 'latex', 'FontSize', 26);
            set(gca, 'FontSize', 18);
            grid on;
            ylim('padded');
            eps_name = strcat('Scenario1_eta', num2str(i));
            saveas(gcf, eps_name, 'epsc');
        end

    case 2

        for i = 1 : n
        
            fig = figure('Visible', 'off');
            legend_name{1} = strcat('$\eta_', num2str(i), '$');
            legend_name{2} = strcat('$\hat{\eta_', num2str(i), '}^{KF}_{R_1}$');
            legend_name{3} = strcat('$\hat{\eta_', num2str(i), '}^{KF}_{R_2}$');
            plot(q.time, rad2deg(q.signals.values(:, i)), 'LineWidth', 1.5);
            hold on;
            plot(q1.time, rad2deg(squeeze(q1.signals.values(i, :))'), 'LineWidth', 1.5);
            hold on;
            plot(q2.time, rad2deg(squeeze(q2.signals.values(i, :))'), 'LineStyle', '-', 'LineWidth', 0.5);
            xlabel('Time [s]');
            ylabel('Joint variable [°]');
            legend(legend_name, 'Location', 'NorthOutside', 'Orientation', 'horizontal', 'Box', 'off', 'Interpreter', 'latex', 'FontSize', 26);
            set(gca, 'FontSize', 18);
            grid on;
            ylim('padded');
            eps_name = strcat('Scenario2_eta', num2str(i));
            saveas(gcf, eps_name, 'epsc');
        end

    case 3

        for i = 1 : n
        
            fig = figure('Visible', 'off');
            legend_name{1} = strcat('$\eta_', num2str(i), '$');
            legend_name{2} = strcat('$\hat{\eta_', num2str(i), '}^{KF}$');
            plot(q.time, rad2deg(q.signals.values(:, i)), 'LineWidth', 1.5);
            hold on;
            plot(q1.time, rad2deg(squeeze(q1.signals.values(i, :))'), 'Color', [0.9290 0.6940 0.1250], 'LineWidth', 1.5);
            xlabel('Time [s]');
            ylabel('Joint variable [°]');
            legend(legend_name, 'Location', 'NorthOutside', 'Orientation', 'horizontal', 'Box', 'off', 'Interpreter', 'latex', 'FontSize', 26);
            set(gca, 'FontSize', 18);
            grid on;
            ylim('padded');
            eps_name = strcat('Scenario3_eta', num2str(i));
            saveas(gcf, eps_name, 'epsc');
        end

end

% Remove blocks
global markers_shoulder markers_forearm markers_hand;
remove_blocks_simulink(markers_shoulder, markers_forearm, markers_hand);
end