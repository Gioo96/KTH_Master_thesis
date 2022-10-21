function show_results(directory, markers_num_list, sim_num_list, sigma_Q2, sigma_R2, scenario_num, save)

 %% Run Human arm aparameters
run('human_ar_p.m');

global kf ekf noise;
global p_0p v_0p;
global markers_shoulder markers_forearm markers_hand;

%% Set mex
set_Mex(directory);

%% Comment/Uncomment blocks

% Uncomment System and Show Result block
set_param('master_thesis_simulink/System', 'commented', 'off');
set_param('master_thesis_simulink/System/Show Results', 'commented', 'off');
set_param('master_thesis_simulink/System/Show Results/LS', 'commented', 'off');
set_param('master_thesis_simulink/System/Show Results/KF', 'commented', 'off');
set_param('master_thesis_simulink/System/Show Results/EKF', 'commented', 'off');
set_param('master_thesis_simulink/System/LS', 'commented', 'on');
set_param('master_thesis_simulink/System/LKF', 'commented', 'on');
set_param('master_thesis_simulink/System/EKF', 'commented', 'on');
set_param('master_thesis_simulink/System/EKF1', 'commented', 'on');
set_param('master_thesis_simulink/System/Human arm', 'commented', 'on');
set_param('master_thesis_simulink/System/Ref', 'commented', 'on');
set_param('master_thesis_simulink/System/Noisy p', 'commented', 'on');
set_param('master_thesis_simulink/System/Noisy pdot', 'commented', 'on');

% Comment Ros2Matlab and Experiments block
set_param('master_thesis_simulink/Ros2Matlab', 'commented', 'on ');
set_param('master_thesis_simulink/Experiments', 'commented', 'on ');

markers_shoulder_num = str2double(directory(2));
markers_forearm_num = str2double(directory(5));
markers_hand_num = str2double(directory(8));
fprintf('Set of markers: SHOULDER: %d, FOREARM: %d, HAND: %d\n', markers_shoulder_num, markers_forearm_num, markers_hand_num);

%% Load noise
noise = load(strcat('Simulations/', directory, '/noise.mat'));
noise = noise.noise;

% Loop
num = 0;
for i = 1:size(markers_num_list, 1)

    markers_num = markers_num_list(i);
    % Markers
    % Load corresponding set of markers
    markers_shoulder = load(strcat('Simulations/', directory, '/Markers_Pos_', num2str(markers_num), '/markers_shoulder.mat'));
    markers_shoulder = markers_shoulder.markers_shoulder;
    markers_forearm = load(strcat('Simulations/', directory, '/Markers_Pos_', num2str(markers_num), '/markers_forearm.mat'));
    markers_forearm = markers_forearm.markers_forearm;
    markers_hand = load(strcat('Simulations/', directory, '/Markers_Pos_', num2str(markers_num), '/markers_hand.mat'));
    markers_hand = markers_hand.markers_hand;

    % Number of markers
    m = str2num(directory(2)) + str2num(directory(5)) +  str2num(directory(8));

    %% KF

    % F matrix --> df/dq evaluated at the equilibrium point
    kf.F = full(f_Fekf_mex(kf.q_eq, markers_shoulder, markers_forearm, markers_hand, kf.pdot_eq));
    % G matrix --> df/du evaluated at the equilibrium point
    kf.G = full(f_Gekf_mex(kf.q_eq, markers_shoulder, markers_forearm, markers_hand, kf.pdot_eq));
    
    % H matrix --> dPhi/dq (Jacobian J) evaluated at the equilibrium point
    kf.J = full(f_J_mex(kf.q_eq, markers_shoulder, markers_forearm, markers_hand));
    % J matrix --> dPhi/du = 0 
    kf.I = zeros(3*m, 3*m);
    
    % p_eq
    kf.p_eq = full(f_Phi_mex(kf.q_eq, markers_shoulder, markers_forearm, markers_hand));

    kf.Q = sigma_Q2*eye(n);
    kf.R = sigma_R2*noise.Rp;

    % Check DETECTABILITY of (F, J) and STABILIZABILITY of (F, Q)
    [is_stabilizable, is_detectable] = is_stabilizable_detectable(kf.F, kf.Q, kf.J);
    
    if (is_stabilizable && is_detectable)
    
        %% Compute Pk, Pk_, Kk offline
    
        % List of Pk for k = 1, .., N
        kf.Pk = [kf.P0];
        % List of Pk_ for k = 1, .., N
        kf.Pk_ = [];
        % List of Kk for k = 1, .., N
        kf.Kk = [];
        
        % Compute Pk, Pk_, Kk offline
        N_samples = 50 / 0.01;
        for k = 1 : N_samples
        
            Pk_ = kf.F * kf.Pk(1:n, n*(k-1)+1:n*(k-1)+n) * kf.F' + kf.Q;
            Kk = Pk_ * kf.J' / (kf.J * Pk_ * kf.J' + kf.R);
            Pk = (eye(n) - Kk * kf.J) * Pk_ * (eye(size(kf.F, 1)) - Kk * kf.J)' + Kk * kf.R * Kk';
            kf.Pk_ = [kf.Pk_, Pk_];
            kf.Kk = [kf.Kk, Kk];
            kf.Pk = [kf.Pk, Pk];
        end
    end

    %% EKF
    
    ekf.Q = sigma_Q2*eye(n);
    ekf.R = sigma_R2*noise.Rp;

    for j = 1:size(sim_num_list(i, :), 2)

        sim_num = sim_num_list(i, j);
        num = num + 1;

        if (sim_num ~= 0)

            try

                % Load corresponding measurements
                p_0p = load(strcat('Simulations/', directory, '/Markers_Pos_', num2str(markers_num), '/Sim_', num2str(sim_num), '/p_0p.mat'));
                p_0p = p_0p.p_0p;
                v_0p = load(strcat('Simulations/', directory, '/Markers_Pos_', num2str(markers_num), '/Sim_', num2str(sim_num), '/v_0p.mat'));
                v_0p = v_0p.v_0p;
                
                %% Run Simulation 

                set_param('master_thesis_simulink', 'StopTime', 'p_0p.time(end)');
                output = sim('master_thesis_simulink.slx');
                q_LS = output.q_LS;
                q_KF = output.q_KF;
                q_EKF = output.q_EKF;
        
            catch
    
                disp(['Marker: ', num2str(markers_num), ', Sim: ', num2str(sim_num)]);
                disp('ERROR');
            end

                %% RMSE
                % ||e(1)||, ||e(2)||, ..., ||e(N)||
                norm_error_LS = vecnorm(output.error_FK__LS.signals.values);
                norm_error_KF = vecnorm(output.error_FK__KF.signals.values);
                norm_error_EKF = vecnorm(output.error_FK__EKF.signals.values);
                
                % ||e(1)||^2, ||e(2)||^2, ..., ||e(N)||^2
                norm2_error_LS = norm_error_LS.^2;
                norm2_error_KF = norm_error_KF.^2;
                norm2_error_EKF = norm_error_EKF.^2;
                
                % 1/(3m)*||e(1)||^2, 1/(3m)*||e(2)||^2, ..., 1/(3*m)*||e(N)||^2
                mean_norm2_error_LS = norm2_error_LS./(3*m);
                mean_norm2_error_KF = norm2_error_KF./(3*m);
                mean_norm2_error_EKF = norm2_error_EKF./(3*m);
                
                % sqrt(1/(3m)*||e(1)||^2),  sqrt(1/(3m)*||e(2)||^2), ...,  sqrt(1/(3m)*||e(N)||^2)
                RMSE_LS_t = sqrt(mean_norm2_error_LS);
                RMSE_KF_t = sqrt(mean_norm2_error_KF);
                RMSE_EKF_t = sqrt(mean_norm2_error_EKF);
                
                % DATA RMSE
                RMSE_LS = sqrt(1/(3*m) * mean(norm2_error_LS));
                RMSE_KF = sqrt(1/(3*m) * mean(norm2_error_KF));
                RMSE_EKF = sqrt(1/(3*m) * mean(norm2_error_EKF));
        
                disp(['Marker: ', num2str(markers_num), ', Sim: ', num2str(sim_num)]);
                disp(['RMSE LS = ', num2str(RMSE_LS)]);
                disp(['RMSE LKF = ', num2str(RMSE_KF)]);
                disp(['RMSE EKF = ', num2str(RMSE_EKF)]);
                disp(['Time of simulation: ', num2str(p_0p.time(end))]);
    

                % plot p_0
                fig = figure(4*(num-1)+1);
                set(fig, 'position', [10, 10, 1300, 900]);
                sgtitle('p^{0^\prime}');
                
                for k = 1 : m
                
                    title_name_x = strcat('$M', num2str(k), '_{x}^0$');
                    title_name_y = strcat('$M', num2str(k), '_{y}^0$');
                    title_name_z = strcat('$M', num2str(k), '_{z}^0$');
                    
                    subplot(m, 3, (k-1)*3+1);
                    plot(p_0p.time, p_0p.signals.values((k-1)*3+1, :));
                    title(title_name_x, Interpreter = "latex");
                    xlabel('Time [s]');
                    ylabel('Position [m]');
                    subplot(m, 3, (k-1)*3+2);
                    plot(p_0p.time, p_0p.signals.values((k-1)*3+2, :));
                    title(title_name_y, Interpreter = "latex");
                    xlabel('Time [s]');
                    ylabel('Position [m]');
                    subplot(m, 3, (k-1)*3+3);
                    plot(p_0p.time, p_0p.signals.values((k-1)*3+3, :));
                    title(title_name_z, Interpreter = "latex");
                    xlabel('Time [s]');
                    ylabel('Position [m]');
                end
                
                % plot v_0
                fig = figure(4*(num-1)+2);
                set(fig, 'position', [10, 10, 1300, 900]);
                sgtitle('v^{0^\prime}');
                for k = 1 : m
                
                    title_name_x = strcat('$\dot{M', num2str(k), '}_{x}^0$');
                    title_name_y = strcat('$\dot{M', num2str(k), '}_{y}^0$');
                    title_name_z = strcat('$\dot{M', num2str(k), '}_{z}^0$');
                    
                    subplot(m, 3, (k-1)*3+1);
                    plot(v_0p.time, v_0p.signals.values((k-1)*3+1, :));
                    title(title_name_x, Interpreter = "latex");
                    xlabel('Time [s]');
                    ylabel('Velocity [m/s]');
                    subplot(m, 3, (k-1)*3+2);
                    plot(v_0p.time, v_0p.signals.values((k-1)*3+2, :));
                    title(title_name_y, Interpreter = "latex");
                    xlabel('Time [s]');
                    ylabel('Velocity [m/s]');
                    subplot(m, 3, (k-1)*3+3);
                    plot(v_0p.time, v_0p.signals.values((k-1)*3+3, :));
                    title(title_name_z, Interpreter = "latex");
                    xlabel('Time [s]');
                    ylabel('Velocity [m/s]');
                end

                %% Scenario 1 --> first 6 experiments (LS vs EKF)
                if (scenario_num == 1)

                    % LS vs KF vs EKF Estimation
                    fig = figure(4*(num-1)+3);
                    sgtitle('LS vs EKF Estimation');
                    for k = 1 : n
                    
                        set(fig, 'position', [10, 10, 1300, 900]);
                        legend_name{1} = strcat('$\hat{\eta_', num2str(k), '}^{LS}$');
                        legend_name{2} = strcat('$\hat{\eta_', num2str(k), '}^{EKF}$');
                        subplot(3,3,k);
                        plot(q_LS.time, rad2deg(q_LS.signals.values(k, :)));
                        hold on;
                        plot(q_EKF.time, rad2deg(q_EKF.signals.values(:, k)));
                        xlabel('Time [s]');
                        ylabel('Joint variable [deg]');
                        ylim('padded');
                        legend(legend_name, 'Interpreter', 'latex', 'FontSize', 18);
                        grid on;
                    end
                    
                    % RMSE
                    title_name = '$\sqrt{\frac{1}{3m}\cdot\left\|p^0 - \Phi(\hat{q}^{LS})\right\|^2}$ vs $\sqrt{\frac{1}{3m}\cdot\left\|p^0 - \Phi(\hat{q}^{EKF})\right\|^2}$';
                    fig = figure(4*(num-1)+4);
                    set(fig, 'position', [10, 10, 1300, 900]);
                    plot(q_LS.time, RMSE_LS_t(:, :));
                    hold on;
                    plot(q_EKF.time, RMSE_EKF_t(:, :));
                    xlabel('Time [s]');
                    ylabel('norm error [m]');
                    legend('LS', 'EKF');
                    title(title_name, Interpreter = "latex");

                %% Scenario 2 --> 7th experiment (LKF vs EKF)
                elseif (scenario_num == 2)
                
                    % LS vs KF vs EKF Estimation
                    fig = figure(4*(num-1)+3);
                    sgtitle('LKF vs EKF Estimation');
                    for k = 1 : n
                    
                        set(fig, 'position', [10, 10, 1300, 900]);
                        legend_name{1} = strcat('$\hat{\eta_', num2str(k), '}^{LKF}$');
                        legend_name{2} = strcat('$\hat{\eta_', num2str(k), '}^{EKF}$');
                        subplot(3,3,k);
                        plot(q_KF.time, rad2deg(q_KF.signals.values(k, :)));
                        hold on;
                        plot(q_EKF.time, rad2deg(q_EKF.signals.values(:, k)));
                        xlabel('Time [s]');
                        ylabel('Joint variable [deg]');
                        ylim('padded');
                        legend(legend_name, 'Interpreter', 'latex', 'FontSize', 18);
                        grid on;
                    end
                    
                    % RMSE
                    title_name = '$\sqrt{\frac{1}{3m}\cdot\left\|p^0 - \Phi(\hat{q}^{LKF})\right\|^2}$ vs $\sqrt{\frac{1}{3m}\cdot\left\|p^0 - \Phi(\hat{q}^{EKF})\right\|^2}$';
                    fig = figure(4*(num-1)+4);
                    set(fig, 'position', [10, 10, 1300, 900]);
                    plot(q_KF.time, RMSE_KF_t(:, :));
                    hold on;
                    plot(q_EKF.time, RMSE_EKF_t(:, :));
                    xlabel('Time [s]');
                    ylabel('norm error [m]');
                    legend('LKF', 'EKF');
                    title(title_name, Interpreter = "latex");
                end
                
    
        end
        
    end
end

%% Save EPS figures

if (save)

    if (scenario_num == 1)

        for i = 1 : n
    
            fig = figure('Visible', 'off');
            legend_name{1} = strcat('$\hat{\eta_', num2str(k), '}^{LS}$');
            legend_name{2} = strcat('$\hat{\eta_', num2str(k), '}^{EKF}$');
            plot(q_LS.time, rad2deg(q_LS.signals.values(i, :)), 'LineWidth', 1.5);
            hold on;
            plot(q_EKF.time, rad2deg(q_EKF.signals.values(:, i)), 'LineWidth', 1.5);
            xlabel('Time [s]');
            ylabel('Joint variable [°]');
            legend(legend_name, 'Location', 'NorthOutside', 'Orientation', 'horizontal', 'Box', 'off', 'Interpreter', 'latex', 'FontSize', 26);
            set(gca, 'FontSize', 18);
            grid on;
            ylim('padded');
            eps_name = strcat(directory, '__eta_', num2str(i));
            saveas(gcf, eps_name, 'epsc');
        end
        
        
        fig = figure('Visible', 'off');
        legend_name{1} = strcat('LS');
        legend_name{2} = strcat('EKF');
        plot(q_EKF.time, RMSE_LS_t(:, :), 'LineWidth', 1.5);
        hold on;
        plot(q_EKF.time, RMSE_EKF_t(:, :), 'LineWidth', 1.5);
        xlabel('Time [s]');
        ylabel('RMSE [m]');
        legend(legend_name, 'Location', 'NorthOutside', 'Orientation', 'horizontal', 'Box', 'off', 'Interpreter', 'latex', 'FontSize', 26);
        set(gca, 'FontSize', 18);
        grid on;
        ylim('padded');
        eps_name = strcat(directory, '__RMSE');
        saveas(gcf, eps_name, 'epsc');

    elseif (scenario_num == 2)
     
        for i = 1 : n
    
            fig = figure('Visible', 'off');
            legend_name{1} = strcat('$\hat{\eta_', num2str(k), '}^{LKF}$');
            legend_name{2} = strcat('$\hat{\eta_', num2str(k), '}^{EKF}$');
            plot(q_KF.time, rad2deg(q_KF.signals.values(i, :)), 'LineWidth', 1.5);
            hold on;
            plot(q_EKF.time, rad2deg(q_EKF.signals.values(:, i)), 'LineWidth', 1.5);
            xlabel('Time [s]');
            ylabel('Joint variable [°]');
            legend(legend_name, 'Location', 'NorthOutside', 'Orientation', 'horizontal', 'Box', 'off', 'Interpreter', 'latex', 'FontSize', 26);
            set(gca, 'FontSize', 18);
            grid on;
            ylim('padded');
            eps_name = strcat(directory, '_LKF__eta_', num2str(i));
            saveas(gcf, eps_name, 'epsc');
        end
        
        
        fig = figure('Visible', 'off');
        legend_name{1} = strcat('LKF');
        legend_name{2} = strcat('EKF');
        plot(q_KF.time, RMSE_KF_t(:, :), 'LineWidth', 1.5);
        hold on;
        plot(q_EKF.time, RMSE_EKF_t(:, :), 'LineWidth', 1.5);
        xlabel('Time [s]');
        ylabel('RMSE [m]');
        legend(legend_name, 'Location', 'NorthOutside', 'Orientation', 'horizontal', 'Box', 'off', 'Interpreter', 'latex', 'FontSize', 26);
        set(gca, 'FontSize', 18);
        grid on;
        ylim('padded');
        eps_name = strcat(directory, '_LKF__RMSE');
        saveas(gcf, eps_name, 'epsc');
    end
    
end
end