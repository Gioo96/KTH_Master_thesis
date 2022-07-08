function output = LS_KF_EKF_exp(method_flag, directory)

%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

% Inputs
% -- method_flag         : 'LS'   --> Least Squares is selected
%                        : 'KF'   --> Kalman Filter is selected
%                        : 'EKF'  --> Extended Kalman Filter is selected


% Uncomment Experiments, Ros2Matlab
set_param(strcat('master_thesis_simulink/Experiments/', directory), 'commented', 'off');
set_param(strcat('master_thesis_simulink/Experiments/', directory, '/Experiments'), 'commented', 'off');
set_param(strcat('master_thesis_simulink/Experiments/', directory, '/Model validation'), 'commented', 'on');
set_param(strcat('master_thesis_simulink/Experiments/', directory, '/Noise estimation'), 'commented', 'on');
set_param(strcat('master_thesis_simulink/Ros2Matlab/', directory), 'commented', 'off');

% Comment System
set_param('master_thesis_simulink/System', 'commented', 'on');

% Number of DoF
n = 7;

% Number of markers
m = str2double(directory(2)) + str2double(directory(5)) + str2double(directory(8));

global selected;

%% Select {LS, KF, EKF}

% selected --> -1 : No method has been selected
%               0 : LS
%               1 : KF
%               2 : EKF
%               3 : LS && KF
%               4 : LS && EKF
%               5 : KF && EKF
%               6 : LS && KF && EKF
selected = -1;
number_methods = length(method_flag);

switch number_methods

    %% 1 ELEMENT
    case 1

        % LS
        if (strcmp(method_flag(number_methods), "LS") == 1)

            selected = 0;
        
        % KF
        elseif (strcmp(method_flag(number_methods), "KF") == 1)

            selected = 1;
        
        % EKF
        elseif (strcmp(method_flag(number_methods), "EKF") == 1)

            selected = 2;

        end

    %% 2 ELEMENTS
    case 2

        % LS && KF
        if ((strcmp(method_flag(number_methods-1), "LS") == 1 && strcmp(method_flag(number_methods), "KF") == 1) || (strcmp(method_flag(number_methods), "LS") == 1 && strcmp(method_flag(number_methods-1), "KF") == 1))
    
            selected = 3;
        
        % LS && EKF
        elseif ((strcmp(method_flag(number_methods-1), "LS") == 1 && strcmp(method_flag(number_methods), "EKF") == 1) || (strcmp(method_flag(number_methods), "LS") == 1 && strcmp(method_flag(number_methods-1), "EKF") == 1))
    
            selected = 4;
        
        % KF && EKF
        elseif ((strcmp(method_flag(number_methods-1), "KF") == 1 && strcmp(method_flag(number_methods), "EKF") == 1) || (strcmp(method_flag(number_methods), "KF") == 1 && strcmp(method_flag(number_methods-1), "EKF") == 1))
    
            selected = 5;
        end

    %% 3 ELEMENTS
    case 3

        selected = 6;
end

switch selected

    %% LS
    case 0

        %% Comment / Uncomment blocks
        % Uncomment LS
        set_param(strcat('master_thesis_simulink/Experiments/', directory, '/Experiments/LS'), 'commented', 'off');

        % Comment KF
        set_param(strcat('master_thesis_simulink/Experiments/', directory, '/Experiments/KF'), 'commented', 'on');
        % Comment EKF
        set_param(strcat('master_thesis_simulink/Experiments/', directory, '/Experiments/EKF'), 'commented', 'on');

        %% Simulation
        output = sim("master_thesis_simulink.slx");

        %% Plot results

        % LS Estimation
        legend_name = cell(n, 1);
        fig = figure;
        sgtitle('LS Estimation');
        for i = 1 : n

            legend_name{i} = strcat('q_', num2str(i), '^{LS}');
            set(fig, 'position', [10, 10, 1300, 900]);
            subplot(3,3,i);
            plot(output.q_LS.time, squeeze(output.q_LS.signals.values(i, :))');
            xlabel('Time [s]');
            ylabel('Joint variable [deg]');
            legend(legend_name{i}, 'Location', 'Southwest');
        end

        % p_0 vs Phi(q_LS)
        legend_name = cell(3*m, 1);
        fig = figure;
        set(fig, 'position', [10, 10, 1300, 900]);
        sgtitle('p^0 - \Phi(q_{LS})');
        for i = 1 : m

            title_name_x = strcat('$M', num2str(i), '_{x}^0$ - $\hat{M', num2str(i), '}_{FK, x}^{LS}$');
            title_name_y = strcat('$M', num2str(i), '_{y}^0$ - $\hat{M', num2str(i), '}_{FK, y}^{LS}$');
            title_name_z = strcat('$M', num2str(i), '_{z}^0$ - $\hat{M', num2str(i), '}_{FK, z}^{LS}$');
            
            subplot(m, 3, (i-1)*3+1);
            plot(output.error_FK__LS.time, output.error_FK__LS.signals.values((i-1)*3+1, :));
            title(title_name_x, Interpreter = "latex");
            xlabel('Time [s]');
            ylabel('Error [m]');
            subplot(m, 3, (i-1)*3+2);
            plot(output.error_FK__LS.time, output.error_FK__LS.signals.values((i-1)*3+2, :));
            title(title_name_y, Interpreter = "latex");
            xlabel('Time [s]');
            ylabel('Error [m]');
            subplot(m, 3, (i-1)*3+3);
            plot(output.error_FK__LS.time, output.error_FK__LS.signals.values((i-1)*3+3, :));
            title(title_name_z, Interpreter = "latex");
            xlabel('Time [s]');
            ylabel('Error [m]');
        end

        % ||p_0 - Phi(q_LS)||
        norm_error = vecnorm(output.error_FK__LS.signals.values);
        title_name = '$\|p^0 - \Phi(\hat{q}^{LS})\|$';
        fig = figure;
        set(fig, 'position', [10, 10, 1300, 900]);
        plot(output.error_FK__LS.time, norm_error(:, :));
        xlabel('Time [s]');
        ylabel('norm error [m]');
        title(title_name, Interpreter = "latex");

    %% KF
    case 1
        
        % Global variables
        global markers_shoulder markers_forearm markers_hand;
        global noise;
        global kf;

        %% Comment blocks and set parameters 

        % Uncomment KF vs OBS block
        set_param('master_thesis_simulink/System/KF vs OBS', 'commented', 'off');

        % Comment TRIAL
        set_param('master_thesis_simulink/System/KF vs OBS/trial', 'commented', 'on');

        % Comment LS block 
        set_param('master_thesis_simulink/System/LS', 'commented', 'on');
        
        % Comment EKF block
        set_param('master_thesis_simulink/System/EKF', 'commented', 'on');

        % State space model : Deltaqk+1 = F*Deltaqk + G*Deltauk_noisy + wk
        %                     Deltapk   = H*Deltaqk + vk

        % F matrix --> df/dq evaluated at the equilibrium point
        kf.F = full(f_Fekf_mex(kf.q_eq, markers_shoulder, markers_forearm, markers_hand, kf.u_eq));
        % G matrix --> df/du evaluated at the equilibrium point
        kf.G = full(f_Gekf_mex(kf.q_eq, markers_shoulder, markers_forearm, markers_hand, kf.u_eq));
        
        % H matrix --> dPhi/dq (Jacobian J) evaluated at the equilibrium point
        kf.H = full(f_J_mex(kf.q_eq, markers_shoulder, markers_forearm, markers_hand));
        % J matrix --> dPhi/du = 0 
        kf.J = zeros(3*m, 3*m);

        % p_eq
        kf.p_eq = full(f_Phi_mex(kf.q_eq, markers_shoulder, markers_forearm, markers_hand));

        % F, G, H, J matrices
        set_param('master_thesis_simulink/System/KF vs OBS/Discrete State-Space', 'A', 'kf.F');
        set_param('master_thesis_simulink/System/KF vs OBS/Discrete State-Space', 'B', 'kf.G');
        set_param('master_thesis_simulink/System/KF vs OBS/Discrete State-Space', 'C', 'kf.H');
        set_param('master_thesis_simulink/System/KF vs OBS/Discrete State-Space', 'D', 'kf.J');

        % Initial Condition
        set_param('master_thesis_simulink/System/KF vs OBS/Discrete State-Space', 'InitialCondition', 'q0_model - kf.q_eq');

        % In Simulink the process noise is applied as input so that:
        % Deltaqk+1 = F*Deltaqk + G*Deltauk_noisy + G*nk -----> wk = G*nk
        % Deltapk   = H*Deltaqk + vk
        % var{wk} = Q = G*var{nk}*G'
        % sqrt(Q)*sqrt(Q) = G*N*G'= G*B*B'*G'
        % G*B = sqrt(Q)
        % N = var{vk} = B*B'

        noise.N = linsolve(kf.G, sqrt(noise.Q)) * linsolve(kf.G, sqrt(noise.Q))';
        noise.N_seed = 4;

        %% Compute Pk, Pk_, Kk offline

        % List of Pk for k = 1, .., N
        kf.Pk = [kf.P0];
        % List of Pk_ for k = 1, .., N
        kf.Pk_ = [];
        % List of Kk for k = 1, .., N
        kf.Kk = [];

        % Compute Pk, Pk_, Kk offline
        for k = 1 : N_samples
        
            Pk_ = kf.F * kf.Pk(1:n, n*(k-1)+1:n*(k-1)+n) * kf.F' + noise.Q;
            Kk = Pk_ * kf.H' / (kf.H * Pk_ * kf.H' + noise.R);
            Pk = (eye(n) - Kk * kf.H) * Pk_ * (eye(size(kf.F, 1)) - Kk * kf.H)' + Kk * noise.R * Kk';
            kf.Pk_ = [kf.Pk_, Pk_];
            kf.Kk = [kf.Kk, Kk];
            kf.Pk = [kf.Pk, Pk];
        
        end

        % Simulation
        output = sim("master_thesis_simulink.slx");
        
        %% Plot KF estimate vs CONTINUOS joints' variables
        fig = figure;
        sgtitle("Linearized KF vs True");

        for i = 1 : n
        
            q = strcat('$q_', num2str(i), '$');
            qkf = strcat('$\hat{q}^{KF}_', num2str(i), '$');
            set(fig, 'position', [10, 10, 1300, 900]);
            subplot(3,3,i);
            plot(output.q.time, rad2deg(output.q.signals.values(:, i)), 'LineWidth', 2);
            hold on;
            stairs(output.qk_kf.time, rad2deg(squeeze(output.qk_kf.signals.values(:, 1))'), 'LineWidth', 2);
            legend(q, qkf, 'Location', 'southeast', 'Interpreter', 'latex');
        end

    %% EKF
    case 2

        %% Comment / Uncomment blocks 

        % Uncomment EKF
        set_param(strcat('master_thesis_simulink/Experiments/', directory, '/Experiments/EKF'), 'commented', 'off');

        % Comment KF
        set_param(strcat('master_thesis_simulink/Experiments/', directory, '/Experiments/KF'), 'commented', 'on');
        % Comment LS 
        set_param(strcat('master_thesis_simulink/Experiments/', directory, '/Experiments/LS'), 'commented', 'on');

        %% Simulation
        output = sim("master_thesis_simulink.slx");

        %% Plot results

        % plot p_0
        fig = figure;
        set(fig, 'position', [10, 10, 1300, 900]);
        sgtitle('p^0');
        for i = 1 : m
        
            title_name_x = strcat('$M', num2str(i), '_{x}^0$');
            title_name_y = strcat('$M', num2str(i), '_{y}^0$');
            title_name_z = strcat('$M', num2str(i), '_{z}^0$');
            
            subplot(m, 3, (i-1)*3+1);
            plot(output.p_0.time, output.p_0.signals.values((i-1)*3+1, :));
            title(title_name_x, Interpreter = "latex");
            xlabel('Time [s]');
            ylabel('Position [m]');
            subplot(m, 3, (i-1)*3+2);
            plot(output.p_0.time, output.p_0.signals.values((i-1)*3+2, :));
            title(title_name_y, Interpreter = "latex");
            xlabel('Time [s]');
            ylabel('Position [m]');
            subplot(m, 3, (i-1)*3+3);
            plot(output.p_0.time, output.p_0.signals.values((i-1)*3+3, :));
            title(title_name_z, Interpreter = "latex");
            xlabel('Time [s]');
            ylabel('Position [m]');
        end

        % plot v_0
        fig = figure;
        set(fig, 'position', [10, 10, 1300, 900]);
        sgtitle('v^0');
        for i = 1 : m
        
            title_name_x = strcat('$\dot{M', num2str(i), '}_{x}^0$');
            title_name_y = strcat('$\dot{M', num2str(i), '}_{y}^0$');
            title_name_z = strcat('$\dot{M', num2str(i), '}_{z}^0$');
            
            subplot(m, 3, (i-1)*3+1);
            plot(output.v_0.time, output.v_0.signals.values((i-1)*3+1, :));
            title(title_name_x, Interpreter = "latex");
            xlabel('Time [s]');
            ylabel('Velocity [m/s]');
            subplot(m, 3, (i-1)*3+2);
            plot(output.v_0.time, output.v_0.signals.values((i-1)*3+2, :));
            title(title_name_y, Interpreter = "latex");
            xlabel('Time [s]');
            ylabel('Velocity [m/s]');
            subplot(m, 3, (i-1)*3+3);
            plot(output.v_0.time, output.v_0.signals.values((i-1)*3+3, :));
            title(title_name_z, Interpreter = "latex");
            xlabel('Time [s]');
            ylabel('Velocity [m/s]');
        end

        % EKF Estimation
        fig = figure;
        sgtitle('EKF Estimation');
        for i = 1 : n

            title_name = strcat('q_', num2str(i), '^{EKF}');
            set(fig, 'position', [10, 10, 1300, 900]);
            subplot(3,3,i);
            plot(output.q_EKF.time, output.q_EKF.signals.values(:, i));
            xlabel('Time [s]');
            ylabel('Joint variable [deg]');
            title(title_name);
        end

        % p_0 vs Phi(q_EKF)
        fig = figure;
        set(fig, 'position', [10, 10, 1300, 900]);
        sgtitle('p^0 - \Phi(q_{EKF})');
        for i = 1 : m

            title_name_x = strcat('$M', num2str(i), '_{x}^0$ - $\hat{M', num2str(i), '}_{FK, x}^{EKF}$');
            title_name_y = strcat('$M', num2str(i), '_{y}^0$ - $\hat{M', num2str(i), '}_{FK, y}^{EKF}$');
            title_name_z = strcat('$M', num2str(i), '_{z}^0$ - $\hat{M', num2str(i), '}_{FK, z}^{EKF}$');
            
            subplot(m, 3, (i-1)*3+1);
            plot(output.error_FK__EKF.time, output.error_FK__EKF.signals.values((i-1)*3+1, :));
            title(title_name_x, Interpreter = "latex");
            xlabel('Time [s]');
            ylabel('Error [m]');
            subplot(m, 3, (i-1)*3+2);
            plot(output.error_FK__EKF.time, output.error_FK__EKF.signals.values((i-1)*3+2, :));
            title(title_name_y, Interpreter = "latex");
            xlabel('Time [s]');
            ylabel('Error [m]');
            subplot(m, 3, (i-1)*3+3);
            plot(output.error_FK__EKF.time, output.error_FK__EKF.signals.values((i-1)*3+3, :));
            title(title_name_z, Interpreter = "latex");
            xlabel('Time [s]');
            ylabel('Error [m]');
        end

        % ||p_0 - Phi(q_EKF)||
        norm_error = vecnorm(output.error_FK__EKF.signals.values);
        title_name = '$\|p^0 - \Phi(\hat{q}^{EKF})\|$';
        fig = figure;
        set(fig, 'position', [10, 10, 1300, 900]);
        plot(output.error_FK__EKF.time, norm_error(:, :));
        xlabel('Time [s]');
        ylabel('norm error [m]');
        title(title_name, Interpreter = "latex");

    %% LS && KF
    case 3

        %% Comment blocks and set parameters 

        % Uncomment LS block 
        set_param('master_thesis_simulink/System/LS', 'commented', 'off');

        % Uncomment KF vs OBS block
        set_param('master_thesis_simulink/System/KF vs OBS', 'commented', 'off');

        % Comment TRIAL
        set_param('master_thesis_simulink/System/KF vs OBS/trial', 'commented', 'on');
        
        % Comment EKF block
        set_param('master_thesis_simulink/System/EKF', 'commented', 'on');

        %% LS
        % Least Squares IC
        set_param('master_thesis_simulink/System/LS/Integrator', 'InitialCondition', 'q0_LS');

        %% KF
        % Global variables
        global noise;
        global kf;

        % Number of markers
        m = size(markers.shoulder_variables, 1) + size(markers.forearm_variables, 1) + size(markers.hand_variables, 1);

        % State space model : Deltaqk+1 = F*Deltaqk + G*Deltauk_noisy + wk
        %                     Deltapk   = H*Deltaqk + vk

        % F matrix --> df/dq evaluated at the equilibrium point
        kf.F = full(f_Fekf_mex(kf.q_eq, markers.shoulder_variables, markers.forearm_variables, markers.hand_variables, kf.u_eq));
        % G matrix --> df/du evaluated at the equilibrium point
        kf.G = full(f_Gekf_mex(kf.q_eq, markers.shoulder_variables, markers.forearm_variables, markers.hand_variables, kf.u_eq));
        
        % H matrix --> dPhi/dq (Jacobian J) evaluated at the equilibrium point
        kf.H = full(f_J_mex(kf.q_eq, markers.shoulder_variables, markers.forearm_variables, markers.hand_variables));
        % J matrix --> dPhi/du = 0 
        kf.J = zeros(3*m, 3*m);

        % F, G, H, J matrices
        set_param('master_thesis_simulink/System/KF vs OBS/Discrete State-Space', 'A', 'kf.F');
        set_param('master_thesis_simulink/System/KF vs OBS/Discrete State-Space', 'B', 'kf.G');
        set_param('master_thesis_simulink/System/KF vs OBS/Discrete State-Space', 'C', 'kf.H');
        set_param('master_thesis_simulink/System/KF vs OBS/Discrete State-Space', 'D', 'kf.J');

        % Initial Condition
        set_param('master_thesis_simulink/System/KF vs OBS/Discrete State-Space', 'InitialCondition', 'q0_model - kf.q_eq');

        % In Simulink the process noise is applied as input so that:
        % Deltaqk+1 = F*Deltaqk + G*Deltauk_noisy + G*nk -----> wk = G*nk
        % Deltapk   = H*Deltaqk + vk
        % var{wk} = Q = G*var{nk}*G'
        % sqrt(Q)*sqrt(Q) = G*N*G'= G*B*B'*G'
        % G*B = sqrt(Q)
        % N = var{vk} = B*B'

        noise.N = linsolve(kf.G, sqrt(noise.Q)) * linsolve(kf.G, sqrt(noise.Q))';
        noise.N_seed = 4;

        %% Compute Pk, Pk_, Kk offline

        % List of Pk for k = 1, .., N
        kf.Pk = [kf.P0];
        % List of Pk_ for k = 1, .., N
        kf.Pk_ = [];
        % List of Kk for k = 1, .., N
        kf.Kk = [];

        % Compute Pk, Pk_, Kk offline
        for k = 1 : N_samples
        
            Pk_ = kf.F * kf.Pk(1:n, n*(k-1)+1:n*(k-1)+n) * kf.F' + noise.Q;
            Kk = Pk_ * kf.H' / (kf.H * Pk_ * kf.H' + noise.R);
            Pk = (eye(n) - Kk * kf.H) * Pk_ * (eye(size(kf.F, 1)) - Kk * kf.H)' + Kk * noise.R * Kk';
            kf.Pk_ = [kf.Pk_, Pk_];
            kf.Kk = [kf.Kk, Kk];
            kf.Pk = [kf.Pk, Pk];
        
        end

        %% Simulation
        output = sim("master_thesis_simulink.slx");

        %% Plot KF estimate vs LS estimate vs TRUE joints' variables
        fig = figure;
        sgtitle("LS vs KF vs True");
        
        for i = 1 : n
        
            q = strcat('$q_', num2str(i), '$');
            qkf = strcat('$\hat{q}^{KF}_', num2str(i), '$');
            qls = strcat('$\hat{q}^{LS}_', num2str(i), '$');
            set(fig, 'position', [10, 10, 1300, 900]);
            subplot(3,3,i);
            plot(output.out.time, rad2deg(output.out.signals(1).values(:, i)), 'LineWidth', 2);
            hold on;
            stairs(output.out.time, rad2deg(squeeze(output.out.signals(2).values(i, :))'), 'LineWidth', 2);
            hold on;
            stairs(output.out.time, rad2deg(squeeze(output.out.signals(3).values(i, :))'), 'LineWidth', 2);
            legend(q, qls, qkf, 'Location', 'southeast', 'Interpreter', 'latex');
        end

    %% LS && EKF
    case 5

        %% Comment blocks and set parameters 

        % Uncomment LS block 
        set_param('master_thesis_simulink/System/LS', 'commented', 'off');

        % Uncomment EKF block
        set_param('master_thesis_simulink/System/EKF', 'commented', 'off');

        % Comment KF vs OBS block
        set_param('master_thesis_simulink/System/KF vs OBS', 'commented', 'on');

        % Comment TRIAL
        set_param('master_thesis_simulink/System/KF vs OBS/trial', 'commented', 'on');

        %% LS
        % Least Squares IC
        set_param('master_thesis_simulink/System/LS/Integrator', 'InitialCondition', 'q0_LS');

        %% Simulation
        output = sim("master_thesis_simulink.slx");

        %% Plot KF estimate vs LS estimate vs TRUE joints' variables
        fig = figure;
        sgtitle("LS vs EKF vs True");
        
        for i = 1 : n
        
            q = strcat('$q_', num2str(i), '$');
            qekf = strcat('$\hat{q}^{EKF}_', num2str(i), '$');
            qls = strcat('$\hat{q}^{LS}_', num2str(i), '$');
            set(fig, 'position', [10, 10, 1300, 900]);
            subplot(3,3,i);
            plot(output.out.time, rad2deg(output.out.signals(1).values(:, i)), 'LineWidth', 2);
            hold on;
            stairs(output.out.time, rad2deg(squeeze(output.out.signals(2).values(i, :))'), 'LineWidth', 2);
            hold on;
            stairs(output.out.time, rad2deg(squeeze(output.out.signals(4).values(:, i))'), 'LineWidth', 2);
            legend(q, qls, qekf, 'Location', 'southeast', 'Interpreter', 'latex');
        end

        %% Plot ||pk-Phi(qk_est)|| over time
        fig1 = figure;

        n = size(output.p_noisy_meas.signals.values, 1);
        kf_error_ekf = zeros(n, 1);
        kf_error_ls = zeros(n, 1);
        for i = 1:n

            kf_error_ls(i) = norm(output.p_noisy_meas.signals.values(i,:) - full(f_f_mex(output.out.signals(2).values(:, i), markers_shoulder, markers_forearm, markers_hand)));
            kf_error_ekf(i) = norm(output.p_noisy_meas.signals.values(i,:) - full(f_f_mex(output.out.signals(4).values(i, :), markers_shoulder, markers_forearm, markers_hand)));
        end
        leg_ekf = strcat('$||p_k - \Phi(\hat{q_k}^{ekf})||', '$');
        leg_ls = strcat('$||p_k - \Phi(\hat{q_k}^{ls})||', '$');
        plot(output.p_noisy_meas.time, kf_error_ls);
        hold on;
        plot(output.p_noisy_meas.time, kf_error_ekf);
        legend(leg_ls, leg_ekf, 'Location', 'southeast', 'Interpreter', 'latex');
        title("FK error");

    %% KF && EKF
    case 6

        %% Comment blocks and set parameters 

        % Comment LS block 
        set_param('master_thesis_simulink/System/LS', 'commented', 'on');

        % Uncomment EKF block
        set_param('master_thesis_simulink/System/EKF', 'commented', 'off');

        % Uncomment KF vs OBS block
        set_param('master_thesis_simulink/System/KF vs OBS', 'commented', 'off');

        % Comment TRIAL
        set_param('master_thesis_simulink/System/KF vs OBS/trial', 'commented', 'on');

        %% KF
        % Global variables
        global noise;
        global kf;

        % Number of markers
        m = size(markers.shoulder_variables, 1) + size(markers.forearm_variables, 1) + size(markers.hand_variables, 1);

        % State space model : Deltaqk+1 = F*Deltaqk + G*Deltauk_noisy + wk
        %                     Deltapk   = H*Deltaqk + vk

        % F matrix --> df/dq evaluated at the equilibrium point
        kf.F = full(f_Fekf_mex(kf.q_eq, markers.shoulder_variables, markers.forearm_variables, markers.hand_variables, kf.u_eq));
        % G matrix --> df/du evaluated at the equilibrium point
        kf.G = full(f_Gekf_mex(kf.q_eq, markers.shoulder_variables, markers.forearm_variables, markers.hand_variables, kf.u_eq));
        
        % H matrix --> dPhi/dq (Jacobian J) evaluated at the equilibrium point
        kf.H = full(f_J_mex(kf.q_eq, markers.shoulder_variables, markers.forearm_variables, markers.hand_variables));
        % J matrix --> dPhi/du = 0 
        kf.J = zeros(3*m, 3*m);

        % F, G, H, J matrices
        set_param('master_thesis_simulink/System/KF vs OBS/Discrete State-Space', 'A', 'kf.F');
        set_param('master_thesis_simulink/System/KF vs OBS/Discrete State-Space', 'B', 'kf.G');
        set_param('master_thesis_simulink/System/KF vs OBS/Discrete State-Space', 'C', 'kf.H');
        set_param('master_thesis_simulink/System/KF vs OBS/Discrete State-Space', 'D', 'kf.J');

        % Initial Condition
        set_param('master_thesis_simulink/System/KF vs OBS/Discrete State-Space', 'InitialCondition', 'q0_model - kf.q_eq');

        % In Simulink the process noise is applied as input so that:
        % Deltaqk+1 = F*Deltaqk + G*Deltauk_noisy + G*nk -----> wk = G*nk
        % Deltapk   = H*Deltaqk + vk
        % var{wk} = Q = G*var{nk}*G'
        % sqrt(Q)*sqrt(Q) = G*N*G'= G*B*B'*G'
        % G*B = sqrt(Q)
        % N = var{vk} = B*B'

        noise.N = linsolve(kf.G, sqrt(noise.Q)) * linsolve(kf.G, sqrt(noise.Q))';
        noise.N_seed = 4;

        % Compute Pk, Pk_, Kk offline
        % List of Pk for k = 1, .., N
        kf.Pk = [kf.P0];
        % List of Pk_ for k = 1, .., N
        kf.Pk_ = [];
        % List of Kk for k = 1, .., N
        kf.Kk = [];

        % Compute Pk, Pk_, Kk offline
        for k = 1 : N_samples
        
            Pk_ = kf.F * kf.Pk(1:n, n*(k-1)+1:n*(k-1)+n) * kf.F' + noise.Q;
            Kk = Pk_ * kf.H' / (kf.H * Pk_ * kf.H' + noise.R);
            Pk = (eye(n) - Kk * kf.H) * Pk_ * (eye(size(kf.F, 1)) - Kk * kf.H)' + Kk * noise.R * Kk';
            kf.Pk_ = [kf.Pk_, Pk_];
            kf.Kk = [kf.Kk, Kk];
            kf.Pk = [kf.Pk, Pk];
        
        end

        %% Simulation
        output = sim("master_thesis_simulink.slx");

        %% Plot KF estimate vs EKF estimate vs TRUE joints' variables
        fig = figure;
        sgtitle("KF vs EKF vs True");
        
        for i = 1 : n
        
            q = strcat('$q_', num2str(i), '$');
            qkf = strcat('$\hat{q}^{KF}_', num2str(i), '$');
            qekf = strcat('$\hat{q}^{EKF}_', num2str(i), '$');
            set(fig, 'position', [10, 10, 1300, 900]);
            subplot(3,3,i);
            plot(output.out.time, rad2deg(output.out.signals(1).values(:, i)), 'LineWidth', 2);
            hold on;
            stairs(output.out.time, rad2deg(squeeze(output.out.signals(3).values(i, :))'), 'LineWidth', 2);
            hold on;
            stairs(output.out.time, rad2deg(squeeze(output.out.signals(4).values(i, :))'), 'LineWidth', 2);
            legend(q, qkf, qekf, 'Location', 'southeast', 'Interpreter', 'latex');
        end

    %% LS && KF && EKF
    case 7

        %% Comment blocks and set parameters 

        % Uncomment LS block 
        set_param('master_thesis_simulink/System/LS', 'commented', 'off');

        % Uncomment EKF block
        set_param('master_thesis_simulink/System/EKF', 'commented', 'off');

        % Uncomment KF vs OBS block
        set_param('master_thesis_simulink/System/KF vs OBS', 'commented', 'off');

        % Comment TRIAL
        set_param('master_thesis_simulink/System/KF vs OBS/trial', 'commented', 'on');

        %% LS
        % Least Squares IC
        set_param('master_thesis_simulink/System/LS/Integrator', 'InitialCondition', 'q0_LS');

        %% KF
        % Global variables
        global noise;
        global kf;

        % Number of markers
        m = size(markers.shoulder_variables, 1) + size(markers.forearm_variables, 1) + size(markers.hand_variables, 1);

        % State space model : Deltaqk+1 = F*Deltaqk + G*Deltauk_noisy + wk
        %                     Deltapk   = H*Deltaqk + vk

        % F matrix --> df/dq evaluated at the equilibrium point
        kf.F = full(f_Fekf_mex(kf.q_eq, markers.shoulder_variables, markers.forearm_variables, markers.hand_variables, kf.u_eq));
        % G matrix --> df/du evaluated at the equilibrium point
        kf.G = full(f_Gekf_mex(kf.q_eq, markers.shoulder_variables, markers.forearm_variables, markers.hand_variables, kf.u_eq));
        
        % H matrix --> dPhi/dq (Jacobian J) evaluated at the equilibrium point
        kf.H = full(f_J_mex(kf.q_eq, markers.shoulder_variables, markers.forearm_variables, markers.hand_variables));
        % J matrix --> dPhi/du = 0 
        kf.J = zeros(3*m, 3*m);

        % F, G, H, J matrices
        set_param('master_thesis_simulink/System/KF vs OBS/Discrete State-Space', 'A', 'kf.F');
        set_param('master_thesis_simulink/System/KF vs OBS/Discrete State-Space', 'B', 'kf.G');
        set_param('master_thesis_simulink/System/KF vs OBS/Discrete State-Space', 'C', 'kf.H');
        set_param('master_thesis_simulink/System/KF vs OBS/Discrete State-Space', 'D', 'kf.J');

        % Initial Condition
        set_param('master_thesis_simulink/System/KF vs OBS/Discrete State-Space', 'InitialCondition', 'q0_model - kf.q_eq');

        % In Simulink the process noise is applied as input so that:
        % Deltaqk+1 = F*Deltaqk + G*Deltauk_noisy + G*nk -----> wk = G*nk
        % Deltapk   = H*Deltaqk + vk
        % var{wk} = Q = G*var{nk}*G'
        % sqrt(Q)*sqrt(Q) = G*N*G'= G*B*B'*G'
        % G*B = sqrt(Q)
        % N = var{vk} = B*B'

        noise.N = linsolve(kf.G, sqrt(noise.Q)) * linsolve(kf.G, sqrt(noise.Q))';
        noise.N_seed = 4;

        % Compute Pk, Pk_, Kk offline
        % List of Pk for k = 1, .., N
        kf.Pk = [kf.P0];
        % List of Pk_ for k = 1, .., N
        kf.Pk_ = [];
        % List of Kk for k = 1, .., N
        kf.Kk = [];

        % Compute Pk, Pk_, Kk offline
        for k = 1 : N_samples
        
            Pk_ = kf.F * kf.Pk(1:n, n*(k-1)+1:n*(k-1)+n) * kf.F' + noise.Q;
            Kk = Pk_ * kf.H' / (kf.H * Pk_ * kf.H' + noise.R);
            Pk = (eye(n) - Kk * kf.H) * Pk_ * (eye(size(kf.F, 1)) - Kk * kf.H)' + Kk * noise.R * Kk';
            kf.Pk_ = [kf.Pk_, Pk_];
            kf.Kk = [kf.Kk, Kk];
            kf.Pk = [kf.Pk, Pk];
        
        end

        %% Simulation
        output = sim("master_thesis_simulink.slx");

        %% Plot KF estimate vs EKF estimate vs TRUE joints' variables
        fig = figure;
        sgtitle("LS vs KF vs EKF vs True");
        
        for i = 1 : n
        
            q = strcat('$q_', num2str(i), '$');
            qls = strcat('$\hat{q}^{LS}_', num2str(i), '$');
            qkf = strcat('$\hat{q}^{KF}_', num2str(i), '$');
            qekf = strcat('$\hat{q}^{EKF}_', num2str(i), '$');
            set(fig, 'position', [10, 10, 1300, 900]);
            subplot(3,3,i);
            plot(output.out.time, rad2deg(output.out.signals(1).values(:, i)), 'LineWidth', 2);
            hold on;
            stairs(output.out.time, rad2deg(squeeze(output.out.signals(2).values(i, :))'), 'LineWidth', 2);
            hold on;
            stairs(output.out.time, rad2deg(squeeze(output.out.signals(3).values(i, :))'), 'LineWidth', 2);
            hold on;
            stairs(output.out.time, rad2deg(squeeze(output.out.signals(4).values(i, :))'), 'LineWidth', 2);
            legend(q, qls, qkf, qekf, 'Location', 'southeast', 'Interpreter', 'latex');
        end

        %% Plot ||pk-Phi(qk_est)|| over time
        output.p_noisy_meas
        fig1 = figure;
        sgtitle("FK error")

        %n = length(output.p_noisy_meas)

end
end