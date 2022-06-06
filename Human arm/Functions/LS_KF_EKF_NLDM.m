function LS_KF_EKF_NLDM(method_flag, simModel_flag, markers, N_samples, C_code_folder)

%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

% Inputs
% -- q0                  : Matrix containing the IC --> ROW 1 : Simulink Model IC / NLDM IC (EKF) / LDM IC (KF)
%                                                       ROW 2 : LS IC
% -- method_flag         : 'LS'   --> Least Squares is selected
%                        : 'KF'   --> Kalman Filter is selected
%                        : 'EKF'  --> Extended Kalman Filter is selected
%                        : 'NLDM' --> Non Linear Discrete Model is selected
% -- simModel_flag       : 'free'         --> free fall motion is simulated
%                        : 'precompiuted' --> precompiuted trajectory is simulated
% -- markers             : []                   -->  No Markers are needed here
%                        : [markers variables]  -->  KF
% -- kf_ekf              : Struct containing Kalman Filter or Extended Kalman Filter parameters such as : -- Initial states estimate 
%                                                                                                         -- Initial state covariance estimate
%                        : [markers variables]  -->  KF
% -- N_samples           : Number of samples
% -- C_code_folder       : Folder containing the mex functions needed, depending on the location of the markers
%                          --> 'S1_F1_H1'
%                          --> 'S4_F3_H2'

% Uncomment System
set_param('master_thesis_simulink/System', 'commented', 'off');

% Comment Ros2Matlab
set_param('master_thesis_simulink/Ros2Matlab', 'commented', 'on');

% Number of DoF
n = 7;

global markers_shoulder markers_forearm markers_hand;
set_markers_simulink(markers_shoulder, markers_forearm, markers_hand);

% Model IC
%q0_model = q0(1, :);
set_param('master_thesis_simulink/System/Human arm/RightShoulder_joint/jRightShoulder_rotx', 'PositionTargetValue', 'q0_model(1)');
set_param('master_thesis_simulink/System/Human arm/RightShoulder_joint/jRightShoulder_roty', 'PositionTargetValue', 'q0_model(2)');
set_param('master_thesis_simulink/System/Human arm/RightShoulder_joint/jRightShoulder_rotz', 'PositionTargetValue', 'q0_model(3)');
set_param('master_thesis_simulink/System/Human arm/Elbow_joint/jRightElbow_rotx', 'PositionTargetValue', 'q0_model(4)');
set_param('master_thesis_simulink/System/Human arm/Wrist_joint/jRightWrist_rotx', 'PositionTargetValue', 'q0_model(5)');
set_param('master_thesis_simulink/System/Human arm/Wrist_joint/jRightWrist_roty', 'PositionTargetValue', 'q0_model(6)');
set_param('master_thesis_simulink/System/Human arm/Wrist_joint/jRightWrist_rotz', 'PositionTargetValue', 'q0_model(7)');

%% Add path 'C code/C_code_folder' <-- target_folder
set_Mex(C_code_folder);

%% Select {Free fall motion, Precompiuted trajectory}
switch simModel_flag

    % FREE FALL MOTION
    case 'free'

        %% Joints' trajectory
        % q1
        set_param('master_thesis_simulink/System/Human arm/RightShoulder_joint/jRightShoulder_rotx', 'TorqueActuationMode', 'NoTorque', 'MotionActuationMode', 'ComputedMotion');
        
        % q2
        set_param('master_thesis_simulink/System/Human arm/RightShoulder_joint/jRightShoulder_roty', 'TorqueActuationMode', 'NoTorque', 'MotionActuationMode', 'ComputedMotion');
        
        % q3
        set_param('master_thesis_simulink/System/Human arm/RightShoulder_joint/jRightShoulder_rotz', 'TorqueActuationMode', 'NoTorque', 'MotionActuationMode', 'ComputedMotion');
        
        % q4
        set_param('master_thesis_simulink/System/Human arm/Elbow_joint/jRightElbow_rotx', 'TorqueActuationMode', 'NoTorque', 'MotionActuationMode', 'ComputedMotion');

        % q5
        set_param('master_thesis_simulink/System/Human arm/Wrist_joint/jRightWrist_rotx', 'TorqueActuationMode', 'NoTorque', 'MotionActuationMode', 'ComputedMotion');

        % q6
        set_param('master_thesis_simulink/System/Human arm/Wrist_joint/jRightWrist_roty', 'TorqueActuationMode', 'NoTorque', 'MotionActuationMode', 'ComputedMotion');
        
        % q7
        set_param('master_thesis_simulink/System/Human arm/Wrist_joint/jRightWrist_rotz', 'TorqueActuationMode', 'NoTorque', 'MotionActuationMode', 'ComputedMotion');
    
    case 'precomputed'

        disp('aaa')
        % Joints' trajectory
        % q1
        set_param('master_thesis_simulink/System/Human arm/RightShoulder_joint/jRightShoulder_rotx', 'TorqueActuationMode', 'ComputedTorque', 'MotionActuationMode', 'InputMotion');
        add_line('master_thesis_simulink/System/Human arm/RightShoulder_joint', get_param('master_thesis_simulink/System/Human arm/RightShoulder_joint/c1', 'PortHandles').RConn, get_param('master_thesis_simulink/System/Human arm/RightShoulder_joint/jRightShoulder_rotx', 'PortHandles').LConn(2));
        disp('aaa')
        % q2
        set_param('master_thesis_simulink/System/Human arm/RightShoulder_joint/jRightShoulder_roty', 'TorqueActuationMode', 'ComputedTorque', 'MotionActuationMode', 'InputMotion');
        add_line('master_thesis_simulink/System/Human arm/RightShoulder_joint', get_param('master_thesis_simulink/System/Human arm/RightShoulder_joint/c2', 'PortHandles').RConn, get_param('master_thesis_simulink/System/Human arm/RightShoulder_joint/jRightShoulder_roty', 'PortHandles').LConn(2));
        
        % q3
        set_param('master_thesis_simulink/System/Human arm/RightShoulder_joint/jRightShoulder_rotz', 'TorqueActuationMode', 'ComputedTorque', 'MotionActuationMode', 'InputMotion');
        add_line('master_thesis_simulink/System/Human arm/RightShoulder_joint', get_param('master_thesis_simulink/System/Human arm/RightShoulder_joint/c3', 'PortHandles').RConn, get_param('master_thesis_simulink/System/Human arm/RightShoulder_joint/jRightShoulder_rotz', 'PortHandles').LConn(2));
        
        % q4
        set_param('master_thesis_simulink/System/Human arm/Elbow_joint/jRightElbow_rotx', 'TorqueActuationMode', 'ComputedTorque', 'MotionActuationMode', 'InputMotion');
        add_line('master_thesis_simulink/System/Human arm/Elbow_joint', get_param('master_thesis_simulink/System/Human arm/Elbow_joint/c1', 'PortHandles').RConn, get_param('master_thesis_simulink/System/Human arm/Elbow_joint/jRightElbow_rotx', 'PortHandles').LConn(2));
        
        % q5
        set_param('master_thesis_simulink/System/Human arm/Wrist_joint/jRightWrist_rotx', 'TorqueActuationMode', 'ComputedTorque', 'MotionActuationMode', 'InputMotion');
        add_line('master_thesis_simulink/System/Human arm/Wrist_joint', get_param('master_thesis_simulink/System/Human arm/Wrist_joint/c1', 'PortHandles').RConn, get_param('master_thesis_simulink/System/Human arm/Wrist_joint/jRightWrist_rotx', 'PortHandles').LConn(2));
        
        % q6
        set_param('master_thesis_simulink/System/Human arm/Wrist_joint/jRightWrist_roty', 'TorqueActuationMode', 'ComputedTorque', 'MotionActuationMode', 'InputMotion');
        add_line('master_thesis_simulink/System/Human arm/Wrist_joint', get_param('master_thesis_simulink/System/Human arm/Wrist_joint/c2', 'PortHandles').RConn, get_param('master_thesis_simulink/System/Human arm/Wrist_joint/jRightWrist_roty', 'PortHandles').LConn(2));
        
        % q7
        set_param('master_thesis_simulink/System/Human arm/Wrist_joint/jRightWrist_rotz', 'TorqueActuationMode', 'ComputedTorque', 'MotionActuationMode', 'InputMotion');
        add_line('master_thesis_simulink/System/Human arm/Wrist_joint', get_param('master_thesis_simulink/System/Human arm/Wrist_joint/c3', 'PortHandles').RConn, get_param('master_thesis_simulink/System/Human arm/Wrist_joint/jRightWrist_rotz', 'PortHandles').LConn(2));
end

%% Select {LS, KF, EKF, NLDM}

% selected --> -1 : No method has been selected
%               0 : LS
%               1 : KF
%               2 : EKF
%               3 : NLDM
%               4 : LS && KF
%               5 : LS && EKF
%               6 : KF && EKF
%               7 : LS && KF && EKF
selected = -1;
number_methods = length(method_flag);

switch number_methods

    %% 1 ELEMENT
    case 1

        % LS
        if (strcmp(method_flag(number_methods), "LS") == 1)

            selected = 0;
        
        % KF
        else if (strcmp(method_flag(number_methods), "KF") == 1)

            selected = 1;
        
        % EKF
        else if (strcmp(method_flag(number_methods), "EKF") == 1)

            selected = 2;

        % NLDM
        else if (strcmp(method_flag(number_methods), "NLDM") == 1)

            selected = 3;
        end
        end
        end
        end

    %% 2 ELEMENTS
    case 2

        % LS && KF
        if ((strcmp(method_flag(number_methods-1), "LS") == 1 && strcmp(method_flag(number_methods), "KF") == 1) || (strcmp(method_flag(number_methods), "LS") == 1 && strcmp(method_flag(number_methods-1), "KF") == 1))
    
            selected = 4;
        
        % LS && EKF
        else if ((strcmp(method_flag(number_methods-1), "LS") == 1 && strcmp(method_flag(number_methods), "EKF") == 1) || (strcmp(method_flag(number_methods), "LS") == 1 && strcmp(method_flag(number_methods-1), "EKF") == 1))
    
            selected = 5;
        
        % KF && EKF
        else if ((strcmp(method_flag(number_methods-1), "KF") == 1 && strcmp(method_flag(number_methods), "EKF") == 1) || (strcmp(method_flag(number_methods), "KF") == 1 && strcmp(method_flag(number_methods-1), "EKF") == 1))
    
            selected = 6;
        end
        end
        end

    %% 3 ELEMENTS
    case 3

        selected = 7;
end

switch selected

    %% LS
    case 0

        %% Comment blocks and set parameters 

        % Uncomment LS block 
        set_param('master_thesis_simulink/System/LS', 'commented', 'off');
        
        % Comment EKF block
        set_param('master_thesis_simulink/System/EKF', 'commented', 'on');
        
        % Comment KF vs OBS block
        set_param('master_thesis_simulink/System/KF vs OBS', 'commented', 'on');

        % Least Squares IC
        set_param('master_thesis_simulink/System/LS/Integrator', 'InitialCondition', 'q0_LS');

        %% Simulation

        output = sim("master_thesis_simulink.slx");
        %% Plot results

        legend_name = cell(n, 1);
        fig = figure;
        for i = 1 : n

            legend_name{i} = strcat('q_', num2str(i), ' - q_', num2str(i), '^{LS}');
            set(fig, 'position', [10, 10, 1300, 900]);
            subplot(3,3,i);
            plot(output.q.time, rad2deg(output.q.signals.values(:, i) - squeeze(output.q_LS.signals.values(i, :))'));
            legend(legend_name{i}, 'Location', 'southeast');
        end

    %% KF
    case 1
        
        % Global variables
        global noise;
        global kf;

        % Number of markers
        m = size(markers.shoulder_variables, 1) + size(markers.forearm_variables, 1) + size(markers.hand_variables, 1);

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
            stairs(output.qk_kf.time, rad2deg(squeeze(output.qk_kf.signals.values(i, :))'), 'LineWidth', 2);
            legend(q, qkf, 'Location', 'southeast', 'Interpreter', 'latex');
        end

    %% EKF
    case 2

        %% Comment blocks and set parameters 

        % Uncomment EKF block
        set_param('master_thesis_simulink/System/EKF', 'commented', 'off');

        % Uncomment Extended Kalman Filter
        set_param('master_thesis_simulink/System/EKF/Extended Kalman Filter', 'commented', 'off');

        % Comment KF vs OBS block
        set_param('master_thesis_simulink/System/KF vs OBS', 'commented', 'on');

        % Comment LS block 
        set_param('master_thesis_simulink/System/LS', 'commented', 'on');

        %% Simulation
        output = sim("master_thesis_simulink.slx");
        
        %% Plot EKF estimate vs CONTINUOS joints' variables
        fig = figure;
        sgtitle("EKF vs True");
        
        for i = 1 : n
        
            q = strcat('$q_', num2str(i), '$');
            qekf = strcat('$\hat{q}^{EKF}_', num2str(i), '$');
            set(fig, 'position', [10, 10, 1300, 900]);
            subplot(3,3,i);
            plot(output.q.time, rad2deg(output.q.signals.values(:, i)), 'LineWidth', 2);
            hold on;
            stairs(output.qk_ekf.time, rad2deg(squeeze(output.qk_ekf.signals.values(i, :))'), 'LineWidth', 2);
            legend(q, qekf, 'Location', 'southeast', 'Interpreter', 'latex');
        end

    %% NLDM
    case 3

        % Uncomment EKF block
        set_param('master_thesis_simulink/System/EKF', 'commented', 'off');
        
        % Comment Extended Kalman Filter
        set_param('master_thesis_simulink/System/EKF/Extended Kalman Filter', 'commented', 'on');
        
        % Comment LS block 
        set_param('master_thesis_simulink/System/LS', 'commented', 'on');
        
        % Comment KF vs OBS block
        set_param('master_thesis_simulink/System/KF vs OBS', 'commented', 'on');
        
        %% Simulation
        output = sim("master_thesis_simulink.slx");
        
        %% Plot Discrete Non Linear Model vs true joints' variables (q vs qk)
        fig = figure;
        sgtitle("NLDM (10ms) vs CONTINUOS NL");
        for i = 1 : n
        
            q = strcat('q_', num2str(i));
            qk = strcat('q^k_', num2str(i));
            set(fig, 'position', [10, 10, 1300, 900]);
            subplot(3,3,i);
            plot(output.q.time, rad2deg(output.q.signals.values(:, i)));
            hold on;
            stairs(output.qk.time, rad2deg(squeeze(output.qk.signals.values(i, :))'));
            legend(q, qk, 'Location', 'southeast');
        end

    %% LS && KF
    case 4

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
            stairs(output.out.time, rad2deg(squeeze(output.out.signals(4).values(i, :))'), 'LineWidth', 2);
            legend(q, qls, qekf, 'Location', 'southeast', 'Interpreter', 'latex');
        end

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

end


% If 'precomputed' --> delete connections
if (strcmp(simModel_flag, 'precomputed') == 1)

    delete_line('master_thesis_simulink/System/Human arm/RightShoulder_joint', get_param('master_thesis_simulink/System/Human arm/RightShoulder_joint/c1', 'PortHandles').RConn, get_param('master_thesis_simulink/System/Human arm/RightShoulder_joint/jRightShoulder_rotx', 'PortHandles').LConn(2));
    delete_line('master_thesis_simulink/System/Human arm/RightShoulder_joint', get_param('master_thesis_simulink/System/Human arm/RightShoulder_joint/c2', 'PortHandles').RConn, get_param('master_thesis_simulink/System/Human arm/RightShoulder_joint/jRightShoulder_roty', 'PortHandles').LConn(2));
    delete_line('master_thesis_simulink/System/Human arm/RightShoulder_joint', get_param('master_thesis_simulink/System/Human arm/RightShoulder_joint/c3', 'PortHandles').RConn, get_param('master_thesis_simulink/System/Human arm/RightShoulder_joint/jRightShoulder_rotz', 'PortHandles').LConn(2));
    delete_line('master_thesis_simulink/System/Human arm/Elbow_joint', get_param('master_thesis_simulink/System/Human arm/Elbow_joint/c1', 'PortHandles').RConn, get_param('master_thesis_simulink/System/Human arm/Elbow_joint/jRightElbow_rotx', 'PortHandles').LConn(2));

    delete_line('master_thesis_simulink/System/Human arm/Wrist_joint', get_param('master_thesis_simulink/System/Human arm/Wrist_joint/c1', 'PortHandles').RConn, get_param('master_thesis_simulink/System/Human arm/Wrist_joint/jRightWrist_rotx', 'PortHandles').LConn(2));
    delete_line('master_thesis_simulink/System/Human arm/Wrist_joint', get_param('master_thesis_simulink/System/Human arm/Wrist_joint/c2', 'PortHandles').RConn, get_param('master_thesis_simulink/System/Human arm/Wrist_joint/jRightWrist_roty', 'PortHandles').LConn(2));
    delete_line('master_thesis_simulink/System/Human arm/Wrist_joint', get_param('master_thesis_simulink/System/Human arm/Wrist_joint/c3', 'PortHandles').RConn, get_param('master_thesis_simulink/System/Human arm/Wrist_joint/jRightWrist_rotz', 'PortHandles').LConn(2));
end


end