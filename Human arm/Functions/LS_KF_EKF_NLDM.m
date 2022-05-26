function LS_KF_EKF_NLDM(q0, method_flag, simModel_flag, C_code_folder)

%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

% Inputs
% -- q0                  : Matrix containing the IC --> ROW 1 : Simulink Model IC
%                                                       ROW 2 : LS IC
% -- sampling_time       : Sampling time
% -- method_flag         : 'LS' --> Least Squares is selected
%                        : 'KF' --> Kalman Filter is selected
%                        : 'EKF' --> Extended Kalman Filter is selected
%                        : 'NLDM' --> Non Linear Discrete Model is selected
% -- simModel_flag       : 'free'         --> free fall motion is simulated
%                        : 'precompiuted' --> precompiuted trajectory is simulated
% -- C_code_folder       : Folder containing the mex functions needed, depending on the location of the markers
%                          --> 'S1_F1_H1'
%                          --> 'S4_F3_H2'

% Uncomment System
set_param('master_thesis_simulink/System', 'commented', 'off');

% Comment Ros2Matlab
set_param('master_thesis_simulink/Ros2Matlab', 'commented', 'on');

% Global variables
global q0_model;
global q0_LS;
global q0_NLDM;

% Number of DoF
n = 7;

% Model IC
q0_model = q0(1, :);
set_param('master_thesis_simulink/System/Human arm/RightShoulder_joint/jRightShoulder_rotx', 'PositionTargetValue', 'q0_model(1)');
set_param('master_thesis_simulink/System/Human arm/RightShoulder_joint/jRightShoulder_roty', 'PositionTargetValue', 'q0_model(2)');
set_param('master_thesis_simulink/System/Human arm/RightShoulder_joint/jRightShoulder_rotz', 'PositionTargetValue', 'q0_model(3)');
set_param('master_thesis_simulink/System/Human arm/Elbow_joint/jRightElbow_rotx', 'PositionTargetValue', 'q0_model(4)');
set_param('master_thesis_simulink/System/Human arm/Wrist_joint/jRightWrist_rotx', 'PositionTargetValue', 'q0_model(5)');
set_param('master_thesis_simulink/System/Human arm/Wrist_joint/jRightWrist_roty', 'PositionTargetValue', 'q0_model(6)');
set_param('master_thesis_simulink/System/Human arm/Wrist_joint/jRightWrist_rotz', 'PositionTargetValue', 'q0_model(7)');

% NLDM IC
q0_NLDM = q0_model;

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
    
    case 'precompiuted'

        % Joints' trajectory
        % q1
        set_param('master_thesis_simulink/System/Human arm/RightShoulder_joint/jRightShoulder_rotx', 'TorqueActuationMode', 'ComputedTorque', 'MotionActuationMode', 'InputMotion');
        add_line('master_thesis_simulink/System/Human arm/RightShoulder_joint', get_param('master_thesis_simulink/System/Human arm/RightShoulder_joint/c1', 'PortHandles').RConn, get_param('master_thesis_simulink/System/Human arm/RightShoulder_joint/jRightShoulder_rotx', 'PortHandles').LConn(2));
        
        % q2
        set_param('master_thesis_simulink/System/Human arm/RightShoulder_joint/jRightShoulder_roty', 'TorqueActuationMode', 'ComputedTorque', 'MotionActuationMode', 'InputMotion');
        add_line('master_thesis_simulink/System/Human arm/RightShoulder_joint', get_param('master_thesis_simulink/System/Human arm/RightShoulder_joint/c2', 'PortHandles').RConn, get_param('master_thesis_simulink/System/Human arm/RightShoulder_joint/jRightShoulder_roty', 'PortHandles').LConn(2));
        
        % q3
        set_param('master_thesis_simulink/System/Human arm/RightShoulder_joint/jRightShoulder_rotz', 'TorqueActuationMode', 'ComputedTorque', 'MotionActuationMode', 'InputMotion');
        add_line('master_thesis_simulink/System/Human arm/RightShoulder_joint', get_param('master_thesis_simulink/System/Human arm/RightShoulder_joint/c3', 'PortHandles').RConn, get_param('master_thesis_simulink/System/Human arm/RightShoulder_joint/jRightShoulder_rotz', 'PortHandles').LConn(2));
        
        % q4
        set_param('master_thesis_simulink/System/Human arm/Elbow_joint/jRightElbow_rotx', 'TorqueActuationMode', 'ComputedTorque', 'MotionActuationMode', 'InputMotion');
        add_line('master_thesis_simulink/System/Human arm/Elbow_joint', get_param('master_thesis_simulink/System/Human arm/Elbow_joint/c1', 'PortHandles').RConn, get_param('master_thesis_simulink/System/Human arm/Elbow_joint/jRightElbow_rotx', 'PortHandles').LConn(2));
        
end

%% Select {LS, KF, EKF}
switch method_flag

    % LEAST SQUARES
    case 'LS'

        %% Comment blocks and set parameters 

        % Uncomment LS block 
        set_param('master_thesis_simulink/System/LS', 'commented', 'off');
        
        % Comment EKF block
        set_param('master_thesis_simulink/System/EKF', 'commented', 'on');
        
        % Comment KF vs OBS block
        set_param('master_thesis_simulink/System/KF vs OBS', 'commented', 'on');

        % Least Squares IC
        q0_LS = q0(2, :)';
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

    % NLDM
    case 'NLDM'

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
end

% If 'precompiuted' --> delete connections
if (strcmp(simModel_flag, 'precompiuted') == 1)

    delete_line('master_thesis_simulink/System/Human arm/RightShoulder_joint', get_param('master_thesis_simulink/System/Human arm/RightShoulder_joint/c1', 'PortHandles').RConn, get_param('master_thesis_simulink/System/Human arm/RightShoulder_joint/jRightShoulder_rotx', 'PortHandles').LConn(2));
    delete_line('master_thesis_simulink/System/Human arm/RightShoulder_joint', get_param('master_thesis_simulink/System/Human arm/RightShoulder_joint/c2', 'PortHandles').RConn, get_param('master_thesis_simulink/System/Human arm/RightShoulder_joint/jRightShoulder_roty', 'PortHandles').LConn(2));
    delete_line('master_thesis_simulink/System/Human arm/RightShoulder_joint', get_param('master_thesis_simulink/System/Human arm/RightShoulder_joint/c3', 'PortHandles').RConn, get_param('master_thesis_simulink/System/Human arm/RightShoulder_joint/jRightShoulder_rotz', 'PortHandles').LConn(2));
    delete_line('master_thesis_simulink/System/Human arm/Elbow_joint', get_param('master_thesis_simulink/System/Human arm/Elbow_joint/c1', 'PortHandles').RConn, get_param('master_thesis_simulink/System/Human arm/Elbow_joint/jRightElbow_rotx', 'PortHandles').LConn(2));
end


end