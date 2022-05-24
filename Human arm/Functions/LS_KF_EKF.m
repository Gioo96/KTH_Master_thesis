function LS_KF_EKF(method_flag, q0, simModel_flag)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

% Inputs
% -- method_flag         : 'LS' --> Least Squares is selected
%                        : 'KF' --> Kalman Filter is selected
%                        : 'EKF' --> Extended Kalman Filter is selected
% -- noise               : Struct containing the noise parameters
% -- q0                  : Matrix containing the IC --> ROW 1 : Simulink Model IC
%                                                       ROW 2 : LS IC
% -- simModel_flag       : 'free'         --> free fall motion is simulated
%                        : 'precompiuted' --> precompiuted trajectory is simulated

% Uncomment System
set_param('master_thesis_simulink/System', 'commented', 'off');

% Comment Ros2Matlab
set_param('master_thesis_simulink/Ros2Matlab', 'commented', 'on');

% Global variables
global q0_model;
global q0_LS;

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

        % If 'precompiuted' --> delete connections
        output = sim("master_thesis_simulink.slx");

        if (strcmp(simModel_flag, 'precompiuted') == 1)

            delete_line('master_thesis_simulink/System/Human arm/RightShoulder_joint', get_param('master_thesis_simulink/System/Human arm/RightShoulder_joint/c1', 'PortHandles').RConn, get_param('master_thesis_simulink/System/Human arm/RightShoulder_joint/jRightShoulder_rotx', 'PortHandles').LConn(2));
            delete_line('master_thesis_simulink/System/Human arm/RightShoulder_joint', get_param('master_thesis_simulink/System/Human arm/RightShoulder_joint/c2', 'PortHandles').RConn, get_param('master_thesis_simulink/System/Human arm/RightShoulder_joint/jRightShoulder_roty', 'PortHandles').LConn(2));
            delete_line('master_thesis_simulink/System/Human arm/RightShoulder_joint', get_param('master_thesis_simulink/System/Human arm/RightShoulder_joint/c3', 'PortHandles').RConn, get_param('master_thesis_simulink/System/Human arm/RightShoulder_joint/jRightShoulder_rotz', 'PortHandles').LConn(2));
            delete_line('master_thesis_simulink/System/Human arm/Elbow_joint', get_param('master_thesis_simulink/System/Human arm/Elbow_joint/c1', 'PortHandles').RConn, get_param('master_thesis_simulink/System/Human arm/Elbow_joint/jRightElbow_rotx', 'PortHandles').LConn(2));
        end

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

end

end