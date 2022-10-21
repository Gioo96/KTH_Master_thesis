% Run human_arm_parameters
run('human_arm_parameters.m');

%% Simulink

% q0 model
q0_model = zeros(7, 1);

% Simulation of 20s
simulink.time = 100;

%% Initialize ROS

rosinit;

%% Callback --> start simulation when M0_W(1) > 0

name = 'Super_marker_0';

for i = 1:1

    % Comment System
    set_param('master_thesis_simulink/System', 'commented', 'on');

    % Comment Experiments
    set_param('master_thesis_simulink/Experiments', 'commented', 'on');
    
    % Uncomment Ros2Matlab
    set_param('master_thesis_simulink/Ros2Matlab', 'commented', 'off');

    % Define object of the class
    tuning = callback_class(name, shoulder_num, forearm_num, hand_num);
    [~, out] = tuning.simulation_tuningParameters;

    % Plot data
    figure;
    sgtitle(strcat('tSh^{M_0} : Simulation ', num2str(i)));
    subplot(1,3,1);
    scatter(out.tSh_0.time, out.tSh_0.signals.values(1, :), "filled");
    legend('x', Location='southeast');
    subplot(1,3,2);
    scatter(out.tSh_0.time, out.tSh_0.signals.values(2, :), 'filled');
    legend('y', Location='southeast');
    subplot(1,3,3);
    scatter(out.tSh_0.time, out.tSh_0.signals.values(3, :), 'filled');
    legend('z', Location='southeast');
 
    % Compute average values of translactional vector tSh_0 
    sum_tSh_0 = zeros(3, 1);
    for j = 1:length(out.tSh_0.time)

        sum_tSh_0 = sum_tSh_0 +  out.tSh_0.signals.values(:, j);
    end
    tSh_0_est = sum_tSh_0 / length(out.tSh_0.time);
    disp('tSh_0 estimate');
    disp(tSh_0_est);
end


% Shutdown ROS
rosshutdown;


