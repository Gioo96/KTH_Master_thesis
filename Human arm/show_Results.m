% function show_Results(q0_LS, q0_EKF, P0_EKF, R_EKF, Q_EKF, directory, markers_num, sim_num)
function show_Results(directory, markers_num, sim_num)

 
%% Run Human arm aparameters
run('human_arm_parameters_exp.m');

global kf ekf noise;
global p_0p v_0p;
global markers_shoulder markers_forearm markers_hand;
% ls.q0 = q0_LS;
% ekf.q0 = q0_EKF;
% ekf.P0 = P0_EKF;

%% Comment/Uncomment blocks

% Uncomment System and Show Result block
set_param('master_thesis_simulink/System', 'commented', 'off ');
set_param('master_thesis_simulink/System/Show Results', 'commented', 'off ');

% Comment Ros2Matlab and Experiments block
set_param('master_thesis_simulink/Ros2Matlab', 'commented', 'on ');
set_param('master_thesis_simulink/Experiments', 'commented', 'on ');

markers_shoulder_num = str2double(directory(2));
markers_forearm_num = str2double(directory(5));
markers_hand_num = str2double(directory(8));
fprintf('Set of markers: SHOULDER: %d, FOREARM: %d, HAND: %d\n', markers_shoulder_num, markers_forearm_num, markers_hand_num);

% Load corresponding set of markers
markers_shoulder = load(strcat('Simulations/', directory, '/Markers_Pos_', num2str(markers_num), '/markers_shoulder.mat'));
markers_shoulder = markers_shoulder.markers_shoulder;
markers_forearm = load(strcat('Simulations/', directory, '/Markers_Pos_', num2str(markers_num), '/markers_forearm.mat'));
markers_forearm = markers_forearm.markers_forearm;
markers_hand = load(strcat('Simulations/', directory, '/Markers_Pos_', num2str(markers_num), '/markers_hand.mat'));
markers_hand = markers_hand.markers_hand;

pause(0.1);
if (markers_shoulder_num== 1 && markers_forearm_num == 1 && markers_hand_num == 2)

    fprintf('Markers SHOULDER: %6.4f\n                 %6.4f\n                 %6.4f\n', markers_shoulder');
    fprintf('Markers FOREARM: %6.4f\n                %6.4f\n                 %6.4f\n', markers_forearm');
    fprintf('Markers HAND: %6.4f  %6.4f\n              %6.4f %6.4f\n               %6.4f  %6.4f\n', markers_hand');
end

%% Load noise

noise = load(strcat('Simulations/', directory, '/noise.mat'));
noise = noise.noise;

%% EKF

% Noise
if isempty(ekf.R)

    ekf.R = noise.R;
end

if isempty(ekf.Q)

    ekf.Q = noise.Q;
end

%% KF

% Noise
if isempty(kf.R)

    kf.R = noise.R;
end

if isempty(kf.Q)

    kf.Q = noise.Q;
end

% F matrix --> df/dq evaluated at the equilibrium point
kf.F = full(f_Fekf_mex(kf.q_eq, markers_shoulder, markers_forearm, markers_hand, kf.v_0p_eq));
% G matrix --> df/du evaluated at the equilibrium point
kf.G = full(f_Gekf_mex(kf.q_eq, markers_shoulder, markers_forearm, markers_hand, kf.v_0p_eq));

% H matrix --> dPhi/dq (Jacobian J) evaluated at the equilibrium point
kf.H = full(f_J_mex(kf.q_eq, markers_shoulder, markers_forearm, markers_hand));
% J matrix --> dPhi/du = 0 
kf.J = zeros(3*m, 3*m);

% p_eq
kf.p_0p_eq = full(f_Phi_mex(kf.q_eq, markers_shoulder, markers_forearm, markers_hand));

% Compute Pk, Pk_, Kk offline

% List of Pk for k = 1, .., N
kf.Pk = [kf.P0];
% List of Pk_ for k = 1, .., N
kf.Pk_ = [];
% List of Kk for k = 1, .., N
kf.Kk = [];

% Compute Pk, Pk_, Kk offline
for k = 1 : number_samples

    Pk_ = kf.F * kf.Pk(1:n, n*(k-1)+1:n*(k-1)+n) * kf.F' + kf.Q;
    Kk = Pk_ * kf.H' / (kf.H * Pk_ * kf.H' + kf.R);
    Pk = (eye(n) - Kk * kf.H) * Pk_ * (eye(size(kf.F, 1)) - Kk * kf.H)' + Kk * kf.R * Kk';
    kf.Pk_ = [kf.Pk_, Pk_];
    kf.Kk = [kf.Kk, Kk];
    kf.Pk = [kf.Pk, Pk];
end
%% Loop through each simulation
fprintf('Sim n. %d\n', sim_num);

% Load corresponding measurements
p_0p = load(strcat('Simulations/', directory, '/Markers_Pos_', num2str(markers_num), '/Sim_', num2str(sim_num), '/p_0p.mat'));
p_0p = p_0p.p_0p;
v_0p = load(strcat('Simulations/', directory, '/Markers_Pos_', num2str(markers_num), '/Sim_', num2str(sim_num), '/v_0p.mat'));
v_0p = v_0p.v_0p;

%% Run Simulation 
pause(0.1);
output = sim('master_thesis_simulink.slx');


%% p_0' and v_0' just once 
% plot p_0
fig = figure;
set(fig, 'position', [10, 10, 1300, 900]);
sgtitle('p^0');
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
fig = figure;
set(fig, 'position', [10, 10, 1300, 900]);
sgtitle('v^0');
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

%% Plot results LS vs KF vs EKF

% LS vs KF vs EKF Estimation
fig = figure;
sgtitle('LS vs KF vs EKF Estimation');
for k = 1 : n

    title_name = strcat('$\hat{q}_', num2str(k), '$'); 
    set(fig, 'position', [10, 10, 1300, 900]);
    subplot(3,3,k);
    plot(output.q_LS.time, rad2deg(output.q_LS.signals.values(k, :)));
    hold on;
    plot(output.q_KF.time, rad2deg(output.q_KF.signals.values(k, :)));
    hold on;
    plot(output.q_EKF.time, rad2deg(output.q_EKF.signals.values(:, k)));
    xlabel('Time [s]');
    ylabel('Joint variable [deg]');
    legend('LS', 'KF', 'EKF');
    title(title_name, Interpreter = "latex");
end

% p_0 - Phi(q_LS) vs p_0 - Phi(q_KF) vs p_0 - Phi(q_EKF)
fig = figure;
set(fig, 'position', [10, 10, 1300, 900]);
sgtitle('p^0-\Phi(q_{LS}) vs p^0-\Phi(q_{KF}) vs p^0-\Phi(q_{EKF})');
for k = 1 : m

    title_name_x = strcat('$p', num2str(k), '_{x}^0$ - $\hat{p', num2str(k), '}_{x}$');
    title_name_y = strcat('$p', num2str(k), '_{y}^0$ - $\hat{p', num2str(k), '}_{y}$');
    title_name_z = strcat('$p', num2str(k), '_{z}^0$ - $\hat{p', num2str(k), '}_{z}$');
    
    subplot(m, 3, (k-1)*3+1);
    plot(output.error_FK__LS.time, output.error_FK__LS.signals.values((k-1)*3+1, :));
    hold on;
    plot(output.error_FK__KF.time, output.error_FK__KF.signals.values((k-1)*3+1, :));
    hold on;
    plot(output.error_FK__EKF.time, output.error_FK__EKF.signals.values((k-1)*3+1, :));
    legend('LS', 'KF', 'EKF');
    title(title_name_x, Interpreter = "latex");
    xlabel('Time [s]');
    ylabel('Error [m]');
    subplot(m, 3, (k-1)*3+2);
    plot(output.error_FK__LS.time, output.error_FK__LS.signals.values((k-1)*3+2, :));
    hold on;
    plot(output.error_FK__KF.time, output.error_FK__KF.signals.values((k-1)*3+2, :));
    hold on;
    plot(output.error_FK__EKF.time, output.error_FK__EKF.signals.values((k-1)*3+2, :));
    legend('LS', 'KF', 'EKF');
    title(title_name_y, Interpreter = "latex");
    xlabel('Time [s]');
    ylabel('Error [m]');
    subplot(m, 3, (k-1)*3+3);
    plot(output.error_FK__LS.time, output.error_FK__LS.signals.values((k-1)*3+3, :));
    hold on;
    plot(output.error_FK__KF.time, output.error_FK__KF.signals.values((k-1)*3+3, :));
    hold on;
    plot(output.error_FK__EKF.time, output.error_FK__EKF.signals.values((k-1)*3+3, :));
    legend('LS', 'KF', 'EKF');
    title(title_name_z, Interpreter = "latex");
    xlabel('Time [s]');
    ylabel('Error [m]');
end

% ||p_0 - Phi(q_LS)|| vs ||p_0 - Phi(q_KF)|| vs ||p_0 - Phi(q_EKF)||
norm_error_LS = vecnorm(output.error_FK__LS.signals.values);
norm_error_KF = vecnorm(output.error_FK__KF.signals.values);
norm_error_EKF = vecnorm(output.error_FK__EKF.signals.values);
title_name = '$\|p^0 - \Phi(\hat{q}^{LS})\|$ vs $\|p^0 - \Phi(\hat{q}^{KF})\|$ vs $\|p^0 - \Phi(\hat{q}^{EKF})\|$';
fig = figure;
set(fig, 'position', [10, 10, 1300, 900]);
plot(output.error_FK__LS.time, norm_error_LS(:, :));
hold on;
plot(output.error_FK__KF.time, norm_error_KF(:, :));
hold on;
plot(output.error_FK__EKF.time, norm_error_EKF(:, :));
xlabel('Time [s]');
ylabel('norm error [m]');
legend('LS', 'KF', 'EKF');
title(title_name, Interpreter = "latex");

end