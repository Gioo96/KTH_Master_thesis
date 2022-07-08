%% Run human_arm_parameters
run('human_arm_parameters_exp.m');

%% Simulink time
simulink.time = 10;

% Uncomment Ros2Matlab, Experiments
set_param(strcat('master_thesis_simulink/Ros2Matlab/', directory), 'commented', 'off');
set_param(strcat('master_thesis_simulink/Experiments/', directory), 'commented', 'off');
set_param(strcat('master_thesis_simulink/Experiments/', directory, '/Experiments'), 'commented', 'on');
set_param(strcat('master_thesis_simulink/Experiments/', directory, '/Model validation'), 'commented', 'off');

% Comment System
set_param('master_thesis_simulink/System', 'commented', 'on');

% Simulate Model
out = sim('master_thesis_simulink.slx');
markers_Bases = out.markers_Bases.signals.values(:, end);
markers_shoulder = markers_Bases(1:3);
markers_forearm = markers_Bases(4:6);
markers_hand = [markers_Bases(7:9) markers_Bases(10:12)];

% Plot p_0 vs p_FK
fig = figure;
set(fig, 'position', [10, 10, 1500, 500]);
sgtitle('p^0 vs p_{FK}^0');

for i = 1:m

    subplot(2,2,i);
    % p^0
    plot(out.p0_vs_pFK.time, out.p0_vs_pFK.signals.values((i-1)*6+1, :)); % x
    hold on;
    plot(out.p0_vs_pFK.time, out.p0_vs_pFK.signals.values((i-1)*6+2, :)); % y
    hold on;
    plot(out.p0_vs_pFK.time, out.p0_vs_pFK.signals.values((i-1)*6+3, :)); % z
    hold on;
    % p_FK
    plot(out.p0_vs_pFK.time, out.p0_vs_pFK.signals.values((i-1)*6+4, :)); % x
    hold on;
    plot(out.p0_vs_pFK.time, out.p0_vs_pFK.signals.values((i-1)*6+5, :)); % y
    hold on;
    plot(out.p0_vs_pFK.time, out.p0_vs_pFK.signals.values((i-1)*6+6, :)); % z
    legend('p^0:x', 'p^0:y', 'p^0:z', 'p^0_{FK}:x', 'p^0_{FK}:y', 'p^0_{FK}:z', Location='west');
    title(strcat('Marker', num2str(i)));

end    