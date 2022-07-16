%% Run human_arm_parameters
run('human_arm_parameters_exp.m');

%% Simulink time
simulink.time = 10;

%% Comment / Uncomment blocks in Simulink
% Uncomment Ros2Matlab, Experiments
% Ros2Matlab
set_param('master_thesis_simulink/Ros2Matlab', 'commented', 'off');
set_param(strcat('master_thesis_simulink/Ros2Matlab/', directory), 'commented', 'off');
all_Subsystem_Ros2Matlab = find_system('master_thesis_simulink/Ros2Matlab', 'SearchDepth', '1');
all_Subsystem_Ros2Matlab_num = length(all_Subsystem_Ros2Matlab);
for i = 1:all_Subsystem_Ros2Matlab_num

    subsystem = all_Subsystem_Ros2Matlab(i);
    subsystem_name = subsystem{1};
    if (length(subsystem_name) > 34)
        if (strcmp(subsystem_name(35:42), directory) ~= 1)
    
            set_param(strcat('master_thesis_simulink/Ros2Matlab/', subsystem_name(35:42)), 'commented', 'on');
        end
    end
end

% Experiments
set_param('master_thesis_simulink/Experiments', 'commented', 'off');
set_param(strcat('master_thesis_simulink/Experiments/', directory), 'commented', 'off');
all_Subsystem_Experiments = find_system('master_thesis_simulink/Experiments', 'SearchDepth', '1');
all_Subsystem_Experiments_num = length(all_Subsystem_Experiments);
for i = 1:all_Subsystem_Experiments_num

    subsystem = all_Subsystem_Experiments(i);
    subsystem_name = subsystem{1};
    if (length(subsystem_name) > 35)
        if (strcmp(subsystem_name(36:43), directory) ~= 1)
    
            set_param(strcat('master_thesis_simulink/Experiments/', subsystem_name(36:43)), 'commented', 'on');
        end
    end
end
set_param(strcat('master_thesis_simulink/Experiments/', directory, '/Experiments'), 'commented', 'on');
set_param(strcat('master_thesis_simulink/Experiments/', directory, '/Model validation'), 'commented', 'off');
set_param(strcat('master_thesis_simulink/Experiments/', directory, '/Noise estimation'), 'commented', 'on');

% Comment System
set_param('master_thesis_simulink/System', 'commented', 'on');

% Simulate Model
out = sim('master_thesis_simulink.slx');
markers_Bases = out.markers_Bases.signals.values(:, end);

% S1_F1_H2
if (shoulder_num == 1 && forearm_num == 1 && hand_num == 2)

    markers_shoulder = markers_Bases(1:3);
    markers_forearm = markers_Bases(4:6);
    markers_hand = [markers_Bases(7:9) markers_Bases(10:12)];

% S0_F1_H2
elseif (shoulder_num == 0 && forearm_num == 1 && hand_num == 2)

    markers_shoulder = -1;
    markers_forearm = markers_Bases(2:4);
    markers_hand = [markers_Bases(5:7) markers_Bases(8:10)];

% S1_F0_H2
elseif (shoulder_num == 1 && forearm_num == 0 && hand_num == 2)

    markers_shoulder = markers_Bases(1:3);
    markers_forearm = -1;
    markers_hand = [markers_Bases(5:7) markers_Bases(8:10)];

% S2_F0_H2
elseif (shoulder_num == 2 && forearm_num == 0 && hand_num == 2)

    markers_shoulder = [markers_Bases(1:3) markers_Bases(4:6)];
    markers_forearm = -1;
    markers_hand = [markers_Bases(8:10) markers_Bases(11:13)];

% S0_F2_H2
elseif (shoulder_num == 0 && forearm_num == 2 && hand_num == 2)

    markers_shoulder = -1;
    markers_forearm = [markers_Bases(2:4) markers_Bases(5:7)];
    markers_hand = [markers_Bases(8:10) markers_Bases(11:13)];

% S1_F0_H3
elseif (shoulder_num == 1 && forearm_num == 0 && hand_num == 3)

    markers_shoulder = markers_Bases(1:3);
    markers_forearm = -1;
    markers_hand = [markers_Bases(5:7) markers_Bases(8:10) markers_Bases(11:13)];

% S1_F2_H2
elseif (shoulder_num == 1 && forearm_num == 2 && hand_num == 2)

    markers_shoulder = markers_Bases(1:3);
    markers_forearm = [markers_Bases(4:6) markers_Bases(7:9)];
    markers_hand = [markers_Bases(10:12) markers_Bases(13:15)];
end

clear markers_Bases;

% Save
save(strcat('/home/gioel/Documents/Master_thesis/Human arm/Simulations/', directory, '/Markers_Pos_6/markers_shoulder.mat'), 'markers_shoulder');
save(strcat('/home/gioel/Documents/Master_thesis/Human arm/Simulations/', directory, '/Markers_Pos_6/markers_forearm.mat'), 'markers_forearm');
save(strcat('/home/gioel/Documents/Master_thesis/Human arm/Simulations/', directory, '/Markers_Pos_6/markers_hand.mat'), 'markers_hand');

% Plot p_0 vs p_FK
if (shoulder_num == 1 && forearm_num == 2 && hand_num == 2)

    fig = figure;
    set(fig, 'position', [10, 10, 1500, 500]);
    sgtitle('p^{0p} vs p_{FK}^{0p}');
    
    for i = 1:m
    
        subplot(3,2,i);
        % p^0
        plot(out.p0p_vs_pFK.time, out.p0p_vs_pFK.signals.values((i-1)*6+1, :)); % x
        hold on;
        plot(out.p0p_vs_pFK.time, out.p0p_vs_pFK.signals.values((i-1)*6+2, :)); % y
        hold on;
        plot(out.p0p_vs_pFK.time, out.p0p_vs_pFK.signals.values((i-1)*6+3, :)); % z
        hold on;
        % p_FK
        plot(out.p0p_vs_pFK.time, out.p0p_vs_pFK.signals.values((i-1)*6+4, :)); % x
        hold on;
        plot(out.p0p_vs_pFK.time, out.p0p_vs_pFK.signals.values((i-1)*6+5, :)); % y
        hold on;
        plot(out.p0p_vs_pFK.time, out.p0p_vs_pFK.signals.values((i-1)*6+6, :)); % z
        legend('p^0:x', 'p^0:y', 'p^0:z', 'p^0_{FK}:x', 'p^0_{FK}:y', 'p^0_{FK}:z', Location='west');
        title(strcat('Marker', num2str(i)));
    
    end   
else

    fig = figure;
    set(fig, 'position', [10, 10, 1500, 500]);
    sgtitle('p^{0p} vs p_{FK}^{0p}');
    
    for i = 1:m
    
        subplot(2,2,i);
        % p^0
        plot(out.p0p_vs_pFK.time, out.p0p_vs_pFK.signals.values((i-1)*6+1, :)); % x
        hold on;
        plot(out.p0p_vs_pFK.time, out.p0p_vs_pFK.signals.values((i-1)*6+2, :)); % y
        hold on;
        plot(out.p0p_vs_pFK.time, out.p0p_vs_pFK.signals.values((i-1)*6+3, :)); % z
        hold on;
        % p_FK
        plot(out.p0p_vs_pFK.time, out.p0p_vs_pFK.signals.values((i-1)*6+4, :)); % x
        hold on;
        plot(out.p0p_vs_pFK.time, out.p0p_vs_pFK.signals.values((i-1)*6+5, :)); % y
        hold on;
        plot(out.p0p_vs_pFK.time, out.p0p_vs_pFK.signals.values((i-1)*6+6, :)); % z
        legend('p^0:x', 'p^0:y', 'p^0:z', 'p^0_{FK}:x', 'p^0_{FK}:y', 'p^0_{FK}:z', Location='west');
        title(strcat('Marker', num2str(i)));
    
    end   
end 

clear fig;
clear all_Subsystem_Experiments all_Subsystem_Experiments_num all_Subsystem_Ros2Matlab all_Subsystem_Ros2Matlab_num subsystem_name subsystem;