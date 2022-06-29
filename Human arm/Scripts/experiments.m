%% Position of markers wrt Base Frames
Super_marker_1 = [0; -0.155; 0];
Super_marker_2 = [0;-0.07; 0];
Super_marker_3 = [-0.03; -0-06; 0];
Super_marker_4 = [0.03; -0-13; 0];

markers_shoulder = Super_marker_1;
markers_forearm = Super_marker_2;
markers_hand = [Super_marker_3 Super_marker_4];

sample_Time = 0.01;
simulink.time = Inf;
%% Add path 'C code/C_code_folder' <-- target_folder
shoulder_num = 1;
forearm_num = 1;
hand_num = 2;
m = shoulder_num + forearm_num + hand_num;
directory = strcat('S', num2str(shoulder_num), '_F', num2str(forearm_num), '_H', num2str(hand_num));
set_Mex(directory);

%% EKF
ekf.q0 = zeros(7, 1);
ekf.P0 = diag(ones(7, 1));