%% Run Human arm aparameters
run('human_arm_parameters.m')

%% Simulation time
simulink.time = 20;

%% Add path 'C code/C_code_folder' <-- target_folder
set_Mex(directory);

%% tSh_0, 0 Configuration
tSh_0_0Conf = [0.01; -0.229; -0.261];

%% Compute markers flag
compute_markers = false;

%% Run function

% LS
method_flag = "LS";
q0_LS = zeros(7, 1);
global selected;
selected = -1;
LS_KF_EKF_exp(method_flag, compute_markers, []);