%% HUMAN ARM PARAMETERS

% Number of DoF
n = 7;

% CHANGE HERE NUMBER OF MARKERS
shoulder_num = 1;
forearm_num = 1;
hand_num = 2;
m = shoulder_num + forearm_num + hand_num;

% Shoulder
arm.shoulder.length = 0.28; % [m]

% Forearm
arm.forearm.length = 0.26; % [m]

% Markers
arm.markers.links = [1*ones(shoulder_num, 1);
                2*ones(forearm_num, 1);
                3*ones(hand_num, 1)];

% tSh_0, 0 Configuration
tSh_0_0Conf = [0.01; -0.229; -0.261];

% tSh_0, Robust
tSh_0_Robust = [0.03; -0.246; -0.32];

% Name directory
directory = strcat('S', num2str(shoulder_num), '_F', num2str(forearm_num), '_H', num2str(hand_num));

% Add path 'C code/C_code_folder' <-- target_folder
set_Mex(directory);
%% Global variables

global kf;
global ekf;
global noise;
global markers_shoulder markers_forearm markers_hand;
global selected;
selected = -1;

%% Sample Time

sample_Time = 0.01;

%% Simulation time

simulink.time = 1;

%% Number of samples

number_samples = simulink.time/sample_Time;

%% Noise (ESTIMATED) --> ASSUMPTION: All markers have the same noise
%noise.R = 7.9099e-07 * diag(ones(m*3, 1)); % ESTIMATED
%noise.Q = 1 * diag(ones(n, 1));
%noise.Nu = 0.0285 * diag(ones(m*3, 1)); % ESTIMATED
%noise.Nu = 0.0285 * diag(ones(m*3, 1));
%noise.Nu_seed = 3;