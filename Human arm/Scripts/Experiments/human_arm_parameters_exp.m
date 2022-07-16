%% HUMAN ARM PARAMETERS

% Number of DoF
n = 7;

% CHANGE HERE NUMBER OF MARKERS
shoulder_num = 1;
forearm_num = 1;
hand_num = 2;
m = shoulder_num + forearm_num + hand_num;

% Shoulder
arm.shoulder.length = 0.25; % [m]
arm.shoulder.radius = 0.05; % [m] % SIMULINK HUMAN ARM
arm.shoulder.mass = 2.4459; % [Kg] % SIMULINK HUMAN ARM
arm.shoulder.moment_inertia = [0.0059341, 0.0010123, 0.0059341]; % [Kg*m^2] % SIMULINK HUMAN ARM

% Forearm
arm.forearm.length = 0.25; % [m]
arm.forearm.radius = 0.03; % [m] % SIMULINK HUMAN ARM
arm.forearm.mass = 1.578; % [Kg] % SIMULINK HUMAN ARM
arm.forearm.moment_inertia = [0.0089856, 0.00041027, 0.0089856]; % [Kg*m^2] % SIMULINK HUMAN ARM

% Hand
arm.hand.dimensions = [0.1, 0.18, 0.04]; % [m] % SIMULINK HUMAN ARM
arm.hand.mass = 0.4734; % [Kg] % SIMULINK HUMAN ARM
arm.hand.moment_inertia = [0.0015629, 0.0007401900000000001, 0.0021389]; % [Kg*m^2] % SIMULINK HUMAN ARM

% Markers
arm.markers.radius = 0.01; % [m] % SIMULINK HUMAN ARM
arm.markers.links = [1*ones(shoulder_num, 1);
                2*ones(forearm_num, 1);
                3*ones(hand_num, 1)];

% tSh_0, 0 Configuration
%tSh_0_0Conf = [0.01; -0.229; -0.261];

% tSh_0, Robust
%tSh_0_Robust = [0.03; -0.246; -0.32];

% Name directory
directory = strcat('S', num2str(shoulder_num), '_F', num2str(forearm_num), '_H', num2str(hand_num));

% Add path 'C code/C_code_folder' <-- target_folder
set_Mex(directory);
%% Global variables

global ls;
global kf;
global ekf;
global noise;
global markers_shoulder markers_forearm markers_hand;
global p_0p v_0p;
global selected;
selected = -1;

%% Sample Time

sample_Time = 0.01;

%% Simulation time

simulink.time = 50;

%% Number of samples

number_samples = simulink.time/sample_Time;