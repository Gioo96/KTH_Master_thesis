%% HUMAN ARM PARAMETERS

% Number of DoF
n = 7;

% Shoulder
arm.shoulder.length = 0.25; % [m]
arm.shoulder.radius = 0.05; % [m]
arm.shoulder.mass = 2.4459; % [Kg]
arm.shoulder.moment_inertia = [0.0059341, 0.0010123, 0.0059341]; % [Kg*m^2]

% Forearm
arm.forearm.length = 0.25; % [m]
arm.forearm.radius = 0.03; % [m]
arm.forearm.mass = 1.578; % [Kg]
arm.forearm.moment_inertia = [0.0089856, 0.00041027, 0.0089856]; % [Kg*m^2]

% Hand
arm.hand.dimensions = [0.1, 0.18, 0.04]; % [m]
arm.hand.mass = 0.4734; % [Kg]
arm.hand.moment_inertia = [0.0015629, 0.0007401900000000001, 0.0021389]; % [Kg*m^2]

% Markers
arm.markers.radius = 0.01; % [m]