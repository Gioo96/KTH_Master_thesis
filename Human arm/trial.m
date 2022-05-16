% CasADi
import casadi.*

% Human arm parameters
run('human_arm_param.m');

%% Set parameters

% Sampling time
sample_Time = 0.01;

% Simulation time
simulink.time = 4.5;

% Simulation IC [rad]
simulink.q0 = zeros(n, 1);

% NLDM IC
set_param('master_thesis_simulink/NLDM delay', 'InitialCondition', 'simulink.q0');

% Marker is placed in the : shoulder --> 1
%                           forearm  --> 2
%                           hand     --> 3
marker.links = [1;
                1;
                1;
                1;
                2;
                2;
                2;
                3;
                3];
marker.shoulder_variables = [arm.shoulder.radius+arm.markers.radius -3/4*arm.shoulder.length 0;
                   -arm.shoulder.radius-arm.markers.radius -3/4*arm.shoulder.length 0;
                   arm.shoulder.radius+arm.markers.radius -1/4*arm.shoulder.length 0;
                   -arm.shoulder.radius-arm.markers.radius -1/4*arm.shoulder.length 0]';
marker.forearm_variables = [arm.forearm.radius+arm.markers.radius -3/4*arm.forearm.length 0;
                   -arm.forearm.radius-arm.markers.radius -1/2*arm.forearm.length 0;
                   arm.forearm.radius+arm.markers.radius -1/4*arm.forearm.length 0]';
marker.hand_variables = [0 -3/4*arm.hand.dimensions(2) arm.hand.dimensions(3)/2+arm.markers.radius;
                   0 -1/4*arm.hand.dimensions(2) arm.hand.dimensions(3)/2+arm.markers.radius]';

% Number of markers
m = size(marker.links, 1);

% Noise (TO BE ESTIMATED)
noise.measurement_var = 0.00001 * ones(m*3, 1);
noise.measurement_seed = 1;
noise.input_var = 0.0000001 * ones(m*3, 1);
noise.input_seed = 2;

% Comment Kalman Filter function
set_param('master_thesis_simulink/Kalman Filter','commented','on');

% Comment Extended Kalman Filter function
set_param('master_thesis_simulink/Extended Kalman Filter','commented','on');

% Comment Linearized Discrete model
set_param('master_thesis_simulink/Linearized Discrete Model','commented','on');

% Comment State-Observer
set_param('master_thesis_simulink/State-observer','commented','on');

%% FORWARD KINEMATICS

% Number of markers placed in the SHOULDER: s-1
s = 1;

% Number of markers placed in the FOREARM: f-1
f = 1;

% Number of markers placed in the HAND: h-1
h = 1;

for i = 1 : m

    switch marker.links(i)

        case 1
            s = s + 1;
        
        case 2
            f = f + 1;

        case 3
            h = h + 1;

    end
end

% Forward kinematics initialization
model.Phi = SX.sym('phi', [m * 3, 1]);

% Joint variables
model.q = SX.sym('q', [n, 1]);

% Forward kinematics computation
% SHOULDER
if (s > 1)
    for s = 1 : size(marker.shoulder_variables, 2)
        
        T = shoulder_Phi(marker.shoulder_variables(:, s), model.q);
        model.Phi((s-1)*3+1 : (s-1)*3+3) = T(1:3, 4);
    end
end

% FOREARM
i = 1;
if (f > 1)
    for f = size(marker.shoulder_variables, 2)+1 : size(marker.shoulder_variables, 2)+size(marker.forearm_variables, 2)
        
        T = forearm_Phi(marker.forearm_variables(:, i), model.q);
        model.Phi((f-1)*3+1 : (f-1)*3+3) = T(1:3, 4);
        i = i + 1;
    end
end

% HAND
j = 1;
if (h > 1)
    for h = size(marker.shoulder_variables, 2)+size(marker.forearm_variables, 2)+1 : size(marker.shoulder_variables, 2)+size(marker.forearm_variables, 2)+size(marker.hand_variables, 2)
    
        T = hand_Phi(marker.hand_variables(:, j), model.q);
        model.Phi((h-1)*3+1 : (h-1)*3+3) = T(1:3, 4);
        j = j + 1;
    end
end

% Phi function
functions.f_Phi = Function('f_Phi', {model.q}, {model.Phi});

%% JACOBIAN

% Jacobian computation
model.J = jacobian(model.Phi, model.q);

% Jacobian function
functions.f_J = Function('f_J', {model.q}, {model.J});

%% JACOBIAN PSEUDOINVERSE

% PseudoInverse computation
model.Jpseudo = pinv(model.J);

% PseudoInverse function
functions.f_Jpseudo = Function('f_Jpseudo', {model.q}, {model.Jpseudo});

%% Discretization Runge-Kutta
% qk+1 = qk + 1/6*Ts*(k1 + 2k2 + 2k3 + k4) ---> qk+1 = f(qk, uk)
 
% qdot = Jpseudo * u
rk.u = SX.sym('u', [m*3, 1]);
rk.qdot = model.Jpseudo * rk.u;

% Function used to compute k1, k2, k3, k4
functions.f_RungeKutta = Function('f_RungeKutta', {model.q, rk.u}, {rk.qdot});

% k1, k2, k3, k4 computation
rk.k1 = functions.f_RungeKutta(model.q, rk.u);
rk.k2 = functions.f_RungeKutta(model.q + sample_Time*rk.k1/2, rk.u);
rk.k3 = functions.f_RungeKutta(model.q + sample_Time*rk.k2/2, rk.u);
rk.k4 = functions.f_RungeKutta(model.q + sample_Time*rk.k3, rk.u);

% Function f computation
model.f = model.q + 1/6*sample_Time*(rk.k1 + 2*rk.k2 + 2*rk.k3 + rk.k4);
functions.f_f = Function('f_f', {model.q, rk.u}, {model.f});

%% Linearization around eq. point (eq. point : q_eq = 0, u_eq = 0)
% qk+1 = Fqk + Guk
% pk = Hqk + Juk

% F, G in terms of symbolic variables
model.linearized.F_symb = jacobian(model.f, model.q);
model.linearized.G_symb = jacobian(model.f, rk.u);

% Function F, G
functions.f_Fekf = Function('f_Fekf', {model.q, rk.u}, {model.linearized.F_symb});
functions.f_Gekf = Function('f_Gekf', {model.q, rk.u}, {model.linearized.G_symb});

% Equilibrium point
kf.equilibrium_q = zeros(n, 1);
kf.equilibrium_u = zeros(3*m, 1);

%% Generate the mex functions

opts = struct('main', true, 'mex', true);

% Generate Jpseudo
functions.f_Jpseudo.generate('f_Jpseudo_mex.c', opts);

% Generate f
functions.f_f.generate('f_f_mex.c', opts);

% Generate Phi
functions.f_Phi.generate('f_Phi_mex.c', opts);
% 
% Generate F
functions.f_Fekf.generate('f_Fekf_mex.c', opts);
% 
% Generate G
functions.f_Gekf.generate('f_Gekf_mex.c', opts);
% 
% Generate H
functions.f_J.generate('f_Hekf_mex.c', opts);



