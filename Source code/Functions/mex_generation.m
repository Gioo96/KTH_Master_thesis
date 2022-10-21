function f_J = mex_generation(shoulder_num, forearm_num, hand_num, generate)

%% Description

% mex_generation generates the following mex functions: - FK: Forward Kinematics
%                                                       - DK: Differential Kinematics
%                                                       - f: Discretization of DF
%                                                       - F_ekf
%                                                       - G_ekf
%                                                       - H_ekf

% Inputs:
% -- shoulder_num   : Number of markers on the shoulder
% -- forearm_num    : Number of markers on the forearm
% -- hand_num       : Number of markers on the hand
% -- generate       : true --> generate mex functions
%                   : false --> do not generate mex functions

% Outputs:
% -- f_J            : Jacobian function which is needed by 'singular_config' function

%% FORWARD KINEMATICS

% CasADi
import casadi.*

% Number of joints
n = 7;
% Number of markers
m = shoulder_num + forearm_num + hand_num;

% Name the markers which are placed in the shoulder, forearm and hand
m_shoulder_str = []; % Name of markers placed in the SHOULDER
m_forearm_str = []; % Name of markers placed in the FOREARM
m_hand_str = []; % Name of markers placed in the HAND

if (shoulder_num > 0)

    for s = 1:shoulder_num
        
        m_shoulder_str = [m_shoulder_str, 's' + string(s)];
    end
end

if (forearm_num > 0)

    for f = 1:forearm_num
        
        m_forearm_str = [m_forearm_str, 'f' + string(f)];
    end
end

if (hand_num > 0)

    for h = 1:hand_num
        
        m_hand_str = [m_hand_str, 'h' + string(h)];
    end
end

% Change directory
directory = strcat('C code/S', num2str(shoulder_num), '_F', num2str(forearm_num), '_H', num2str(hand_num), '/');
cd (directory);

% SHOULDER, FOREARM, HAND variables
shou_vars = [];
fore_vars = [];
hand_vars = [];

% Joint variables
q = SX.sym('q', [n, 1]);

% Forward kinematics initialization
Phi = SX.sym('phi', [m * 3, 1]);

% Forward kinematics computation
% SHOULDER
for s = 1 : shoulder_num
    
    [T, variable] = FK_shoulder(char(m_shoulder_str(s)), q);
    shou_vars = [shou_vars, variable];
    Phi((s-1)*3+1 : (s-1)*3+3) = T(1:3, 4);
end

% FOREARM
i = 1;
for f = shoulder_num+1 : shoulder_num+forearm_num
    
    [T, variable] = FK_forearm(char(m_forearm_str(i)), q);
    fore_vars = [fore_vars, variable];
    Phi((f-1)*3+1 : (f-1)*3+3) = T(1:3, 4);
    i = i + 1;
end

% HAND
j = 1;
for h = shoulder_num+forearm_num+1 : shoulder_num+forearm_num+hand_num

    [T, variable] = FK_hand(char(m_hand_str(j)), q);
    hand_vars = [hand_vars, variable];
    Phi((h-1)*3+1 : (h-1)*3+3) = T(1:3, 4);
    j = j + 1;
end

% Phi function
f_Phi = Function('f_Phi', {q, shou_vars, fore_vars, hand_vars}, {Phi});

%% JACOBIAN

% Jacobian computation
J = jacobian(Phi, q);

% Jacobian function
f_J = Function('f_J', {q, shou_vars, fore_vars, hand_vars}, {J});

%% JACOBIAN PSEUDOINVERSE

% PseudoInverse computation
Jpseudo = (J'*J)^(-1) * J';
Jpinv = pinv(J);

% Damped LS PseudoInverse computation
%Jpseudo_Damped = (J'*J + eye(n)*exp(-det(J'*J)))^(-1) * J';

% PseudoInverse function
f_Jpseudo = Function('f_Jpseudo', {q, shou_vars, fore_vars, hand_vars}, {Jpseudo});
f_Jpinv = Function('f_Jpinv', {q, shou_vars, fore_vars, hand_vars}, {Jpinv});

% Damped LS PseudoInverse function
%f_Jpseudo_Damped = Function('f_Jpseudo_Damped', {q, shou_vars, fore_vars, hand_vars}, {Jpseudo_Damped});

%% Discretization Runge-Kutta
% qk+1 = qk + 1/6*Ts*(k1 + 2k2 + 2k3 + k4) ---> qk+1 = f(qk, pdot)
 
% qdot = Jpseudo * pdot
pdot = SX.sym('pdot', [m*3, 1]);
qdot = Jpseudo * pdot;

% k1, k2, k3, k4 computation
f_RungeKutta = Function('f_RungeKutta', {q, shou_vars, fore_vars, hand_vars, pdot}, {qdot});
sample_Time = 0.01; % Sampling time
k1 = f_RungeKutta(q, shou_vars, fore_vars, hand_vars, pdot);
k2 = f_RungeKutta(q + sample_Time*k1/2, shou_vars, fore_vars, hand_vars, pdot);
k3 = f_RungeKutta(q + sample_Time*k2/2, shou_vars, fore_vars, hand_vars, pdot);
k4 = f_RungeKutta(q + sample_Time*k3, shou_vars, fore_vars, hand_vars, pdot);

% Function f computation
f = q + 1/6*sample_Time*(k1 + 2*k2 + 2*k3 + k4);
f_f = Function('f_f', {q, shou_vars, fore_vars, hand_vars, pdot}, {f});

% qdot = Jpseudo * pdot
pdot_pinv = SX.sym('pdot', [m*3, 1]);
qdot_pinv = Jpinv * pdot_pinv;

% k1, k2, k3, k4 computation
f_RungeKutta_pinv = Function('f_RungeKutta_pinv', {q, shou_vars, fore_vars, hand_vars, pdot}, {qdot_pinv});
sample_Time = 0.01; % Sampling time
k1_pinv = f_RungeKutta_pinv(q, shou_vars, fore_vars, hand_vars, pdot_pinv);
k2_pinv = f_RungeKutta_pinv(q + sample_Time*k1_pinv/2, shou_vars, fore_vars, hand_vars, pdot_pinv);
k3_pinv = f_RungeKutta_pinv(q + sample_Time*k2_pinv/2, shou_vars, fore_vars, hand_vars, pdot_pinv);
k4_pinv = f_RungeKutta_pinv(q + sample_Time*k3_pinv, shou_vars, fore_vars, hand_vars, pdot_pinv);

% Function f computation
f_pinv = q + 1/6*sample_Time*(k1_pinv + 2*k2_pinv + 2*k3_pinv + k4_pinv);
f_f_pinv = Function('f_f_pinv', {q, shou_vars, fore_vars, hand_vars, pdot_pinv}, {f_pinv});

%% Linearization
% qk+1 = F*qk + G*pdotk
% pk = Jqk

% F, G in terms of symbolic variables
F = jacobian(f, q);
G = jacobian(f, pdot);

% Function F, G
f_F = Function('f_F', {q, shou_vars, fore_vars, hand_vars, pdot}, {F});
f_G = Function('f_G', {q, shou_vars, fore_vars, hand_vars, pdot}, {G});

% F, G in terms of symbolic variables
F_pinv = jacobian(f_pinv, q);
G_pinv = jacobian(f_pinv, pdot_pinv);

% Function F, G
f_F_pinv = Function('f_F_pinv', {q, shou_vars, fore_vars, hand_vars, pdot_pinv}, {F_pinv});
f_G_pinv = Function('f_G_pinv', {q, shou_vars, fore_vars, hand_vars, pdot_pinv}, {G_pinv});

%% Generate the mex functions

switch generate

    case 'true'

        opts = struct('main', true, 'mex', true);
        
        % Generate Phi
        %f_Phi.generate('f_Phi_mex.c', opts);
        %mex f_Phi_mex.c;

        % Generate J
        %f_J.generate('f_J_mex.c', opts);
        %mex f_J_mex.c;
        
        % Generate Jpseudo
        %f_Jpseudo.generate('f_Jpseudo_mex.c', opts);
        %mex f_Jpseudo_mex.c;
        
        % Generate f
        f_f.generate('f_f_mex.c', opts);
        mex f_f_mex.c;
        
        % Generate F
        f_F.generate('f_Fekf_mex.c', opts);
        mex f_Fekf_mex.c;
        
        % Generate G
        f_G.generate('f_Gekf_mex.c', opts);
        mex f_Gekf_mex.c;
        
        % Generate J
        f_J.generate('f_Jekf_mex.c', opts);
        mex f_Jekf_mex.c;
        

end

%% Rechange directory
cd ../..;
end