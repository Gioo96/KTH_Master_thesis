function mex_generation(links, sample_Time)

% mex_generation generates the following mex functions: - Phi
%                                                       - Jpseudo
%                                                       - f
%                                                       - F_ekf
%                                                       - G_ekf
%                                                       - H_ekf

% Inputs:
% -- links          : Vector containing indices {1, 2, 3} related to the corresponding marker position
% -- sample_Time    : Sampling time

%  CasADi
import casadi.*

%% FORWARD KINEMATICS

% Number of joints
n = 7;
% Number of markers
m = size(links, 1);

% Name the markers which are placed in the shoulder, forearm and hand
m_shoulder_str = []; % Name of markers placed in the SHOULDER
m_forearm_str = []; % Name of markers placed in the FOREARM
m_hand_str = []; % Name of markers placed in the HAND

s = 1;
f = 1;
h = 1;
for i = 1 : m

    switch links(i)

        case 1
            m_shoulder_str = [m_shoulder_str, 's' + string(s)];
            s = s + 1;
        
        case 2
            m_forearm_str = [m_forearm_str, 'f' + string(f)];
            f = f + 1;

        case 3
            m_hand_str = [m_hand_str, 'h' + string(h)];
            h = h + 1;

    end
end

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
for s = 1 : size(m_shoulder_str, 2)
    
    [T, variable] = FK_shoulder(char(m_shoulder_str(s)), q);
    shou_vars = [shou_vars, variable];
    Phi((s-1)*3+1 : (s-1)*3+3) = T(1:3, 4);
end

% FOREARM
i = 1;
for f = size(m_shoulder_str, 2)+1 : size(m_shoulder_str, 2)+size(m_forearm_str, 2)
    
    [T, variable] = FK_forearm(char(m_forearm_str(i)), q);
    fore_vars = [fore_vars, variable];
    Phi((f-1)*3+1 : (f-1)*3+3) = T(1:3, 4);
    i = i + 1;
end

% HAND
j = 1;
for h = size(m_shoulder_str, 2)+size(m_forearm_str, 2)+1 : size(m_shoulder_str, 2)+size(m_forearm_str, 2)+size(m_hand_str, 2)

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
Jpseudo = pinv(J);

% PseudoInverse function
f_Jpseudo = Function('f_Jpseudo', {q, shou_vars, fore_vars, hand_vars}, {Jpseudo});

%% Discretization Runge-Kutta
% qk+1 = qk + 1/6*Ts*(k1 + 2k2 + 2k3 + k4) ---> qk+1 = f(qk, u_noisyk)
 
% qdot = Jpseudo * u_noisy
u_noisy = SX.sym('u_noisy', [m*3, 1]);
qdot = Jpseudo * u_noisy;

% Function used to compute k1, k2, k3, k4
f_RungeKutta = Function('f_RungeKutta', {q, shou_vars, fore_vars, hand_vars, u_noisy}, {qdot});

% k1, k2, k3, k4 computation
k1 = f_RungeKutta(q, shou_vars, fore_vars, hand_vars, u_noisy);
k2 = f_RungeKutta(q + sample_Time*k1/2, shou_vars, fore_vars, hand_vars, u_noisy);
k3 = f_RungeKutta(q + sample_Time*k2/2, shou_vars, fore_vars, hand_vars, u_noisy);
k4 = f_RungeKutta(q + sample_Time*k3, shou_vars, fore_vars, hand_vars, u_noisy);

% Function f computation
f = q + 1/6*sample_Time*(k1 + 2*k2 + 2*k3 + k4);
f_f = Function('f_f', {q, shou_vars, fore_vars, hand_vars, u_noisy}, {f});

%% Linearization around eq. point
% qk+1 = Fqk + Guk
% pk = Hqk + Juk

% F, G in terms of symbolic variables
F = jacobian(f, q);
G = jacobian(f, u_noisy);

% Function F, G
f_F = Function('f_F', {q, shou_vars, fore_vars, hand_vars, u_noisy}, {F});
f_G = Function('f_G', {q, shou_vars, fore_vars, hand_vars, u_noisy}, {G});

%% Generate the mex functions

opts = struct('main', true, 'mex', true);

% % Generate Phi
% f_Phi.generate('f_Phi_mex.c', opts);
% mex f_Phi_mex.c;
% 
% Generate J
f_J.generate('f_J_mex.c', opts);
mex f_J_mex.c;
%
% % Generate Jpseudo
% f_Jpseudo.generate('f_Jpseudo_mex.c', opts);
% mex f_Jpseudo_mex.c;
<<<<<<< HEAD
% Generate Jpseudo
f_J.generate('f_J_mex.c', opts);
mex f_J_mex.c;
=======
>>>>>>> 7bd921c256ee69a625810d4d01884223ebd803fc
% 
% % Generate f
% f_f.generate('f_f_mex.c', opts);
% mex f_f_mex.c;
% 
% % Generate F
% f_F.generate('f_Fekf_mex.c', opts);
% mex f_Fekf_mex.c;
% 
% % Generate G
% f_G.generate('f_Gekf_mex.c', opts);
% mex f_Gekf_mex.c;
% 
% % Generate H
% f_J.generate('f_Hekf_mex.c', opts);
% mex f_Hekf_mex.c;

end