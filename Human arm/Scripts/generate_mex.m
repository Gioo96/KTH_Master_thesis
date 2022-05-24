



%% Set parameters

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

% Noise
noise.measurement_var = 0.00001 * ones(m*3, 1);
noise.measurement_seed = 1;
noise.process_var = 0.00001 * diag(ones(n, 1));
noise.process_seed = 2;
noise.input_var = 0.0000001 * ones(m*3, 1);
noise.input_seed = 3;


% Joints' trajectory
% q1
% set_param('master_thesis_simulink/Human arm/RightShoulder_joint/jRightShoulder_rotx', 'TorqueActuationMode', 'NoTorque', 'MotionActuationMode', 'ComputedMotion');
set_param('master_thesis_simulink/System/Human arm/RightShoulder_joint/jRightShoulder_rotx', 'TorqueActuationMode', 'ComputedTorque', 'MotionActuationMode', 'InputMotion');

% q2
% set_param('master_thesis_simulink/Human arm/RightShoulder_joint/jRightShoulder_roty', 'TorqueActuationMode', 'NoTorque', 'MotionActuationMode', 'ComputedMotion');
set_param('master_thesis_simulink/System/Human arm/RightShoulder_joint/jRightShoulder_roty', 'TorqueActuationMode', 'ComputedTorque', 'MotionActuationMode', 'InputMotion');

% q3
% set_param('master_thesis_simulink/Human arm/RightShoulder_joint/jRightShoulder_rotz', 'TorqueActuationMode', 'NoTorque', 'MotionActuationMode', 'ComputedMotion');
set_param('master_thesis_simulink/System/Human arm/RightShoulder_joint/jRightShoulder_rotz', 'TorqueActuationMode', 'ComputedTorque', 'MotionActuationMode', 'InputMotion');

% q4
% set_param('master_thesis_simulink/Human arm/Elbow_joint/jRightElbow_rotx', 'TorqueActuationMode', 'NoTorque', 'MotionActuationMode', 'ComputedMotion');
set_param('master_thesis_simulink/System/Human arm/Elbow_joint/jRightElbow_rotx', 'TorqueActuationMode', 'ComputedTorque', 'MotionActuationMode', 'InputMotion');

% %% FORWARD KINEMATICS
% 
% % Name the markers which are placed in the shoulder, forearm and hand
% marker.m_shoulder_str = []; % Name of markers placed in the SHOULDER
% marker.m_forearm_str = []; % Name of markers placed in the FOREARM
% marker.m_hand_str = []; % Name of markers placed in the HAND
% s = 1;
% f = 1;
% h = 1;
% for i = 1 : m
% 
%     switch marker.links(i)
% 
%         case 1
%             marker.m_shoulder_str = [marker.m_shoulder_str, 's' + string(s)];
%             s = s + 1;
%         
%         case 2
%             marker.m_forearm_str = [marker.m_forearm_str, 'f' + string(f)];
%             f = f + 1;
% 
%         case 3
%             marker.m_hand_str = [marker.m_hand_str, 'h' + string(h)];
%             h = h + 1;
% 
%     end
% end
% 
% % Forward kinematics initialization
% model.Phi = SX.sym('phi', [m * 3, 1]);
% 
% % SHOULDER, FOREARM, HAND variables
% model.arm_parameters.shou_vars = [];
% model.arm_parameters.fore_vars = [];
% model.arm_parameters.hand_vars = [];
% 
% % Joint variables
% model.q = SX.sym('q', [n, 1]);
% 
% % Forward kinematics computation
% % SHOULDER
% for s = 1 : size(marker.m_shoulder_str, 2)
%     
%     [T, variable] = FK_shoulder(char(marker.m_shoulder_str(s)), model.q);
%     model.arm_parameters.shou_vars = [model.arm_parameters.shou_vars, variable];
%     model.Phi((s-1)*3+1 : (s-1)*3+3) = T(1:3, 4);
% end
% 
% % FOREARM
% i = 1;
% for f = size(marker.m_shoulder_str, 2)+1 : size(marker.m_shoulder_str, 2)+size(marker.m_forearm_str, 2)
%     
%     [T, variable] = FK_forearm(char(marker.m_forearm_str(i)), model.q);
%     model.arm_parameters.fore_vars = [model.arm_parameters.fore_vars, variable];
%     model.Phi((f-1)*3+1 : (f-1)*3+3) = T(1:3, 4);
%     i = i + 1;
% end
% 
% % HAND
% j = 1;
% for h = size(marker.m_shoulder_str, 2)+size(marker.m_forearm_str, 2)+1 : size(marker.m_shoulder_str, 2)+size(marker.m_forearm_str, 2)+size(marker.m_hand_str, 2)
% 
%     [T, variable] = FK_hand(char(marker.m_hand_str(j)), model.q);
%     model.arm_parameters.hand_vars = [model.arm_parameters.hand_vars, variable];
%     model.Phi((h-1)*3+1 : (h-1)*3+3) = T(1:3, 4);
%     j = j + 1;
% end
% 
% % Phi function
% functions.f_Phi = Function('f_Phi', {model.q, model.arm_parameters.shou_vars, model.arm_parameters.fore_vars, model.arm_parameters.hand_vars}, {model.Phi});
% 
% %% JACOBIAN
% 
% % Jacobian computation
% model.J = jacobian(model.Phi, model.q);
% 
% % Jacobian function
% functions.f_J = Function('f_J', {model.q, model.arm_parameters.shou_vars, model.arm_parameters.fore_vars, model.arm_parameters.hand_vars}, {model.J});
% 
% %% JACOBIAN PSEUDOINVERSE
% 
% % PseudoInverse computation
% model.Jpseudo = pinv(model.J);
% 
% % PseudoInverse function
% functions.f_Jpseudo = Function('f_Jpseudo', {model.q, model.arm_parameters.shou_vars, model.arm_parameters.fore_vars, model.arm_parameters.hand_vars}, {model.Jpseudo});
% 
% %% Discretization Runge-Kutta
% % qk+1 = qk + 1/6*Ts*(k1 + 2k2 + 2k3 + k4) ---> qk+1 = f(qk, u_noisyk)
%  
% % qdot = Jpseudo * u_noisy
% rk.u_noisy = SX.sym('u_noisy', [m*3, 1]);
% rk.qdot = model.Jpseudo * rk.u_noisy;
% 
% % Function used to compute k1, k2, k3, k4
% functions.f_RungeKutta = Function('f_RungeKutta', {model.q, model.arm_parameters.shou_vars, model.arm_parameters.fore_vars, model.arm_parameters.hand_vars, rk.u_noisy}, {rk.qdot});
% 
% % k1, k2, k3, k4 computation
% rk.k1 = functions.f_RungeKutta(model.q, model.arm_parameters.shou_vars, model.arm_parameters.fore_vars, model.arm_parameters.hand_vars, rk.u_noisy);
% rk.k2 = functions.f_RungeKutta(model.q + sample_Time*rk.k1/2, model.arm_parameters.shou_vars, model.arm_parameters.fore_vars, model.arm_parameters.hand_vars, rk.u_noisy);
% rk.k3 = functions.f_RungeKutta(model.q + sample_Time*rk.k2/2, model.arm_parameters.shou_vars, model.arm_parameters.fore_vars, model.arm_parameters.hand_vars, rk.u_noisy);
% rk.k4 = functions.f_RungeKutta(model.q + sample_Time*rk.k3, model.arm_parameters.shou_vars, model.arm_parameters.fore_vars, model.arm_parameters.hand_vars, rk.u_noisy);
% 
% % Function f computation
% model.f = model.q + 1/6*sample_Time*(rk.k1 + 2*rk.k2 + 2*rk.k3 + rk.k4);
% functions.f_f = Function('f_f', {model.q, model.arm_parameters.shou_vars, model.arm_parameters.fore_vars, model.arm_parameters.hand_vars, rk.u_noisy}, {model.f});
% 
% %% Linearization around eq. point (eq. point : q_eq = 0, u_eq = 0)
% % qk+1 = Fqk + Guk
% % pk = Hqk + Juk
% 
% % F, G in terms of symbolic variables
% model.linearized.F_symb = jacobian(model.f, model.q);
% model.linearized.G_symb = jacobian(model.f, rk.u_noisy);
% 
% % Function F, G
% functions.f_Fekf = Function('f_Fekf', {model.q, model.arm_parameters.shou_vars, model.arm_parameters.fore_vars, model.arm_parameters.hand_vars, rk.u_noisy}, {model.linearized.F_symb});
% functions.f_Gekf = Function('f_Gekf', {model.q, model.arm_parameters.shou_vars, model.arm_parameters.fore_vars, model.arm_parameters.hand_vars, rk.u_noisy}, {model.linearized.G_symb});
% 
% % Equilibrium point
% % kf.equilibrium_q = zeros(n, 1);
% % kf.equilibrium_u = zeros(3*m, 1);
% 
% %% Generate the mex functions
% 
% % opts = struct('main', true, 'mex', true);
% % 
% % % Generate Jpseudo
% % functions.f_Jpseudo.generate('f_Jpseudo_mex.c', opts);
% % mex f_Jpseudo_mex.c;
% % 
% % % Generate f
% % functions.f_f.generate('f_f_mex.c', opts);
% % mex f_f_mex.c;
% % 
% % % Generate Phi
% % functions.f_Phi.generate('f_Phi_mex.c', opts);
% % mex f_Phi_mex.c;
% % 
% % % Generate F
% % functions.f_Fekf.generate('f_Fekf_mex.c', opts);
% % mex f_Fekf_mex.c;
% % 
% % % Generate G
% % functions.f_Gekf.generate('f_Gekf_mex.c', opts);
% % mex f_Gekf_mex.c;
% % 
% % % Generate H
% % functions.f_J.generate('f_Hekf_mex.c', opts);
% % mex f_Hekf_mex.c;