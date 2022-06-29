function [Phi_shou, shoulder_variable] = FK_shoulder_Experiments(marker_variable, q)

% shoulder_Phi computes the position of the marker (placed in the SHOULDER) expressed wrt Wrold RF

% Human arm parameters
run('human_arm_parameters');

% CasADi
import casadi.*

% Position of marker placed in the SHOULDER wrt Shoulder_RF
shoulder_variable = SX.sym(marker_variable, [3, 1]);

%
% SHOULDER
%

Rx_sh = [cos(q(1)) -sin(q(1)) 0;
               sin(q(1)) cos(q(1)) 0;
               0 0 1];
Ry_sh = [cos(q(2)) -sin(q(2)) 0;
               sin(q(2)) cos(q(2)) 0;
               0 0 1];
Rz_sh = [cos(q(3)) -sin(q(3)) 0;
               sin(q(3)) cos(q(3)) 0;
               0 0 1];

% TRANSLATION: Shoulder_RF wrt M0
trans_BS_M0 = [-0.008;-0.17; -0.29]; % TO BE EVALUATED
% ROTATION: Shoulder_RF wrt M0
Rx_M0 = align.Rz * align.Rx * Rx_sh; % Rotation along x
Ry_x = align.Ry * align.Rz * Ry_sh; % Rotation along y
RBS_y = align.Rx * align.Ry * Rz_sh; % Rotation along z
RBS_M0 = Rx_M0 * Ry_x * RBS_y;

% TRANSLATION: Marker wrt B_shoulder
trans_M_BS = shoulder_variable;
% ROTATION+TRANSLATION: Marker to World
Phi_shou = [RBS_M0 RBS_M0*trans_M_BS+trans_BS_M0;0 0 0 1];

end