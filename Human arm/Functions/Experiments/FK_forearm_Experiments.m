function [Phi_fore, forearm_variable] = FK_forearm_Experiments(marker_variable, q)

% forearm_Phi computes the position of the marker (placed in the FOREARM) expressed wrt Wrold RF

% Human arm parameters
run('human_arm_parameters');

% CasADi
import casadi.*

% Position of marker placed in the FOREARM wrt M0
forearm_variable = SX.sym(marker_variable, [3, 1]);

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

%
% ELBOW
%
Rz_el = [cos(q(4)) -sin(q(4)) 0;
            sin(q(4)) cos(q(4)) 0;
            0 0 1];

% TRANSLATION: Shoulder_RF_final wrt Shoulder_RF
trans_FS_BS = [0; -arm.shoulder.length; 0];
% ROTATION+TRANSLATION: Shoulder_RF_final wrt M0
TFS_M0 = [RBS_M0 RBS_M0*trans_FS_BS+trans_BS_M0;0 0 0 1];
% ROTATION: Elbow_RF wrt Shoulder_RF_final
RBE_FS = Rz_el;

% TRANSLATION: Marker wrt Elbow_RF
trans_M_BE = forearm_variable;
% ROTATION+TRANSLATION: Marker wrt Shoulder_RF_final
TM_FS = [RBE_FS RBE_FS*trans_M_BE;0 0 0 1];
% ROTATION+TRANSLATION: Marker to World
Phi_fore = TFS_M0 * TM_FS;

end