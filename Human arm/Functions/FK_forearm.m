function [Phi_fore, forearm_variable] = FK_forearm(marker_variable, q)

% forearm_Phi computes the position of the marker (placed in the FOREARM) expressed wrt Wrold RF

% Human arm parameters
run('human_arm_parameters');

% CasADi
import casadi.*

% Position of marker placed in the FOREARM wrt its base RF
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

% ROTATION: W --> B_shoulder
RW_x = align.Rz * align.Rx * Rx_sh; % W --> shoulder.Rx
Rx_y = align.Ry * align.Rz * Ry_sh; % shoulder.Rx --> shoulder.Ry
Ry_BS = align.Rx * align.Ry * Rz_sh; % shoulder.Ry --> B_shoulder
RW_BS = RW_x * Rx_y * Ry_BS; % W --> B_shoulder

%
% ELBOW
%
Rx_el = [cos(q(4)) -sin(q(4)) 0;
            sin(q(4)) cos(q(4)) 0;
            0 0 1];

% TRANSLATION: F_shoulder wrt B_shoulder
trans_BS_FS = [0; -arm.shoulder.length; 0];
% ROTATION+TRANSLATION: F_shoulder to World
TW_FS = [RW_BS RW_BS * trans_BS_FS;0 0 0 1];
% ROTATION: F_shoulder --> B_elbow
RFS_x = align.Rz * align.Rx * Rx_el; % F_shoulder --> elbow.Rx
Rx_z = (align.Rz * align.Rx)'; % elbow.Rx --> elbow.Rz
RFS_BE = RFS_x * Rx_z; % F_shoulder --> B_elbow

% TRANSLATION: Marker wrt B_elbow
trans_BE_M = forearm_variable;
% ROTATION+TRANSLATION: Marker to F_shoulder
TFS_M = [RFS_BE RFS_BE * trans_BE_M;0 0 0 1];
% ROTATION+TRANSLATION: Marker to World
Phi_fore = TW_FS * TFS_M;

end