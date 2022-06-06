function [Phi_hand, hand_variable] = FK_hand(marker_variable, q)

% hand_Phi computes the position of the marker (placed in the HAND) expressed wrt Wrold RF

% Human arm parameters
run('human_arm_parameters');

% CasADi
import casadi.*

% Position of marker placed in the HAND wrt its base RF
hand_variable = SX.sym(marker_variable, [3, 1]);

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

%
% WRIST
%
Rx_wr = [cos(q(5)) -sin(q(5)) 0;
            sin(q(5)) cos(q(5)) 0;
            0 0 1];
Ry_wr = [cos(q(6)) -sin(q(6)) 0;
            sin(q(6)) cos(q(6)) 0;
            0 0 1];
Rz_wr = [cos(q(7)) -sin(q(7)) 0;
            sin(q(7)) cos(q(7)) 0;
            0 0 1];

% TRANSLATION: F_elbow wrt B_elbow
trans_BE_FE = [0; -arm.forearm.length; 0];
% ROTATION+TRANSLATION: F_elbow to F_shoulder
TFS_FE = [RFS_BE RFS_BE * trans_BE_FE;0 0 0 1];
% ROTATION+TRANSLATION: F_elbow to W
TW_FE = TW_FS * TFS_FE;
% ROTATION: F_elbow --> B_wrist
RFE_x = align.Rz * align.Rx * Rx_wr; % F_elbow --> wrist.Rx
Rx_y = align.Ry * align.Rz * Ry_wr; % wrist.Rx --> wrist.Ry
Ry_BW = align.Rx * align.Ry * Rz_wr; % wrist.Ry --> B_wrist
RFE_BW = RFE_x * Rx_y * Ry_BW; % F_elbow --> B_wrist

% TRANSLATION: Marker wrt B_wrist
trans_BW_M = hand_variable;
% ROTATION+TRANSLATION: Marker to F_elbow
TFE_M = [RFE_BW RFE_BW * trans_BW_M;0 0 0 1];
% ROTATION+TRANSLATION: Marker to World
Phi_hand = TW_FE * TFE_M;

end