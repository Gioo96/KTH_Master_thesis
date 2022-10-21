function [T_forearm, forearm_variable] = FK_forearm(marker_variable, q)

%% Description

% FK_forearm computes the FK of a marker placed on the FOREARM

% Inputs:
% -- marker_variable   : Variable to identify a specific marker
% -- q                 : joints variable

% Outputs:
% -- T_forearm         : Homogeneous transformation matrix of one marker on the FOREARM expressed wrt RF_W
% -- forearm_variable  : CasADi forearm variable wrt RF_El

%% Function

% CasADi
import casadi.*

% Human arm parameters
run('human_ar_p.m');

% Position of marker placed in the FOREARM wrt its base RF
forearm_variable = SX.sym(marker_variable, [3, 1]);

% Align Reference frames
align.Rx = [1 0 0;
            0 0 -1;
            0 1 0];
align.Ry = [0 0 1;
            0 1 0;
            -1 0 0];
align.Rz = [0 -1 0;
            1 0 0;
            0 0 1];

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
Rz_el = [cos(q(4)) -sin(q(4)) 0;
            sin(q(4)) cos(q(4)) 0;
            0 0 1];

% TRANSLATION: F_shoulder wrt B_shoulder
trans_BS_FS = [0; -arm.shoulder.length; 0];
% ROTATION+TRANSLATION: F_shoulder to World
TW_FS = [RW_BS RW_BS * trans_BS_FS;0 0 0 1];
% ROTATION: F_shoulder --> B_elbow
RFS_BE = Rz_el; % F_shoulder --> B_elbow

% TRANSLATION: Marker wrt B_elbow
trans_BE_M = forearm_variable;
% ROTATION+TRANSLATION: Marker to F_shoulder
TFS_M = [RFS_BE RFS_BE * trans_BE_M;0 0 0 1];
% ROTATION+TRANSLATION: Marker to World
T_forearm = TW_FS * TFS_M;

end