function [T_shoulder, shoulder_variable] = FK_shoulder(marker_variable, q)

%% Description

% FK_shoulder computes the FK of a marker placed on the SHOULDER

% Inputs:
% -- marker_variable   : Variable to identify a specific marker
% -- q                 : joints variable

% Outputs:
% -- T_shoulder        : Homogeneous transformation matrix of one marker on the SHOULDER expressed wrt RF_W
% -- shoulder_variable : CasADi shoulder variable wrt RF_Sh

%% Function

% CasADi
import casadi.*

% Position of marker placed in the SHOULDER wrt its base RF
shoulder_variable = SX.sym(marker_variable, [3, 1]);

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

% TRANSLATION: Marker wrt B_shoulder
trans_BS_M = shoulder_variable;
% ROTATION+TRANSLATION: Marker to World
T_shoulder = [RW_BS RW_BS * trans_BS_M;0 0 0 1];

end