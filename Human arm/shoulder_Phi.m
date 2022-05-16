function Phi_shou = shoulder_Phi(marker, q)

% shoulder_Phi computes the position of the marker (placed in the SHOULDER) expressed wrt Wrold RF

% Human arm parameters
run('human_arm_param');

% CasADi
import casadi.*

% marker:      Position of marker placed in the SHOULDER wrt its base RF

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
trans_BS_M = marker;
% ROTATION+TRANSLATION: Marker to World
Phi_shou = [RW_BS RW_BS * trans_BS_M;0 0 0 1];

end