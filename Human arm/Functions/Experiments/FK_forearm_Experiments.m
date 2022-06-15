function [Phi_fore, forearm_variable] = FK_forearm_Experiments(marker_variable)

% forearm_Phi computes the position of the marker (placed in the FOREARM) expressed wrt Wrold RF

% Human arm parameters
run('human_arm_parameters');

% CasADi
import casadi.*

% Position of marker placed in the FOREARM wrt M0
forearm_variable = SX.sym(marker_variable, [3, 1]);

% ROTATION+TRANSLATION: Marker to M0 (I don't care about rotation)
Phi_fore = [eye(3, 3) forearm_variable;0 0 0 1];

end