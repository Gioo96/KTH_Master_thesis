function [Phi_hand, hand_variable] = FK_hand_Experiments(marker_variable)

% hand_Phi computes the position of the marker (placed in the HAND) expressed wrt Wrold RF

% Human arm parameters
run('human_arm_parameters');

% CasADi
import casadi.*

% Position of marker placed in the HAND wrt its base RF
hand_variable = SX.sym(marker_variable, [3, 1]);

% ROTATION+TRANSLATION: Marker to M0 (I don't care about rotation)
Phi_hand = [eye(3, 3) hand_variable;0 0 0 1];

end