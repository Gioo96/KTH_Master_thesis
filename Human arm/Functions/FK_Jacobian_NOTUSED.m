function [p_W, J_mi] = FK_Jacobian_i(p_base, link, q, arm)

% Computes : 
% -- the position of the marker wrt World RF, given a
% specific joint variables configuration
% -- Jacobian matrix ROW related to the given marker and configuration

% Inputs:
% -- p_base : generic marker 3-coordinates wrt Base RF
% -- arm    : it contains human arm parameters
% -- link   : it can be either 1 (shoulder), 2 (forearm), 3 (hand)
% -- q      : joint variables configuration at the given time istant

% Output:
% -- p_W    : position of the marker wrt World RF 
% -- J_mi   : Jacobian contribution related to the given marker mi

p_W = [0;0;0];
J_mi = zeros(3, 7);
switch link

    % Shoulder
    case 1

        p_W = [p_base(3)*sin(q(2)) + p_base(1)*cos(q(2))*cos(q(3)) - p_base(2)*cos(q(2))*sin(q(3));
               p_base(1)*(cos(q(1))*sin(q(3)) + cos(q(3))*sin(q(1))*sin(q(2))) + p_base(2)*(cos(q(1))*cos(q(3)) - sin(q(1))*sin(q(2))*sin(q(3))) - p_base(3)*cos(q(2))*sin(q(1));
               p_base(1)*(sin(q(1))*sin(q(3)) - cos(q(1))*cos(q(3))*sin(q(2))) + p_base(2)*(cos(q(3))*sin(q(1)) + cos(q(1))*sin(q(2))*sin(q(3))) + p_base(3)*cos(q(1))*cos(q(2))];

        J_mi = [0 p_base(3)*cos(q(2))-p_base(1)*cos(q(3))*sin(q(2))+p_base(2)*sin(q(2))*sin(q(3)) -p_base(2)*cos(q(2))*cos(q(3))-p_base(1)*cos(q(2))*sin(q(3)) 0 0 0 0;
                -p_base(1)*(sin(q(1))*sin(q(3))-cos(q(1))*cos(q(3))*sin(q(2)))-p_base(2)*(cos(q(3))*sin(q(1))+cos(q(1))*sin(q(2))*sin(q(3)))-p_base(3)*cos(q(1))*cos(q(2)) p_base(3)*sin(q(1))*sin(q(2))+p_base(1)*cos(q(2))*cos(q(3))*sin(q(1))-p_base(2)*cos(q(2))*sin(q(1))*sin(q(3)) p_base(1)*(cos(q(1))*cos(q(3))-sin(q(1))*sin(q(2))*sin(q(3)))-p_base(2)*(cos(q(1))*sin(q(3))+cos(q(3))*sin(q(1))*sin(q(2))) 0 0 0 0;
                p_base(1)*(cos(q(1))*sin(q(3))+cos(q(3))*sin(q(1))*sin(q(2)))+p_base(2)*(cos(q(1))*cos(q(3))-sin(q(1))*sin(q(2))*sin(q(3)))-p_base(3)*cos(q(2))*sin(q(1)) p_base(2)*cos(q(1))*cos(q(2))*sin(q(3))-p_base(1)*cos(q(1))*cos(q(2))*cos(q(3))-p_base(3)*cos(q(1))*sin(q(2)) p_base(1)*(cos(q(3))*sin(q(1))+cos(q(1))*sin(q(2))*sin(q(3)))-p_base(2)*(sin(q(1))*sin(q(3))-cos(q(1))*cos(q(3))*sin(q(2))) 0 0 0 0];

    % ForeArm
    case 2

        p_W = [sin(q(2))*(p_base(3)*cos(q(4)) + p_base(2)*sin(q(4))) + p_base(1)*cos(q(2))*cos(q(3)) + arm.shoulder.length*cos(q(2))*sin(q(3)) - cos(q(2))*sin(q(3))*(p_base(2)*cos(q(4)) - p_base(3)*sin(q(4)));
               p_base(1)*(cos(q(1))*sin(q(3)) + cos(q(3))*sin(q(1))*sin(q(2))) - arm.shoulder.length*(cos(q(1))*cos(q(3)) - sin(q(1))*sin(q(2))*sin(q(3))) + (cos(q(1))*cos(q(3)) - sin(q(1))*sin(q(2))*sin(q(3)))*(p_base(2)*cos(q(4)) - p_base(3)*sin(q(4))) - cos(q(2))*sin(q(1))*(p_base(3)*cos(q(4)) + p_base(2)*sin(q(4)));
               p_base(1)*(sin(q(1))*sin(q(3)) - cos(q(1))*cos(q(3))*sin(q(2))) - arm.shoulder.length*(cos(q(3))*sin(q(1)) + cos(q(1))*sin(q(2))*sin(q(3))) + (cos(q(3))*sin(q(1)) + cos(q(1))*sin(q(2))*sin(q(3)))*(p_base(2)*cos(q(4)) - p_base(3)*sin(q(4))) + cos(q(1))*cos(q(2))*(p_base(3)*cos(q(4)) + p_base(2)*sin(q(4)))];

        J_mi = [0 cos(q(2))*(p_base(3)*cos(q(4))+p_base(2)*sin(q(4)))-p_base(1)*cos(q(3))*sin(q(2))-arm.shoulder.length*sin(q(2))*sin(q(3))+sin(q(2))*sin(q(3))*(p_base(2)*cos(q(4))-p_base(3)*sin(q(4))) arm.shoulder.length*cos(q(2))*cos(q(3))-p_base(1)*cos(q(2))*sin(q(3))-cos(q(2))*cos(q(3))*(p_base(2)*cos(q(4))-p_base(3)*sin(q(4))) sin(q(2))*(p_base(2)*cos(q(4))-p_base(3)*sin(q(4)))+cos(q(2))*sin(q(3))*(p_base(3)*cos(q(4))+p_base(2)*sin(q(4))) 0 0 0;
                arm.shoulder.length*(cos(q(3))*sin(q(1))+cos(q(1))*sin(q(2))*sin(q(3)))-p_base(1)*(sin(q(1))*sin(q(3))-cos(q(1))*cos(q(3))*sin(q(2)))-(cos(q(3))*sin(q(1))+cos(q(1))*sin(q(2))*sin(q(3)))*(p_base(2)*cos(q(4))-p_base(3)*sin(q(4)))-cos(q(1))*cos(q(2))*(p_base(3)*cos(q(4))+p_base(2)*sin(q(4))) sin(q(1))*sin(q(2))*(p_base(3)*cos(q(4))+p_base(2)*sin(q(4)))+p_base(1)*cos(q(2))*cos(q(3))*sin(q(1))+arm.shoulder.length*cos(q(2))*sin(q(1))*sin(q(3))-cos(q(2))*sin(q(1))*sin(q(3))*(p_base(2)*cos(q(4))-p_base(3)*sin(q(4))) p_base(1)*(cos(q(1))*cos(q(3))-sin(q(1))*sin(q(2))*sin(q(3)))+arm.shoulder.length*(cos(q(1))*sin(q(3))+cos(q(3))*sin(q(1))*sin(q(2)))-(cos(q(1))*sin(q(3))+cos(q(3))*sin(q(1))*sin(q(2)))*(p_base(2)*cos(q(4))-p_base(3)*sin(q(4))) -(cos(q(1))*cos(q(3))-sin(q(1))*sin(q(2))*sin(q(3)))*(p_base(3)*cos(q(4))+p_base(2)*sin(q(4)))-cos(q(2))*sin(q(1))*(p_base(2)*cos(q(4))-p_base(3)*sin(q(4))) 0 0 0;
                p_base(1)*(cos(q(1))*sin(q(3))+cos(q(3))*sin(q(1))*sin(q(2)))-arm.shoulder.length*(cos(q(1))*cos(q(3))-sin(q(1))*sin(q(2))*sin(q(3)))+(cos(q(1))*cos(q(3))-sin(q(1))*sin(q(2))*sin(q(3)))*(p_base(2)*cos(q(4))-p_base(3)*sin(q(4)))-cos(q(2))*sin(q(1))*(p_base(3)*cos(q(4))+p_base(2)*sin(q(4))) cos(q(1))*cos(q(2))*sin(q(3))*(p_base(2)*cos(q(4))-p_base(3)*sin(q(4)))-p_base(1)*cos(q(1))*cos(q(2))*cos(q(3))-arm.shoulder.length*cos(q(1))*cos(q(2))*sin(q(3))-cos(q(1))*sin(q(2))*(p_base(3)*cos(q(4))+p_base(2)*sin(q(4))) p_base(1)*(cos(q(3))*sin(q(1))+cos(q(1))*sin(q(2))*sin(q(3)))+arm.shoulder.length*(sin(q(1))*sin(q(3))-cos(q(1))*cos(q(3))*sin(q(2)))-(sin(q(1))*sin(q(3))-cos(q(1))*cos(q(3))*sin(q(2)))*(p_base(2)*cos(q(4))-p_base(3)*sin(q(4))) cos(q(1))*cos(q(2))*(p_base(2)*cos(q(4))-p_base(3)*sin(q(4)))-(cos(q(3))*sin(q(1))+cos(q(1))*sin(q(2))*sin(q(3)))*(p_base(3)*cos(q(4))+p_base(2)*sin(q(4))) 0 0 0];
    % Hand
    case 3
        p_W = [(cos(q(4))*sin(q(2)) + cos(q(2))*sin(q(3))*sin(q(4)))*(p_base(1)*(sin(q(5))*sin(q(7)) - cos(q(5))*cos(q(7))*sin(q(6))) + p_base(2)*(cos(q(7))*sin(q(5)) + cos(q(5))*sin(q(6))*sin(q(7))) + p_base(3)*cos(q(5))*cos(q(6))) + (sin(q(2))*sin(q(4)) - cos(q(2))*cos(q(4))*sin(q(3)))*(p_base(1)*(cos(q(5))*sin(q(7)) + cos(q(7))*sin(q(5))*sin(q(6))) + p_base(2)*(cos(q(5))*cos(q(7)) - sin(q(5))*sin(q(6))*sin(q(7))) - p_base(3)*cos(q(6))*sin(q(5))) + arm.shoulder.length*cos(q(2))*sin(q(3)) - arm.forearm.length*sin(q(2))*sin(q(4)) + cos(q(2))*cos(q(3))*(p_base(3)*sin(q(6)) + p_base(1)*cos(q(6))*cos(q(7)) - p_base(2)*cos(q(6))*sin(q(7))) + arm.forearm.length*cos(q(2))*cos(q(4))*sin(q(3));
               (cos(q(4))*(cos(q(1))*cos(q(3)) - sin(q(1))*sin(q(2))*sin(q(3))) - cos(q(2))*sin(q(1))*sin(q(4)))*(p_base(1)*(cos(q(5))*sin(q(7)) + cos(q(7))*sin(q(5))*sin(q(6))) + p_base(2)*(cos(q(5))*cos(q(7)) - sin(q(5))*sin(q(6))*sin(q(7))) - p_base(3)*cos(q(6))*sin(q(5))) - (sin(q(4))*(cos(q(1))*cos(q(3)) - sin(q(1))*sin(q(2))*sin(q(3))) + cos(q(2))*cos(q(4))*sin(q(1)))*(p_base(1)*(sin(q(5))*sin(q(7)) - cos(q(5))*cos(q(7))*sin(q(6))) + p_base(2)*(cos(q(7))*sin(q(5)) + cos(q(5))*sin(q(6))*sin(q(7))) + p_base(3)*cos(q(5))*cos(q(6))) - arm.shoulder.length*(cos(q(1))*cos(q(3)) - sin(q(1))*sin(q(2))*sin(q(3))) + (cos(q(1))*sin(q(3)) + cos(q(3))*sin(q(1))*sin(q(2)))*(p_base(3)*sin(q(6)) + p_base(1)*cos(q(6))*cos(q(7)) - p_base(2)*cos(q(6))*sin(q(7))) - arm.forearm.length*cos(q(4))*(cos(q(1))*cos(q(3)) - sin(q(1))*sin(q(2))*sin(q(3))) + arm.forearm.length*cos(q(2))*sin(q(1))*sin(q(4));
               (cos(q(4))*(cos(q(3))*sin(q(1)) + cos(q(1))*sin(q(2))*sin(q(3))) + cos(q(1))*cos(q(2))*sin(q(4)))*(p_base(1)*(cos(q(5))*sin(q(7)) + cos(q(7))*sin(q(5))*sin(q(6))) + p_base(2)*(cos(q(5))*cos(q(7)) - sin(q(5))*sin(q(6))*sin(q(7))) - p_base(3)*cos(q(6))*sin(q(5))) - (sin(q(4))*(cos(q(3))*sin(q(1)) + cos(q(1))*sin(q(2))*sin(q(3))) - cos(q(1))*cos(q(2))*cos(q(4)))*(p_base(1)*(sin(q(5))*sin(q(7)) - cos(q(5))*cos(q(7))*sin(q(6))) + p_base(2)*(cos(q(7))*sin(q(5)) + cos(q(5))*sin(q(6))*sin(q(7))) + p_base(3)*cos(q(5))*cos(q(6))) - arm.shoulder.length*(cos(q(3))*sin(q(1)) + cos(q(1))*sin(q(2))*sin(q(3))) + (sin(q(1))*sin(q(3)) - cos(q(1))*cos(q(3))*sin(q(2)))*(p_base(3)*sin(q(6)) + p_base(1)*cos(q(6))*cos(q(7)) - p_base(2)*cos(q(6))*sin(q(7))) - arm.forearm.length*cos(q(4))*(cos(q(3))*sin(q(1)) + cos(q(1))*sin(q(2))*sin(q(3))) - arm.forearm.length*cos(q(1))*cos(q(2))*sin(q(4))];

        sigma_1 = p_base(3)*sin(q(6))+p_base(1)*cos(q(6))*cos(q(7))-p_base(2)*cos(q(6))*sin(q(7));
        sigma_2 = sin(q(1))*sin(q(3))-cos(q(1))*cos(q(3))*sin(q(2));
        sigma_3 = cos(q(1))*sin(q(3))+cos(q(3))*sin(q(1))*sin(q(2));
        sigma_4 = p_base(3)*sin(q(5))*sin(q(6))+p_base(1)*cos(q(6))*cos(q(7))*sin(q(5))-p_base(2)*cos(q(6))*sin(q(5))*sin(q(7));
        sigma_5 = p_base(3)*cos(q(5))*sin(q(6))+p_base(1)*cos(q(5))*cos(q(6))*cos(q(7))-p_base(2)*cos(q(5))*cos(q(6))*sin(q(7));
        sigma_6 = p_base(3)*cos(q(6))-p_base(1)*cos(q(7))*sin(q(6))+p_base(2)*sin(q(6))*sin(q(7));
        sigma_7 = sin(q(2))*sin(q(4))-cos(q(2))*cos(q(4))*sin(q(3));
        sigma_8 = cos(q(4))*sin(q(2))+cos(q(2))*sin(q(3))*sin(q(4));
        sigma_9 = p_base(2)*cos(q(6))*cos(q(7))+p_base(1)*cos(q(6))*sin(q(7));
        sigma_18 = cos(q(5))*cos(q(7))-sin(q(5))*sin(q(6))*sin(q(7));
        sigma_19 = cos(q(5))*sin(q(7))+cos(q(7))*sin(q(5))*sin(q(6));
        sigma_20 = sin(q(5))*sin(q(7))-cos(q(5))*cos(q(7))*sin(q(6));
        sigma_21 = cos(q(7))*sin(q(5))+cos(q(5))*sin(q(6))*sin(q(7));
        sigma_22 = cos(q(1))*cos(q(3))-sin(q(1))*sin(q(2))*sin(q(3));
        sigma_23 = cos(q(3))*sin(q(1))+cos(q(1))*sin(q(2))*sin(q(3));
        sigma_10 = p_base(1)*sigma_19+p_base(2)*sigma_18-p_base(3)*cos(q(6))*sin(q(5));
        sigma_11 = p_base(1)*sigma_20+p_base(2)*sigma_21+p_base(3)*cos(q(5))*cos(q(6));
        sigma_12 = p_base(1)*sigma_18-p_base(2)*sigma_19;
        sigma_13 = p_base(1)*sigma_21-p_base(2)*sigma_20;
        sigma_14 = cos(q(4))*sigma_22-cos(q(2))*sin(q(1))*sin(q(4));
        sigma_15 = sin(q(4))*sigma_22+cos(q(2))*cos(q(4))*sin(q(1));
        sigma_16 = sin(q(4))*sigma_23-cos(q(1))*cos(q(2))*cos(q(4));
        sigma_17 = cos(q(4))*sigma_23+cos(q(1))*cos(q(2))*sin(q(4));

        J_mi = [0 (cos(q(2))*cos(q(4))-sin(q(2))*sin(q(3))*sin(q(4)))*sigma_11+(cos(q(2))*sin(q(4))+cos(q(4))*sin(q(2))*sin(q(3)))*sigma_10-arm.forearm.length*cos(q(2))*sin(q(4))-arm.shoulder.length*sin(q(2))*sin(q(3))-cos(q(3))*sin(q(2))*sigma_1-arm.forearm.length*cos(q(4))*sin(q(2))*sin(q(3)) arm.shoulder.length*cos(q(2))*cos(q(3))-cos(q(2))*sin(q(3))*sigma_1+cos(q(2))*cos(q(3))*sin(q(4))*sigma_11-cos(q(2))*cos(q(3))*cos(q(4))*sigma_10+arm.forearm.length*cos(q(2))*cos(q(3))*cos(q(4)) sigma_8*sigma_10-sigma_7*sigma_11-arm.forearm.length*cos(q(4))*sin(q(2))-arm.forearm.length*cos(q(2))*sin(q(3))*sin(q(4)) sigma_8*sigma_10-sigma_7*sigma_11 sigma_7*sigma_4-sigma_8*sigma_5+cos(q(2))*cos(q(3))*sigma_6 sigma_8*sigma_13+sigma_7*sigma_12-cos(q(2))*cos(q(3))*sigma_9;            
                arm.shoulder.length*sigma_23+sigma_16*sigma_11-sigma_17*sigma_10-sigma_2*sigma_1+arm.forearm.length*cos(q(4))*sigma_23+arm.forearm.length*cos(q(1))*cos(q(2))*sin(q(4)) (cos(q(4))*sin(q(1))*sin(q(2))+cos(q(2))*sin(q(1))*sin(q(3))*sin(q(4)))*sigma_11+(sin(q(1))*sin(q(2))*sin(q(4))-cos(q(2))*cos(q(4))*sin(q(1))*sin(q(3)))*sigma_10+arm.shoulder.length*cos(q(2))*sin(q(1))*sin(q(3))-arm.forearm.length*sin(q(1))*sin(q(2))*sin(q(4))+cos(q(2))*cos(q(3))*sin(q(1))*sigma_1+arm.forearm.length*cos(q(2))*cos(q(4))*sin(q(1))*sin(q(3)) arm.shoulder.length*sigma_3+sigma_22*sigma_1+arm.forearm.length*cos(q(4))*sigma_3+sin(q(4))*sigma_3*sigma_11-cos(q(4))*sigma_3*sigma_10 arm.forearm.length*sin(q(4))*sigma_22-sigma_15*sigma_10-sigma_14*sigma_11+arm.forearm.length*cos(q(2))*cos(q(4))*sin(q(1)) -sigma_14*sigma_11-sigma_15*sigma_10 sigma_15*sigma_5+sigma_3*sigma_6+sigma_14*sigma_4 sigma_14*sigma_12-sigma_15*sigma_13-sigma_9*sigma_3;
                sigma_14*sigma_10-sigma_15*sigma_11-arm.shoulder.length*sigma_22+sigma_3*sigma_1-arm.forearm.length*cos(q(4))*sigma_22+arm.forearm.length*cos(q(2))*sin(q(1))*sin(q(4)) arm.forearm.length*cos(q(1))*sin(q(2))*sin(q(4))-(cos(q(1))*sin(q(2))*sin(q(4))-cos(q(1))*cos(q(2))*cos(q(4))*sin(q(3)))*sigma_10-arm.shoulder.length*cos(q(1))*cos(q(2))*sin(q(3))-(cos(q(1))*cos(q(4))*sin(q(2)) + cos(q(1))*cos(q(2))*sin(q(3))*sin(q(4)))*sigma_11-cos(q(1))*cos(q(2))*cos(q(3))*sigma_1-arm.forearm.length*cos(q(1))*cos(q(2))*cos(q(4))*sin(q(3)) arm.shoulder.length*sigma_2+sigma_23*sigma_1+arm.forearm.length*cos(q(4))*sigma_2+sin(q(4))*sigma_2*sigma_11-cos(q(4))*sigma_2*sigma_10 arm.forearm.length*sin(q(4))*sigma_23-sigma_16*sigma_10-sigma_17*sigma_11-arm.forearm.length*cos(q(1))*cos(q(2))*cos(q(4)) -sigma_17*sigma_11-sigma_16*sigma_10 sigma_16*sigma_5+sigma_2*sigma_6+sigma_17*sigma_4 sigma_12*sigma_17-sigma_9*sigma_2-sigma_13*sigma_16];
end
end