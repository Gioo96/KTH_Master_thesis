function [sh_markers_list, fo_markers_list, ha_markers_list] = singular_config(q_trajectory, f_J, sh_number, fo_number, ha_number, rcond_thresh)

% Run Human Arm parameters
run('human_arm_param.m');

% Initialization
sh_markers_list = [];
fo_markers_list = [];
ha_markers_list = [];

for i = 1 : 10

    disp(i);
    %% Polar coordinates
    % SHOULDER
    % theta
    th_smax = 2*pi;
    th_smin = 0;
    th_s = (th_smax-th_smin).*rand(1, sh_number) + th_smin;
    % height
    h_smax = 0;
    h_smin = -arm.shoulder.length;
    h_s = (h_smax-h_smin).*rand(1, sh_number) + h_smin;

    % FOREARM
    % theta
    th_fmax = 2*pi;
    th_fmin = 0;
    th_f = (th_fmax-th_fmin).*rand(1, fo_number) + th_fmin;
    % height
    h_fmax = 0;
    h_fmin = -arm.forearm.length;
    h_f = (h_fmax-h_fmin).*rand(1, fo_number) + h_fmin;

    %% Cartesian coordinates
    % SHOULDER
    sx = -arm.shoulder.radius * cos(th_s);
    sy = h_s;
    sz = arm.shoulder.radius * sin(th_s);
    s_markers = [sx; sy; sz];

    % FOREARM
    fx = -arm.forearm.radius * cos(th_f);
    fy = h_f;
    fz = arm.forearm.radius * sin(th_f);
    f_markers = [fx; fy; fz];

    % HAND
    h_xmax = arm.hand.dimensions(1)/2;
    h_xmin = -h_xmax;
    hx = (h_xmax-h_xmin).*rand(1, ha_number) + h_xmin;
    h_ymax = 0;
    h_ymin = -arm.hand.dimensions(2);
    hy = (h_ymax-h_ymin).*rand(1, ha_number) + h_ymin;
    hz = (arm.hand.dimensions(3)/2 + arm.markers.radius)*ones(1, ha_number);
    h_markers = [hx; hy; hz];

%     disp("SHOULDER")
%     disp(s_markers)
%     disp("FOREARM")
%     disp(f_markers)
%     disp("HAND")
%     disp(h_markers)
    %% See if there are singularities for a specific marker position for every point of the given trajectory 

    % Boolean variable to see if there are singularities along the trajectory
    is_Singular = false;
    for j = 1 :  length(q_trajectory.time)

        q = q_trajectory.signals.values(j, :)';
        J = full(f_J(q, s_markers, f_markers, h_markers));

        %if (ss(2,1) < 0 && ss(2,1) > -0.47828 && hh(2,1) < 0 && hh(2,2) < 0)
        rcond_num = rcond(J'*J);
        if rcond_num < rcond_thresh

            is_Singular = true;
            disp(rcond_num)
        end

        if is_Singular

            break;
        end
    end

    % If there are no singularities then the set of markers is good
    if ~is_Singular

        sh_markers_list = [sh_markers_list s_markers];
        fo_markers_list = [fo_markers_list f_markers];
        ha_markers_list = [ha_markers_list h_markers];
end
end