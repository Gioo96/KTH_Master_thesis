function [non_singular_perc, quasi_singular_perc, singular_perc, m_best] = singular_config(q_trajectory, sh_number, fo_number, ha_number, f_J, cond_thresh, max_number_simulations, save_markers)

%% Description

% singular_config randomly computes 'max_number_simulations' set of markers and it finds the best in terms of condition number

% Inputs:
% -- q_trajectory           : Reference joint-space trajectory
% -- sh_number              : Number of markers on the shoulder
% -- fo_number              : Number of markers on the forearm
% -- ha_number              : Number of markers on the hand
% -- f_J                    : Jacobian function
% -- cond_thresh            : Condition number threshold --> if above cond_thresh then q is close to be singular
% -- max_number_simulations : Maximum number of simulations
% -- save_markers           : true: save markers of ith combination
%                             false: do not save them

% Outputs:
% -- condition_number       : List containing all condition number averages foreach simulation
%                             --> 0 if at least one singular or quasi-singular point
% -- quasi_singular_perc    : List containing all quasi-singular points, in terms of percentage, foreach simulation
% -- singular_perc          : List containing all singular points, in terms of percentage, foreach simulation
% -- m_best                 : best set of markers among the number of simulations

%% Function

% Human Arm parameters
run('human_ar_p.m');

% C code folder
C_code_folder = strcat('S', num2str(sh_number), '_F', num2str(fo_number), '_H', num2str(ha_number));

% Comment LS, EKF, KF
set_param('master_thesis_simulink/System', 'commented', 'off');
set_param('master_thesis_simulink/Ros2Matlab', 'commented', 'on');
set_param('master_thesis_simulink/Experiments', 'commented', 'on');
set_param('master_thesis_simulink/System/Human arm', 'commented', 'off');
set_param('master_thesis_simulink/System/Show Results', 'commented', 'on');
set_param('master_thesis_simulink/System/LS', 'commented', 'on');
set_param('master_thesis_simulink/System/LKF', 'commented', 'on');
set_param('master_thesis_simulink/System/EKF', 'commented', 'on');

%% Add path 'C code/C_code_folder' <-- target_folder
set_Mex(C_code_folder);

num_simulations = 0; % Total number of simulations
mean_cond_min = 1e+10;

% Best set of markers
m_shoulder_best = zeros(3, sh_number);
m_forearm_best = zeros(3, fo_number);
m_hand_best = zeros(3, ha_number);
m_best = [m_shoulder_best, m_forearm_best, m_hand_best];

% Quasi-singular points percentage foreach simulation
quasi_singular_perc = [];

% Singular points percentage foreach simulation
singular_perc = [];

% Non-Singular points percentage foreach simulation
non_singular_perc = [];

while (num_simulations < max_number_simulations)

    % Increment number of simulations
    num_simulations = num_simulations + 1;

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
    sx = (-arm.shoulder.radius-arm.markers.radius) * cos(th_s);
    sy = h_s;
    sz = (arm.shoulder.radius+arm.markers.radius) * sin(th_s);
    m_shoulder = [sx; sy; sz];

    % FOREARM
    fx = (-arm.forearm.radius-arm.markers.radius) * cos(th_f);
    fy = h_f;
    fz = (arm.forearm.radius+arm.markers.radius) * sin(th_f);
    m_forearm = [fx; fy; fz];

    % HAND
    h_xmax = arm.hand.dimensions(1)/2;
    h_xmin = -h_xmax;
    hx = (h_xmax-h_xmin).*rand(1, ha_number) + h_xmin;
    h_ymax = 0;
    h_ymin = -arm.hand.dimensions(2);
    hy = (h_ymax-h_ymin).*rand(1, ha_number) + h_ymin;
    hz = (arm.hand.dimensions(3)/2 + arm.markers.radius)*ones(1, ha_number);
    m_hand = [hx; hy; hz];

    %% Save markers of the ith simulation
    if (save_markers)

        m_shoulder_best = m_shoulder;
        m_forearm_best = m_forearm;
        m_hand_best = m_hand;
        m_best = [m_shoulder_best, m_forearm_best, m_hand_best];
     
    end
    %% See if there are singularities for a specific marker position for every point of the given trajectory 

    % Sum of all condition numbers along the trajectory
    sum_cond = 0;

    % Count the number of SINGULAR points along the trajectory
    count_singularities = 0;

    % Count the number of CLOSE TO BE SINGULAR points along the trajectory
    count_close_singularities = 0;

    % Count the number of NON-SINGULAR points along the trajectory
    count_non_singularities = 0;

    % Loop foreach point of the trajectory
    for j = 1:length(q_trajectory.time)

        q = q_trajectory.signals.values(j, :)';

        % Jacobian evaluation J(q)
        J = full(f_J(q, m_shoulder, m_forearm, m_hand));

        % Rank[J(q)]
        r = rank(J);

        % J is SINGULAR
        if (r < min(size(J,1), size(J,2)))

            count_singularities = count_singularities + 1;

        else

            % Condition number
            cond_num = cond(J);

            % J is QUASI-SINGULAR
            if (cond_num > cond_thresh)

                count_close_singularities = count_close_singularities + 1;

            % J is NOT-SINGULAR
            else

                count_non_singularities = count_non_singularities + 1;
                sum_cond = sum_cond + cond_num;
            end
        end
    end

    % Non-singularities
    %disp(strcat('NON-SINGULAR points [%]: ', num2str(count_non_singularities/length(q_trajectory.time)*100),' %'));
    non_singular_perc = [non_singular_perc count_non_singularities/length(q_trajectory.time)*100];
    
    % Singularities
    %disp(strcat('SINGULAR points [%]: ', num2str(count_singularities/length(q_trajectory.time)*100), ' %'));
    singular_perc = [singular_perc count_singularities/length(q_trajectory.time)*100];
    
    % Quasi-singularities
    %disp(strcat('CLOSE TO BE SINGULAR points [%]: ', num2str(count_close_singularities/length(q_trajectory.time)*100), ' %'));
    quasi_singular_perc = [quasi_singular_perc count_close_singularities/length(q_trajectory.time)*100];
   

    % 100% NON-SINGULAR points
    if (count_singularities == 0 && count_close_singularities == 0)

        % Mean of condition number along the trajectory
        mean_cond = sum_cond / length(q_trajectory.time);
        
        % The current set of MARKERS is better than the previous one
        if (mean_cond < mean_cond_min)

            mean_cond_min = mean_cond;
            m_shoulder_best = m_shoulder;
            m_forearm_best = m_forearm;
            m_hand_best = m_hand;
            m_best = [m_shoulder_best, m_forearm_best, m_hand_best];
        end
    end
end
end