function singular_config(q_trajectory, sh_number, fo_number, ha_number, rcond_thresh, max_number_simulations)

global f_J;
global q0_model;

% Human Arm parameters
run('human_arm_parameters.m');

% C _code_folder
C_code_folder = strcat('S', num2str(sh_number), '_F', num2str(fo_number), '_H', num2str(ha_number));

% Comment LS, EKF, KF
set_param('master_thesis_simulink/System/LS', 'commented', 'on');
set_param('master_thesis_simulink/System/KF vs OBS', 'commented', 'on');
set_param('master_thesis_simulink/System/EKF', 'commented', 'on');

% Initialization
sh_markers_list = [];
fo_markers_list = [];
ha_markers_list = [];

%% Add path 'C code/C_code_folder' <-- target_folder
set_Mex(C_code_folder);

num_simulations_tot = 0;
num_simulations = 0;
markers_found = false;
mean_rcond_max = -1;
while ~markers_found

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

    %% See if there are singularities for a specific marker position for every point of the given trajectory 

    % Boolean variable to see if there are singularities along the trajectory
    is_Singular = false;
    sum_rcond = 0;
    for j = 1 :  length(q_trajectory.time)

        q = q_trajectory.signals.values(j, :)';
        J = full(f_J(q, m_shoulder, m_forearm, m_hand));
        rcond_num = rcond(J'*J);
        sum_rcond = sum_rcond + rcond_num;
        if rcond_num < rcond_thresh

            is_Singular = true;
        end

        if is_Singular

            break;
        end
    end

    mean_rcond = sum_rcond / length(q_trajectory.time);
    % If there are no singularities then the set of markers is good
    if ~is_Singular

        if (mean_rcond > mean_rcond_max)

            mean_rcond_max = mean_rcond;
            num_simulations = num_simulations + 1;
            disp(mean_rcond_max);
        end

        if (num_simulations == 1)
        
            markers_found = true;
        end
    end

    num_simulations_tot = num_simulations_tot + 1;
    num_simulations_tot
    if (num_simulations_tot > max_number_simulations)

        disp('No set of markers found !!')
        break;
    end
end

if markers_found

    disp(strcat('Set of SHOULDER markers: ', num2str(m_shoulder)));
    disp(strcat('Set of FOREARM markers: ', num2str(m_forearm)));
    disp(strcat('Set of HAND markers: ', num2str(m_hand)));
    sh_markers_list = [sh_markers_list m_shoulder];
    fo_markers_list = [fo_markers_list m_forearm];
    ha_markers_list = [ha_markers_list m_hand];

    % Run simulation with given trasjectory and set of markers
    set_markers_simulink(m_shoulder, m_forearm, m_hand);
    q0_model = q_trajectory.signals.values(1, :)';

    % Comment/Uncomment blocks
    set_param('master_thesis_simulink/System', 'commented', 'off');
    set_param('master_thesis_simulink/Ros2Matlab', 'commented', 'on');
    set_param('master_thesis_simulink/Experiments', 'commented', 'on');
    simulink.time = 10;
    set_param('master_thesis_simulink', 'StopTime', 'simulink.time');
    sim('master_thesis_simulink.slx');

    % Remove blocks
    remove_blocks_simulink(sh_number, fo_number, ha_number);
end
end