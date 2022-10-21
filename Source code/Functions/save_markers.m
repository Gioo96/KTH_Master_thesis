function save_markers(directory)

%% Description

% save_markers save set of markers given the directory

%% Function

% Run Human arm parameters
run('human_ar_p.m');

%% Polar coordinates
% Number of markers
sh_number = str2double(directory(2));
fo_number = str2double(directory(5));
ha_number = str2double(directory(8));

if (sh_number == 1 && fo_number == 1 && ha_number ==2)

    disp('6th combination already computed')
else

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
    
    markers_best = [m_shoulder, m_forearm, m_hand];
    if (sh_number == 0 && fo_number == 1 && ha_number == 2)
    
        num_sim = 1;
        name_str = ['markers_best_', num2str(num_sim), 'st.mat'];
    
    elseif (sh_number == 1 && fo_number == 0 && ha_number == 2)
    
        num_sim = 2;
        name_str = ['markers_best_', num2str(num_sim), 'nd.mat'];
    
    elseif (sh_number == 0 && fo_number == 2 && ha_number == 2)
    
        num_sim = 4;
        name_str = ['markers_best_', num2str(num_sim), 'th.mat'];
    
    elseif (sh_number == 1 && fo_number == 0 && ha_number == 3)
    
        num_sim = 5;
        name_str = ['markers_best_', num2str(num_sim), 'th.mat'];
    
    elseif (sh_number == 2 && fo_number == 0 && ha_number == 2)
    
        num_sim = 8;
        name_str = ['markers_best_', num2str(num_sim), 'th.mat'];
    
    elseif (sh_number == 1 && fo_number == 2 && ha_number == 2)
    
        num_sim = 9;
        name_str = ['markers_best_', num2str(num_sim), 'th.mat'];
    end
    
    save(name_str, 'markers_best');
end
 
end