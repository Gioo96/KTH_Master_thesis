function set_markers_simulink(sh_markers, fo_markers, ha_markers)

% Set global variables
global markers_shoulder markers_forearm markers_hand;
markers_shoulder = sh_markers;
markers_forearm = fo_markers;
markers_hand = ha_markers;

if (size(sh_markers, 2) == 0)

    markers_shoulder = -1;

elseif (size(fo_markers, 2) == 0)

    markers_forearm = -1;

elseif (size(ha_markers, 2) == 0)

    markers_hand = -1;
    
end

% Number of markers in the shoulder, forearm, hand
sh_number = size(sh_markers, 2);
fo_number = size(fo_markers, 2);
ha_number = size(ha_markers, 2);

% Count number of markers
count = 0;
if (sh_number > 0)

    % Add Markers block
    Markers = add_block('master_thesis_simulink/System/Human arm/Markers', 'master_thesis_simulink/System/Human arm/RightShoulder/Markers', 'MakeNameUnique', 'on');
    set_param(Markers, 'commented', 'off');
    set_param(Markers, 'Position', [-180 265 -90 385]);
    W_out = get_param('master_thesis_simulink/System/Human arm/RightShoulder/W', 'PortHandles').RConn;
    M_in1 = get_param(Markers, 'PortHandles').LConn(1);
    M_in2 = get_param(Markers, 'PortHandles').LConn(2);
    add_line('master_thesis_simulink/System/Human arm/RightShoulder', W_out, M_in1, 'autorouting', 'on');
    R_out = get_param('master_thesis_simulink/System/Human arm/RightShoulder/Reference Frame', 'PortHandles').RConn;
    add_line('master_thesis_simulink/System/Human arm/RightShoulder', R_out, M_in2, 'autorouting', 'on');

    % Inside Markers --> create more markers
    path = 'master_thesis_simulink/System/Human arm/RightShoulder/Markers';
    current_position = get_param(strcat(path, '/M1'), 'Position');
    count = count + 1;
    pi_name = strcat('p_', num2str(count));
    pdoti_name = strcat('pdot_', num2str(count));
    set_param(strcat(path, '/p_1'), 'GotoTag', pi_name);
    set_param(strcat(path, '/pdot_1'), 'GotoTag', pdoti_name);
    % Set parameters for M1
    set_param(strcat(path, '/M1/translation_x'), 'TranslationCartesianOffset', '[markers_shoulder(1,1) 0 0]');
    set_param(strcat(path, '/M1/translation_y'), 'TranslationCartesianOffset', '[0 markers_shoulder(2,1) 0]');
    set_param(strcat(path, '/M1/translation_z'), 'TranslationCartesianOffset', '[0 0 markers_shoulder(3,1)]');
    set_param(strcat(path, '/M1/translation_R2Base'), 'TranslationCartesianOffset', '[0 arm.shoulder.length/2 0]');
    set_param(strcat(path, '/M1/p_1'), 'Name', 'p_1');
    set_param(strcat(path, '/M1/pdot_1'), 'Name', 'pdot_1');
    current_position_pi = get_param(strcat(path, '/p_1'), 'Position');
    current_position_pdoti = get_param(strcat(path, '/pdot_1'), 'Position');

    if (sh_number > 1)

        for i = 2 : sh_number

            count = count + 1;

            % Copy and paste M1 block
            Mi = add_block(strcat(path, '/M1'), strcat(path, '/M1'), 'MakeNameUnique', 'on');
            set_param(Mi, 'Position', current_position + [0 80 0 80]);
            current_position = get_param(Mi, 'Position');
            Mi_in1 = get_param(Mi, 'PortHandles').LConn(1);
            Mi_in2 = get_param(Mi, 'PortHandles').LConn(2);
            W_out = get_param(strcat(path, '/W')', 'PortHandles').RConn;
            R_out = get_param(strcat(path, '/R')', 'PortHandles').RConn;
            add_line(path, W_out, Mi_in1, 'autorouting', 'on');
            add_line(path, R_out, Mi_in2, 'autorouting', 'on');

            % Set markers parameters
            marker_i = strcat(path, '/M', num2str(i));
            set_param(strcat(marker_i, '/translation_x'), 'TranslationCartesianOffset', strcat('[markers_shoulder(1,', num2str(i), ') 0 0]'));
            set_param(strcat(marker_i, '/translation_y'), 'TranslationCartesianOffset', strcat('[0 markers_shoulder(2,', num2str(i), ') 0]'));
            set_param(strcat(marker_i, '/translation_z'), 'TranslationCartesianOffset', strcat('[0 0 markers_shoulder(3,', num2str(i), ')]'));
            set_param(strcat(marker_i, '/translation_R2Base'), 'TranslationCartesianOffset', '[0 arm.shoulder.length/2 0]');

            % Add connections and output outside M2,... blocks
            pi = add_block(strcat(path, '/p_1'), strcat(path, '/p_1'), 'MakeNameUnique', 'on');
            pi_out = get_param('master_thesis_simulink/System/Human arm/RightShoulder/Markers/M2', 'PortHandles').LConn(1);
            pi_in = get_param(pi, 'PortHandles').Inport;
            set_param(pi, 'Position', current_position_pi + [0 80 0 80]);
            current_position_pi = get_param(pi, 'Position');
            add_line('master_thesis_simulink/System/Human arm/RightShoulder/Markers', get_param(Mi, 'PortHandles').Outport(1), pi_in, 'autorouting', 'on');
            pi_name = strcat('p_', num2str(count));
            set_param(pi, 'GotoTag', pi_name);

            pdoti = add_block(strcat(path, '/pdot_1'), strcat(path, '/pdot_1'), 'MakeNameUnique', 'on');
            pdoti_out = get_param('master_thesis_simulink/System/Human arm/RightShoulder/Markers/M2', 'PortHandles').LConn(2);
            pdoti_in = get_param(pdoti, 'PortHandles').Inport;
            set_param(pdoti, 'Position', current_position_pdoti + [0 80 0 80]);
            current_position_pdoti = get_param(pdoti, 'Position');
            add_line('master_thesis_simulink/System/Human arm/RightShoulder/Markers', get_param(Mi, 'PortHandles').Outport(2), pdoti_in, 'autorouting', 'on');
            pdoti_name = strcat('pdot_', num2str(count));
            set_param(pdoti, 'GotoTag', pdoti_name);
            set_param(pdoti, 'Name', pdoti_name);
        end
    end
end

if (fo_number > 0)

    % Add Markers block
    Markers = add_block('master_thesis_simulink/System/Human arm/Markers', 'master_thesis_simulink/System/Human arm/RightForeArm/Markers', 'MakeNameUnique', 'on');
    set_param(Markers, 'commented', 'off');
    set_param(Markers, 'Position', [-180 265 -90 385]);
    W_out = get_param('master_thesis_simulink/System/Human arm/RightForeArm/W', 'PortHandles').RConn;
    M_in1 = get_param(Markers, 'PortHandles').LConn(1);
    M_in2 = get_param(Markers, 'PortHandles').LConn(2);
    add_line('master_thesis_simulink/System/Human arm/RightForeArm', W_out, M_in1, 'autorouting', 'on');
    R_out = get_param('master_thesis_simulink/System/Human arm/RightForeArm/Reference Frame', 'PortHandles').RConn;
    add_line('master_thesis_simulink/System/Human arm/RightForeArm', R_out, M_in2, 'autorouting', 'on');

    % Inside Markers --> create more markers
    path = 'master_thesis_simulink/System/Human arm/RightForeArm/Markers';
    current_position = get_param(strcat(path, '/M1'), 'Position');
    count = count + 1;
    pi_name = strcat('p_', num2str(count));
    pdoti_name = strcat('pdot_', num2str(count));
    set_param(strcat(path, '/p_1'), 'GotoTag', pi_name);
    set_param(strcat(path, '/pdot_1'), 'GotoTag', pdoti_name);
    % Set parameters for M1
    set_param(strcat(path, '/M1/translation_x'), 'TranslationCartesianOffset', '[markers_forearm(1,1) 0 0]');
    set_param(strcat(path, '/M1/translation_y'), 'TranslationCartesianOffset', '[0 markers_forearm(2,1) 0]');
    set_param(strcat(path, '/M1/translation_z'), 'TranslationCartesianOffset', '[0 0 markers_forearm(3,1)]');
    set_param(strcat(path, '/M1/translation_R2Base'), 'TranslationCartesianOffset', '[0 arm.forearm.length/2 0]');
    set_param(strcat(path, '/M1/p_1'), 'Name', 'p_1');
    set_param(strcat(path, '/M1/pdot_1'), 'Name', 'pdot_1');
    current_position_pi = get_param(strcat(path, '/p_1'), 'Position');
    current_position_pdoti = get_param(strcat(path, '/pdot_1'), 'Position');

    if (fo_number > 1)

        for i = 2 : fo_number

            count = count + 1;

            % Copy and paste M1 block
            Mi = add_block(strcat(path, '/M1'), strcat(path, '/M1'), 'MakeNameUnique', 'on');
            set_param(Mi, 'Position', current_position + [0 80 0 80]);
            current_position = get_param(Mi, 'Position');
            Mi_in1 = get_param(Mi, 'PortHandles').LConn(1);
            Mi_in2 = get_param(Mi, 'PortHandles').LConn(2);
            W_out = get_param(strcat(path, '/W')', 'PortHandles').RConn;
            R_out = get_param(strcat(path, '/R')', 'PortHandles').RConn;
            add_line(path, W_out, Mi_in1, 'autorouting', 'on');
            add_line(path, R_out, Mi_in2, 'autorouting', 'on');

            % Set markers parameters
            marker_i = strcat(path, '/M', num2str(i));
            set_param(strcat(marker_i, '/translation_x'), 'TranslationCartesianOffset', strcat('[markers_forearm(1,', num2str(i), ') 0 0]'));
            set_param(strcat(marker_i, '/translation_y'), 'TranslationCartesianOffset', strcat('[0 markers_forearm(2,', num2str(i), ') 0]'));
            set_param(strcat(marker_i, '/translation_z'), 'TranslationCartesianOffset', strcat('[0 0 markers_forearm(3,', num2str(i), ')]'));
            set_param(strcat(marker_i, '/translation_R2Base'), 'TranslationCartesianOffset', '[0 arm.forearm.length/2 0]');

            % Add connections and output outside M2,... blocks
            pi = add_block(strcat(path, '/p_1'), strcat(path, '/p_1'), 'MakeNameUnique', 'on');
            pi_out = get_param('master_thesis_simulink/System/Human arm/RightForeArm/Markers/M2', 'PortHandles').LConn(1);
            pi_in = get_param(pi, 'PortHandles').Inport;
            set_param(pi, 'Position', current_position_pi + [0 80 0 80]);
            current_position_pi = get_param(pi, 'Position');
            add_line('master_thesis_simulink/System/Human arm/RightForeArm/Markers', get_param(Mi, 'PortHandles').Outport(1), pi_in, 'autorouting', 'on');
            pi_name = strcat('p_', num2str(count));
            set_param(pi, 'GotoTag', pi_name);

            pdoti = add_block(strcat(path, '/pdot_1'), strcat(path, '/pdot_1'), 'MakeNameUnique', 'on');
            pdoti_out = get_param('master_thesis_simulink/System/Human arm/RightForeArm/Markers/M2', 'PortHandles').LConn(2);
            pdoti_in = get_param(pdoti, 'PortHandles').Inport;
            set_param(pdoti, 'Position', current_position_pdoti + [0 80 0 80]);
            current_position_pdoti = get_param(pdoti, 'Position');
            add_line('master_thesis_simulink/System/Human arm/RightForeArm/Markers', get_param(Mi, 'PortHandles').Outport(2), pdoti_in, 'autorouting', 'on');
            pdoti_name = strcat('pdot_', num2str(count));
            set_param(pdoti, 'GotoTag', pdoti_name);
        end
    end
end

if (ha_number > 0)
    
    % Add Markers block
    Markers = add_block('master_thesis_simulink/System/Human arm/Markers', 'master_thesis_simulink/System/Human arm/RightHand/Markers', 'MakeNameUnique', 'on');
    set_param(Markers, 'commented', 'off');
    set_param(Markers, 'Position', [-180 265 -90 385]);
    W_out = get_param('master_thesis_simulink/System/Human arm/RightHand/W', 'PortHandles').RConn;
    M_in1 = get_param(Markers, 'PortHandles').LConn(1);
    M_in2 = get_param(Markers, 'PortHandles').LConn(2);
    add_line('master_thesis_simulink/System/Human arm/RightHand', W_out, M_in1, 'autorouting', 'on');
    R_out = get_param('master_thesis_simulink/System/Human arm/RightHand/Reference Frame', 'PortHandles').RConn;
    add_line('master_thesis_simulink/System/Human arm/RightHand', R_out, M_in2, 'autorouting', 'on');
    
    % Inside Markers --> create more markers
    path = 'master_thesis_simulink/System/Human arm/RightHand/Markers';
    current_position = get_param(strcat(path, '/M1'), 'Position');
    count = count + 1;
    pi_name = strcat('p_', num2str(count));
    pdoti_name = strcat('pdot_', num2str(count));
    set_param(strcat(path, '/p_1'), 'GotoTag', pi_name);
    set_param(strcat(path, '/pdot_1'), 'GotoTag', pdoti_name);
    % Set parameters for M1
    set_param(strcat(path, '/M1/translation_x'), 'TranslationCartesianOffset', '[markers_hand(1,1) 0 0]');
    set_param(strcat(path, '/M1/translation_y'), 'TranslationCartesianOffset', '[0 markers_hand(2,1) 0]');
    set_param(strcat(path, '/M1/translation_z'), 'TranslationCartesianOffset', '[0 0 markers_hand(3,1)]');
    set_param(strcat(path, '/M1/translation_R2Base'), 'TranslationCartesianOffset', '[0 arm.hand.dimensions(2)/2 0]');
    set_param(strcat(path, '/M1/p_1'), 'Name', 'p_1');
    set_param(strcat(path, '/M1/pdot_1'), 'Name', 'pdot_1');
    current_position_pi = get_param(strcat(path, '/p_1'), 'Position');
    current_position_pdoti = get_param(strcat(path, '/pdot_1'), 'Position');

    if (ha_number > 1)

        for i = 2 : ha_number

            count = count + 1;
            % Copy and paste M1 block
            Mi = add_block(strcat(path, '/M1'), strcat(path, '/M1'), 'MakeNameUnique', 'on');
            set_param(Mi, 'Position', current_position + [0 80 0 80]);
            current_position = get_param(Mi, 'Position');
            Mi_in1 = get_param(Mi, 'PortHandles').LConn(1);
            Mi_in2 = get_param(Mi, 'PortHandles').LConn(2);
            W_out = get_param(strcat(path, '/W')', 'PortHandles').RConn;
            R_out = get_param(strcat(path, '/R')', 'PortHandles').RConn;
            add_line(path, W_out, Mi_in1, 'autorouting', 'on');
            add_line(path, R_out, Mi_in2, 'autorouting', 'on');

            % Set markers parameters
            marker_i = strcat(path, '/M', num2str(i));
            set_param(strcat(marker_i, '/translation_x'), 'TranslationCartesianOffset', strcat('[markers_hand(1,', num2str(i), ') 0 0]'));
            set_param(strcat(marker_i, '/translation_y'), 'TranslationCartesianOffset', strcat('[0 markers_hand(2,', num2str(i), ') 0]'));
            set_param(strcat(marker_i, '/translation_z'), 'TranslationCartesianOffset', strcat('[0 0 markers_hand(3,', num2str(i), ')]'));
            set_param(strcat(marker_i, '/translation_R2Base'), 'TranslationCartesianOffset', '[0 arm.hand.dimensions(2)/2 0]');

            % Add connections and output outside M2,... blocks
            pi = add_block(strcat(path, '/p_1'), strcat(path, '/p_1'), 'MakeNameUnique', 'on');
            pi_out = get_param('master_thesis_simulink/System/Human arm/RightHand/Markers/M2', 'PortHandles').LConn(1);
            pi_in = get_param(pi, 'PortHandles').Inport;
            set_param(pi, 'Position', current_position_pi + [0 80 0 80]);
            current_position_pi = get_param(pi, 'Position');
            add_line('master_thesis_simulink/System/Human arm/RightHand/Markers', get_param(Mi, 'PortHandles').Outport(1), pi_in, 'autorouting', 'on');
            pi_name = strcat('p_', num2str(count));
            set_param(pi, 'GotoTag', pi_name);

            pdoti = add_block(strcat(path, '/pdot_1'), strcat(path, '/pdot_1'), 'MakeNameUnique', 'on');
            pdoti_out = get_param('master_thesis_simulink/System/Human arm/RightHand/Markers/M2', 'PortHandles').LConn(2);
            pdoti_in = get_param(pdoti, 'PortHandles').Inport;
            set_param(pdoti, 'Position', current_position_pdoti + [0 80 0 80]);
            current_position_pdoti = get_param(pdoti, 'Position');
            add_line('master_thesis_simulink/System/Human arm/RightHand/Markers', get_param(Mi, 'PortHandles').Outport(2), pdoti_in, 'autorouting', 'on');
            pdoti_name = strcat('pdot_', num2str(count));
            set_param(pdoti, 'GotoTag', pdoti_name);
            
       end
   end
end

% Mux, Mux1
curr_pos_Mux = get_param('master_thesis_simulink/System/Human arm/Mux', 'Position') - [120 40 80 310];
curr_pos_Mux1 = get_param('master_thesis_simulink/System/Human arm/Mux1', 'Position') - [120 40 80 310];
set_param('master_thesis_simulink/System/Human arm/Mux', 'Inputs', 'm');
set_param('master_thesis_simulink/System/Human arm/Mux1', 'Inputs', 'm');
for i = 1 : count
    
    p_from = add_block('simulink/Signal Routing/From', strcat('master_thesis_simulink/System/Human arm/p_', num2str(i)), 'MakeNameUnique', 'on');
    set_param(p_from, 'GotoTag', strcat('p_', num2str(i)));
    mux_in_i = get_param('master_thesis_simulink/System/Human arm/Mux', 'PortHandles').Inport(i);
    p_out_i = get_param(p_from, 'PortHandles').Outport;

    pdot_from = add_block('simulink/Signal Routing/From', strcat('master_thesis_simulink/System/Human arm/pdot_', num2str(i)), 'MakeNameUnique', 'on');
    set_param(pdot_from, 'GotoTag', strcat('pdot_', num2str(i)));
    mux1_in_i = get_param('master_thesis_simulink/System/Human arm/Mux1', 'PortHandles').Inport(i);
    pdot_out_i = get_param(pdot_from, 'PortHandles').Outport;
    if i == 1
    
        set_param(p_from, 'Position', curr_pos_Mux);
        set_param(pdot_from, 'Position', curr_pos_Mux1);
    end
    set_param(p_from, 'Position', curr_pos_Mux + [0 70 0 70]);
    add_line('master_thesis_simulink/System/Human arm/', p_out_i, mux_in_i, 'autorouting', 'on');
    curr_pos_Mux = get_param(p_from, 'Position');

    set_param(pdot_from, 'Position', curr_pos_Mux1 + [0 70 0 70]);
    add_line('master_thesis_simulink/System/Human arm/', pdot_out_i, mux1_in_i, 'autorouting', 'on');
    curr_pos_Mux1 = get_param(pdot_from, 'Position');
end
end