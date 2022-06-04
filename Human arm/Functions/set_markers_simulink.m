function set_markers_simulink(sh_markers, fo_markers, ha_markers)

% Number of markers in the shoulder, forearm, hand
sh_number = size(sh_markers, 2);
fo_number = size(fo_markers, 2);
ha_number = size(ha_markers, 2);
% Delete all the Markers blocks
% delete_block('master_thesis_simulink/System/Human arm/RightShoulder/Markers');
% delete_block('master_thesis_simulink/System/Human arm/RightForeArm/Markers');
% delete_block('master_thesis_simulink/System/Human arm/RightHand/Markers');

if (sh_number > 0)

    % Add Markers block
    Markers = add_block('master_thesis_simulink/System/Human arm/Markers', 'master_thesis_simulink/System/Human arm/RightShoulder/Markers', 'MakeNameUnique', 'on');
    set_param(Markers, 'Position', [-180 265 -90 385]);
    W_out = get_param('master_thesis_simulink/System/Human arm/RightShoulder/W', 'PortHandles');
    M_in = get_param(Markers, 'PortHandles');
    add_line('master_thesis_simulink/System/Human arm/RightShoulder', W_out.RConn, M_in.LConn(1), 'autorouting', 'on');
    R_out = get_param('master_thesis_simulink/System/Human arm/RightShoulder/Reference Frame', 'PortHandles');
    add_line('master_thesis_simulink/System/Human arm/RightShoulder', R_out.RConn, M_in.LConn(2), 'autorouting', 'on');

    % Inside Markers --> create more markers
    path = 'master_thesis_simulink/System/Human arm/RightShoulder/Markers';
    current_position = get_param(strcat(path, '/M1'), 'Position');
    if (sh_number > 1)

        % Set parameters for M1
        set_param(strcat(path, '/M1/translation_x'), 'TranslationCartesianOffset', '[sh_markers(1,1) 0 0]');
        set_param(strcat(path, '/M1/translation_y'), 'TranslationCartesianOffset', '[0 sh_markers(2,1) 0]');
        set_param(strcat(path, '/M1/translation_z'), 'TranslationCartesianOffset', '[0 0 sh_markers(3,1)]');
        set_param(strcat(path, '/M1/p_1'), 'Name', 'p_1');
        set_param(strcat(path, '/M1/pdot_1'), 'Name', 'pdot_1');
%         for i = 2 : sh_number
% 
%             % Copy and paste M1 block
%             Mi = add_block(strcat(path, '/M1'), strcat(path, '/M1'), 'MakeNameUnique', 'on');
%             set_param(Mi, 'Position', current_position + [0 80 0 80]);
%             current_position = get_param(Mi, 'Position');
%         end
    end
end

if (fo_number > 0)

    % Add Markers block
    Markers = add_block('master_thesis_simulink/System/Human arm/Markers', 'master_thesis_simulink/System/Human arm/RightForeArm/Markers', 'MakeNameUnique', 'on');
    set_param(Markers, 'Position', [-180 265 -90 385]);
    W_out = get_param('master_thesis_simulink/System/Human arm/RightForeArm/W', 'PortHandles');
    M_in = get_param(Markers, 'PortHandles');
    add_line('master_thesis_simulink/System/Human arm/RightForeArm', W_out.RConn, M_in.LConn(1), 'autorouting', 'on');
    R_out = get_param('master_thesis_simulink/System/Human arm/RightForeArm/Reference Frame', 'PortHandles');
    add_line('master_thesis_simulink/System/Human arm/RightForeArm', R_out.RConn, M_in.LConn(2), 'autorouting', 'on');
end

if (ha_number > 0)
    
    % Add Markers block
    Markers = add_block('master_thesis_simulink/System/Human arm/Markers', 'master_thesis_simulink/System/Human arm/RightHand/Markers', 'MakeNameUnique', 'on');
    set_param(Markers, 'Position', [-180 265 -90 385]);
    W_out = get_param('master_thesis_simulink/System/Human arm/RightHand/W', 'PortHandles');
    M_in = get_param(Markers, 'PortHandles');
    add_line('master_thesis_simulink/System/Human arm/RightHand', W_out.RConn, M_in.LConn(1), 'autorouting', 'on');
    R_out = get_param('master_thesis_simulink/System/Human arm/RightHand/Reference Frame', 'PortHandles');
    add_line('master_thesis_simulink/System/Human arm/RightHand', R_out.RConn, M_in.LConn(2), 'autorouting', 'on');
    
end
end