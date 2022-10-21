function remove_blocks_simulink(shoulder_markers, forearm_markers, hand_markers)

% Number of markers
sh_number = size(shoulder_markers, 2);
fo_number = size(forearm_markers, 2);
ha_number = size(hand_markers, 2);
if (shoulder_markers == -1)

    sh_number = 0;
end

if (forearm_markers == -1)

    fo_number = 0;
end

if (hand_markers == -1)

    ha_number = 0;
end

m = sh_number + fo_number + ha_number;

% Remove blocks
if (sh_number > 0)

    Markers_lines =  get_param('master_thesis_simulink/System/Human arm/RightShoulder/Markers', 'LineHandles');
    input_lines = Markers_lines.LConn;
    delete_line(input_lines);
    delete_block('master_thesis_simulink/System/Human arm/RightShoulder/Markers');
end

if (fo_number > 0)
    
    Markers_lines =  get_param('master_thesis_simulink/System/Human arm/RightForeArm/Markers', 'LineHandles');
    input_lines = Markers_lines.LConn;
    delete_line(input_lines);
    delete_block('master_thesis_simulink/System/Human arm/RightForeArm/Markers');
end

if (ha_number > 0)
    
    Markers_lines =  get_param('master_thesis_simulink/System/Human arm/RightHand/Markers', 'LineHandles');
    input_lines = Markers_lines.LConn;
    delete_line(input_lines);
    delete_block('master_thesis_simulink/System/Human arm/RightHand/Markers');
end

for j = 1:m

    p_from = strcat('master_thesis_simulink/System/Human arm/p_', num2str(j));
    mux_in_i = get_param('master_thesis_simulink/System/Human arm/Mux', 'PortHandles').Inport(j);
    p_out_i = get_param(p_from, 'PortHandles').Outport;
    delete_line('master_thesis_simulink/System/Human arm/', p_out_i, mux_in_i);
    delete_block(p_from);

    pdot_from = strcat('master_thesis_simulink/System/Human arm/pdot_', num2str(j));
    mux1_in_i = get_param('master_thesis_simulink/System/Human arm/Mux1', 'PortHandles').Inport(j);
    pdot_out_i = get_param(pdot_from, 'PortHandles').Outport;
    delete_line('master_thesis_simulink/System/Human arm/', pdot_out_i, mux1_in_i);
    delete_block(pdot_from);

end
end