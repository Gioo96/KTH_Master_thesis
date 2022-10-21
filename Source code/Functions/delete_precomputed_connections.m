function delete_precomputed_connections()

%% Description
% delete_precomputed_connections deletes the block connections

%% Function

% Number of joints
n = 7;

for i = 1 : n

    if (i <= 3)

        c = strcat('c', num2str(i));
        c_shoulder =  get_param(strcat('master_thesis_simulink/System/Human arm/RightShoulder_joint/', c), 'LineHandles');
        c_out = c_shoulder.RConn;
        delete_line(c_out);
    end

    if (i == 4)

        c = strcat('c', num2str(i-3));
        c_elbow =  get_param(strcat('master_thesis_simulink/System/Human arm/Elbow_joint/', c), 'LineHandles');
        c_out = c_elbow.RConn;
        delete_line(c_out);
    end

    if (i > 4)

        c = strcat('c', num2str(i-4));
        c_wrist =  get_param(strcat('master_thesis_simulink/System/Human arm/Wrist_joint/', c), 'LineHandles');
        c_out = c_wrist.RConn;
        delete_line(c_out);
    end
end
end