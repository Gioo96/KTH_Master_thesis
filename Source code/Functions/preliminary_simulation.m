function preliminary_simulation(set_combinations, number_simulations, q_trajectory, save_markers_nonSingular_combinations)

%% Description

% preliminary_simulation , foreach set of combination, performes
% 'number_simulations' simulations generating random markers in each 
% one of them and computes the percentage of singular, non-singular and 
% quasi-singular points along the trajectory 'q_trajectory'

% Inputs:
% -- set_combinations                      : set of markers' combinations
% -- number_simulations                    : number of simulations
% -- q_trajectory                          : assigned trajectory
% -- save_markers_nonSingular_combinations : [3 7 10] can be either false or true

%% Function

% Number of combinations
number_combinations = size(set_combinations, 1);

% Cobdition number threshold above which a point is considered singular
condition_number_thresh = 1e+3;

fig = figure();
for i = 1:number_combinations

    % Number of markers on the shoulder, forearm, hand
    shoulder_num = set_combinations(i, 1); 
    forearm_num = set_combinations(i, 2); 
    hand_num = set_combinations(i, 3);
    m = shoulder_num + forearm_num + hand_num;
    f_J = mex_generation(shoulder_num, forearm_num, hand_num, 'false');

    % 3th combination
    if (shoulder_num == 1 && forearm_num == 1 && hand_num == 1)

        save_markers = save_markers_nonSingular_combinations(1);

    % 7th combination
    elseif (shoulder_num == 1 && forearm_num == 2 && hand_num == 1)

        save_markers = save_markers_nonSingular_combinations(2);

    % 10th combination
    elseif (shoulder_num == 2 && forearm_num == 2 && hand_num == 1)

        save_markers = save_markers_nonSingular_combinations(3);

    else

        save_markers = false;
    end

    % Find percentages foreach simulation and find best set of markers for a given trajectory and specific number of markers
    [ns, qs, s, markers_best] = singular_config(q_trajectory, shoulder_num, forearm_num, hand_num, f_J, condition_number_thresh, number_simulations, save_markers);

    switch m

        % 3 markers
        case 3

            if (shoulder_num == 1 && forearm_num == 1 && hand_num == 1)

                % Save markers
                if (save_markers)

                    save('markers_best_3th.mat', 'markers_best');
                end
            end

        % 4 markers
        case 4

            if (shoulder_num == 1 && forearm_num == 1 && hand_num == 2)

                % Save best markers
                %save('markers_best_6th.mat', 'markers_best');

            elseif (shoulder_num == 1 && forearm_num == 2 && hand_num == 1)

                % Save markers
                if (save_markers)

                    save('markers_best_7th.mat', 'markers_best');
                end
            end

        % 5 markers
        case 5

            if (shoulder_num == 2 && forearm_num == 2 && hand_num == 1)

                % Save markers
                if (save_markers)

                    save('markers_best_10th.mat', 'markers_best');
                end
            end
    end

    legend_name_ns = 'NS points';
    legend_name_s = 'S points';
    legend_name_qs = 'QS points';
    % Save EPS figures
    fig1 = figure('Visible', 'off');
    set(0, 'currentfigure', fig1);
    stem(ns, 'LineStyle', ':', 'LineWidth', 1, 'Marker', 'o', 'MarkerSize', 9);
    hold on;
    stem(s, 'LineStyle', ':', 'LineWidth', 1, 'Marker', '*', 'MarkerSize', 9);
    hold on;
    stem(qs, 'LineStyle', ':', 'LineWidth', 1, 'Marker', 'diamond', 'MarkerSize', 9);
    xlabel('# Simulation');
    ylabel('Percentage [%]');
    legendInfo{1} = legend_name_ns;
    legendInfo{2} = legend_name_s;
    legendInfo{3} = legend_name_qs;
    legend(legendInfo, 'Location', 'NorthOutside', 'Orientation', 'horizontal', 'Box', 'off', 'Interpreter', 'latex', 'FontSize', 26);
    set(gca, 'FontSize', 18);
    grid on;
    eps_name = strcat('Preliminary_sim_m', num2str(i));
    saveas(gcf, eps_name, 'epsc');

    % Plot figures
    set(0, 'currentfigure', fig);
    subplot(5, 2, i);
    stem(ns, 'LineStyle', ':', 'LineWidth', 1, 'Marker', 'o', 'MarkerSize', 9);
    hold on;
    stem(s, 'LineStyle', ':', 'LineWidth', 1, 'Marker', '*', 'MarkerSize', 9);
    hold on;
    stem(qs, 'LineStyle', ':', 'LineWidth', 1, 'Marker', 'diamond', 'MarkerSize', 9);
    xlabel('# Simulation');
    ylabel('Percentage [%]');
    legendInfo{1} = legend_name_ns;
    legendInfo{2} = legend_name_s;
    legendInfo{3} = legend_name_qs;
    legend(legendInfo, 'Interpreter', 'latex');
    grid on;

end
end