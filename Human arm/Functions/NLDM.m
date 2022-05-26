function NLDM(sampling_Time, C_code_folder)

%% Sampling Time
sample_Time = sampling_Time;

%% Simulation
out_NLDM = sim("master_thesis_simulink.slx");

%% Plot Discrete Non Linear Model vs true joints' variables (q vs qk)
fig_NLDM = figure;
sgtitle("NLDM (10ms) vs CONTINUOS NL");
for i = 1 : n

    q = strcat('q_', num2str(i));
    qk = strcat('q^k_', num2str(i));
    set(fig_NLDM, 'position', [10, 10, 1300, 900]);
    subplot(3,3,i);
    plot(out_NLDM.q.time, rad2deg(out_NLDM.q.signals.values(:, i)));
    hold on;
    stairs(out_NLDM.qk.time, rad2deg(squeeze(out_NLDM.qk.signals.values(i, :))'));
    legend(q, qk, 'Location', 'southeast');
end

end