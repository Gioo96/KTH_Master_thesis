function animatePendulum(t, theta_actual, theta_measured, theta_estimated, theta_initial)

[~, L, ~] = pendparams;

if isempty(theta_estimated) && isempty(theta_measured)

    theta_estimated = zeros(length(theta_actual),1);
    theta_measured = zeros(length(theta_actual),1);
    visibility = ["on","off","off"];
elseif isempty(theta_estimated)

    theta_estimated = zeros(length(theta_actual),1);
    visibility = ["on","off","on"];
elseif isempty(theta_measured)

    theta_measured = zeros(length(theta_actual),1);
    visibility = ["on","on","off"];
else

    visibility = ["on","on","on"];
end

clf
tiledlayout(2,1)           
nexttile 
hold on
plot(t, theta_actual, 'g', LineWidth = 1.5, Visible = visibility(1));
plot(t, theta_estimated, 'b', LineWidth = 1, Visible = visibility(2));
plot(t, theta_measured, 'r', LineWidth = 0.5, Visible = visibility(3));

axis([t(1) t(end) -abs(theta_initial)-10 abs(theta_initial)+10])
title('System Response')

% Convert from degrees to radians
theta_actual_rad = deg2rad(theta_actual);
theta_estimated_rad = deg2rad(theta_estimated);
theta_measured_rad = deg2rad(theta_measured);

hold off

x_actual = L * sin(theta_actual_rad);
y_actual = -L * cos(theta_actual_rad);

x_estimated = L * sin(theta_estimated_rad);
y_estimated = -L * cos(theta_estimated_rad);

x_measured =  L * sin(theta_measured_rad);
y_measured = -L * cos(theta_measured_rad);

hold on
a1 = plot(t(1), Marker=".", MarkerSize=20, Color="g");
e1 = plot(t(1), Marker=".", MarkerSize=20, Color="b");
m1 = plot(t(1), Marker=".", MarkerSize=20, Color="r");

xlabel("Time (sec)")
ylabel("Angular position (deg)")
leg = ["Pendulum response","Kalman Filter estimate","Measured response"];
ind = (visibility == "on");
leg(ind == 0) = '';
legend(leg, Location='eastoutside');
hold off

nexttile
hold on;
a2 = plot([0, x_actual(1,1)],[0, y_actual(1,1)], ...
    Marker=".", MarkerSize=25, LineWidth=2, Color='green', Visible = visibility(1));

e2 = plot([0, x_estimated(1,1)],[0, y_estimated(1,1)], ...
    Marker=".", MarkerSize=20, LineWidth=2, Color='blue', Visible = visibility(2));

m2 = plot([0, x_measured(1,1)],[0, y_measured(1,1)], ...
    Marker=".", MarkerSize=20, LineWidth=2, Color='red', Visible = visibility(3));
hold off;
axis equal
axis([-1.2*L 1.2*L -1.2*L 0.2*L])
ht = title("Time: "+t(1)+" sec");   

for id = 1:length(t)
    set(a1, XData = t(id),YData = theta_actual(id))
    set(a2, XData = [0, x_actual(id)], YData = [0, y_actual(id)], Visible = visibility(1))
        
    set(e1, XData = t(id), YData = theta_estimated(id), Visible = visibility(2))
    set(e2, XData = x_estimated(id), YData = y_estimated(id))  
    
    set(m1, XData = t(id), YData = theta_measured(id), Visible = visibility(3))
    set(m2, XData = x_measured(id), YData = y_measured(id))
        
    set(ht, String = "Time: " + t(id) + " sec")
    
    drawnow
end

end