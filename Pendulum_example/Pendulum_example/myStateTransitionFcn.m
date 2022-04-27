function x = myStateTransitionFcn(x,u)

% Write your code here to define the discrete-time state update equations
% you've found in Part3-task15 of the Kalman filter virtual lab 
ts = 0.01;
g = 9.81;
L = 1;
x = [x(1) + ts*x(2);  
     x(2) - ts*g/L*sin(x(1)) + ts/(L^2)*u];
end