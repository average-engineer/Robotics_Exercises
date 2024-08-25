%% Open-Loop dynamics of a simple pendulum
% Ashutosh Mukherjee
function [x_dot] = openLoop_pend(x,t,l,g)
x_dot1 = x(2);
x_dot2 = -(g/l)*sin(x(1));
x_dot = [x_dot1;x_dot2];
end