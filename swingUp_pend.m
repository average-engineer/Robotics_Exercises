%% Closed-Loop dynamics of a simple pendulum (Swing-UP Controller=
% Ashutosh Mukherjee
function [x_dot] = swingUp_pend(x,t,l,b,g,m)
K = 100;
u_max = 1;
% Desired Energy
E_des = 2*m*g*l;
E_act = 0.5*m*(l^2)*(x(2)^2) + m*g*l*(1-cos(x(1)));
if x(2)*cos(x(1)) >= 0
    u = K*(E_des - E_act);
else
    u = -K*(E_des - E_act);
end
% Clipping
u = min(max(-u_max,u),u_max);
xd1 = x(2);
xd2 = (u/(m*l^2)) - (b*x(2)/m*l) - (g*sin(x(1))/l);
x_dot = [xd1;xd2];