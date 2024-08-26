%% Energy-based Swing-Up and Balancing of Simple Pendulum
% Ashutosh Mukherjee
function [x_dot] = swingUp_Balance_Pend(x,t,l,b,g,m,u_max)
% Strategy:
% Perform swing-up using energy error till a close value to 180 degs is
% reached
% Define a threshold for linear behaviour of the pendulum about its upright
% equilibrium point
% Switch to a LQR Controller once the linear threshold is reached in order
% to balance the pendulum at its upright position

%% Define Linear Threshold
linEps = 10; % degrees
% Check if the system is within the linear threshold
if (180/pi)*abs(pi - x(1)) < linEps
    LINSYS = 1;
else
    LINSYS = 0;
end

%% Switch Controllers
switch LINSYS
    case 0 % Non-Linear System (Perform Energy Swing-Up)
        K = 10;
        % Desired Energy
        E_des = 2*m*g*l;
        E_act = 0.5*m*(l^2)*(x(2)^2) + m*g*l*(1-cos(x(1)));
        if x(2) >= 0
            u = K*(E_des - E_act);
        else
            u = -K*(E_des - E_act);
        end
        % Clipping
        u = min(max(-u_max,u),u_max);
        xd1 = x(2);
        xd2 = (u/(m*l^2)) - (b*x(2)/m*l) - (g*sin(x(1))/l);

    case 1 % Linear System (Perform LQR)
        A = [0,1;-g/l,-b/(m*l)];
        B = [0;1/(m*l*l)];
        Q = eye(2);
        R = 1;
        [K_lqr,~,~] = lqr(A,B,Q,R);
        z(1) = pi - x(1);
        z(2) = -x(2);
        z = z(:);
        z_dot = (A-B*K_lqr)*z;
        xd1 = -z_dot(1);
        xd2 = -z_dot(2);
end

x_dot = [xd1;xd2];
end