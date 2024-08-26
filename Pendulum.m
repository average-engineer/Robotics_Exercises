%% Dynamics and Control of a Simple Pendulum
% Ashutosh Mukherjee
clc
clearvars
close all
%% Properties
m = 0.5;
L = 0.5;
b = 0.1;
g = 9.81;
t_min = 0;
t_max = 5;
N = 1000;
time = linspace(t_min,t_max,N);
u_max = 1; % Actuator Limit
%% Initial Conditions
x1_0 = 0*(pi/180);
x2_0 = 0;
%% Euler Intergration
% [theta,theta_dot] = eulerInt(time,[x1_0,x2_0],L,g,m,b);

%% Runge-Kutta Integration
% [t,result] = ode45(@(t,x)openLoop_pend(x,t,L,g),time,[x1_0;x2_0]);
% [t,result] = ode45(@(t,x)swingUp_pend(x,t,L,b,g,m),time,[x1_0;x2_0]);
[t,result] = ode45(@(t,x)swingUp_Balance_Pend(x,t,L,b,g,m,u_max),time,[x1_0;x2_0]);
%% Plotting
% theta = theta*(180/pi);
% theta_dot = theta_dot*(180/pi);
theta_rk = result(:,1)*(180/pi);
theta_dot_rk = result(:,2)*(180/pi);
figure
subplot(2,1,1)
hold on
% plot(time,theta,'LineWidth',2,'DisplayName','Euler')
plot(time,theta_rk,'LineWidth',2,'DisplayName','Runge-Kutta')
xlabel('Time (s)')
ylabel('Angle (deg)')
legend
grid on
subplot(2,1,2)
hold on
% plot(time,theta_dot,'LineWidth',2,'DisplayName','Euler')
plot(time,theta_dot_rk,'LineWidth',2,'DisplayName','Runge-Kutta')
xlabel('Time (s)')
ylabel('Anglular Rate (deg/s)')
legend
grid on

%% LQR
% A = [0,1;-g/L,-b/(m*L)];
% B = [0;1/(m*L*L)];
% Q = eye(2);
% R = 1;
% [K_lqr,M_lqr,lambda] = lqr(A,B,Q,R);
% [t,z] = ode45(@(t,z)fullState_LinCL(z,t,A,B,K_lqr),time,[45*pi/180;0]);
% theta_balance = pi - z;
% theta_balance = theta_balance*(180/pi);

