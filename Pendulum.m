%% Dynamics and Control of a Simple Pendulum
% Ashutosh Mukherjee
clc
clearvars
close all
addpath("C:\Users\ashut\Desktop\Werk\Chalmers_Robotics\Robotics_Exercises\Report\matlabtikz")
%% Properties
m = 0.5;
L = 0.5;
b = 0.1;
g = 9.81;
t_min = 0;
t_max = 10;
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
[t,x_1] = ode45(@(t,x)swingUp_pend(x,t,L,b,g,m),time,[x1_0;x2_0]);
[t,x_2] = ode45(@(t,x)swingUp_Balance_Pend(x,t,L,b,g,m,u_max),time,[x1_0;x2_0]);
%% Plotting
% theta = theta*(180/pi);
% theta_dot = theta_dot*(180/pi);
theta_2 = x_2(:,1)*(180/pi);
theta_1 = x_1(:,1)*(180/pi);
figure
% subplot(2,1,1)
hold on
% plot(time,theta,'LineWidth',2,'DisplayName','Euler')
plot(time,theta_1,'LineWidth',2,'DisplayName','Energy-Based')
plot(time,theta_2,'LineWidth',2,'DisplayName','Energy-Based +  LQR')
xlabel('Time (s)')
ylabel('Angle (deg)')
lgd = legend;
lgd.Location = 'southeast';
grid on
% subplot(2,1,2)
% hold on
% plot(time,theta_dot,'LineWidth',2,'DisplayName','Euler')
% plot(time,theta_dot_rk,'LineWidth',2,'DisplayName','Runge-Kutta')
% xlabel('Time (s)')
% ylabel('Anglular Rate (deg/s)')
% legend
% grid on
matlab2tikz();


