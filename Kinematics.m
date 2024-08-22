%% Kinematics of a robotic manipulator
% Ashutosh Mukherjee
% Based on formulation given in Introduction to Robotics by J.Craig
clc
clearvars
close all
format short

%% Reading the input parameter file
file = fopen("dhParam.txt",'r');

NJ = fscanf(file,'%d',1); % Number of Joints
DOF = fscanf(file,'%d',1); % Degrees of Freedom

alpha = zeros(NJ,1);    % alpha(k-1)  =   Twist Angle (Angle b/w Z(k-1) & Z(k) about X(k-1))
a = zeros(NJ,1);        % a(k-1)      =   Link Length (Distance from Z(k-1) to Z(k) along X(k-1))
theta = zeros(NJ,1);    % theta(k)  =   Joint Angle (Angle b/w X(k-1) & X(k) along Z(k))
d = zeros(NJ,1);        % d(k)      =   Joint Offset (Distance from X(k-1) to X(k) along Z(k))

for k=1:NJ
    alpha(k) = fscanf(file,'%f',1)*pi/180;
    a(k) = fscanf(file,'%f',1);
    d(k) = fscanf(file,'%f',1);
    theta(k) = fscanf(file,'%f',1)*pi/180;
end


%% Homogeneous Tranformation matrix from end-effector to base joint

T = eye(4);
for k = NJ:-1:1
    T = [cos(theta(k)), -sin(theta(k)), 0 , a(k);
        sin(theta(k))*cos(alpha(k)) , cos(theta(k))*cos(alpha(k)) , -sin(alpha(k)) , -sin(alpha(k))*d(k);
        sin(theta(k))*sin(alpha(k)) , cos(theta(k))*sin(alpha(k)) , cos(alpha(k)) , cos(alpha(k))*d(k);
        zeros(1,3),1]*T;
end
%% Forward Kinematics
% Assuming the trajectory of the end-effector in this case coincides with
% the origin of the prismatic joint frame
disp("*******************Forward Kinematics*********************\n")
r_fwd = T(1:3,end)


%% Workspace Plotting
% Assumed joint contraints
% q1 \in [-pi/2,pi/2] (radians)
q1_min = -pi/2;
q1_max = pi/2;
% q2 \in [-pi,pi] (radians)
q2_min = 0;
q2_max = pi;
% q3 \in [0,1] (meters)
q3_min = 0;
q3_max = 1;

% Grid Creation
q1Grid = linspace(q1_min,q1_max,50);
q2Grid = linspace(q2_min,q2_max,50);
%q3Grid = linspace(q3_min,q3_max,50);
q3Grid = [q3_min,q3_max];
% Using the analytical expression of Fwd Kinematics to compute the
% workspace points
figure
hold on
count = 1;
% 3 Nested Loops (obviously not the most efficient)
for kk = 1:length(q1Grid)
    for jj = 1:length(q2Grid)
        for ii = 1:length(q3Grid)
            p_x(count) = q3Grid(ii)*cos(q1Grid(kk))*sin(q2Grid(jj));
            p_y(count) = q3Grid(ii)*sin(q1Grid(kk))*sin(q2Grid(jj));
            p_z(count) = q3Grid(ii)*cos(q2Grid(jj));
            count = count + 1;
        end
    end
end
plot3(p_x,p_y,p_z,'*','color','b')
grid on


%% Inverse Kinematics 
disp("*******************Inverse Kinematics*********************\n")
[q1,q2,q3] = invKinematics(r_fwd(1),r_fwd(2),r_fwd(3))
function [q1,q2,q3] = invKinematics(x,y,z)
if x == y
    q1 = NaN; % Arbritary value within the defined range
else
    q1 = atan(y/x);
end
q1 = (180/pi)*q1; % Converting to degrees
q2 = atan2(sqrt(x^2 + y^2),z);
q2 = (180/pi)*q2; % Converting to degrees
q3 = sqrt(x^2 + y^2 + z^2);
end