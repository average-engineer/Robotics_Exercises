%% Forward of a robotic manipulator
% Ashutosh Mukherjee
% Based on formulation given in Introduction to Robotics by J.Craig
clc
clearvars
close all
format short


% INPUT VARIBLES
NJ = 3; % Number of Joints

syms q1 q2 q3

% alpha(k-1)  =   Twist Angle (Angle b/w Z(k-1) & Z(k) about X(k-1))
% a(k-1)      =   Link Length (Distance from Z(k-1) to Z(k) along X(k-1))
% theta(k)  =   Joint Angle (Angle b/w X(k-1) & X(k) along Z(k))
% d(k)      =   Joint Offset (Distance from X(k-1) to X(k) along Z(k))

alpha = [0;-90*pi/180;90*pi/180];
a = zeros(NJ,1); 
d = [0;0;q3];
theta = [q1;q2;0];


% CALCULATE THE Homogeneous Tranformation matrix from end-effector to base
% joint
T = eye(4);
for k = NJ:-1:1
    T = [cos(theta(k)), -sin(theta(k)), 0 , a(k);
        sin(theta(k))*cos(alpha(k)) , cos(theta(k))*cos(alpha(k)) , -sin(alpha(k)) , -sin(alpha(k))*d(k);
        sin(theta(k))*sin(alpha(k)) , cos(theta(k))*sin(alpha(k)) , cos(alpha(k)) , cos(alpha(k))*d(k);
        zeros(1,3),1]*T;
end

T = simplify(T)