%% Closed-Loop linearized system with full-state feedback
% Ashutosh Mukherjee
function [z_dot] = fullState_LinCL(z,t,A,B,K)
z_dot = (A-B*K)*z;