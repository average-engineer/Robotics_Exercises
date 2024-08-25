%% Euler-Integrator
% Ashutosh Mukherjee
function [x1Vec,x2Vec] = eulerInt(t,x_0,l,g)
x1Vec = zeros(length(t),1);
x2Vec = zeros(length(t),1);
x1Vec(1) = x_0(1);
x2Vec(2) = x_0(2);

dt = (t(end) - t(1))/length(t);

int_count = 1;

while int_count < length(t)
    int_count = int_count + 1;
    x_dot = openLoop_pend([x1Vec(int_count - 1),x2Vec(int_count - 1)],t,l,g);
    x1Vec(int_count) = x1Vec(int_count - 1) + dt*x_dot(1);
    x2Vec(int_count) = x2Vec(int_count - 1) + dt*x_dot(2);
end
