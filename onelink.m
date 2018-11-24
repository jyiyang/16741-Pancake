% initial height
h = 5;
% gravitational constant
g = 9.81;
% v2 = unknown m/s
v1 = [0, 10];
syms v2;

m = 1;
% pancake is 2m long lol
L = 2;
% distance from one end of the pancake to the icr
l = v2*L / (v1 - v2);
% instanesous angular velocity
w = v1 / (L + l);
% Moment of inertia from parallel axis theorem
I_ic = m*L^2 / 12 + m*(0.5*L + l)^2;
% Kinectic energy
K = 0.5*m*((v1 + v2) / 2)^2 + 0.5*I_ic*w^2;
% Potential energy
P = m*g*h;

% Evaluate
v2s = double(solve(K == P));
ls = v2s.*L ./ (v1 - v2s);

% calculate next location
x1 = t_step * v1;

