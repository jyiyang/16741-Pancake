clear all
% gravitational constant
g = 9.81;
m = 0.01;
% pancake is 0.01m
L = 1;
theta_dd = 0;
theta_d = 0;
theta = 0;
Izz = 0.666667*m*L^2;
x = 0; x_d = 0;
y = 0; y_d = 5;
% damping factor
cw = 0.005; cp = 1.12 * ( L / 2)^2 * pi;

t_step = 0.01;
t_max = 2;
t = linspace(0, t_max, t_max / t_step);

xx = zeros(1, size(t, 2));
yy = zeros(1, size(t, 2));
theta_x = zeros(1, size(t, 2));
e1x = zeros(1, size(t, 2));
e1y = zeros(1, size(t, 2));
e2x = zeros(1, size(t, 2));
e2y = zeros(1, size(t, 2));

for i = 1:size(t, 2)
    figure(3)
    axis equal
    grid on

    A = [1, 0, L/2*sin(theta);
         0, 1, -L/2*cos(theta);
         -m*L/2*sin(theta), m*L/2*cos(theta), Izz - m*L^2/4];
    
    b = -[-theta_d^2 * L / 2 * cos(theta) + sign(x_d) * cp * sin(x_d)^2;
          theta_d^2 * L / 2 * sin(theta) + g + sign(y_d) * cp * cos(y_d)^2;
          m*g*L/2*cos(theta) + cw * theta_d];
    
    sol = A \ b;
    theta_dd = sol(3);
    x_dd = sol(1);
    y_dd = sol(2);
    
    theta_d = theta_d + theta_dd * t_step;
    theta = theta + theta_d * t_step;

    x_d = x_d + x_dd * t_step;
    y_d = y_d + y_dd * t_step;
    x = x + x_d * t_step;
    y = y + y_d * t_step;
    
    xx(i) = x + L*cos(theta) /2;
    yy(i) = y - L*sin(theta) /2;
    theta_x(i) = theta;
    
    e1x(i) = x;
    e1y(i) = y;    
    
    e2x(i) = x + L*cos(theta);
    e2y(i) = y - L*sin(theta);
    
    line([e1x(i); e2x(i)], [e1y(i); e2y(i)])
    LL = (e2x(i)^2 + e2y(i)^2 - (e1x(i)^2 + e1y(i)^2))^2;
    if mod(i, 10) == 0
        drawnow 
    end
end

figure(1)
subplot(3, 1, 1)
plot(t, xx);
subplot(3, 1, 2)
plot(t, yy);
subplot(3, 1, 3)
plot(t, theta_x);

figure(2)
plot(xx, yy)
