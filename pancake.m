function [pt_array] = pancake(x, y, theta, x_d, y_d, theta_d, pk_l, pk_m)

    % gravitational constant
    g = 9.81;
    m = pk_m;
    % pancake is 0.01m
    L = pk_l;
    theta_dd = 0;
    Izz = 0.666667*m*L^2;
    % damping factor
    cw = 0.005; cp = 1.12 * ( L / 2)^2 * pi;

    t_step = 0.001;
    t_max = 2;
    t = linspace(0, t_max, t_max / t_step);

    xx = zeros(1, size(t, 2));
    yy = zeros(1, size(t, 2));
    theta_x = zeros(1, size(t, 2));
    e1x = zeros(1, size(t, 2));
    e1y = zeros(1, size(t, 2));
    e2x = zeros(1, size(t, 2));
    e2y = zeros(1, size(t, 2));
    
    pt_array = zeros(size(t, 2), 4);

    for i = 1:size(t, 2)

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

        LL = (e2x(i)^2 + e2y(i)^2 - (e1x(i)^2 + e1y(i)^2))^2;
    end
    
    pt_array(:, 1) = e1x';
    pt_array(:, 2) = e1y';
    pt_array(:, 3) = e2x';
    pt_array(:, 4) = e2y';


end
