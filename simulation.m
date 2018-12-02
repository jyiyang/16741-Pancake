clear *;
close all;

load('flip_actions.mat');
fig = figure(1);

speed = 5;
for i = 1:size(jt_array, 1)
    1
    if (mod(i, speed) == 1) || (i == size(jt_array, 1))   
        
        jt1 = jt_array(i, 1, :);
        jt2 = jt_array(i, 2, :);
        jt3 = jt_array(i, 3, :);
        jt4 = jt_array(i, 4, :);
        T_rst = T_array(i, :);
        r = 0.1;

        clf(fig)
        axis equal;
        grid on;
        hold on;
        xlim([-1, 4]);
        ylim([-2, 5]);
        [x, y, z] = sphere();
        surf(x * r + jt1(1), y * r + jt1(2), z * r, 'EdgeColor', 'g', 'FaceColor', 'g');
        surf(x * r + jt2(1), y * r + jt2(2), z * r, 'EdgeColor', 'g', 'FaceColor', 'g');
        surf(x * r + jt3(1), y * r + jt3(2), z * r, 'EdgeColor', 'g', 'FaceColor', 'g');
        %surf(x * r + jt4(1), y * r + jt4(2), z * r, 'EdgeColor', 'g', 'FaceColor', 'g');

        [x, y, z] = cylinder2P(r/2,20,[jt1(1), jt1(2), 0],[jt2(1), jt2(2), 0]);
        surf(x, y, z, 'EdgeColor', 'r', 'FaceColor', 'r');
        [x, y, z] = cylinder2P(r/2,20,[jt2(1), jt2(2), 0],[jt3(1), jt3(2), 0]);
        surf(x, y, z, 'EdgeColor', 'r', 'FaceColor', 'r');
        [x, y, z] = cylinder2P(r/2,20,[jt3(1), jt3(2), 0],[jt4(1), jt4(2), 0]);
        surf(x, y, z, 'EdgeColor', 'r', 'FaceColor', 'r');
        title(strcat('T1: ', num2str(T_rst(1)), '  T2: ', num2str(T_rst(2)), '  T3: ', num2str(T_rst(3))))
        
        ex2 = jt4(1);
        ey2 = jt4(2);
        ex1 = jt3(1);
        ey1 = jt3(2);
        cx = (ex1 + ex2) / 2;
        cy = (ey1 + ey2) / 2;
        lnx = ex2 - ex1;
        lny = ey2 - ey1;
        nmx = lny / lnx;
        nmy = -1;
        plotCircle3D([cx, cy, 0], [nmx, nmy, 0], 0.25);       
        drawnow;

    end

end

T_rst = T_array(end, :);
for i = 1:size(pc_point_array, 1)
    2
    if (mod(i, speed) == 1) || (i == size(pc_point_array, 1))  
        if i < size(jtc_array, 1)
            jt1 = jtc_array(i, 1, :);
            jt2 = jtc_array(i, 2, :);
            jt3 = jtc_array(i, 3, :);
            jt4 = jtc_array(i, 4, :);
            T_rst = Tc_array(i, :);
        else
            jt1 = jtc_array(end, 1, :);
            jt2 = jtc_array(end, 2, :);
            jt3 = jtc_array(end, 3, :);
            jt4 = jtc_array(end, 4, :);
            T_rst = Tc_array(end, :);
        end
        
        r = 0.1;

        clf(fig)
        axis equal;
        grid on;
        hold on;
        xlim([-1, 4]);
        ylim([-2, 5]);
        [x, y, z] = sphere();
        surf(x * r + jt1(1), y * r + jt1(2), z * r, 'EdgeColor', 'g', 'FaceColor', 'g');
        surf(x * r + jt2(1), y * r + jt2(2), z * r, 'EdgeColor', 'g', 'FaceColor', 'g');
        surf(x * r + jt3(1), y * r + jt3(2), z * r, 'EdgeColor', 'g', 'FaceColor', 'g');
        %surf(x * r + jt4(1), y * r + jt4(2), z * r, 'EdgeColor', 'g', 'FaceColor', 'g');

        [x, y, z] = cylinder2P(r/2,20,[jt1(1), jt1(2), 0],[jt2(1), jt2(2), 0]);
        surf(x, y, z, 'EdgeColor', 'r', 'FaceColor', 'r');
        [x, y, z] = cylinder2P(r/2,20,[jt2(1), jt2(2), 0],[jt3(1), jt3(2), 0]);
        surf(x, y, z, 'EdgeColor', 'r', 'FaceColor', 'r');
        [x, y, z] = cylinder2P(r/2,20,[jt3(1), jt3(2), 0],[jt4(1), jt4(2), 0]);
        surf(x, y, z, 'EdgeColor', 'r', 'FaceColor', 'r');
        title(strcat('T1: ', num2str(T_rst(1)), '  T2: ', num2str(T_rst(2)), '  T3: ', num2str(T_rst(3))))
        
        ex1 = pc_point_array(i, 1);
        ey1 = pc_point_array(i, 2);
        ex2 = pc_point_array(i, 3);
        ey2 = pc_point_array(i, 4);
        cx = (ex1 + ex2) / 2;
        cy = (ey1 + ey2) / 2;
        
        lnx = ex2 - ex1;
        lny = ey2 - ey1;
        nmx = lny / lnx;
        nmy = -1;
        plotCircle3D([cx, cy, 0], [nmx, nmy, 0], 0.25);       
        drawnow;

    end
end