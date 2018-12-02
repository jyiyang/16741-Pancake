close all;
clear *;

syms mm1 mm2 mm3 aa1 aa2 aa3 th_1 th_2 th_3 th_1_d th_2_d th_3_d th_1_dd th_2_dd th_3_dd real;
I1 = [0,0,0;0,0,0;0,0,1/3*mm1*aa1^2];
I2 = [0,0,0;0,0,0;0,0,1/3*mm2*aa2^2];
I3 = [0,0,0;0,0,0;0,0,1/3*mm3*aa3^2];
%I1 = [0,0,0;0,0,0;0,0,i1];
%I2 = [0,0,0;0,0,0;0,0,i2];
%I3 = [0,0,0;0,0,0;0,0,i3];
Jv1 = [-aa1/2*sin(th_1), 0,0;aa1/2*cos(th_1), 0,0;0,0,0];
Jv2 = [-aa1*sin(th_1)-aa2/2*sin(th_1+th_2), -aa2/2*sin(th_1+th_2),0;aa1*cos(th_1)+aa2/2*cos(th_1+th_2), aa2/2*cos(th_1+th_2),0;0,0,0];
Jv3 = [-aa1*sin(th_1)-aa2*sin(th_1+th_2)-aa3/2*sin(th_1+th_2+th_3), -aa2*sin(th_1+th_2)-aa3/2*sin(th_1+th_2+th_3),-aa3/2*sin(th_1+th_2+th_3);aa1*cos(th_1)+aa2*cos(th_1+th_2)+aa3/2*cos(th_1+th_2+th_3), aa2*cos(th_1+th_2)+aa3/2*cos(th_1+th_2+th_3),aa3/2*cos(th_1+th_2+th_3);0,0,0];
Jw1=[0,0,0;0,0,0;1,0,0];
Jw2=[0,0,0;0,0,0;1,1,0];
Jw3=[0,0,0;0,0,0;1,1,1];

D = mm1 * (Jv1' * Jv1) + mm2 * (Jv2' * Jv2) + mm3 * (Jv3' * Jv3) + Jw1' * I1 * Jw1 + Jw2' * I2 * Jw2 + Jw3' * I3 * Jw3;
D = simplify(D);
M = D * [th_1_dd; th_2_dd; th_3_dd];

C = [aa1;aa2;aa3];
for k = 1:3
    switch k
        case 1
            thk = th_1;
        case 2
            thk = th_2;
        case 3
            thk = th_3;
    end
    c_cell = 0;
    for i = 1:3
        switch i
            case 1
                thi = th_1;
                thi_d = th_1_d;
            case 2
                thi = th_2;
                thi_d = th_2_d;
            case 3
                thi = th_3;
                thi_d = th_3_d;
        end
        for j = 1:3
            switch j
                case 1
                    thj = th_1;
                    thj_d = th_1_d;
                case 2
                    thj = th_2;
                    thj_d = th_2_d;
                case 3
                    thj = th_3;
                    thj_d = th_3_d;
            end
            c_ijk = 1/2 * (diff(D(k, j), thi) + diff(D(k, i), thj) - diff(D(i, j), thk));
            c_cell = c_cell + c_ijk * thi_d * thj_d;
        end
    end
    C(k) = simplify(c_cell);
end

G = [aa1;aa2;aa3];
O = [aa1/2*sin(th_1); aa1*sin(th_1) + aa2/2*sin(th_1+th_2); aa1*sin(th_1) + aa2*sin(th_1+th_2) + aa3/2*sin(th_1+th_2+th_3)];
for k = 1:3
    g_cell = 0;
    switch k
        case 1
            thk = th_1;
        case 2
            thk = th_2;
        case 3
            thk = th_3;
    end
    for i = 1:3
        switch i
            case 1
                mi = mm1;
                thi_d = th_1_d;
            case 2
                mi = mm2;
                thi_d = th_2_d;
            case 3
                mi = mm3;
                thi_d = th_3_d;
        end
        oi = O(i);
        g_cell = g_cell + mi * -9.81 * diff(oi, thk);
    end
    G(k) = simplify(g_cell);
end

T = M + C + G;
T = simplify(T);
save('sym_equation', 'T', 'M', 'C', 'G', 'aa1', 'aa2', 'aa3', 'mm1', 'mm2', 'mm3', 'th_1', 'th_2', 'th_3', 'th_1_d', 'th_2_d', 'th_3_d', 'th_1_dd', 'th_2_dd', 'th_3_dd');


