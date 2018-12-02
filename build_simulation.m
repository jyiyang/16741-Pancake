clear *;
close all;

load('sym_equation.mat');

th1 = 0;
th2 = 0;
th3 = -pi/4;

tg_vx = -1;
tg_vy = 5;
tg_w = 2 * pi;

a1 = 1;
a2 = 1;
a3 = 1;
m1 = 1;
m2 = 1;
m3 = 1;

pk_l = 0.5;
pk_m = 0.1;
m3 = m3 + pk_m;

pass_flag = false;

while (pass_flag == false)
    tg_th1 = (rand() - 0.5) * 2 * pi/8;
    tg_th2 = (rand() - 0.5) * 2 * pi/8;
    tg_th3 = -(tg_th1 + tg_th2);
    if (tg_th2 ~= 0)
        de_th_d = inverse_vel_kinematics(tg_vx, tg_vy, tg_w, a1, a2, a3, tg_th1, tg_th2);
    end
    if ((tg_th1 * de_th_d(1)) > 0) && ((tg_th2 * de_th_d(2)) > 0) && (((tg_th3 - th3) * de_th_d(3)) > 0)
        pass_flag = true;
    end
    
end

tt1 = 2 * (tg_th1 - th1) / de_th_d(1);
tt2 = 2 * (tg_th2 - th2) / de_th_d(2);
tt3 = 2 * (tg_th3 - th3) / de_th_d(3);
alpha1 = de_th_d(1) / tt1;
alpha2 = de_th_d(2) / tt2;
alpha3 = de_th_d(3) / tt3;
max_tt = max([tt1, tt2, tt3]);
wtt1 = max_tt - tt1;
wtt2 = max_tt - tt2;
wtt3 = max_tt - tt3;

t = 0:0.001:max_tt;

h1=th1;
h2=th2;
h3=th3;
v1 = 0;
v2 = 0;
v3 = 0;
jt_array = zeros(length(t), 4, 2);
T_array = zeros(length(t), 3);
for i = 1:length(t)
    [t(i), max_tt]
    
    time = t(i);
    if time > wtt1
        h1 = th1 + 1/2 * alpha1 * (time - wtt1)^2;
        v1 = alpha1 * (time - wtt1);
    end
    if time > wtt2
        h2 = th2 + 1/2 * alpha2 * (time - wtt2)^2;
        v2 = alpha2 * (time - wtt2);
    end
    if time > wtt3
        h3 = th3 + 1/2 * alpha3 * (time - wtt3)^2;
        v3 = alpha3 * (time - wtt3);
    end
    jt1 = [0,0];
    jt2 = [a1 * cos(h1), a1 * sin(h1)];
    jt3 = [a1 * cos(h1) + a2 * cos(h1 + h2), a1 * sin(h1) + a2 * sin(h1 + h2)];
    jt4 = [a1 * cos(h1) + a2 * cos(h1 + h2) + a3 * cos(h1 + h2 + h3), a1 * sin(h1) + a2 * sin(h1 + h2) + a3 * sin(h1 + h2 + h3)];
    
    T_rst = -double(subs(T, [aa1, aa2, aa3, mm1, mm2, mm3, th_1, th_2, th_3, th_1_d, th_2_d, th_3_d, th_1_dd, th_2_dd, th_3_dd], [a1, a2, a3, m1, m2, m3, h1, h2, h3, v1, v2, v3, alpha1, alpha2, alpha3]));
    
    jt_array(i, 1, :) = jt1;
    jt_array(i, 2, :) = jt2;
    jt_array(i, 3, :) = jt3;
    jt_array(i, 4, :) = jt4;
    T_array(i, :) = T_rst';
    
end

T_rst = -double(subs(T, [aa1, aa2, aa3, mm1, mm2, mm3, th_1, th_2, th_3, th_1_d, th_2_d, th_3_d, th_1_dd, th_2_dd, th_3_dd], [a1, a2, a3, m1, m2, m3, h1, h2, h3, 0, 0, 0, 0, 0, 0]));
T_array = [T_array; T_rst'];
atmp = (a3 - pk_l) / 2;
pc_point_array = pancake(a1 * cos(h1) + a2 * cos(h1 + h2) + atmp * cos(h1 + h2 + h3), a1 * sin(h1) + a2 * sin(h1 + h2) + atmp * sin(h1 + h2 + h3), h1 + h2 + h3, tg_vx, tg_vy, -tg_w, pk_l, pk_m);

pass_flag = false;
offset = 0;
while (pass_flag == false)
    pc_center = [(pc_point_array(end - offset, 1) + pc_point_array(end - offset, 3)) / 2, (pc_point_array(end - offset, 2) + pc_point_array(end - offset, 4)) / 2];
    jt3_goal = [pc_center(1) - a3 / 2, pc_center(2)];
    if (jt3_goal(1)^2 + jt3_goal(2)^2) < (a1 + a2)^2
        pass_flag = true;
    else
        offset = offset + 1;
    end
end
pancake_time = (size(pc_point_array, 1) - offset) * 0.001;
pc_point_array = pc_point_array(1:size(pc_point_array, 1) - offset, :);
th1_curr = h1;
th2_curr = h2;
th3_curr = h3;
th2_goal = acos((jt3_goal(1)^2 + jt3_goal(2)^2 - a1^2 - a2^2) / (2*a1*a2));
th1_goal = atan2(jt3_goal(2), jt3_goal(1)) - atan2(a2*sin(th2_goal), a1+a2*cos(th2_goal));
th3_goal = -(th1_goal + th2_goal);
th1_dist = mod(th1_goal - th1_curr, 2*pi);
th2_dist = mod(th2_goal - th2_curr, 2*pi);
th3_dist = mod(th3_goal - th3_curr, 2*pi);
if th1_dist > pi
    th1_dist = th1_dist - 2 * pi;
end
if th2_dist > pi
    th2_dist = th2_dist - 2 * pi;
end
if th3_dist > pi
    th3_dist = th3_dist - 2 * pi;
end
arm_time = pancake_time / 4;
alpha1 = th1_dist / (arm_time / 2)^2;
alpha2 = th2_dist / (arm_time / 2)^2;
alpha3 = th3_dist / (arm_time / 2)^2;

t_arm = 0:0.001:arm_time;
jtc_array = zeros(length(t_arm), 4, 2);
Tc_array = zeros(length(t_arm), 3);
for i = 1:numel(t_arm)
    [t_arm(i), arm_time]
    time = t_arm(i);
    v1_half = alpha1 * (arm_time / 2);
    v2_half = alpha2 * (arm_time / 2);
    v3_half = alpha3 * (arm_time / 2);
    if i < (numel(t_arm) / 2)
        h1 = th1_curr + 1/2 * alpha1 * time^2;
        h2 = th2_curr + 1/2 * alpha2 * time^2;
        h3 = th3_curr + 1/2 * alpha3 * time^2;
        v1 = alpha1 * time;
        v2 = alpha2 * time;
        v3 = alpha3 * time;
    else
        tmp_time = time - arm_time / 2;
        h1 = th1_curr + 1/2 * alpha1 * (arm_time / 2)^2 + (v1_half * tmp_time - 1/2 * alpha1 * tmp_time^2);
        h2 = th2_curr + 1/2 * alpha2 * (arm_time / 2)^2 + (v2_half * tmp_time - 1/2 * alpha2 * tmp_time^2);
        h3 = th3_curr + 1/2 * alpha3 * (arm_time / 2)^2 + (v3_half * tmp_time - 1/2 * alpha3 * tmp_time^2);
        v1 = v1_half - alpha1 * tmp_time;
        v2 = v2_half - alpha2 * tmp_time;
        v3 = v3_half - alpha3 * tmp_time;
    end
    
    jt1 = [0,0];
    jt2 = [a1 * cos(h1), a1 * sin(h1)];
    jt3 = [a1 * cos(h1) + a2 * cos(h1 + h2), a1 * sin(h1) + a2 * sin(h1 + h2)];
    jt4 = [a1 * cos(h1) + a2 * cos(h1 + h2) + a3 * cos(h1 + h2 + h3), a1 * sin(h1) + a2 * sin(h1 + h2) + a3 * sin(h1 + h2 + h3)];
    
    T_rst = -double(subs(T, [aa1, aa2, aa3, mm1, mm2, mm3, th_1, th_2, th_3, th_1_d, th_2_d, th_3_d, th_1_dd, th_2_dd, th_3_dd], [a1, a2, a3, m1, m2, m3, h1, h2, h3, v1, v2, v3, alpha1, alpha2, alpha3]));
    
    jtc_array(i, 1, :) = jt1;
    jtc_array(i, 2, :) = jt2;
    jtc_array(i, 3, :) = jt3;
    jtc_array(i, 4, :) = jt4;
    Tc_array(i, :) = T_rst';
end
T_rst = -double(subs(T, [aa1, aa2, aa3, mm1, mm2, mm3, th_1, th_2, th_3, th_1_d, th_2_d, th_3_d, th_1_dd, th_2_dd, th_3_dd], [a1, a2, a3, m1, m2, m3, h1, h2, h3, 0, 0, 0, 0, 0, 0]));
Tc_array = [Tc_array; T_rst'];

save('flip_actions.mat', 'jt_array', 'T_array', 'pc_point_array', 'jtc_array', 'Tc_array');
