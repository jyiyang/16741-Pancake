function desired_th_d = inverse_vel_kinematics(vx, vy, w, a1, a2, a3, th1, th2)
    
    a3 = a3 / 2;
    J = [-a1*sin(th1)-a2*sin(th1+th2), -a2*sin(th1+th2), 0; a1*cos(th1)+a2*cos(th1+th2)+a3, a2*cos(th1+th2)+a3, a3;1, 1, 1];
    desired_th_d = J \ [vx; vy; w];

end