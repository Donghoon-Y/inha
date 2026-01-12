function dx = attitude_ode(t, x, J, T)
    w = x(5:7);
    q = x(1:4);
    h = J*w;
    omega = [0, w(3), -w(2), w(1);
            -w(3), 0, w(1), w(2);
            w(2), -w(1), 0, w(3);
            -w(1), -w(2), -w(3), 0];

    dq = 0.5*omega*q;
    dw = J\(T-cross(w,h));

    dx = [dq; dw];

end