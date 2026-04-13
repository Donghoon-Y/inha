function dx = attitude_ode_pd(~, x, J, qd, wd,Kp, Kd, tau_max)
    q = x(1:4);         % [qx;qy;qz;qw]  (I->B)
    w = x(5:7);         % body rates [rad/s]

    qe = quatMultiply(qd, quatConj(q));

    % unwinding prevention: choose shortest rotation
    if qe(4) < 0
        qe = -qe;
    end

    eq = qe(1:3);   % vector part
    ew = (w-wd);
    T = -Kp*eq - Kd*ew;

    if ~isempty(tau_max)
        T = min(max(T, -tau_max), tau_max);
    end

    % --- rigid body dynamics ---
    h  = J*w;
    dw = J\(T - cross(w, h));

    % --- quaternion kinematics ---
    dq = quatKinematics(q, w);  

    dx = [dq; dw];
end
