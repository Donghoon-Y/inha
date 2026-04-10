function [dx, w_des_out] = attitude_mode_switch(t, x, J, Kp, Kd, tau_max, r_eci_sat, v_eci_sat, unit_rho_sun, r_gs_ecef, angle, dt, t_utc0)
    q = x(1:4);
    w = x(5:7);
    q = q/norm(q);
    C_bi = dcm_q(q);

    k = round(t/dt) + 1;
    if k < 1, k = 1; end
    if k > size(r_eci_sat, 1), k = size(r_eci_sat, 1); end

    t_current = t_utc0 + seconds(t);
    r_sat = r_eci_sat(k, :).'*1000;
    v_sat = v_eci_sat(k, :).'*1000;
    sun_point = unit_rho_sun(k,:).';

    r_gs_eci = ecef2eci(t_current, r_gs_ecef);
    We_vec = [0; 0; 7.292115e-5];
    v_gs_eci = cross(We_vec, r_gs_eci);

    r_rel = r_gs_eci - r_sat;
    u_zenith = r_gs_eci/norm(r_gs_eci);
    u_gs2sat = -r_rel/norm(r_rel);
    sin_elv = dot(u_gs2sat, u_zenith);
    elv = rad2deg(asin(sin_elv));
    f_dot = norm(cross(r_sat, v_sat))/norm(r_sat)^2;

    if elv >= angle
        MODE = 'GROUND';
    else
        MODE = 'SUN';
    end

    if strcmp(MODE, 'GROUND')
        v_rel = v_gs_eci - v_sat;
        u_vec = r_rel/norm(r_rel);
        w_des = cross(u_vec, v_rel)/norm(r_rel);          % ← 여기서 w_des 계산
        w_dot_des = cross(u_vec, r_sat)*(f_dot)^2/norm(r_rel) - 2*w_des*u_vec'*v_rel/norm(r_rel);

        u_vec_body = C_bi*u_vec;
        x_b = [1; 0; 0];
        prime_axis = cross(u_vec_body, x_b);
        axis_dot = max(min(dot(u_vec_body, x_b), 1.0), -1.0);
        theta = acos(axis_dot);

        if norm(prime_axis) < 1e-6
            q_err_scalar = 1; q_err_vec = [0; 0; 0];
        else
            prime_axis = prime_axis/norm(prime_axis);
            q_err_scalar = cos(theta/2);
            q_err_vec = prime_axis * sin(theta/2);
        end
        q_err = [q_err_vec; q_err_scalar];

    else
        w_des = [0; 0; 0];                                % ← Sun 모드는 0
        w_dot_des = [0; 0; 0];
        x_b = [1; 0; 0];
        s_i = sun_point;
        if norm(s_i) < 1e-12, s_i = [1;0;0]; else, s_i = s_i/norm(s_i); end

        C_bi = dcm_q(q);
        s_b = C_bi * s_i;
        s_b = s_b / norm(s_b);

        prime_axis = cross(s_b, x_b);
        axis_dot = max(min(dot(s_b, x_b), 1.0), -1.0);
        theta = acos(axis_dot);

        if norm(prime_axis) < 1e-6
            q_err_scalar = 1; q_err_vec = [0; 0; 0];
        else
            prime_axis = prime_axis/norm(prime_axis);
            q_err_scalar = cos(theta/2);
            q_err_vec = prime_axis * sin(theta/2);
        end
        q_err = [q_err_vec; q_err_scalar];
    end

    w_err = w - C_bi*w_des;
    T_feedforward = J*(C_bi*w_dot_des);
    T_gyro = cross(w, J*w);

    sign_q4 = sign(q_err(4));
    if sign_q4 == 0, sign_q4 = 1; end

    T_req = -Kp*sign_q4*q_err_vec - Kd*w_err + T_feedforward + T_gyro;
    T = max(min(T_req, tau_max), -tau_max);

    h  = J*w;
    dw = J\(T - cross(w, h));
    dq = quatKinematics(q, w);

    dx = [dq; dw];
    w_des_out = C_bi * w_des;   % ← 바디→ECI 변환해서 반환 (ECI 기준으로 통일)
end