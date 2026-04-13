function dx = attitude_ode_sun_pd(t, x, J, Kp, Kd, tau_max, sun_hat_eci_hist, z_b, dt)

    q = x(1:4);   
    w = x(5:7);     

    % normalize quaternion and target 
    q = q / norm(q);

    z_b = z_b / norm(z_b);

    N = size(sun_hat_eci_hist, 1);

    k = round(t) + 1;           
    if k < 1
        k = 1;
    elseif k > N
        k = N;
    end

    s_i = sun_hat_eci_hist(k, :).';   % 3x1
    ns = norm(s_i);
    if ns < 1e-12
        s_i = [1;0;0];               
    else
        s_i = s_i / ns;
    end

    %Sun position vector Body Frame Rotation
    C_bi = dcm_q(q);
    s_b  = C_bi * s_i;
    s_b  = s_b / norm(s_b);  

    prime_axis = cross(s_b, z_b); % 회전축
    axis_dot = dot(s_b, z_b);
    axis_dot = max(min(axis_dot, 1.0), -1.0);

    theta = acos(axis_dot);

    %회전축 Normalize
    if norm(prime_axis) < 1e-6
        q_err_scalar = 1;
        q_err_vec = [0; 0; 0];
    else 
         prime_axis = prime_axis/norm(prime_axis);
         q_err_scalar = cos(theta/2);
         q_err_vec = prime_axis * sin(theta/2);
    end 
       

    sign_q4 = sign(q_err_scalar);

    if sign_q4 == 0 
        sign_q4 = 1; %부호가 없으면 양수처리
    end

    %Control law3 적용

    T_req = -Kp*sign_q4*q_err_vec -Kd*w;

    T = min(tau_max, T_req);

   
    % --- rigid body dynamics ---
    h  = J*w;
    dw = J\(T - cross(w, h));

    % --- quaternion kinematics ---
    dq = quatKinematics(q, w);

    dx = [dq; dw];
end
