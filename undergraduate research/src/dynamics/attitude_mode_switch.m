function dx = attitude_mode_switch(t, x, J, Kp, Kd, tau_max, r_eci_sat, v_eci_sat, unit_rho_sun, r_gs_ecef, angle,dt)
     %상태변수
     q = x(1:4);
     w = x(5:7);
    
     %Normalize
     q = q/norm(q);

     C_bi = dcm_q(q);

     %시간 매칭
     k = round(t/dt) + 1;

     if k < 1, k =1; end
     if k > size(r_eci_sat, 1), k = size(r_eci_sat, 1); end
     %현재상태(m로 통일)
     r_sat = r_eci_sat(k, :).'*1000; %3*1로 맞추기
     v_sat = v_eci_sat(k, :).'*1000;
     sun_point = unit_rho_sun(k,:).'; %단위벡터

     %지상국 ECI로 변환
     We = 7.292115e-5; % [rad/s]
     theta = We * t;   % 회전각
     R_ecef2eci = [cos(theta), -sin(theta), 0;
                  sin(theta),  cos(theta), 0;
                           0,           0, 1];

     r_gs_eci = R_ecef2eci*r_gs_ecef;
     v_gs_eci = cross([0;0;We], r_gs_eci);

     %모드 변환 기준 설정
     r_rel = r_gs_eci - r_sat;

     u_zenith = r_gs_eci/norm(r_gs_eci);
     u_gs2sat = -r_rel/norm(r_rel);

     sin_elv = dot(u_gs2sat,u_zenith);
     elv = rad2deg(asin(sin_elv));
     f_dot = norm(cross(r_sat, v_sat))/norm(r_sat)^2;

     if elv >= angle
         MODE = 'GROUND' ;
     else
         MODE = 'SUN';
     end

     if strcmp(MODE, 'GROUND')
         v_rel = v_gs_eci - v_sat;
         u_vec = r_rel/norm(r_rel);
         u_dot = (eye(3)-u_vec*u_vec')*v_rel/norm(r_rel);
         w_des = cross(u_vec, v_rel)/norm(r_rel);
         w_dot_des = cross(u_vec, r_sat)*(f_dot)^2/norm(r_rel) - 2*w_des*u_vec'*v_rel/norm(r_rel);

         %Target Frame 설정
         u_vec_body = C_bi*u_vec;
         x_b = [1; 0; 0]; 

         prime_axis = cross(u_vec_body, x_b); % 회전축
         axis_dot = dot(u_vec_body, x_b);
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

         q_err = [q_err_vec; q_err_scalar];
        
     else
         w_des = [0; 0; 0];
         w_dot_des = [0; 0; 0];
         x_b = [1; 0; 0];
         s_i = sun_point;   % 3x1
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

         prime_axis = cross(s_b, x_b); % 회전축
         axis_dot = dot(s_b, x_b);
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

         q_err = [q_err_vec; q_err_scalar];
     end

     
     w_err = w-C_bi*w_des;

     T_feedforward = J*(C_bi*w_dot_des);
     
     T_gyro = cross(w, J*w);

     sign_q4 = sign(q_err(4));
     if sign_q4 == 0, sign_q4 = 1; end

     T_req = -Kp*sign_q4*q_err_vec -Kd*w_err + T_feedforward + T_gyro;

     T = max(min(T_req, tau_max), -tau_max);

   
    % --- rigid body dynamics ---
     h  = J*w;
     dw = J\(T - cross(w, h));

    % --- quaternion kinematics ---
     dq = quatKinematics(q, w);

     dx = [dq; dw];

end