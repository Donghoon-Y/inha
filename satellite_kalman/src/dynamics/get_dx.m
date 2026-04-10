function dx = get_dx(t, x, J, Kp, Kd, tau_max, r_eci_sat, v_eci_sat, unit_rho_sun, r_gs_ecef, angle, dt, t_utc0)
% GET_DX  attitude_mode_switch의 dx만 반환하는 래퍼 함수
%
% attitude_mode_switch가 [dx, w_des_out] 두 개를 반환하기 때문에
% q_rungekutta4에 직접 넘길 수 없어서 이 래퍼를 사용합니다.
%
% 입력/출력은 attitude_mode_switch와 동일하되 dx만 반환합니다.
 
dx = attitude_mode_switch(t, x, J, Kp, Kd, tau_max, ...
                          r_eci_sat, v_eci_sat, unit_rho_sun, ...
                          r_gs_ecef, angle, dt, t_utc0);
end
 