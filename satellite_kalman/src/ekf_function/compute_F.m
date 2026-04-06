function F = compute_F(q, w, J)
    % 1. dq_dot/dq (4x4)
    Omega = 0.5 * [ 0,  w(3), -w(2), w(1);
                   -w(3),  0,  w(1), w(2);
                    w(2), -w(1),  0, w(3);
                   -w(1), -w(2), -w(3), 0 ];

    % 2. dq_dot/dw (4x3) - Xi matrix
    Xi = 0.5 * [ q(4), -q(3),  q(2);
                 q(3),  q(4), -q(1);
                -q(2),  q(1),  q(4);
                -q(1), -q(2), -q(3)];

    % 3. dw_dot/dw (3x3)
    J_inv = inv(J);
    skew_w = [0 -w(3) w(2); w(3) 0 -w(1); -w(2) w(1) 0]; % skew 함수 대체
    skew_Jw = [0 -(J(3,3)*w(3)) (J(2,2)*w(2)); (J(3,3)*w(3)) 0 -(J(1,1)*w(1)); -(J(2,2)*w(2)) (J(1,1)*w(1)) 0]; 
    df_dw = J_inv * (skew_Jw - skew_w * J);
    
    F = [Omega, Xi; zeros(3,4), df_dw];
end