function H = jacobian_h_q(q, v)
    q1 = q(1); q2 = q(2); q3 = q(3); q4 = q(4);
    vx = v(1); vy = v(2); vz = v(3);
    
    % C(q)*v 를 [q1, q2, q3, q4]로 각각 편미분한 행렬
    H = 2 * [ ...
        q1*vx + q2*vy + q3*vz,  -q2*vx + q1*vy - q4*vz, -q3*vx + q4*vy + q1*vz,   q4*vx + q3*vy - q2*vz;
        q2*vx - q1*vy + q4*vz,   q1*vx + q2*vy + q3*vz, -q4*vx - q2*vy + q1*vz,  -q3*vx + q4*vy + q1*vz;
        q3*vx - q4*vy - q1*vz,   q4*vx + q2*vy - q1*vz,  q1*vx + q2*vy + q3*vz,   q2*vx - q1*vy + q4*vz ];
end