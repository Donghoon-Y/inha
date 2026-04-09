function H = jacobian_h_q(q, v)
% jacobian_h_q : H = d(C(q)*v)/dq  →  크기 3x4
% dcm_q (JPL, q=[qx;qy;qz;qw]) 기준, SymPy 심볼릭 미분으로 검증.
% 기존 코드는 12개 원소 중 9개가 틀려 있었음.

    q1 = q(1); q2 = q(2); q3 = q(3); q4 = q(4);
    vx = v(1); vy = v(2); vz = v(3);

    H = 2 * [ ...
    %     ∂/∂q1                    ∂/∂q2                    ∂/∂q3                    ∂/∂q4
      q1*vx+q2*vy+q3*vz,   q1*vy-q2*vx-q4*vz,   q1*vz-q3*vx+q4*vy,  -q2*vz+q3*vy+q4*vx; ...
     -q1*vy+q2*vx+q4*vz,   q1*vx+q2*vy+q3*vz,   q2*vz-q3*vy-q4*vx,   q1*vz-q3*vx+q4*vy; ...
     -q1*vz+q3*vx-q4*vy,  -q2*vz+q3*vy+q4*vx,   q1*vx+q2*vy+q3*vz,  -q1*vy+q2*vx+q4*vz  ];
end