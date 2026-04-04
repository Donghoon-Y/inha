function eul = dcm2eul321(C_bi)
    theta = -asin( max(-1,min(1, C_bi(1,3))) );

    % avoid gimbal lock
    cth = cos(theta);
    if abs(cth) < 1e-12
        psi = 0; %gimbal lock이면 yaw를 고정
        phi = atan2(C_bi(2,1), C_bi(2,2));
    else
        phi = atan2(C_bi(2,3), C_bi(3,3));
        psi = atan2(C_bi(1,2), C_bi(1,1));
    end

    eul = [phi; theta; psi];
end
