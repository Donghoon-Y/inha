function dq = quatKinematics(q, w)
    % scalar-last
    wq = [w; 0];                 % [wx;wy;wz;0]
    dq = 0.5 * quatMultiply(q, wq);
end
