function q = quatMultiply(q1, q2)
    % q = q1 ⊗ q2  , scalar-last [v;s]
    v1 = q1(1:3);  s1 = q1(4);
    v2 = q2(1:3);  s2 = q2(4);

    v = s1*v2 + s2*v1 + cross(v1, v2);
    s = s1*s2 - dot(v1, v2);

    q = [v; s];
end
