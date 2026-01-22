function A = orb2eci(raan, inc)
    A = [cos(raan), -sin(raan)*cos(inc), sin(raan)*sin(inc);
        sin(raan), cos(raan)*cos(inc), -cos(raan)*sin(inc);
        0, sin(inc), cos(inc)];
end