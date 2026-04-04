function [r_sun_eci, v_sun_eci] = sun_eci_from_utc(t_utc)

    %Time
    JD = juliandate(t_utc);
    T = (JD-2451545.0)/36525.0;
    
    %Mean equation
    M = 357.52911 + (35999.05029*T) - (0.0001537*T.^2);
    L0 = 280.46646 + 36000.76983*T + 0.0003032*T.^2;
    L0 = mod(L0,360);
    M  = mod(M,360);
    e = 0.0167086;
    C = 2*e*sind(M) + 5/4*e^2*sind(2*M)+ 13/12*e^3*sind(3*M);
    lam = L0 + C;
    t_a = M + C;

    R_AU = (1 - e.^2) ./ (1 + e.*cosd(t_a));

    eps = 23.439291 - 0.0130042*T; 
    x = cosd(lam);
    y = cosd(eps).*sind(lam);
    z = sind(eps).*sind(lam);

    AU_km = 149597870.7;
    r_sun_eci = [x(:), y(:), z(:)] .* (R_AU(:)*AU_km);
    v_sun_eci = r_sun_eci ./ vecnorm(r_sun_eci,2,2);
end
