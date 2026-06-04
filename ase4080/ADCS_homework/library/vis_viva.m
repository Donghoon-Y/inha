%% Vis-viva eq
%  For any Keplerian Orbit.
% 
%  Input
%       r0_ECI [km] [3x1]
%       v0_ECI [km] [3x1]
%       output_idx : 1 = a / 2 = v / 3 = r
%  Output
%       a (Semimajor axis) [km] [1x1]
%  Parameter
%       Mu = 398600.4418 (Earth gravitational constant) [1x1] [km3 / s2]     @WGS84
%

function output = vis_viva(r,v,a,idx)
mu = 398600.4418;
r = norm(r);
v = norm(v);

if idx == 1
    % Semi Major Axis - a
    output = 1/(2/r - v^2/mu);
elseif idx == 2
    % Speed - v
    output = sqrt(mu*(2/r-1/a));
elseif idx == 3
    % Distance - r
    output = 1/( v^2/(2*mu) + 1/(2*a) );
else
    fprintf('Invalid Index.')
end

end



