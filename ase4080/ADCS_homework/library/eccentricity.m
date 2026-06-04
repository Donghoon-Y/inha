%% eccentricity (vector)
%  For any Keplerian Orbit.
% 
%  Input
%       r0_ECI [km] [3x1]
%       v0_ECI [km] [3x1]
%  Output
%       e (Eccentricity vector) [] [3x1]
%       ee (Eccentricity) [] [1x1]
%  Parameter
%       Mu = 398600.4418 (Earth gravitational constant) [1x1] [km3 / s2]     @WGS84
%

function [e,ee] = eccentricity(r,v)
mu = 398600.4418;
h = cross(r,v);

e = 1/mu * ( cross(v,h)-r*(mu/norm(r)) );
ee = norm(e);

if norm(ee) < 0.0001
    e = [];
    ee = 0;
end



