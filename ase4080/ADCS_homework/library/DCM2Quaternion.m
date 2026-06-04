 %% Conversion of DCM to Quaternion
%  Input
%       DCM
%  Output
%       Quaternion
%

function [Quaternion] = DCM2Quaternion(DCM)

q4 = 1/2 * sqrt(1 + DCM(1, 1) + DCM(2, 2) + DCM(3, 3));
q1 = 1/(4*q4) * (DCM(2, 3) - DCM(3, 2));
q2 = 1/(4*q4) * (DCM(3, 1) - DCM(1, 3));
q3 = 1/(4*q4) * (DCM(1, 2) - DCM(2, 1));

Quaternion = [q1 q2 q3 q4]';

end