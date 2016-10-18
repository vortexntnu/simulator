function R = Reuler(q)
%REULER - Calculates the Euler angle rotation matrix of the input vector.
% The rotation assumes roll-pitch-yaw parametrization.
%
% Syntax: R = Reuler(q)
%
% Input:
%    q - Input vector (3x1)
%    q = [phi theta psi]'
%
% Output:
%    R - Rotation matrix (3x3)

phi   = q(1);
theta = q(2);
psi   = q(3);

r11 =   cos(psi) * cos(theta);
r12 = - sin(psi) * cos(phi) + cos(psi) * sin(theta) * sin(phi);
r13 =   sin(psi) * sin(phi) + cos(psi) * cos(phi) * sin(theta);
r21 =   sin(psi) * cos(theta);
r22 =   cos(psi) * cos(phi) + sin(phi) * sin(theta) * sin(psi);
r23 = - cos(psi) * sin(phi) + sin(theta) * sin(psi) * cos(phi);
r31 = - sin(theta);
r32 =   cos(theta) * sin(phi);
r33 =   cos(theta) * cos(phi);

R = [r11 r12 r13
     r21 r22 r23
     r31 r32 r33];

end
