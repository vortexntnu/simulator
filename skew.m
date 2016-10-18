function S = skew(x)
%SKEW - Calculates the skew-symmetrix matrix of the input vector.
%
% Syntax:  S = skew(x)
%
% Inputs:
%    x - Input vector (3x1)
%
% Outputs:
%    S - Skew-symmetric form of input (3x3)

S = [   0   -x(3)   x(2);
      x(3)     0   -x(1);
     -x(2)   x(1)     0 ];

end
