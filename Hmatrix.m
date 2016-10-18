function H = Hmatrix(r)
%HMATRIX Summary of this function goes here
%   Detailed explanation goes here

S = skew(r);
H = [  eye(3)     S'
     zeros(3) eye(3)];

end

