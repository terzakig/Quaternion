%           George Terzakis, 2017
%
%         University of Portsmouth
%
%      Matlab Code based on the contents of:
%
% "Modified Rodrigues Parameters: An Efficient Reprepsentation of
% Orientation in 3D Vision and Graphics"
% G. Terzakis, M. Lourakis and D. Ait-Boudaoud

% SLERP derivative!
function dslerp = QuatSlerpDeriv(p, q, t)
    % obtain the angle between the quaternions as unit vectors
    Phi = acos(p'*q);
    if abs(Phi) < 0.00001
        dslerp = -cos((1-t)*Phi)*p + sin(t*Phi)*q; 
    else
        dslerp = Phi * (-cos((1-t)*Phi)*p + sin(t*Phi)*q)/sin(Phi);
    end
