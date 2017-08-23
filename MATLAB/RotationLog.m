%           George Terzakis, 2017
%
%         University of Portsmouth
%
%      Matlab Code based on the contents of:
%
% "Modified Rodrigues Parameters: An Efficient Reprepsentation of
% Orientation in 3D Vision and Graphics"
% G. Terzakis, M. Lourakis and D. Ait-Boudaoud

% Rotation logarithm
function Ux = RotationLog(R)
theta = acos((trace(R)-1)*0.5);
if (theta < 0.00001)
    Ux = 0.5 * (R-R');
else
    Ux = 0.5 * theta / sin(theta) * (R - R');
end