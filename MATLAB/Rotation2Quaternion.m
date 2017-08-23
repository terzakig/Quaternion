%           George Terzakis, 2017
%
%         University of Portsmouth
%
%      Matlab Code based on the contents of:
%
% "Modified Rodrigues Parameters: An Efficient Reprepsentation of
% Orientation in 3D Vision and Graphics"
% G. Terzakis, M. Lourakis and D. Ait-Boudaoud

% rotation to quaternion. Using log(R)
function [s, v] = Rotation2Quaternion(R)

Ux = RotationLog(R);
u = [Ux(3, 2); Ux(1, 3); Ux(2, 1)];
theta = norm(u);

if theta < 0.00001
    s = 1; v = [0; 0; 0];
else
    s = cos(theta/2); v = [sin(theta/2)*u/theta];
end
