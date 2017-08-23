%           George Terzakis, 2017
%
%         University of Portsmouth
%
%      Matlab Code based on the contents of:
%
% "Modified Rodrigues Parameters: An Efficient Reprepsentation of
% Orientation in 3D Vision and Graphics"
% G. Terzakis, M. Lourakis and D. Ait-Boudaoud

% The Jacobian of the quaternion scalar part in terms of thge AXis-Angle parameters
function dsdu = QuatScalarJacWRTAA(s, v)
% s : Quaternion scalar part.
% v : The quaternion vector part.

% Easy peesy...
dsdu = -0.5 * [v(1), v(2), v(3)];
