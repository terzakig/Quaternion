%           George Terzakis, 2017
%
%         University of Portsmouth
%
%      Matlab Code based on the contents of:
%
% "Modified Rodrigues Parameters: An Efficient Reprepsentation of
% Orientation in 3D Vision and Graphics"
% G. Terzakis, M. Lourakis and D. Ait-Boudoud

% The derivative of the quaternion scalar WRT MRPs
function dsdpsi = QuatScalarJacWRTMRPs(s, v)
% s: The quaternion scalar part
% v: The quaternion vector part

% Simple formula: -(1 + s) * v'
dsdpsi = -(1 + s) * [v(1), v(2), v(3)];