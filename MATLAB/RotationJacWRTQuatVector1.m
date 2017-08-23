%           George Terzakis, 2017
%
%         University of Portsmouth
%
%      Matlab Code based on the contents of:
%
% "Modified Rodrigues Parameters: An Efficient Reprepsentation of
% Orientation in 3D Vision and Graphics"
% G. Terzakis, M. Lourakis and D. Ait-Boudaoud

% derivative of a rotation matrix wrt v1
function dRdv1 = RotationJacWRTQuatVector1(s, v)
% INPUT
% s: The quaternion scalar part
% v: the quaternion vector part

v1=v(1); v2 = v(2); v3 = v(3);

dRdv1 = 2*[v1 v2 v3; 
           v2 -v1 -s; 
           v3 s -v1];
