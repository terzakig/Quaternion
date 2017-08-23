%           George Terzakis, 2017
%
%         University of Portsmouth
%
%      Matlab Code based on the contents of:
%
% "Modified Rodrigues Parameters: An Efficient Reprepsentation of
% Orientation in 3D Vision and Graphics"
% G. Terzakis, M. Lourakis and D. Ait-Boudaoud

% MRPs to quaternion
function [s, v] = MRPs2Quaternion(psi)
v = 2*psi / (1 + norm(psi)^2);
s = ( 1 - norm(psi)^2 ) / ( 1 + norm(psi)^2 );
