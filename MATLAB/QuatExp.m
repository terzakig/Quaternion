%           George Terzakis, 2017
%
%         University of Portsmouth
%
%      Matlab Code based on the contents of:
%
% "Modified Rodrigues Parameters: An Efficient Reprepsentation of
% Orientation in 3D Vision and Graphics"
% G. Terzakis, M. Lourakis and D. Ait-Boudaoud

% Exponential of 3D vector. 
% NOTE: This exponential incorporates a 1/2 factor so that it maps
% axis-angle vectors to the quaternions that correspond to the actual
% rotation.
function [s, v] = QuatExp(axis)
    
angle = norm(axis);
if (angle^2 > 0.00001)
    v = sin(angle/2) * axis / angle;
else
    v = [0;0;0];
end    
       
s = cos(angle / 2);
