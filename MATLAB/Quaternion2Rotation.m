%           George Terzakis, 2017
%
%         University of Portsmouth
%
%      Matlab Code based on the contents of:
%
% "Modified Rodrigues Parameters: An Efficient Reprepsentation of
% Orientation in 3D Vision and Graphics"
% G. Terzakis, M. Lourakis and D. Ait-Boudaoud

% Get a rotation matrix from a unit quaternion.
function R = Quaternion2Rotation(s, v)
% s: The quaternion scalar part.
% v: The quaternion vector part

v1 = v(1); v2 = v(2); v3 = v(3);
R = [s^2+v1^2-v2^2-v3^2 , 2*(v1*v2-s*v3) , 2*(v1*v3+s*v2);
      2*(v1*v2+s*v3) , s^2-v1^2+v2^2-v3^2 , 2*(v2*v3-s*v1); 
      2*(v1*v3-s*v2) , 2*(v2*v3+s*v1), s^2-v1^2-v2^2+v3^2];