%           George Terzakis, 2017
%
%         University of Portsmouth
%
%      Matlab Code based on the contents of:
%
% "Modified Rodrigues Parameters: An Efficient Reprepsentation of
% Orientation in 3D Vision and Graphics"
% G. Terzakis, M. Lourakis and D. Ait-Boudaoud

% quaternion logarithm (the axis-angle vector of the ROTATION, i.e. 2*θ)
function u = QuatLog(s, v)
       
cosq = s;
angle = 2 * acos(cosq);
    
u = v;
if abs(angle) < 1e-4
    u = u / (0.5 - angle^2 / 48);
else
    u = u * angle / sin(angle / 2);
 end
    
