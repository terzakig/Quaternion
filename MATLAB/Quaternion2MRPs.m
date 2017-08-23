%           George Terzakis, 2017
%
%         University of Portsmouth
%
%      Matlab Code based on the contents of:
%
% "Modified Rodrigues Parameters: An Efficient Reprepsentation of
% Orientation in 3D Vision and Graphics"
% G. Terzakis, M. Lourakis and D. Ait-Boudaoud

% Quaternion 2 MRPs
function psi = Quaternion2MRPs(s, v)
if abs(s+1) < 0.000001 
    psi = sqrt(10000)*[1;1;1];
else
    psi = v / (1 + s);
end
