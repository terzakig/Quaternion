%           George Terzakis, 2017
%
%         University of Portsmouth
%
%      Matlab Code based on the contents of:
%
% "Modified Rodrigues Parameters: An Efficient Reprepsentation of
% Orientation in 3D Vision and Graphics"
% G. Terzakis, M. Lourakis and D. Ait-Boudoud

% The derivative of the quaternion vector part WRT MRPs
function dvdpsi = QuatVectorJacWRTMRPs(s, v)
% s: The quaternion scalar part
% v: The quaternion vector part

% Simple formula: (1 + s) * I3 - v*v'
dvdpsi = (1 + s) * eye(3) - v*v';