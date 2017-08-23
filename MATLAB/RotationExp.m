%           George Terzakis, 2017
%
%         University of Portsmouth
%
%      Matlab Code based on the contents of:
%
% "Modified Rodrigues Parameters: An Efficient Reprepsentation of
% Orientation in 3D Vision and Graphics"
% G. Terzakis, M. Lourakis and D. Ait-Boudaoud

% The Rodrigues formula
function R = RotationExp(u)
theta = norm(u);
if theta < 0.00001
    % 3d order approximating sin(theta)/theta
    alpha = 1 + theta^2 / 6;
    % 2nd order approximation of (1-cos(theta))/theta
    beta = 0.5;
else
    alpha = sin(theta)/theta;
    beta = (1-cos(theta))/theta^2;
end
Ux = [0 -u(3) u(2); u(3) 0 -u(1); -u(2) u(1) 0];
R = eye(3) + alpha *Ux + beta * Ux^2;
