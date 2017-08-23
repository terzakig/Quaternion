%           George Terzakis, 2017
%
%         University of Portsmouth
%
%      Matlab Code based on the contents of:
%
% "Modified Rodrigues Parameters: An Efficient Reprepsentation of
% Orientation in 3D Vision and Graphics"
% G. Terzakis, M. Lourakis and D. Ait-Boudaoud

% Jacobian of the quaternion vector part WRT axis-angle parameters
function dvdu = QuatVectorJacWRTAA(s, v)

% s : The quaternion scalar part.
% v : The quaternion vector part.

% Get the logarithm of the quaternion
u = QuatLog(s, v);
u1 = u(1); u2 = u(2); u3 = u(3);
theta = norm(u);
if (theta < 10e-5)
    dvdu = eye(3);
else
    % the derivative (formula lifted verbatim from Diebel's report on
    % orientation parametrizations - Stanford university)   
    dvdu = [2*(u2^2+u3^2)*sin(theta/2)+(u1^2)*theta*cos(theta/2) , u1*u2*(theta*cos(theta/2)-2*sin(theta/2)) ,          u1*u3* (theta*cos(theta/2)-2*sin(theta/2));
         u1*u2*(theta*cos(theta/2)-2*sin(theta/2)) ,          2*(u1^2+u3^2)*sin(theta/2)+(u2^2)*theta*cos(theta/2) , u2*u3* (theta*cos(theta/2)-2*sin(theta/2));
         u1*u3*(theta*cos(theta/2)-2*sin(theta/2)) ,          u2*u3*(theta*cos(theta/2)-2*sin(theta/2))   ,        2*(u1^2+u2^2)*sin(theta/2)+(u3^2)*theta*cos(theta/2)];

    dvdu = dvdu / (2*theta^3);
end
