%           George Terzakis, 2017
%
%         University of Portsmouth
%
%      Matlab Code based on the contents of:
%
% "Modified Rodrigues Parameters: An Efficient Reprepsentation of
% Orientation in 3D Vision and Graphics"
% G. Terzakis, M. Lourakis and D. Ait-Boudaoud

% Rotation matrix derivative WRT axis-angle #i
% Employing the Gallego - Yezzi formula
function dRdui = RotationJacWRTaxisangle(u, i)
theta = norm(u);
if theta < 0.0001
    % just using the infinitesimal generators at the identity
    if i == 1
        G = [0 0 0 ; 0 0 -1; 0 1 0];
    elseif i == 2
        G = [0 0 1; 0 0 0; -1 0 0];
    else
        G = [0 -1 0 ; 1 0 0; 0 0 0];
    end
    dRdui = G;
else
    Ux = [0 -u(3) u(2); u(3) 0 -u(1); -u(2) u(1) 0];
    R = eye(3) + sin(theta)/theta*Ux + (1-cos(theta))/(theta^2)*Ux^2;
    ei = [0; 0; 0];
    ei(i) = 1;
    temp = cross(u, (eye(3) -R)*ei);
    Tmpx = [0 -temp(3) temp(2); temp(3) 0 -temp(1); -temp(2) temp(1) 0]; 
    dRdui = (u(i) * Ux + Tmpx)/theta^2 * R;
end;