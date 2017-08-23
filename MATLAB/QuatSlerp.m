%           George Terzakis, 2017
%
%         University of Portsmouth
%
%      Matlab Code based on the contents of:
%
% "Modified Rodrigues Parameters: An Efficient Reprepsentation of
% Orientation in 3D Vision and Graphics"
% G. Terzakis, M. Lourakis and D. Ait-Boudaoud


% SLERP!
% NOTE: In this case, quaternion s are treated as vectors, since the actual
% ordering of the scalar part (before or after the vector part) does not
% matter.
function slerp = QuatSlerp(p, q, t)
    % obtain the angle between the quaternions as unit vectors
    costheta = p'*q;
    if (1 + costheta > 0.00001) % Not close to pi...
        start = p;
        finish = q;
        if (1 - costheta > 0.00001)
            theta = acos(costheta);
            alpha = sin((1 - t) * theta) /sin(theta);
            beta = sin(t * theta) / sin(theta);        
        else
            alpha = 1 - t;
            beta = t;
        end    
    else
        % very close (or equal) to pi
        start = p;
        alpha = 1;
        beta = 0;
        finish = -q;

        end
    end

            
    slerp = alpha * start + beta * finish;
                                  