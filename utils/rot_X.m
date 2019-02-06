function [ rot ] = rot_X( phi )
% Create rotation matrix around X vector in the right-hand positive
% direction
% Phi in Radian
% Roll

rot = [ 1 0 0;
        0 cos(phi) sin(phi);
        0 -sin(phi) cos(phi)];

end

