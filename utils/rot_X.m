function [ rot ] = rot_X( phi )
% Create rotation matrix around Z vector in the right-hand positive
% direction. phi in radians


rot = [ 1 0 0;
        0 cos(phi) sin(phi);
        0 -sin(phi) cos(phi)];

end

