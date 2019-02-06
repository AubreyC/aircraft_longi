function [ rot ] = rot_y( theta )
% Create rotation matrix around Z vector in the right-hand positive
% direction. theta in radians

rot = [ cos(theta) 0 -sin(theta);
        0          1           0;
       sin(theta) 0  cos(theta)];

end