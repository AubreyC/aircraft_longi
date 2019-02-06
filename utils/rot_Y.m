function [ rot ] = rot_y( theta )
% Create rotation matrix around Y vector in the right-hand positive
% direction
% Theta in Radian
% Pitch 

rot = [ cos(theta) 0 -sin(theta);
        0          1           0;
       sin(theta) 0  cos(theta)];

end