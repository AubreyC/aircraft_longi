function [ rot ] = rot_Z( psi )
% Create rotation matrix around Z vector in the right-hand positive
% direction. psi in radians

rot = [ cos(psi) sin(psi)   0; 
       -sin(psi) cos(psi)   0;
        0           0       1];

end
