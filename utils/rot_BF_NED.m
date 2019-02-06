function [ rot ] = rot_BF_NED( phi, theta, psi )
% Create rotation matrix for Vehicle - NED to Body Frame

rot = rot_X(phi)*rot_Y(theta)*rot_Z(psi);

end