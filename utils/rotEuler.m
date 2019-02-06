function [ rot ] = rotEuler( phi, theta, psi )
% Create rotation matrix for the Euler Angle
rot = rot_X(phi)*rot_Y(theta)*rot_Z(psi);

end