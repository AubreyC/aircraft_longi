function [ rot ] = rot_BF_NED( phi, theta, psi )
% Create frame rotation matrix: NED to Body Frame
% Angles in radians

rot = rot_X(phi)*rot_Y(theta)*rot_Z(psi);

end