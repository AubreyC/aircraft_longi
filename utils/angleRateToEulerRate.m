function [ eulerAngleDot ] = angleRateToEulerRate(eulerAngles, angleRate)
%Convert angle rate (p, q, r) expressed in Body Frame to Euler angles rate
%(phi_dot, theta_dot, psi_dot)
% Small Unmanned Aircraft, Randal W. Beard, chap 3.3
%Param: 
% angleRate = [p;q;r]
% eulerAngles = [phi; theta; psi]
%Output: 
% eulerAngleDot = [phiDot; thetaDot; psiDot]
phi = eulerAngles(1);
theta = eulerAngles(2);
psi = eulerAngles(3);

mat = [1, sin(phi)*tan(theta), cos(phi)*tan(theta);...
       0, cos(phi),            -sin(phi);...
       0, sin(phi)/cos(theta), cos(phi)/cos(theta)];

eulerAngleDot = mat*angleRate;

end

